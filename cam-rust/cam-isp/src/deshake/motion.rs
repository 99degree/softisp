//! Motion estimation for deshake — block matching, diamond search, weighted median.

/// Motion estimation result for a single block.
#[derive(Debug, Clone, Copy)]
pub struct BlockMatch {
    pub dx: f32,
    pub dy: f32,
    pub cost: f32,
}

/// Compute SAD between two blocks, normalized by block area.
pub fn block_sad(
    curr: &[u8], cw: u32,
    prev: &[u8], pw: u32,
    cx: u32, cy: u32,
    sx: i32, sy: i32,
    size: u32,
) -> f32 {
    let mut sad = 0.0f32;
    for dy in 0..size {
        for dx in 0..size {
            let cu = ((cy + dy) * cw + (cx + dx)) as usize;
            let pu = ((sy + dy as i32) * pw as i32 + (sx + dx as i32)) as usize;
            let a = curr.get(cu).copied().unwrap_or(0) as f32;
            let b = prev.get(pu).copied().unwrap_or(0) as f32;
            sad += (a - b).abs();
        }
    }
    sad / (size * size) as f32
}

/// Compute variance of a block (texture metric). Low = flat region.
pub fn block_variance(data: &[u8], stride: u32, bx: u32, by: u32, size: u32) -> f32 {
    let mut sum = 0.0f32;
    let mut sum_sq = 0.0f32;
    let count = (size * size) as f32;
    for dy in 0..size {
        for dx in 0..size {
            let idx = ((by + dy) * stride + (bx + dx)) as usize;
            let v = data.get(idx).copied().unwrap_or(0) as f32;
            sum += v;
            sum_sq += v * v;
        }
    }
    let mean = sum / count;
    sum_sq / count - mean * mean
}

/// 3-step logarithmic diamond search for motion estimation.
/// Checks at most 1 + 8*log2(search_radius) positions instead of
/// (2*radius+1)^2 for full scan. Typically 25-33 SAD ops vs 1089.
pub fn diamond_search(
    curr_gray: &[u8], cw: u32,
    prev_gray: &[u8], pw: u32, ph: u32,
    bx: u32, by: u32,
    block_size: u32,
    search_radius: i32,
) -> Option<BlockMatch> {
    let cx = bx as i32;
    let cy = by as i32;
    let max_x = (pw as i32) - block_size as i32;
    let max_y = (ph as i32) - block_size as i32;

    // Initialize at current position (no motion)
    let zero_cost = block_sad(curr_gray, cw, prev_gray, pw, bx, by, cx, cy, block_size);
    let mut best_dx = 0i32;
    let mut best_dy = 0i32;
    let mut best_cost = zero_cost;

    // 3-step search: start with biggest step, halve each iteration
    let mut step = (search_radius as u32).next_power_of_two() as i32 / 2;
    while step >= 1 {
        let mut improved = false;
        // Check 4 cardinal directions
        for (dsx, dsy) in &[(step, 0), (-step, 0), (0, step), (0, -step)] {
            let sx = cx + best_dx + dsx;
            let sy = cy + best_dy + dsy;
            if sx < 0 || sy < 0 || sx > max_x || sy > max_y {
                continue;
            }
            let cost = block_sad(curr_gray, cw, prev_gray, pw, bx, by, sx, sy, block_size);
            if cost < best_cost {
                best_cost = cost;
                best_dx += dsx;
                best_dy += dsy;
                improved = true;
            }
        }
        if !improved {
            // Try 4 diagonal directions if cardinal didn't help
            for (dsx, dsy) in &[(step, step), (step, -step), (-step, step), (-step, -step)] {
                let sx = cx + best_dx + dsx;
                let sy = cy + best_dy + dsy;
                if sx < 0 || sy < 0 || sx > max_x || sy > max_y {
                    continue;
                }
                let cost = block_sad(curr_gray, cw, prev_gray, pw, bx, by, sx, sy, block_size);
                if cost < best_cost {
                    best_cost = cost;
                    best_dx += dsx;
                    best_dy += dsy;
                    improved = true;
                }
            }
        }
        if !improved {
            step /= 2;
        }
    }

    Some(BlockMatch {
        dx: best_dx as f32,
        dy: best_dy as f32,
        cost: best_cost,
    })
}

/// Full exhaustive search for motion estimation.
/// O((2R+1)^2) SADs — use only for small search radii or debugging.
pub fn full_search(
    curr_gray: &[u8], cw: u32,
    prev_gray: &[u8], pw: u32, ph: u32,
    bx: u32, by: u32,
    block_size: u32,
    search_radius: i32,
) -> Option<BlockMatch> {
    let cx = bx as i32;
    let cy = by as i32;
    let max_x = (pw as i32) - block_size as i32;
    let max_y = (ph as i32) - block_size as i32;

    let mut best = BlockMatch { dx: 0.0, dy: 0.0, cost: f32::MAX };

    for sy in (cy - search_radius).max(0)..=(cy + search_radius).min(max_y) {
        for sx in (cx - search_radius).max(0)..=(cx + search_radius).min(max_x) {
            let cost = block_sad(curr_gray, cw, prev_gray, pw, bx, by, sx, sy, block_size);
            if cost < best.cost {
                best = BlockMatch {
                    dx: (sx - cx) as f32,
                    dy: (sy - cy) as f32,
                    cost,
                };
            }
        }
    }

    Some(best)
}

/// Parabolic interpolation for sub-pixel refinement.
/// Fits a parabola to the SAD curve at (x-1, x, x+1) and returns the minimum.
pub fn parabolic_refine(
    curr_gray: &[u8], cw: u32,
    prev_gray: &[u8], pw: u32,
    bx: u32, by: u32,
    best_dx: f32, best_dy: f32,
    block_size: u32,
) -> (f32, f32) {
    let cx = bx as f32;
    let cy = by as f32;
    let bx_u = bx;
    let by_u = by;

    // Refine X axis
    let cost_left = block_sad(curr_gray, cw, prev_gray, pw, bx_u, by_u,
        (cx + best_dx - 1.0) as i32, (cy + best_dy) as i32, block_size);
    let cost_center = block_sad(curr_gray, cw, prev_gray, pw, bx_u, by_u,
        (cx + best_dx) as i32, (cy + best_dy) as i32, block_size);
    let cost_right = block_sad(curr_gray, cw, prev_gray, pw, bx_u, by_u,
        (cx + best_dx + 1.0) as i32, (cy + best_dy) as i32, block_size);

    let denom_x = 2.0 * (cost_left + cost_right - 2.0 * cost_center);
    let refined_dx = if denom_x.abs() > 1e-6 {
        best_dx - (cost_right - cost_left) / denom_x
    } else {
        best_dx
    };

    // Refine Y axis
    let cost_left = block_sad(curr_gray, cw, prev_gray, pw, bx_u, by_u,
        (cx + refined_dx) as i32, (cy + best_dy - 1.0) as i32, block_size);
    let cost_center = block_sad(curr_gray, cw, prev_gray, pw, bx_u, by_u,
        (cx + refined_dx) as i32, (cy + best_dy) as i32, block_size);
    let cost_right = block_sad(curr_gray, cw, prev_gray, pw, bx_u, by_u,
        (cx + refined_dx) as i32, (cy + best_dy + 1.0) as i32, block_size);

    let denom_y = 2.0 * (cost_left + cost_right - 2.0 * cost_center);
    let refined_dy = if denom_y.abs() > 1e-6 {
        best_dy - (cost_right - cost_left) / denom_y
    } else {
        best_dy
    };

    (refined_dx, refined_dy)
}

/// Cost-weighted median of motion vectors.
/// Used to aggregate per-block motion vectors into a single global vector.
pub fn weighted_median(values: &[f32], weights: &[f32]) -> f32 {
    assert_eq!(values.len(), weights.len());
    if values.is_empty() {
        return 0.0;
    }
    let mut pairs: Vec<(f32, f32)> = values.iter().zip(weights.iter()).map(|(&v, &w)| (v, 1.0 / (w + 1e-6))).collect();
    pairs.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    let total_weight: f32 = pairs.iter().map(|(_, w)| w).sum();
    let mut acc = 0.0;
    for (v, w) in &pairs {
        acc += w;
        if acc >= total_weight * 0.5 {
            return *v;
        }
    }
    pairs.last().map_or(0.0, |(v, _)| *v)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_block_sad_identical() {
        let data = vec![128u8; 64 * 64];
        let sad = block_sad(&data, 64, &data, 64, 16, 16, 16, 16, 8);
        assert_eq!(sad, 0.0);
    }

    #[test]
    fn test_block_sad_different() {
        let a = vec![100u8; 64 * 64];
        let b = vec![200u8; 64 * 64];
        let sad = block_sad(&a, 64, &b, 64, 0, 0, 0, 0, 8);
        assert!((sad - 100.0).abs() < 0.01);
    }

    #[test]
    fn test_block_variance_flat() {
        let data = vec![128u8; 64 * 64];
        let var = block_variance(&data, 64, 0, 0, 8);
        assert_eq!(var, 0.0);
    }

    #[test]
    fn test_weighted_median_single() {
        assert_eq!(weighted_median(&[5.0], &[1.0]), 5.0);
    }

    #[test]
    fn test_weighted_median_three() {
        let v = weighted_median(&[1.0, 5.0, 10.0], &[1.0, 10.0, 1.0]);
        assert!((v - 5.0).abs() < 0.5);
    }

    #[test]
    fn test_weighted_median_outlier_suppressed() {
        let v = weighted_median(&[0.0, 5.0, 100.0], &[1.0, 100.0, 1.0]);
        assert!((v - 5.0).abs() < 1.0);
    }
}
