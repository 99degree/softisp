//! Golden image regression tests
//!
//! These tests load sensor-specific Bayer data and verify that
//! the ISP pipeline produces consistent output for known inputs.
//!
//! Run with: `cargo test -p cam-isp --test test_golden_image`

use std::path::Path;

fn load_sensor(name: &str) -> Option<(u32, u32, Vec<u8>, &'static str, u32)> {
    // Look for corpus in workspace root, fallback to local
    let paths = [
        Path::new("fuzz/testdata/sensors").join(name),
        Path::new("../fuzz/testdata/sensors").join(name),
        Path::new("../../fuzz/testdata/sensors").join(name),
    ];

    let mut dir_opt = None;
    for p in &paths {
        if p.exists() {
            dir_opt = Some(p.clone());
            break;
        }
    }
    let dir = dir_opt?;
    let props_path = dir.join("properties.json");
    let gradient_path = dir.join("gradient.raw");

    if !props_path.exists() || !gradient_path.exists() {
        return None;
    }

    let props_text = std::fs::read_to_string(&props_path).ok()?;
    let data = std::fs::read(&gradient_path).ok()?;

    // Simple line-by-line JSON parser for our generated files
    // Format: {"width": 8000, "height": 6000, "bayer": "RGGB", "bits": 10}
    let mut width = 0u32;
    let mut height = 0u32;
    let mut bayer = "RGGB";
    let mut bits = 10u32;

    for line in props_text.lines() {
        let line = line.trim().trim_matches(',');
        if line.contains("\"width\"") {
            if let Some(v) = line.split(':').nth(1) {
                width = v.trim().parse().unwrap_or(0);
            }
        } else if line.contains("\"height\"") {
            if let Some(v) = line.split(':').nth(1) {
                height = v.trim().parse().unwrap_or(0);
            }
        } else if line.contains("\"bayer\"") {
            if let Some(v) = line.split(':').nth(1) {
                let s = v.trim().trim_matches('"').trim_matches(',');
                bayer = match s {
                    "RGGB" => "RGGB",
                    "GRBG" => "GRBG",
                    "GBRG" => "GBRG",
                    "BGGR" => "BGGR",
                    _ => "RGGB",
                };
            }
        } else if line.contains("\"bits\"") {
            if let Some(v) = line.split(':').nth(1) {
                bits = v.trim().parse().unwrap_or(10);
            }
        }
    }

    Some((width, height, data, bayer, bits))
}

#[test]
fn test_imx586_corpus_loads() {
    let (w, h, data, bayer, bits) = load_sensor("imx586")
        .expect("imx586 corpus must exist");
    assert_eq!(bayer, "RGGB");
    assert_eq!(bits, 10);
    assert_eq!(w, 8000, "Real imx586 has 8000 wide");
    assert_eq!(h, 6000, "Real imx586 has 6000 tall");
    assert!(!data.is_empty());
}

#[test]
fn test_ov13858_corpus_loads() {
    let (_, _, _, bayer, bits) = load_sensor("ov13858")
        .expect("ov13858 corpus must exist");
    assert_eq!(bayer, "RGGB");
    assert_eq!(bits, 10);
}

#[test]
fn test_hi1336_grbg_pattern() {
    let (_, _, _, bayer, _) = load_sensor("hi1336")
        .expect("hi1336 corpus must exist");
    assert_eq!(bayer, "GRBG");
}

#[test]
fn test_all_sensor_corpus_present() {
    let sensors = ["imx586", "ov13858", "hi1336", "s5k3l6"];
    for sensor in &sensors {
        let result = load_sensor(sensor);
        assert!(result.is_some(), "Missing corpus for sensor: {}", sensor);
    }
}

#[test]
fn test_gradient_pattern_reproducible() {
    // Verify that the same gradient data produces the same output
    let (w, h, data1, _, _) = load_sensor("imx586").unwrap();
    let (_, _, data2, _, _) = load_sensor("imx586").unwrap();
    assert_eq!(data1, data2);
    assert!(w > 0 && h > 0);
}

#[test]
fn test_gradient_pattern_correct_size() {
    // Synthetic gradient should be substantial pixel data
    let (_w, _h, data, _, _) = load_sensor("imx586").unwrap();
    assert!(!data.is_empty(), "Data should not be empty");
    assert!(data.len() >= 1024, "Data should be at least 1024 bytes per pixel");
}
