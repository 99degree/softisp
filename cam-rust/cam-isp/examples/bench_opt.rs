fn main() {
    let _ = env_logger::builder().is_test(false).filter_level(log::LevelFilter::Debug).try_init();
    cam_isp::init();

    let sensor_w = 3840u32;
    let sensor_h = 2160u32;
    let post_w = 960u32;
    let post_h = 540u32;
    let packed_w = (sensor_w / 2) as i64;
    let full_w = sensor_w as i64;
    let full_h = sensor_h as i64;
    let ds_w = 1920i64;
    let ds_h = 1080i64;
    let post_w_i = post_w as i64;
    let post_h_i = post_h as i64;

    let mut raw_buf = Vec::with_capacity((sensor_w * sensor_h * 2) as usize);
    let mut rng_state = 42u64;
    for _ in 0..sensor_w * sensor_h {
        rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let val = (rng_state >> 22) as u16 & 0x3FF;
        raw_buf.extend_from_slice(&val.to_le_bytes());
    }
    let raw = raw_buf;

    let use_native = std::env::var("MODE").unwrap_or_default() == "native";
    let pw = if use_native { full_w } else { packed_w };
    let elem_type = if use_native { 5 } else { 6 };

    let mut blocks: Vec<Box<dyn cam_isp::pipeline::IspBlock>> = vec![
        Box::new(cam_isp::blocks::RawInputBlock::new().with_elem_type(elem_type).with_concrete_dims(full_h, pw)),
        Box::new(cam_isp::blocks::UnpackCfaBlock::new()
            .with_concrete_width(full_w).with_concrete_dims(full_h, full_w)
            .with_downscale(1)  // no downscale — keep 4K Bayer
            .with_sensor_max(1023.0).with_blc(true)
            .with_mode(if use_native { cam_isp::blocks::UnpackMode::NativeInt16 } else { cam_isp::blocks::UnpackMode::PackedInt32 })),
        Box::new(cam_isp::blocks::IdentityBlock::new("aux_hook_src")),
        Box::new(cam_isp::blocks::IdentityBlock::new("bayer_wb")),
        // Resize Bayer 4K→2K: cheap nearest resize in Bayer domain
        Box::new(cam_isp::blocks::ResizeBlock::new(0.5).with_concrete_dims(full_h, full_w)),
        // DemosaicCcm at 2K (was 4K — 4× fewer pixels)
        Box::new(cam_isp::blocks::DemosaicCcmBlock::new(0).with_concrete_dims(ds_h, ds_w)),
        // Resize RGB 2K→FHD: bilinear downscale
        Box::new(cam_isp::blocks::ResizeBlock::new(0.5).with_concrete_dims(ds_h, ds_w)),
        Box::new(cam_isp::blocks::IdentityBlock::new("tone")),
        Box::new(cam_isp::blocks::IdentityBlock::new("aux_hook_out")),
        Box::new(cam_isp::blocks::FcsBlock::new()),
        Box::new(cam_isp::blocks::LdciBlock::new()),
        Box::new(cam_isp::blocks::EeBlock::new()),
        Box::new(cam_isp::blocks::DisplayBlock::new(post_w).with_pack_rgba(false).with_bg4a(true).with_concrete_dims(post_h_i, post_w_i)),
    ];

    cam_isp::pipeline::GraphComposer::wire_blocks(&mut blocks);

    let mut all = blocks;
    let head = all.remove(0);
    let engine_name = std::env::var("ENGINE").unwrap_or_else(|_| "cpu".to_string());
    let mut engine = match cam_isp::engine::select_engine_by_name(&engine_name) {
        Some(e) => e,
        None => match cam_isp::engine::select_engine() { Some(e) => e, None => Box::new(cam_isp::cpu::CpuEngine::new()) },
    };

    println!("Engine: {} Mode: {} elem={}", engine.backend_name(), if use_native { "NativeInt16" } else { "PackedInt32" }, elem_type);
    if use_native {
        if let Some(mnn) = engine.as_any_mut().downcast_mut::<cam_isp::mnnengine::MnnEngine>() {
            mnn.set_preserve_input_type(true);
            println!("  preserve_input_type=TRUE");
        } else {
            println!("  WARN: downcast failed!");
        }
    }

    let result = engine.build(head, all, None, 21);
    if let Err(ref e) = result {
        eprintln!("Build FAILED: {}", e);
        return;
    }

    println!("Pipeline: {} blocks", if use_native { 8 } else { 8 });
    let mut params = cam_isp::engine::ProcessParams::new(sensor_w, sensor_h, &raw);
    params.target_width = post_w;
    params.target_height = post_h;
    params.sensor_max = 1023.0;
    params.output_format = cam_isp::engine::OutputFormat::FloatBgra;

    let warmup_frames = if use_native { 5 } else { 3 };
    for _ in 0..warmup_frames {
        let _ = engine.process(&params);
    }

    let mut sum_ms = 0.0f64;
    let n_frames = 10;
    for i in 0..n_frames {
        use std::time::Instant;
        let t0 = Instant::now();
        let result = engine.process(&params);
        let ms = t0.elapsed().as_secs_f64() * 1000.0;
        sum_ms += ms;
        match result {
            Ok(frame) => println!("  [{:2}] {:4}×{:<4}  {:7.1}ms  ({} bytes)", i, frame.width, frame.height, ms, frame.data.len()),
            Err(e) => println!("  [{:2}] ERROR: {}  ({:.1}ms)", i, e, ms),
        }
    }
    println!("Average: {:.1}ms ({:.1} FPS)", sum_ms / n_frames as f64, 1000.0 / (sum_ms / n_frames as f64));
}
