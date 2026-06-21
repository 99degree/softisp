    // Set preserve_input_type for NativeInt16 mode
    if use_native {
        if let Some(mnn) = engine.as_any_mut().downcast_mut::<cam_isp::mnnengine::MnnEngine>() {
            mnn.set_preserve_input_type(true);
        }
    }

    let result = engine.build(head, all, None, 21);
    if let Err(ref e) = result {
        eprintln!("Build FAILED: {}", e);
        return;
    }

    println!("Pipeline: 9 blocks (1 Identity)");
    let mut params = cam_isp::engine::ProcessParams::new(sensor_w, sensor_h, &raw);
    params.target_width = post_w;
    params.target_height = post_h;
    params.sensor_max = 1023.0;
    params.output_format = cam_isp::engine::OutputFormat::FloatBgra;

    // Warm-up frames to avoid initial type-mismatch error in native mode
    let warmup_frames = if use_native { 5 } else { 3 };
    for _ in 0..warmup_frames { let _ = engine.process(&params); }

    let mut sum_ms = 0.0f64;
    for i in 0..n_frames {
        use std::time::Instant;
        let t0 = Instant::now();
        let result = engine.process(&params);
        let ms = t0.elapsed().as_secs_f64() * 1000.0;
        sum_ms += ms;
        match result {
            Ok(frame) => println!("  [{:2}] {:4}×{:<4}  {:7.1}ms  ({} bytes)",
                i, frame.width, frame.height, ms, frame.data.len()),
            Err(e) => println!("  [{:2}] ERROR: {}  ({:.1}ms)", i, e, ms),
        }
    }