//! Test CLI for cam-ipa stable API
//!
//! Usage:
//!   cargo run --example ipa_test_cli -p cam-ipa
//!
//! Exercises: IpaContext init/start/configure/process_stats/process_frame/stop

use cam_ipa::{
    IpaConfig, IpaSettings, IpaSensorInfo, IpaFormat, IpaCallbacks,
    ControlList, Buffer, MemBuffer, IpaContext,
};

fn main() {
    env_logger::init();

    println!("=== cam-ipa Test CLI ===");

    // 1. Create config
    let config = IpaConfig {
        image_width: 1920,
        image_height: 1080,
        format: IpaFormat::BGRA,
        num_output_buffers: 2,
        ..IpaConfig::default()
    };
    println!("Config: {}x{} {:?}", config.image_width, config.image_height, config.format);

    // 2. Create IPA context
    let mut ctx = match IpaContext::new(config) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("Failed to create IPA context: {}", e);
            std::process::exit(1);
        }
    };
    println!("IPA context created");

    // 3. Set up callbacks
    let callbacks = IpaCallbacks {
        metadata_ready: Some(Box::new(|frame, metadata| {
            println!("[callback] metadata_ready: frame={} controls={:?}", frame, metadata.data.keys().collect::<Vec<_>>());
        })),
        frame_done: Some(Box::new(|frame, buffer_id| {
            println!("[callback] frame_done: frame={} buffer_id={}", frame, buffer_id);
        })),
        set_sensor_controls: Some(Box::new(|controls| {
            println!("[callback] set_sensor_controls: keys={:?}", controls.data.keys().collect::<Vec<_>>());
        })),
        set_isp_params: Some(Box::new(|| {
            println!("[callback] set_isp_params");
        })),
    };
    ctx.set_callbacks(callbacks);
    println!("Callbacks registered");

    // 4. Create sensor info
    let sensor_info = IpaSensorInfo {
        model: "test_sensor".to_string(),
        bits_per_pixel: 10,
        cfa_pattern: 0,
        active_area_size: cam_ipa::Size { width: 1920, height: 1080 },
        analog_crop: cam_ipa::Rectangle { x: 0, y: 0, width: 1920, height: 1080 },
        output_size: cam_ipa::Size { width: 1920, height: 1080 },
        pixel_rate: 0,
        min_line_length: 0,
        max_line_length: 0,
        min_frame_length: 0,
        max_frame_length: 0,
    };

    // 5. Initialize
    let settings = IpaSettings {
        config_file: None,
        sensor_model: "test_sensor".to_string(),
    };
    match ctx.init(&settings, &sensor_info) {
        Ok(init_result) => {
            println!("IPA init OK: ccm_enabled={}", init_result.ccm_enabled);
        }
        Err(e) => {
            eprintln!("IPA init failed: {}", e);
            std::process::exit(1);
        }
    }
    println!("Initialized: {}", ctx.is_initialized());

    // 6. Start
    match ctx.start() {
        Ok(()) => println!("IPA started"),
        Err(e) => {
            eprintln!("IPA start failed: {}", e);
            std::process::exit(1);
        }
    }
    println!("Started: {}", ctx.is_started());

    // 7. Create test buffers
    let input_size = (1920 * 1080 * 2) as usize; // 16-bit Bayer
    let output_size = (1920 * 1080 * 4) as usize; // BGRA

    let mut input_buf = MemBuffer::new(input_size);
    for (i, b) in input_buf.data.iter_mut().enumerate() {
        *b = (i % 256) as u8;
    }

    let mut output_buf = MemBuffer::new(output_size);

    // 8. Queue request
    let mut controls = ControlList::new();
    controls.set_f32("AnalogueGain", 2.0);
    controls.set_i32("ExposureTime", 2000);
    controls.set_f32_array("ColourGains", &[1.5, 1.2]);

    match ctx.queue_request(1, &controls) {
        Ok(()) => println!("queue_request(1) OK"),
        Err(e) => eprintln!("queue_request failed: {}", e),
    }

    // 9. Compute params
    match ctx.compute_params(1) {
        Ok(()) => println!("compute_params(1) OK"),
        Err(e) => eprintln!("compute_params failed: {}", e),
    }

    // 10. Process stats
    let stats_buf = MemBuffer::new(640 * 480);
    match ctx.process_stats(1, 100, &stats_buf) {
        Ok(metadata) => {
            println!("process_stats OK: {:?}", metadata.data.keys().collect::<Vec<_>>());
        }
        Err(e) => eprintln!("process_stats failed: {}", e),
    }

    // 11. Process frame
    match ctx.process_frame(
        1, 100,
        &mut input_buf, &mut output_buf,
        0, 0,
        1920, 1080,
        &controls,
    ) {
        Ok(()) => println!("process_frame(1) OK"),
        Err(e) => eprintln!("process_frame failed: {}", e),
    }

    // 12. Configure frame backend
    match ctx.configure_frame_backend(IpaFormat::RGBA, 3) {
        Ok(()) => println!("configure_frame_backend(RGBA, 3) OK"),
        Err(e) => eprintln!("configure_frame_backend failed: {}", e),
    }

    // 13. Stop
    ctx.stop();
    println!("Stopped");

    println!("=== cam-ipa Test CLI Complete ===");
}
