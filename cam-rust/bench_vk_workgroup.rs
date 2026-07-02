use cam_isp::{mnn::*, pipeline::*};

fn main() {
    let engine = MnnEngine::create_vulkan(0);
    let variants = [(16,16), (32,8), (64,4)];
    for (wx,wy) in variants {
        let session = engine.create_session();
        // Set workgroup size
        engine.set_workgroup_size(&session, wx, wy);
        
        let mut batch = engine.create_input_tensor(&session, 3840, 2160);
        fill_bayer(batch.slice_mut());
        
        let start = std::time::Instant::now();
        engine.infer(&session, &[batch]);
        let fps = 1000.0 / start.elapsed().as_millis() as f32;
        
        println!("Workgroup {}x{}: {:.1} FPS", wx, wy, fps);
    }
}
fn fill_bayer(dest: &mut [u16]) {
   for (i,v) in dest.iter_mut().enumerate() {
       *v = (i as u16) & 0x0FFF; // synthetic pattern
   }
}
