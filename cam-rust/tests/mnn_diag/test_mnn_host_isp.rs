use std::fs;

fn main() {
    // Use MnnHostModel which wraps the low-level API
    let model_bytes = fs::read("test_nocast.mnn").unwrap_or_else(|_| {
        // Try more paths
        fs::read("/data/data/com.termux/files/home/softisp/cam-rust/test_nocast.mnn").unwrap()
    });
    println!("Model size: {} bytes", model_bytes.len());
    
    // Build model from bytes
    match cam_isp::mnn_host::MnnHostModel::from_bytes(&model_bytes) {
        Ok(model) => {
            println!("Model loaded successfully");
            
            // Create input [1,1,48,64] of f32
            let input: Vec<f32> = (0..(1*1*48*64)).map(|i| (i * 256 / 64 % 256) as f32).collect();
            println!("Input size: {}, first={}, last={}", input.len(), input[0], input[input.len()-1]);
            
            // Run inference
            match model.run(&input, &[1, 1, 48, 64]) {
                Ok(output) => {
                    println!("Output size: {}", output.len());
                    if !output.is_empty() {
                        println!("First 8: {:?}", &output[..output.len().min(8)]);
                        let non_zero: usize = output.iter().filter(|&&v| v != 0.0).count();
                        println!("Non-zero elements: {}", non_zero);
                    }
                }
                Err(e) => println!("Run error: {}", e),
            }
        }
        Err(e) => println!("Load error: {}", e),
    }
}
