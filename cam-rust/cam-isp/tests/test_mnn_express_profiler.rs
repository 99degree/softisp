//! Test MNN Express API with profiler
//! Requires MNN_PIPELINE_PROFILE=ON

#[cfg(test)]
mod tests {
    use cam_isp::mnn_sys::*;
    use std::ffi::CString;
    use std::os::raw::*;

    #[test]
    #[ignore = "Requires MNN_PIPELINE_PROFILE=ON and Express API"]
    fn test_express_profiler() {
        unsafe {
            // Get the global executor
            let executor = mnn_executor_get_global();
            assert!(!executor.is_null(), "Could not get executor");

            // Reset profiler
            println!("[profiler] Resetting profile...");
            mnn_executor_reset_profile(executor);

            // Load a model using Express API
            // For now, just test that we can call the profiler functions
            println!("[profiler] Executor handle: {:?}", executor);

            // Get last GPU time (should be -1 since no inference ran)
            let gpu_time = mnn_executor_get_last_gpu_time_ms(executor);
            println!("[profiler] Last GPU time (before inference): {}ms", gpu_time);

            // Dump profile (should show nothing or empty)
            println!("[profiler] Profile dump:");
            mnn_executor_dump_profile(executor);

            // The Express API would normally:
            // 1. Load a module: auto module = MNN::Express::Module::load(...)
            // 2. Create input: auto input = MNN::Express::_Input(...)
            // 3. Run: auto output = module.onForward({input})
            // 4. This would populate the profiler
            
            // For now, we just verify the API is accessible
            println!("[profiler] Test completed - Express profiler API accessible");
        }
    }
}
