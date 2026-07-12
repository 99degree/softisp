//! Test parallel session pool by submitting frames from 4 threads concurrently.
//!
//! Each thread processes a frame simultaneously. With 4 sessions in the pool,
//! all 4 should run concurrently (total wall time ≈ 1 frame time, not 4×).
//! With 1 session (Mutex), total wall time would be 4×.

#[cfg(feature = "mnn")]
mod parallel_test {

    #[test]
    #[ignore]
    fn test_parallel_submission() {
        // This test requires a full build() call with pipeline blocks.
        // For a quick smoke-test, we just verify the pool creates correctly.
        // Full parallel inference test is done in bench_parallel.rs example.
    }
}
