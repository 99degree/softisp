pub mod scalar;
pub mod neon;
pub mod neon_fp16;
pub mod neon_dotprod;
#[cfg(target_arch = "x86_64")]
pub mod avx2;
#[cfg(target_arch = "x86_64")]
pub mod sse2;
pub mod selector;

pub use selector::{best_backend, active_backend_name, BackendKind, SimdEngine};