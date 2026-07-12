#[cfg(target_arch = "x86_64")]
pub mod avx2;
pub mod neon;
pub mod neon_dotprod;
pub mod neon_fp16;
pub mod scalar;
pub mod selector;
#[cfg(target_arch = "x86_64")]
pub mod sse2;

pub use selector::{active_backend_name, best_backend, BackendKind, SimdEngine};
