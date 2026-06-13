pub mod scalar;
pub mod neon;
pub mod neon_fp16;
pub mod neon_dotprod;
pub mod selector;

pub use selector::{best_backend, active_backend_name, BackendKind, SimdEngine};