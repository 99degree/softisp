//! Last-value cache and reusable host scratch for MNN runtime tensor uploads.
//!
//! `copyFromHostTensor` persists in a session's backend buffer until the tensor
//! is rewritten or the session is resized. Constants in `default_inputs` are
//! re-queued every frame but never change, so uploading them again only puts
//! another host→device barrier on the visible path. The cache is per session
//! slot: pooled sessions own independent backend buffers.
//!
//! Array/raw pending values are built in scratch `Vec`s owned by the same slot,
//! so a steady-state frame allocates no host buffer for the fixed-shape
//! coefficient tensors.

use std::collections::HashMap;

#[derive(Debug, Clone, PartialEq, Eq)]
enum Value {
    Scalar(u32),
    F32(Vec<u32>),
    Bytes(Vec<u8>),
}

#[derive(Debug, Default)]
pub(crate) struct UploadCache {
    values: HashMap<String, Value>,
    f32_scratch: HashMap<String, Vec<f32>>,
    raw_scratch: HashMap<String, Vec<u8>>,
}

impl UploadCache {
    /// Whether a rank-0 scalar still needs to be uploaded.
    pub(crate) fn needs_scalar(&self, name: &str, val: f32) -> bool {
        !matches!(self.values.get(name), Some(Value::Scalar(bits)) if *bits == val.to_bits())
    }

    /// Whether an f32 array still needs to be uploaded.
    pub(crate) fn needs_f32(&self, name: &str, vals: &[f32]) -> bool {
        match self.values.get(name) {
            Some(Value::F32(bits)) => {
                bits.len() != vals.len()
                    || !bits
                        .iter()
                        .zip(vals)
                        .all(|(old, new)| *old == new.to_bits())
            }
            _ => true,
        }
    }

    /// Whether raw serialized tensor bytes still need to be uploaded.
    pub(crate) fn needs_bytes(&self, name: &str, data: &[u8]) -> bool {
        !matches!(self.values.get(name), Some(Value::Bytes(old)) if old.as_slice() == data)
    }

    pub(crate) fn record_scalar(&mut self, name: &str, val: f32) {
        self.values
            .insert(name.to_owned(), Value::Scalar(val.to_bits()));
    }

    pub(crate) fn record_f32(&mut self, name: &str, vals: &[f32]) {
        self.values.insert(
            name.to_owned(),
            Value::F32(vals.iter().map(|v| v.to_bits()).collect()),
        );
    }

    pub(crate) fn record_bytes(&mut self, name: &str, data: &[u8]) {
        self.values
            .insert(name.to_owned(), Value::Bytes(data.to_vec()));
    }

    /// Invalidate uploaded values after `resizeSession` replaces the buffers.
    /// Scratch is plain host memory, resized on next use, so it is kept.
    pub(crate) fn clear(&mut self) {
        self.values.clear();
    }

    /// Borrow a cleared f32 scratch buffer for `name` with room for `len` values.
    pub(crate) fn take_f32(&mut self, name: &str, len: usize) -> Vec<f32> {
        let mut buf = self.f32_scratch.remove(name).unwrap_or_default();
        buf.clear();
        buf.reserve(len);
        buf
    }

    /// Return an f32 buffer after upload or a cache hit so the next frame reuses it.
    pub(crate) fn put_f32(&mut self, name: String, buf: Vec<f32>) {
        self.f32_scratch.insert(name, buf);
    }

    /// Borrow a cleared byte scratch buffer for `name` with room for `len` bytes.
    pub(crate) fn take_raw(&mut self, name: &str, len: usize) -> Vec<u8> {
        let mut buf = self.raw_scratch.remove(name).unwrap_or_default();
        buf.clear();
        buf.reserve(len);
        buf
    }

    /// Return a byte buffer after upload or a cache hit so the next frame reuses it.
    pub(crate) fn put_raw(&mut self, name: String, buf: Vec<u8>) {
        self.raw_scratch.insert(name, buf);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scalar_compares_bit_patterns() {
        let mut cache = UploadCache::default();
        assert!(cache.needs_scalar("ToneBlock/exposure", 1.0));
        cache.record_scalar("ToneBlock/exposure", 1.0);
        assert!(!cache.needs_scalar("ToneBlock/exposure", 1.0));
        assert!(cache.needs_scalar("ToneBlock/exposure", 2.0));
        assert!(cache.needs_scalar("DisplayBlock/gamma", 1.0));

        cache.record_scalar("zero", -0.0);
        assert!(!cache.needs_scalar("zero", -0.0));
        assert!(cache.needs_scalar("zero", 0.0));
    }

    #[test]
    fn array_compares_values_length_and_nan_payload() {
        let mut cache = UploadCache::default();
        let vals = [1.0, f32::NAN, 3.0];
        assert!(cache.needs_f32("DemosaicCcmBlock/w", &vals));
        cache.record_f32("DemosaicCcmBlock/w", &vals);
        assert!(!cache.needs_f32("DemosaicCcmBlock/w", &vals));
        assert!(cache.needs_f32("DemosaicCcmBlock/w", &vals[..2]));
        assert!(cache.needs_f32("DemosaicCcmBlock/w", &[1.0, 4.0, 3.0]));

        let mut other_nan = vals;
        other_nan[1] = f32::from_bits(f32::NAN.to_bits() ^ 1);
        assert!(cache.needs_f32("DemosaicCcmBlock/w", &other_nan));
    }

    #[test]
    fn raw_bytes_and_kind_changes_are_tracked() {
        let mut cache = UploadCache::default();
        let bytes = [0u8, 1, 2, 3];
        assert!(cache.needs_bytes("DisplayBlock/conv_w", &bytes));
        cache.record_bytes("DisplayBlock/conv_w", &bytes);
        assert!(!cache.needs_bytes("DisplayBlock/conv_w", &bytes));
        assert!(cache.needs_bytes("DisplayBlock/conv_w", &bytes[..3]));
        assert!(cache.needs_f32("DisplayBlock/conv_w", &[1.0]));

        cache.record_scalar("DisplayBlock/conv_w", 1.0);
        assert!(cache.needs_bytes("DisplayBlock/conv_w", &bytes));
        assert!(!cache.needs_scalar("DisplayBlock/conv_w", 1.0));
    }

    #[test]
    fn clear_invalidates_every_name() {
        let mut cache = UploadCache::default();
        cache.record_scalar("a", 1.0);
        cache.record_f32("b", &[1.0]);
        cache.record_bytes("c", &[1]);
        assert!(!cache.needs_scalar("a", 1.0));
        assert!(!cache.needs_f32("b", &[1.0]));
        assert!(!cache.needs_bytes("c", &[1]));
        cache.clear();
        assert!(cache.needs_scalar("a", 1.0));
        assert!(cache.needs_f32("b", &[1.0]));
        assert!(cache.needs_bytes("c", &[1]));
    }

    #[test]
    fn f32_scratch_reuses_allocation_per_name() {
        let mut cache = UploadCache::default();
        let a = cache.take_f32("WarpGrid/grid", 1024);
        let a_ptr = a.as_ptr();
        cache.put_f32("WarpGrid/grid".to_owned(), a);

        let b = cache.take_f32("WarpGrid/grid", 16);
        assert_eq!(b.as_ptr(), a_ptr, "scratch should reuse the allocation");
        assert!(b.capacity() >= 1024);
        cache.put_f32("WarpGrid/grid".to_owned(), b);

        let c = cache.take_f32("other", 16);
        assert_ne!(c.as_ptr(), a_ptr, "different names own different buffers");
    }

    #[test]
    fn raw_scratch_reuses_allocation_per_name() {
        let mut cache = UploadCache::default();
        let a = cache.take_raw("DisplayBlock/conv_w", 4096);
        let a_ptr = a.as_ptr();
        cache.put_raw("DisplayBlock/conv_w".to_owned(), a);

        let b = cache.take_raw("DisplayBlock/conv_w", 8);
        assert_eq!(b.as_ptr(), a_ptr, "scratch should reuse the allocation");
        assert!(b.capacity() >= 4096);
    }
}
