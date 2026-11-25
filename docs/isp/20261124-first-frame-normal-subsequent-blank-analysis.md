---
title: "Analysis: First frame normal, subsequent frames blank"
created_at: 2026-11-24T14:13:47.982460556+00:00
question: "Why does SoftISP perf test show only first frame/profile normal, subsequent blank/white?"
attachments: ["/data/data/com.termux/files/home/softisp/cam-rust/cam-isp/src/mnnengine.rs", "/data/data/com.termux/files/home/softisp/cam-rust/cam-isp/tests/probe_4k_perf.rs", "/data/data/com.termux/files/home/softisp/cam-rust/cam-isp/src/blocks/display.rs", "/data/data/com.termux/files/home/softisp/cam-rust/cam-isp/src/engine.rs"]
---

# Analysis: First frame normal, subsequent frames blank

## Question

Why does SoftISP perf test show only first frame/profile normal, subsequent blank/white?

## Context

Running cargo test -p cam-isp --test probe_4k_perf on Android device. First frame/profile (MED FloatRgb) works: "rgb mean [99.4,110.0,110.5], max 110, non-black 100.0%". Subsequent profiles (FUSED, DUAL, WHOLE, UNIFIED) produce "produced a blank frame". Switching to Bgra format works for MED but then fails for FUSED. Warning: "output channel mismatch: graph produced 1 channels, FloatRgb expects 3". After first format (FloatRgb), Bgra works for MED but then fails for FUSED.
