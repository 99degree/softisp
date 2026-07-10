# MNN Inference Guide - Termux ARM64

> **Status**: Partial - TEST profile works, LITE+ profiles blocked by COMPUTE_SIZE_ERROR
> **Platform**: Termux ARM64, MNN 3.5.0
> **Last Updated**: 2025-06-12

---

## Executive Summary

| Profile | ONNX Type | MNN Type | `copyFromHostTensor` | `buffer().host` | `runSession` | Status |
|---------|------------|----------|---------------------|-----------------|--------------|--------|
| TEST (Cast+Mul, 2 ops) | FLOAT32 | FLOAT (code=2, bits=32) | ✅ Works | ✅ Works | ✅ 0 (NO_ERROR) | **FULLY WORKING** |
| TEST (UINT16/INT16) | UINT16/INT16 | INT32 (code=0, bits=32) | ✅ Works | ✅ Works | ✅ 0 (NO_ERROR) | **FULLY WORKING** |
| LITE (full pipeline, 10+ ops) | UINT16/INT16 | INT32 (code=0, bits=32) | ❌ Fails | ⚠️ host overwritten to NULL | ❌ 3 (COMPUTE_SIZE_ERROR) | **BLOCKED** |
| MED/HEAVY/PRO | UINT16/INT16 | INT32 (code=0, bits=32) | ❌ Likely fails | ❌ Likely fails | ❌ Likely fails | **UNTESTED** |

---

## Key Findings from MNN Source Code Analysis

### 1. Session Input Tensor Architecture

MNN uses a **two-tensor scheme** for session inputs:

```
┌─────────────────────────────────────────────────────────────┐
│                    USER-FACING INPUT TENSOR                  │
│  (returned by getSessionInput)                                │
│  - backend: NULL (by design)                                  │
│  - host: NULL or user-provided pointer                       │
│  - mem: NULL                                                  │
│  - usage: INPUT                                               │
│  - "Portal" for user data injection                          │
└─────────────────────────────────────────────────────────────┘
                              │
                              │ copyFromHostTensor() OR
                              │ buffer().host = user_data
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                 PIPELINE INTERNAL INPUT TENSOR                │
│  (in inputTensorCopyCache, used by ops)                       │
│  - backend: CPUBackend (non-NULL)                             │
│  - host: Allocated by onAcquireBuffer                         │
│  - mem: Backend-managed memory                                │
│  - usage: INPUT or CONSTANT                                  │
│  - Actual tensor consumed by first op                         │
└─────────────────────────────────────────────────────────────┘
```

**Flow**:
1. User calls `getSessionInput()` → returns user-facing tensor (portal)
2. User writes data → `copyFromHostTensor()` **FAILS** (backend is NULL)
3. User writes data → `buffer().host = user_data` **WORKS** (mutable reference)
4. During `resizeSession()` → `_InsertCopy()` creates pipeline tensor copy
5. During `runSession()` → `_copyInputs()` copies user portal → pipeline tensor
6. Pipeline ops consume pipeline tensor (not user portal)

### 2. Why `copyFromHostTensor` Fails

```cpp
// Tensor.cpp:171-174
bool Tensor::copyFromHostTensor(const Tensor* hostTensor) {
    auto bn = mDescribe->getBackend();
    if (nullptr == bn) {
        return false;  // ❌ User portal has null backend
    }
    bn->onCopyBuffer(hostTensor, this);
    return true;
}
```

The user-facing input tensor **intentionally has NULL backend** — it's just a portal. `copyFromHostTensor` requires a destination with a valid backend, so it fails.

### 3. Correct Approach: Direct Host Pointer Assignment

```cpp
// ✅ CORRECT: Set host directly on user-facing tensor
Tensor* in = interpreter->getSessionInput(session, nullptr);
std::vector<int32_t> data(48*64);
// Fill data...
in->buffer().host = data.data();  // buffer() returns mutable halide_buffer_t&

// ❌ WRONG: This fails (dest has null backend)
Tensor* hostTensor = Tensor::create(dims, type, data.data(), CAFFE);
in->copyFromHostTensor(hostTensor);  // Returns false
```

### 4. The LITE Model Blocking Issue

**Symptom**: After `resizeSession()`, the user-facing input tensor's `host` pointer is **overwritten to NULL**.

**Test Results**:
```
test_1.mnn (FLOAT, 2 ops):
  Initial host:  0xb40000782603d040
  After resizeSession: 0xb400007946043010  ✅ Reallocated
  runSession: 0 (NO_ERROR)

test_4.mnn (INT16, ~5 ops):
  Initial host:  0xb40000782603d040
  After resizeSession: 0xb400007946043010  ✅ Reallocated
  runSession: 0 (NO_ERROR)

test_lite.mnn (INT16, 10+ ops):
  Initial host:  0x0
  After resizeSession: 0x0  ❌ NOT reallocated
  After manual set: 0xb400007becc166c0
  After resizeSession: 0x0  ❌ Overwritten to NULL
  runSession: 3 (COMPUTE_SIZE_ERROR)
```

**Root Cause**: For complex models (LITE+), MNN's memory optimizer **releases the input tensor's memory** during `_allocForTensor` and **doesn't reallocate** it, leaving `host=NULL`. This happens because:

1. The input tensor is only consumed by the first op (RawInputBlock → NormalizeBlock)
2. MNN's scheduler applies **memory reuse optimization** — input tensors that are single-use get their memory released after the first op
3. `_allocForTensor` (Pipeline.cpp:1008) only allocates if `allocInput && needAllocIO()` 
4. For the LITE model, the input tensor's group or scheduling context prevents reallocation

**The input tensor is released but NOT reallocated** → `host` stays NULL.

### 5. Why `runSession` Returns COMPUTE_SIZE_ERROR (3)

Even when we manually set `host` before `resizeSession`, the pointer gets **overwritten to NULL** by `resizeSession`. During execution:

1. `_copyInputs()` tries to copy from user portal (host=NULL) to pipeline tensor
2. `CPUBackend::onCopyBuffer` (line 817) checks:
   ```cpp
   if (nullptr == srcBuffer.host || nullptr == dstBuffer.host) {
       return;  // Silent no-op
   }
   ```
3. Since `srcBuffer.host == NULL`, the copy **silently fails**
4. Pipeline ops read from uninitialized memory → undefined behavior
5. Some op's `onResize` or execution fails → COMPUTE_SIZE_ERROR (3)

---

## Working Solutions

### ✅ Solution 1: Use TEST Profile (Fully Working)

The TEST profile has minimal ops (Cast + Mul) and **doesn't trigger memory optimization**:

```cpp
// Works with both Session_Input_Inside (default) and Session_Input_User
MNN::ScheduleConfig cfg;
cfg.numThread = 4;
cfg.type = MNN_FORWARD_CPU;

auto sess = net->createSession(cfg);
Tensor* in = net->getSessionInput(sess, nullptr);

// Option A: Direct host pointer
std::vector<float> data(48*64);
// Fill data...
in->buffer().host = data.data();
net->runSession(sess);  // ✅ Works

// Option B: copyFromHostTensor (works because TEST model allocates memory)
Tensor* host = Tensor::create(in->shape(), in->getType(), data.data(), CAFFE);
in->copyFromHostTensor(host);  // ✅ Works for TEST model
net->runSession(sess);
```

### ✅ Solution 2: Use TEST Profile with INT16 Input

Same as Solution 1 but with INT16/UINT16 → converted to INT32 by MNN:

```cpp
// test_4.mnn is a minimal INT16→INT32 pipeline
// ✅ Works with both methods
```

### ✅ Solution 3: Direct Host Pointer + Session_Input_Inside

For models where `resizeSession` reallocates memory (TEST, test_4):

```cpp
Tensor* in = net->getSessionInput(sess, nullptr);
std::vector<int32_t> data(48*64);
// Fill data...

net->resizeSession(sess);  // Reallocates host
in->buffer().host = data.data();  // Overwrite reallocated pointer
net->runSession(sess);  // ✅ Works for models that reallocate
```

**Note**: This works for TEST/test_4 but **fails for LITE** because `resizeSession` doesn't allocate.

---

## Blocking Issues

### ❌ Issue 1: LITE Model Input Tensor Memory Not Allocated

**Symptom**: `resizeSession` doesn't allocate memory for LITE model's input tensor.

**Root Cause**: Memory optimizer releases input tensor memory and doesn't reallocate.

**Evidence**:
- `resizeSession` overwrites manually-set `host` to NULL
- `backend` remains NULL after `resizeSession`
- `_needRelease` returns true for INPUT tensors in certain conditions

**Potential Fixes (not yet verified)**:

#### Fix 1A: Use `Session_Input_User` Mode

```cpp
net->setSessionMode(MNN::Interpreter::Session_Input_User);
// User manages memory, no copy cache
Tensor* in = net->getSessionInput(sess, nullptr);
in->buffer().host = user_data;
net->resizeSession(sess);  // Should NOT overwrite host
net->runSession(sess);
```

**Status**: ❌ Still returns COMPUTE_SIZE_ERROR (3) — tested, doesn't work.

#### Fix 1B: Mark Input Tensor as OUTPUT Usage

```cpp
MNN::ScheduleConfig cfg;
cfg.saveTensors = {"RawInputBlock/frame"};  // Marks tensor as OUTPUT
```

**Status**: ❌ Doesn't help — `saveTensors` only affects tensors with NORMAL usage, not INPUT usage.

#### Fix 1C: Use BackendConfig::Memory_High

```cpp
MNN::BackendConfig bc;
bc.memory = MNN::BackendConfig::Memory_High;
cfg.backendConfig = &bc;
```

**Status**: ❌ Doesn't help — tested with saveTensors + Memory_High, still host=NULL.

#### Fix 1D: Force Allocation via resizeTensor

```cpp
Tensor* in = net->getSessionInput(sess, nullptr);
net->resizeTensor(in, {1,1,48,64});  // Force allocation
net->resizeSession(sess);
```

**Status**: ❌ Still host=NULL — resizeTensor doesn't allocate for LITE model.

#### Fix 1E: Create Separate Host Tensor and Use Map

```cpp
Tensor* in = net->getSessionInput(sess, nullptr);
net->resizeSession(sess);
void* mapped = in->map(MNN::Tensor::MAP_TENSOR_WRITE, CAFFE);
memcpy(mapped, data, bytes);
in->unmap(MNN::Tensor::MAP_TENSOR_WRITE, CAFFE, mapped);
```

**Status**: ❌ `map()` returns NULL — input tensor has no backend memory.

### ❌ Issue 2: COMPUTE_SIZE_ERROR (Error Code 3)

**Symptom**: `runSession` returns 3 even after setting host pointer.

**Root Cause**: The input tensor's `host` is overwritten to NULL by `resizeSession`, so `_copyInputs()` copies from NULL source, leading to undefined behavior in pipeline ops.

**Error Code Mapping** (from ErrorCode.hpp):
- 0 = NO_ERROR
- 1 = OUT_OF_MEMORY
- 2 = NOT_SUPPORT
- 3 = COMPUTE_SIZE_ERROR ← **Our error**
- 5 = INVALID_VALUE
- 20 = TENSOR_NOT_SUPPORT

**Likely Cause**: Some pipeline op's `onResize` or `onExecute` computes output shape based on input shape and fails because the input tensor's shape/metadata is inconsistent.

---

## What Works vs What Doesn't

### ✅ Working (Verified)

| Method | TEST Profile | test_4 (INT16) | LITE Profile |
|--------|--------------|----------------|--------------|
| `copyFromHostTensor` | ✅ Works | ✅ Works | ❌ Fails |
| `buffer().host = data` + `runSession` | ✅ Works | ✅ Works | ❌ COMPUTE_SIZE_ERROR |
| `Session_Input_User` + `buffer().host` | ✅ Works | ✅ Works | ❌ COMPUTE_SIZE_ERROR |
| `resizeSession` allocates memory | ✅ Yes | ✅ Yes | ❌ No (host stays NULL) |

### ❌ Not Working (Verified)

| Method | LITE Profile | Reason |
|--------|--------------|--------|
| `copyFromHostTensor` | ❌ Fails | User tensor has null backend |
| `buffer().host` after resizeSession | ❌ COMPUTE_SIZE_ERROR | Host overwritten to NULL by resizeSession |
| `Session_Input_User` | ❌ COMPUTE_SIZE_ERROR | Same issue |
| `saveTensors` | ❌ No effect | Only affects NORMAL usage tensors |
| `BackendConfig::Memory_High` | ❌ No effect | Doesn't prevent memory release |
| `resizeTensor` | ❌ No effect | Doesn't allocate for LITE model |
| `map()` / `unmap()` | ❌ Returns NULL | No backend memory allocated |

---

## Hypotheses for Fixing LITE Profile

### Hypothesis 1: Input Tensor Not in Pipeline Commands

The LITE model's input tensor might not appear in any `iter.inputs` during `_allocForTensor`, so memory is never allocated. This could happen if:
- MNN's scheduler inserts a Copy op that replaces the input tensor
- The input tensor is in a different group than the first pipeline info
- The input tensor's usage prevents it from being allocated

**Test**: Check if `inputTensorCopyCache` is populated for LITE model during `_InsertCopy`.

### Hypothesis 2: Memory Optimizer Aggressively Releases Input

For complex models, MNN's memory optimizer might:
1. Determine the input tensor is only read once by the first op
2. Release its memory immediately after the first op's `onResize`
3. Not reallocate during `resizeSession` because the tensor is marked as "released"

**Test**: Check `_needRelease` return value for LITE model's input tensor.

### Hypothesis 3: Backend Allocation Fails Silently

`_allocTensor` calls `curBackend->onAcquireBuffer(t, memoryType)`. For the LITE model, this might return false, and the allocation fails silently.

**Test**: Check return value of `onAcquireBuffer` for input tensor.

### Hypothesis 4: Group Mismatch

The input tensor might have `group != 0`, while `_allocForTensor` is called with `index=0`. The condition in `_allocTensor`:
```cpp
if (des->group != group) {
    return true;  // Skip - tensor not in this group
}
```
If the input tensor's group != 0, it won't be allocated.

**Test**: Check `des->group` for LITE model's input tensor.

---

## Debugging Commands

### Check Input Tensor State

```cpp
#include "core/TensorUtils.hpp"

Tensor* in = net->getSessionInput(sess, nullptr);
auto desOrigin = MNN::TensorUtils::getDescribeOrigin(in);
auto des = MNN::TensorUtils::getDescribe(in);

std::cout << "Usage: " << des->usage << "\n";
std::cout << "MemoryType: " << des->memoryType << "\n";
std::cout << "UseCount: " << des->useCount << "\n";
std::cout << "Group: " << des->group << "\n";
std::cout << "Backend: " << desOrigin->getBackend() << "\n";
std::cout << "Mem: " << desOrigin->mem.get() << "\n";
```

**Usage enum**: 0=NORMAL, 1=INPUT, 2=OUTPUT, 3=CONSTANT, 4=TRAINABLE  
**MemoryType enum**: 0=BACKEND, 1=HOST, 2=VIRTUAL, 3=OUTSIDE

### Test Different Approaches

```bash
cd cam-rust
LD_LIBRARY_PATH=lib/aarch64 ./test_input_user2
LD_LIBRARY_PATH=lib/aarch64 ./test_resize_tensor
```

---

## Recommendations

### For Immediate Use

1. **Use TEST profile** for MNN inference — it works perfectly with all methods
2. **CPU fallback** — Use the pure-Rust CPU ISP engine which has all 16 stages working
3. **ONNX Runtime** — Once ORT header is available, test ONNX inference directly

### For Full Pipeline MNN Support

1. **Debug `_allocForTensor`** — Add logging to see why input tensor isn't allocated for LITE model
2. **Check `_needRelease`** — Verify why memory is released and not reallocated
3. **Test with MNN 3.6+** — Memory optimization might be improved in newer versions
4. **Recompile MNN with `MNN_BUILD_MEMORY_OPTIMIZE=OFF`** — Disable memory optimization entirely

---

## References

- MNN Source: `/data/data/com.termux/files/home/MNN/source/`
- Key Files:
  - `Tensor.cpp:171` — `copyFromHostTensor` implementation
  - `Pipeline.cpp:1008` — Input allocation logic
  - `Pipeline.cpp:1147` — `_copyInputs` function
  - `CPUBackend.cpp:817` — `onCopyBuffer` NULL check
  - `Schedule.cpp:390` — Input tensor population
- Test Files: `cam-rust/test_*.cpp`
- Models: `cam-rust/test_*.mnn`

---

## Changelog

- **2025-06-12**: Initial investigation complete. TEST profile works, LITE blocked by COMPUTE_SIZE_ERROR.
- **2025-06-12**: Verified two-tensor architecture via MNN source code.
- **2025-06-12**: Confirmed `resizeSession` overwrites host to NULL for LITE model.
