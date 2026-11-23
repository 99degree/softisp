docs(mnn): why a rebuilt libMNN.so cannot replace the shipped one here

Attempting to rebuild `libMNN.so` from this tree produces a library that fails
to load:

    CANNOT LINK EXECUTABLE …: cannot locate symbol "MNNBinaryAddInt8"
    referenced by lib/aarch64-v8a/libMNN.so

That is not a stale-artifact problem. It is a property of how this checkout was
configured, and it will reproduce on any rebuild.

## The chain

`build_isp/CMakeFiles/4.5.0/CMakeSystem.cmake` resolved the target CPU to the
empty string:

    set(CMAKE_SYSTEM_PROCESSOR "")

`source/backend/cpu/arm/CMakeLists.txt:29` decides whether to include the
AArch64 assembly sources by matching that variable:

    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^aarch64" OR ARCHS STREQUAL "arm64" …)

An empty string matches nothing, so `MNNARM64` was never created — there is no
`CMakeFiles/MNNARM64.dir`, and no `MNNBinaryAddInt8.S` object anywhere in the
build tree.

Meanwhile `source/core/Macro.h:31` still defines `MNN_USE_NEON` from the
preprocessor (the cross-compiler does define `__aarch64__` and `__ARM_NEON`), so
`MNN_USE_NEON` is **on**. The portable int8 fallbacks in
`source/backend/cpu/compute/Int8FunctionsOpt.cpp` sit inside
`#ifndef MNN_USE_NEON` (line 1825), so they were compiled out; the NEON
implementations they defer to are the `.S` files that were never added.

Result: `MNNBinaryAddInt8` and ~110 siblings are referenced by
`CPUBinaryInt8.cpp` and defined nowhere. `MNN_SEP_BUILD=ON` means
`libMNN.so` no longer carries its own kernel objects, so nothing else supplies
them either — and `libMNN_CPU.so` is not produced by the `MNN` target.

## Why the shipped library still works

The committed `libMNN.so` (`09523ac5…`) has **zero** undefined `MNN*` symbols.
It was built with the ARM assembly in place. That is the one to keep.

## What this blocks

`isp.unpack_blc` needs a shape rule in `source/shape/ShapeIspExtra.cpp`, which
links into `libMNN.so`. The rebuilt core library has the rule (the string
`isp.unpack_blc` is present) but cannot be loaded for the reason above, so the
op still fails at session creation on this device:

    [MNN-ISP-CONVERT] onnx domain=isp op_type=isp.unpack_blc -> … type=isp.unpack_blc
    (then) signal: 11

Everything on the other side of the boundary is in place and verified:

  * `isp_spv/isp_unpack_blc.comp` — compiled, bindings 0/3/4/5/6, frame at 3/4
  * `IspSpvLookup.{cpp,hpp}` — embedded and registered
  * `IspOpsetValidate.hpp` — whitelisted, and the rebuilt
    `libMNNConvertDeps.so` (5.4 MB, stripped) carries the entry, so the
    converter no longer reports `INVALID isp.* opset`
  * `ShapeIspExtra.cpp` — the packed→quad rule

## The fix, when a full rebuild is possible

Configure with the processor set explicitly, e.g.

    cmake -DCMAKE_SYSTEM_PROCESSOR=aarch64 … (existing build_isp flags)

which lets `arm/CMakeLists.txt` add `MNNARM64` and links the assembly kernels
back in. That is a full reconfigure and rebuild of the core library.
