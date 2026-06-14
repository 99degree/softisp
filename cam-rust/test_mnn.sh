#!/usr/bin/env bash
# Test MNN backend availability and run benchmarks when libraries are present.
# Requires: libMNN.so, libMNN_Express.so in LD_LIBRARY_PATH or system lib dirs.

set -e

echo "=== MNN Backend Test ==="
echo ""

# Check for MNN shared libraries
echo "Checking for MNN libraries..."
if ldconfig -p 2>/dev/null | grep -q libMNN; then
    echo "  ✓ libMNN found in ldconfig"
else
    echo "  ✗ libMNN NOT found in ldconfig"
fi

if [ -n "$LD_LIBRARY_PATH" ]; then
    echo "  LD_LIBRARY_PATH: $LD_LIBRARY_PATH"
    for lib in libMNN.so libMNN_Express.so libMNN_Vulkan.so libMNN_CL.so; do
        found=0
        for dir in $(echo "$LD_LIBRARY_PATH" | tr ':' '\n'); do
            if [ -f "$dir/$lib" ]; then
                echo "  ✓ $lib found in $dir"
                found=1
                break
            fi
        done
        if [ "$found" -eq 0 ]; then
            echo "  ✗ $lib NOT found in LD_LIBRARY_PATH"
        fi
    done
else
    echo "  LD_LIBRARY_PATH not set"
fi

echo ""

# Try to build with MNN feature
echo "Building cam-isp with 'mnn' feature..."
if cargo check -p cam-isp --features mnn 2>/dev/null; then
    echo "  ✓ cam-isp compiles with 'mnn' feature"
else
    echo "  ✗ cam-isp FAILED to compile with 'mnn' feature"
    echo "  (Check output above for compilation errors)"
    exit 1
fi

echo ""

# Try to run tests (may fail at link time if .so missing)
echo "Attempting to run unit tests with 'mnn' feature..."
if cargo test -p cam-isp --lib --features mnn 2>/tmp/mnn_test.log; then
    echo "  ✓ Unit tests pass with 'mnn' feature"
else
    echo "  ✗ Unit tests failed (see /tmp/mnn_test.log)"
    echo ""
    echo "  Common causes:"
    echo "    - libMNN.so not in LD_LIBRARY_PATH or system lib dir"
    echo "    - MNN libraries not installed on device"
    echo ""
    echo "  To use MNN backend, install MNN libraries and set:"
    echo "    export LD_LIBRARY_PATH=/path/to/mnn/lib:\$LD_LIBRARY_PATH"
fi

echo ""
echo "=== MNN Test Complete ==="
