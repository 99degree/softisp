#!/bin/bash
# SoftISP Fuzzer Runner Script
# Runs fuzz testing for specified duration

set -e

echo "=== SoftISP Fuzzer Runner ==="
echo "Date: $(date)"
echo ""

DURATION=${1:-60}  # Default 60 seconds per target
CORPUS_DIR="fuzz/corpus"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Create corpus directories
mkdir -p "$CORPUS_DIR/fuzz_onnx_parse"
mkdir -p "$CORPUS_DIR/fuzz_pipeline_config"
mkdir -p "$CORPUS_DIR/fuzz_profile"
mkdir -p "$CORPUS_DIR/fuzz_block_id"

run_fuzzer() {
    local target=$1
    local corpus=$2
    
    echo -e "${YELLOW}Fuzzing $target for ${DURATION}s...${NC}"
    
    if cargo fuzz run "$target" "$corpus" -- -max_total_time="$DURATION" -max_len=4096; then
        echo -e "${GREEN}✓ $target completed without crashes${NC}"
        return 0
    else
        echo -e "${RED}✗ $target found a crash!${NC}"
        return 1
    fi
}

CRASHES=0

# Run each fuzzer
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if ! run_fuzzer "fuzz_onnx_parse" "$CORPUS_DIR/fuzz_onnx_parse"; then
    CRASHES=$((CRASHES + 1))
fi

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if ! run_fuzzer "fuzz_pipeline_config" "$CORPUS_DIR/fuzz_pipeline_config"; then
    CRASHES=$((CRASHES + 1))
fi

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if ! run_fuzzer "fuzz_profile" "$CORPUS_DIR/fuzz_profile"; then
    CRASHES=$((CRASHES + 1))
fi

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if ! run_fuzzer "fuzz_block_id" "$CORPUS_DIR/fuzz_block_id"; then
    CRASHES=$((CRASHES + 1))
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if [ $CRASHES -eq 0 ]; then
    echo -e "${GREEN}All fuzzers completed without crashes!${NC}"
    exit 0
else
    echo -e "${RED}$CRASHES fuzzer(s) found crashes!${NC}"
    exit 1
fi
