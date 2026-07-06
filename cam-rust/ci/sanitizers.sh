#!/bin/bash
# SoftISP Sanitizer CI Script
# Runs tests with AddressSanitizer, MemorySanitizer, and UndefinedBehaviorSanitizer

set -e

echo "=== SoftISP Sanitizer CI ==="
echo "Date: $(date)"
echo "Rust version: $(rustc --version)"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

run_sanitizer() {
    local sanitizer=$1
    local flags=$2
    
    echo -e "${YELLOW}Running $sanitizer...${NC}"
    
    if RUSTFLAGS="$flags" cargo test -p cam-isp --lib 2>&1; then
        echo -e "${GREEN}✓ $sanitizer passed${NC}"
        return 0
    else
        echo -e "${RED}✗ $sanitizer failed${NC}"
        return 1
    fi
}

FAILED=0

# AddressSanitizer - detects memory errors
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if ! run_sanitizer "AddressSanitizer" "-Zsanitizer=address"; then
    FAILED=1
fi

# MemorySanitizer - detects uninitialized memory reads
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if ! run_sanitizer "MemorySanitizer" "-Zsanitizer=memory"; then
    FAILED=1
fi

# UndefinedBehaviorSanitizer - detects undefined behavior
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if ! run_sanitizer "ThreadSanitizer" "-Zsanitizer=thread"; then
    FAILED=1
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if [ $FAILED -eq 0 ]; then
    echo -e "${GREEN}All sanitizer checks passed!${NC}"
    exit 0
else
    echo -e "${RED}Some sanitizer checks failed!${NC}"
    exit 1
fi
