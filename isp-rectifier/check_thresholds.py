#!/usr/bin/env python3
"""
CI/CD Threshold Checker for Quantized Models

Reads validation results JSON and enforces accuracy thresholds.
Exits with code 1 if any threshold is exceeded (fails CI).
"""

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, Any

# ANSI colors
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
BOLD = "\033[1m"
RESET = "\033[0m"

# Default thresholds (mean, max) for each output head
DEFAULT_THRESHOLDS = {
    "wbgains": {"mean": 0.01, "max": 0.05, "std": 0.02},
    "ccm": {"mean": 0.01, "max": 0.05, "std": 0.02},
    "tonecurve": {"mean": 0.02, "max": 0.10, "std": 0.03},
    "zoom_factor": {"mean": 0.005, "max": 0.02, "std": 0.01},
}

# Latency thresholds (ms)
LATENCY_THRESHOLDS = {
    "fp32_max_ms": 10.0,
    "quant_max_ms": 5.0,
    "min_speedup": 1.5,
}


def load_results(filepath: str) -> Dict[str, Any]:
    """Load validation results from JSON."""
    with open(filepath, 'r') as f:
        return json.load(f)


def check_threshold(
    name: str,
    value: float,
    threshold: float,
    metric: str,
) -> bool:
    """Check a single threshold."""
    passed = value <= threshold
    status = f"{GREEN}✅{RESET}" if passed else f"{RED}❌{RESET}"
    color = GREEN if passed else RED
    print(f"  {status} {name:12} {metric}: {color}{value:.6f}{RESET} <= {threshold:.6f}")
    return passed


def print_header(title: str):
    print(f"\n{BOLD}{'='*60}{RESET}")
    print(f"{BOLD}{title:^60}{RESET}")
    print(f"{BOLD}{'='*60}{RESET}")


def print_table_header():
    print(f"{'Head':<12} {'Metric':<8} {'Value':>12} {'Threshold':>12} {'Status':>8}")
    print("-" * 60)


def main():
    parser = argparse.ArgumentParser(
        description="Check quantization validation results against thresholds"
    )
    parser.add_argument("results", help="Path to validation_results.json")
    parser.add_argument("--thresholds", help="Path to custom thresholds JSON")
    parser.add_argument("--strict", action="store_true", 
                        help="Fail on warnings (missing heads, etc.)")
    parser.add_argument("--output", help="Write summary to file")
    parser.add_argument("--github-output", action="store_true",
                        help="Format output for GitHub Actions annotations")
    
    args = parser.parse_args()
    
    # Load results
    results = load_results(args.results)
    
    # Load custom thresholds if provided
    thresholds = DEFAULT_THRESHOLDS.copy()
    if args.thresholds:
        with open(args.thresholds, 'r') as f:
            custom = json.load(f)
            thresholds.update(custom)
    
    all_passed = True
    warnings = []
    
    # Check each output head
    print_header("OUTPUT ACCURACY CHECKS")
    print_table_header()
    
    for head_name, limits in thresholds.items():
        if head_name not in results:
            msg = f"Missing results for head: {head_name}"
            warnings.append(msg)
            print(f"  {YELLOW}⚠️ {RESET} {head_name:12} {'MISSING':<8} {'N/A':>12} {'N/A':>12} {'SKIP':>8}")
            if args.strict:
                all_passed = False
            continue
        
        data = results[head_name]
        
        # Check mean
        mean_ok = check_threshold(f"{head_name}_mean", data['mean'], limits['mean'], "mean")
        # Check max
        max_ok = check_threshold(f"{head_name}_max", data['max'], limits['max'], "max")
        # Check std
        std_ok = check_threshold(f"{head_name}_std", data['std'], limits.get('std', float('inf')), "std")
        
        head_passed = mean_ok and max_ok and std_ok
        all_passed = all_passed and head_passed
    
    # Check latency
    print_header("LATENCY CHECKS")
    print_table_header()
    
    if 'latency' in results:
        lat = results['latency']
        
        fp32_ok = check_threshold("FP32 latency", lat['fp32_mean_ms'], LATENCY_THRESHOLDS['fp32_max_ms'], "ms")
        quant_ok = check_threshold("Quant latency", lat['quant_mean_ms'], LATENCY_THRESHOLDS['quant_max_ms'], "ms")
        speedup_ok = check_threshold("Speedup", lat['speedup'], LATENCY_THRESHOLDS['min_speedup'], "x")
        
        all_passed = all_passed and fp32_ok and quant_ok and speedup_ok
    else:
        warnings.append("No latency data in results")
        print(f"  {YELLOW}⚠️ {RESET} {'latency':<12} {'MISSING':<8} {'N/A':>12} {'N/A':>12} {'SKIP':>8}")
    
    # Warnings
    if warnings:
        print_header("WARNINGS")
        for w in warnings:
            print(f"  {YELLOW}⚠️  {w}{RESET}")
        if args.strict:
            all_passed = False
    
    # Summary
    print_header("SUMMARY")
    if all_passed:
        print(f"{GREEN}{BOLD}✅ ALL THRESHOLDS PASSED{RESET}")
        print(f"Quantized model is safe for deployment.")
    else:
        print(f"{RED}{BOLD}❌ THRESHOLD VIOLATIONS DETECTED{RESET}")
        print(f"Quantized model exceeds accuracy/latency limits.")
    
    # GitHub Actions annotations
    if args.github_output:
        print("::group=Threshold Check Results")
        if all_passed:
            print("::notice::All quantization thresholds passed ✅")
        else:
            print("::error::Quantization thresholds failed ❌")
        print("::endgroup::")
    
    # Write output file
    if args.output:
        summary = {
            "passed": all_passed,
            "warnings": warnings,
            "results_checked": args.results,
        }
        with open(args.output, 'w') as f:
            json.dump(summary, f, indent=2)
        print(f"\n📄 Summary written to {args.output}")
    
    sys.exit(0 if all_passed else 1)


if __name__ == "__main__":
    main()