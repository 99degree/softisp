#!/usr/bin/env python3
import numpy as np
import sys

def validate_bayer(filename, width, height, bitdepth, stride, pattern):
    # Load raw buffer
    raw = np.fromfile(filename, dtype=np.uint16)
    expected_size = (stride // 2) * height
    if raw.size != expected_size:
        print(f"[ERROR] Buffer size mismatch: got {raw.size}, expected {expected_size}")
        return

    # Reshape with stride
    frame = raw.reshape((height, stride // 2))

    # Logical crop: remove stride padding
    cropped = frame[:, :width]

    # Normalize
    max_val = (1 << bitdepth) - 1
    norm = cropped.astype(np.float32) / max_val

    print(f"[INFO] Resolution={width}x{height}, Bitdepth={bitdepth}, Pattern={pattern}")
    print(f"[INFO] Min={norm.min():.3f}, Max={norm.max():.3f}, Mean={norm.mean():.3f}")

    # Top-left 2x2 block check
    block = norm[0:2, 0:2]
    print(f"[INFO] Top-left 2x2 block:\n{block}")

    if pattern == "RGGB":
        print("[EXPECT] Top-left pixel is RED")
    elif pattern == "BGGR":
        print("[EXPECT] Top-left pixel is BLUE")
    elif pattern == "GRBG":
        print("[EXPECT] Top-left pixel is GREEN (Red next)")
    elif pattern == "GBRG":
        print("[EXPECT] Top-left pixel is GREEN (Blue next)")
    else:
        print("[ERROR] Unknown pattern")

if __name__ == "__main__":
    if len(sys.argv) < 6:
        print("Usage: test_bayer.py <file> <width> <height> <bitdepth> <stride> <pattern>")
        sys.exit(1)
    validate_bayer(sys.argv[1],
                   int(sys.argv[2]),
                   int(sys.argv[3]),
                   int(sys.argv[4]),
                   int(sys.argv[5]),
                   sys.argv[6])
