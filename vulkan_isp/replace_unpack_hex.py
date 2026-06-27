#!/usr/bin/env python3
"""Replace unpack_demosaic SPIR-V in embedded header with new FCS-fused version."""
import re

HEADER = '/data/data/com.termux/files/home/MNN/tools/converter/source/optimizer/postconvert/isp_spirv_embedded.h'
NEW_HEX = '/data/data/com.termux/files/home/tmp/new_unpack_hex.h'

with open(HEADER) as f:
    header = f.read()

with open(NEW_HEX) as f:
    new_hex_lines = [l.rstrip() for l in f.readlines() if l.strip()]

new_hex_str = '\n'.join(new_hex_lines)

# Replace the old SPIR-V array
old_pattern = r'static const unsigned char g_unpack_demosaic_spv\[\] = \{.*?\n\};\nstatic const int g_unpack_demosaic_spv_len = \d+;'
new_text = f'static const unsigned char g_unpack_demosaic_spv[] = {{\n{new_hex_str}\n}};\nstatic const int g_unpack_demosaic_spv_len = 8660;'

new_header = re.sub(old_pattern, new_text, header, flags=re.DOTALL)

if new_header == header:
    print("ERROR: No replacement made!")
else:
    with open(HEADER, 'w') as f:
        f.write(new_header)
    # Verify
    import subprocess
    r = subprocess.run(['grep', '-c', 'g_unpack_demosaic_spv_len = 8660', HEADER], capture_output=True, text=True)
    print(f"Replaced: verify count = {r.stdout.strip()}")
    r2 = subprocess.run(['wc', '-l', HEADER], capture_output=True, text=True)
    print(f"Header: {r2.stdout.strip()}")
