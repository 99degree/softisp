#!/usr/bin/env python3
"""Remove fused_6in1 SPIR-V from embedded header."""
PATH = '/data/data/com.termux/files/home/MNN/tools/converter/source/optimizer/postconvert/isp_spirv_embedded.h'
with open(PATH) as f:
    lines = f.readlines()

# Find the fused_6in1 block (starts at "static const unsigned char g_fused_6in1_spv[]")
# and remove everything until "static const int g_fused_6in1_spv_len = ..." inclusive
start = None
end = None
for i, line in enumerate(lines):
    if 'g_fused_6in1_spv[]' in line:
        start = i
    if start is not None and 'g_fused_6in1_spv_len' in line:
        end = i + 1
        break

if start is not None and end is not None:
    new_lines = lines[:start] + lines[end:]
    with open(PATH, 'w') as f:
        f.writelines(new_lines)
    print(f"Removed lines {start+1}-{end} ({end-start} lines)")
else:
    print(f"ERROR: start={start} end={end}")
