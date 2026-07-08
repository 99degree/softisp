import sys
data = open(sys.argv[1], 'rb').read()
lines = []
for i in range(0, len(data), 12):
    chunk = data[i:i+12]
    hex_bytes = ', '.join(f'0x{b:02x}' for b in chunk)
    lines.append(f'  {hex_bytes},')
lines[-1] = lines[-1].rstrip(',')
print('static const unsigned char g_argb_convert_spv[] = {')
for l in lines:
    print(l)
print('};')
print(f'static const int g_argb_convert_spv_len = {len(data)};')
