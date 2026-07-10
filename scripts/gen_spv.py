import sys
size_x = int(sys.argv[1])
size_y = int(sys.argv[2])
src = """#version 450
layout(local_size_x = %d, local_size_y = %d, local_size_z = 1) in;
void main() {
    // ... shader payload ...
}
""" % (size_x, size_y)
open("spirv/variant_%dx%d.comp" % (size_x, size_y), "w").write(src); EOF
python gen_spv.py 16 16; echo "Generated shader template"