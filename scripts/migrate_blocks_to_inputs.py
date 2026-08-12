#!/usr/bin/env python3
"""Migrate ISP blocks from baked Const initializers to runtime Input tensors."""

import re, os, sys

BLOCKS_DIR = sys.argv[1] if len(sys.argv) > 1 else "cam-rust/cam-isp/src/blocks"

RE_INIT = re.compile(r'fn\s+initializers\s*\(&self\)\s*->\s*Vec<Vec<u8>>\s*\{')
RE_EXTRA = re.compile(r'fn\s+extra_inputs\s*\(&self\)\s*->\s*Vec<\(String,\s*i64,\s*Vec<i64>\)>\s*\{')
RE_PROTO_START = re.compile(r'Proto::tensor_proto_(float_scalar|float|int64|int32_scalar)\s*\(')


def find_paren_end(text, start):
    depth = 1
    in_string = False
    in_char = False
    i = start + 1
    while i < len(text) and depth > 0:
        c = text[i]
        if in_string:
            if c == '\\':
                i += 2
                continue
            elif c == '"':
                in_string = False
        elif in_char:
            if c == '\\':
                i += 2
                continue
            elif c == "'":
                in_char = False
        else:
            if c == '"':
                in_string = True
            elif c == "'":
                in_char = True
            elif c == '(':
                depth += 1
            elif c == ')':
                depth -= 1
        i += 1
    return i


def find_matching_brace(content, start):
    """Find matching close brace, skipping string literals."""
    depth = 1
    in_string = False
    in_char = False
    i = start
    while i < len(content):
        c = content[i]
        if in_string:
            if c == '\\':
                i += 2
                continue
            elif c == '"':
                in_string = False
        elif in_char:
            if c == '\\':
                i += 2
                continue
            elif c == "'":
                in_char = False
        else:
            if c == '"':
                in_string = True
            elif c == "'":
                in_char = True
            elif c == '{':
                depth += 1
            elif c == '}':
                depth -= 1
                if depth == 0:
                    return i + 1
        i += 1
    return len(content)


def extract_proto_calls(body):
    """Extract (call_type, name_expr, shape_str, data_expr) from Proto calls in body."""
    calls = []
    for m in RE_PROTO_START.finditer(body):
        call_type = m.group(1)
        paren_start = body.index('(', m.start())
        call_end = find_paren_end(body, paren_start)
        call_body = body[paren_start + 1:call_end - 1]

        # Extract name: &format!(...) or plain string literal
        fmt_match = re.search(r'(&format!\s*\(.*?\))', call_body)
        if fmt_match:
            name_expr = fmt_match.group(1).strip()
            # Strip & prefix for use in extra_inputs tuples
            if name_expr.startswith('&'):
                name_expr = name_expr[1:]
        else:
            str_match = re.search(r'"([^"]+)"', call_body)
            if str_match:
                name_expr = f'"{str_match.group(1)}"'
            else:
                continue

        # Extract shape: look for &[...]
        shape_match = re.search(r'&\[(.*?)\]', call_body)
        shape_str = shape_match.group(1).strip() if shape_match else ""

        # Extract data depending on call type
        if call_type in ('float_scalar', 'int32_scalar'):
            # Scalar: data is everything after the last comma
            comma_positions = [i for i, c in enumerate(call_body) if c == ',']
            if comma_positions:
                last_comma = comma_positions[-1]
                data_expr = call_body[last_comma + 1:].strip()
            else:
                data_expr = call_body.strip()
        else:
            # Array: find all &[...] patterns, take the last one as data
            amp_matches = list(re.finditer(r'&\[(.*?)\]', call_body))
            if amp_matches:
                # First &[...] is shape, second is data (if exists)
                if len(amp_matches) >= 2:
                    data_expr = amp_matches[-1].group(0).strip()  # include &[...]
                else:
                    # Only one &[...] — it might be data, or it might be a reference to a variable
                    data_expr = ''
            else:
                # Try to find data after last &
                amp = call_body.rfind('&')
                if amp != -1:
                    data_expr = call_body[amp + 1:].strip().rstrip(',').strip()
                else:
                    data_expr = ''

        calls.append((call_type, name_expr, shape_str, data_expr))
    return calls


def rust_bytes(expr, dtype):
    """Convert a data expression to Rust bytes serialization code."""
    expr = expr.strip()
    if not expr:
        return "vec![]"

    # Scalar float literal
    if re.match(r'^-?\d+\.\d*([eE][+-]?\d+)?$f32$', expr):
        return f"({expr}).to_ne_bytes().to_vec()"
    if re.match(r'^-?\d+\.\d*([eE][+-]?\d+)?$', expr):
        return f"({expr}f32).to_ne_bytes().to_vec()"
    if expr == '0.0':
        return "(0.0f32).to_ne_bytes().to_vec()"
    if expr == '1.0':
        return "(1.0f32).to_ne_bytes().to_vec()"
    if expr == '0.5':
        return "(0.5f32).to_ne_bytes().to_vec()"
    if expr == '0.95':
        return "(0.95f32).to_ne_bytes().to_vec()"
    if expr == '0.05':
        return "(0.05f32).to_ne_bytes().to_vec()"
    if expr == '1e-6f32':
        return "(1e-6f32).to_ne_bytes().to_vec()"

    # Scalar int literal
    if re.match(r'^-?\d+$', expr):
        if dtype == 'int32_scalar':
            return f"({expr}i32).to_ne_bytes().to_vec()"
        elif dtype == 'int64':
            return f"({expr}i64).to_ne_bytes().to_vec()"
        return f"({expr}f32).to_ne_bytes().to_vec()"

    # self.xxx (scalar field)
    if re.match(r'^self\.\w+$', expr):
        return f"({expr}).to_ne_bytes().to_vec()"

    # Array expressions: [val, val, ...] or &[val, val, ...]
    inner_expr = expr[1:] if expr.startswith('&') and expr.startswith('&[') else expr
    if inner_expr.startswith('[') and inner_expr.endswith(']'):
        inner = inner_expr[1:-1]
        parts = [p.strip() for p in inner.split(',')]
        fixed_parts = []
        for p in parts:
            if re.match(r'^-?\d+$', p):
                fixed_parts.append(p + 'i64')
            elif re.match(r'^-?\d+\.\d*([eE][+-]?\d+)?$', p) and not p.endswith('f32'):
                fixed_parts.append(p + 'f32')
            elif re.match(r'^self\.\w+', p):
                fixed_parts.append(p)
            else:
                fixed_parts.append(p)
        fixed = '[' + ', '.join(fixed_parts) + ']'
        return f"{fixed}.iter().flat_map(|v| v.to_ne_bytes()).collect::<Vec<u8>>()"

    # Vec/generator expressions with self. or collect
    if 'self.' in expr or 'collect' in expr or 'map(' in expr or 'flat_map' in expr:
        return f"({expr}).iter().flat_map(|v| v.to_ne_bytes()).collect::<Vec<u8>>()"

    return f"({expr}).as_bytes().to_vec()"


def parse_shape(shape_str):
    if not shape_str:
        return "vec![]"
    parts = [p.strip() for p in shape_str.split(',')]
    return "vec![" + ", ".join(parts) + "]"


def migrate_file(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    matches = list(RE_INIT.finditer(content))
    if not matches:
        return False

    any_migrated = False
    for m_init in reversed(matches):
        init_start = m_init.start()
        init_end = find_matching_brace(content, m_init.end())
        old_block = content[init_start:init_end]

        proto_calls = extract_proto_calls(old_block)
        if not proto_calls:
            continue

        # Check if the initializers body has local variable computations
        body_without_proto = old_block
        for ct, ne, ss, de in proto_calls:
            # Check if shape or data reference local variables (not literal)
            for expr in [ss, de]:
                if expr and re.search(r'\b(self\.|sy|sx|sah|saw|sh|sw|crop_h|crop_w|_oh|_ow|pad_arr|scale_h|scale_w|oc|k|h|w|b|weights|bias|sat_w|lum_w|lap_w|sad_w|raw_w|flip_h|flip_w|input_matrix|output_matrix|bs|height|width|scope)\b', expr):
                    # This Proto call references local variables
                    # We'll still migrate it but with vec![] shape
                    pass

        # Build new initializers
        new_init = "fn initializers(&self) -> Vec<Vec<u8>> {\n        vec![]\n    }"

        # Build extra_inputs (single vec![] block)
        extra_lines = ["fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {"]
        extra_lines.append("        let ns = self.tensor_ns();")
        extra_lines.append("        vec![")
        for call_type, name_expr, shape_str, data_expr in proto_calls:
            if call_type == 'float_scalar':
                dtype = 1
                shape = "vec![]"
            elif call_type == 'float':
                dtype = 1
                shape = parse_shape(shape_str)
            elif call_type == 'int64':
                dtype = 6  # int32 (MNN Vulkan can't handle INT64)
                shape = parse_shape(shape_str)
            elif call_type == 'int32_scalar':
                dtype = 6
                shape = "vec![]"
            else:
                continue
            extra_lines.append(f'            ({name_expr}.to_string(), {dtype}, {shape}),')
        extra_lines.append("        ]")
        extra_lines.append("    }")
        new_extra = "    " + "\n".join(extra_lines)

        # Build extra_input_defaults
        defaults_lines = [
            "fn extra_input_defaults(&self) -> Vec<(String, Vec<u8>)> {",
            "        let ns = self.tensor_ns();",
            "        vec!["
        ]
        for call_type, name_expr, shape_str, data_expr in proto_calls:
            data_code = rust_bytes(data_expr, call_type)
            defaults_lines.append(f'            ({name_expr}.to_string(), {data_code}),')
        defaults_lines.append("        ]")
        defaults_lines.append("    }")
        new_defaults = "    " + "\n".join(defaults_lines)

        # Find the extra_inputs that belongs to this struct (search after init_start)
        m_extra = RE_EXTRA.search(content, init_start)
        if m_extra:
            extra_end = find_matching_brace(content, m_extra.end())
            content = content[:init_start] + new_init + "\n\n" + new_extra + "\n\n" + new_defaults + content[extra_end:]
        else:
            content = content[:init_start] + new_init + "\n\n" + new_extra + "\n\n" + new_defaults + content[init_end:]

        any_migrated = True

    if any_migrated:
        with open(filepath, 'w') as f:
            f.write(content)
    return any_migrated


def main():
    migrated = []
    for fname in sorted(os.listdir(BLOCKS_DIR)):
        if not fname.endswith('.rs'):
            continue
        fpath = os.path.join(BLOCKS_DIR, fname)
        try:
            if migrate_file(fpath):
                migrated.append(fname)
                print(f"  migrated: {fname}")
        except Exception as e:
            print(f"  ERROR in {fname}: {e}", file=sys.stderr)

    print(f"\nMigrated {len(migrated)} block files.")


if __name__ == '__main__':
    main()
