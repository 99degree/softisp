with open('cam-rust/cam-isp/src/mnnengine.rs', 'r', encoding='utf-8') as f:
    content = f.read()

old = '''                let buf_id = buf_id_for_release.unwrap();
                let mut pool = self.buf_pool.lock().unwrap();
                let buf = pool.buffer_mut(buf_id);
                Self::f32_to_u8(&buf)'''

new = '''                let mut pool = self.buf_pool.lock().unwrap();
                let buf = pool.buffer_mut(buf_id);
                // Reinterpret f32 buffer as u8 bytes for clamping conversion.
                let buf = unsafe {
                    std::slice::from_raw_parts_mut(buf.as_mut_ptr() as *mut u8, buf.len() * 4)
                };
                Self::f32_to_u8(buf)'''

if old in content:
    content = content.replace(old, new)
    with open('cam-rust/cam-isp/src/mnnengine.rs', 'w', encoding='utf-8') as f:
        f.write(content)
    print('FIXED')
else:
    print('OLD STRING NOT FOUND')
    lines = content.splitlines()
    for i, line in enumerate(lines[2370:2385], start=2371):
        print(f'{i}: {repr(line)}')
