with open('android-app/src/lib.rs', 'r', encoding='utf-8') as f:
    lines = f.readlines()

old_lines = [
    'use android_activity::ndk::native_window::NativeWindow;\n',
    'use android_activity::{AndroidApp, InputStatus, MainEvent, PollEvent};\n',
    'use cam_isp::engine::{select_engine_by_name, IspEngine};\n',
    'use cam_isp::mnnengine::MnnEngine;\n',
    'use cam_isp::profile::PipelineProfile;\n',
    'use cam_isp::{release_native_window, set_native_window};\n',
]

new_lines = [
    'use android_activity::ndk::native_window::NativeWindow;\n',
    'use android_activity::{AndroidApp, InputStatus, MainEvent, PollEvent};\n',
    'use cam_isp::dual_engine::DualGpuEngine;\n',
    'use cam_isp::engine::{select_engine_by_name, IspEngine};\n',
    'use cam_isp::mnnengine::{MnnBackend, MnnEngine};\n',
    'use cam_isp::profile::PipelineProfile;\n',
    'use cam_isp::{release_native_window, set_native_window};\n',
]

current_lines = lines[18:25]
if current_lines == old_lines:
    lines[18:25] = new_lines
    with open('android-app/src/lib.rs', 'w', encoding='utf-8') as f:
        f.writelines(lines)
    print('UPDATED')
else:
    print('MISMATCH')
    for i, (cur, expected) in enumerate(zip(current_lines, old_lines)):
        if cur != expected:
            print(f'  Line {18+i}:')
            print(f'    CURRENT: {repr(cur)}')
            print(f'    EXPECTED: {repr(expected)}')
