import os, pathlib, re

ndk = pathlib.Path(os.environ.get('ANDROID_NDK_ROOT', pathlib.Path.home() / 'android-sdk/ndk/29.0.14206865'))
root = ndk / 'toolchains/llvm/prebuilt/linux-x86_64/sysroot/usr/include/vulkan'
text = ''
for name in ['vulkan_core.h', 'vulkan_android.h']:
    text += (root / name).read_text()

text = re.sub(r'/\*.*?\*/', '', text, flags=re.S)
text = re.sub(r'//[^\n]*', '', text)

pattern = re.compile(r'VKAPI_ATTR\s+(.+?)\s+VKAPI_CALL\s+(vk\w+)\s*\((.*?)\)\s*;', re.S)
funcs = {}
for m in pattern.finditer(text):
    ret, name, params = m.group(1).strip(), m.group(2), m.group(3).strip()
    funcs[name] = (ret, params)

def split_params(s):
    if not s or s == 'void':
        return []
    out, depth, cur = [], 0, ''
    for ch in s:
        if ch in '<([':
            depth += 1
        elif ch in '>)]':
            depth -= 1
        if ch == ',' and depth == 0:
            out.append(cur.strip())
            cur = ''
        else:
            cur += ch
    if cur.strip():
        out.append(cur.strip())
    return out

def param_name(p):
    p = p.strip()
    mm = re.search(r'([A-Za-z_]\w*)\s*(?:\[[^\]]*\])?\s*$', p)
    return mm.group(1) if mm else ''

def first_handle(params):
    for p in params:
        t = p.rsplit(' ', 1)[0] if ' ' in p else p
        if t.startswith('Vk') and t not in ('VkResult', 'VkBool32'):
            return param_name(p)
    return None

lines = []
lines.append('// Generated passthrough loader for Android hwvulkan HAL drivers.')
lines.append('#define VK_USE_PLATFORM_ANDROID_KHR')
lines.append('#define VK_ENABLE_BETA_EXTENSIONS')
lines.append('#include <vulkan/vulkan.h>')
lines.append('#include <sys/system_properties.h>')
lines.append('#include <dlfcn.h>')
lines.append('#include <cctype>')
lines.append('#include <cstdint>')
lines.append('#include <cstdio>')
lines.append('#include <cstdlib>')
lines.append('#include <cstring>')
lines.append('#include <string>')
lines.append('')
lines.append('static PFN_vkGetInstanceProcAddr g_hal_gipa = nullptr;')
lines.append('static PFN_vkGetInstanceProcAddr g_fallback_gipa = nullptr;')
lines.append('static void* g_hal_handle = nullptr;')
lines.append('static void* g_fallback_handle = nullptr;')
lines.append('static VkInstance g_instance = VK_NULL_HANDLE;')
lines.append('')
lines.append(r'''static bool is_adreno_platform() {
    const char* override = getenv("SOFTISP_VULKAN_PLATFORM");
    if (override != nullptr && override[0] != '\0') {
        std::string value(override);
        for (char& c : value) c = static_cast<char>(std::tolower(c));
        return value.find("adreno") != std::string::npos ||
               value.find("qcom") != std::string::npos;
    }
    const char* keys[] = {"ro.hardware.vulkan", "ro.hardware", "ro.board.platform"};
    for (const char* key : keys) {
        char value[PROP_VALUE_MAX] = {0};
        if (__system_property_get(key, value) <= 0) continue;
        for (char* p = value; *p != '\0'; ++p) *p = static_cast<char>(std::tolower(*p));
        if (std::strstr(value, "adreno") != nullptr || std::strstr(value, "qcom") != nullptr) {
            return true;
        }
    }
    return false;
}''')
lines.append('')
lines.append(r'''static void init_fallback_loader() {
    if (g_fallback_gipa != nullptr) return;
    g_fallback_handle = dlopen("/system/lib64/libvulkan.so", RTLD_NOW | RTLD_LOCAL);
    if (g_fallback_handle == nullptr) {
        g_fallback_handle = dlopen("libvulkan.so.1", RTLD_NOW | RTLD_LOCAL);
    }
    if (g_fallback_handle == nullptr) {
        fprintf(stderr, "[softisp-vulkan] system loader unavailable: %s\n", dlerror());
        return;
    }
    g_fallback_gipa = reinterpret_cast<PFN_vkGetInstanceProcAddr>(
        dlsym(g_fallback_handle, "vkGetInstanceProcAddr"));
    if (g_fallback_gipa == nullptr) {
        fprintf(stderr, "[softisp-vulkan] system loader has no vkGetInstanceProcAddr\n");
        return;
    }
    fprintf(stderr, "[softisp-vulkan] system loader ready\n");
}''')
lines.append('')
lines.append(r'''static void init_backend() {
    if (g_hal_gipa != nullptr || g_fallback_gipa != nullptr) return;
    if (!is_adreno_platform()) {
        init_fallback_loader();
        return;
    }

    const char* override = getenv("SOFTISP_VULKAN_DRIVER");
    const char* path = (override != nullptr && override[0] != '\0')
                           ? override
                           : "libsoftisp_vulkan_adreno.so";
    g_hal_handle = dlopen(path, RTLD_NOW | RTLD_LOCAL);
    if (g_hal_handle == nullptr && (override == nullptr || override[0] == '\0')) {
        path = "/vendor/lib64/hw/vulkan.adreno.so";
        g_hal_handle = dlopen(path, RTLD_NOW | RTLD_LOCAL);
    }
    if (g_hal_handle == nullptr) {
        fprintf(stderr, "[softisp-vulkan] Adreno HAL unavailable (%s): %s\n", path, dlerror());
        init_fallback_loader();
        return;
    }

    auto* module = reinterpret_cast<uint64_t*>(dlsym(g_hal_handle, "HMI"));
    if (module == nullptr) {
        fprintf(stderr, "[softisp-vulkan] %s has no HMI\n", path);
        init_fallback_loader();
        return;
    }
    auto* methods = reinterpret_cast<uint64_t*>(module[4]);
    auto open_fn = reinterpret_cast<int (*)(uint64_t*, const char*, uint64_t**)>(methods[0]);
    uint64_t* device = nullptr;
    const char* device_id = getenv("SOFTISP_HWVULKAN_DEVICE_ID");
    if (device_id == nullptr || device_id[0] == '\0') device_id = "vulkan";
    if (open_fn(module, device_id, &device) != 0 || device == nullptr) {
        fprintf(stderr, "[softisp-vulkan] HAL open failed for id=%s\n", device_id);
        init_fallback_loader();
        return;
    }
    g_hal_gipa = reinterpret_cast<PFN_vkGetInstanceProcAddr>(device[17]);
    fprintf(stderr, "[softisp-vulkan] HAL ready: %s id=%s\n", path, device_id);
}''')
lines.append('')
lines.append('__attribute__((constructor)) static void init_early() { init_backend(); }')
lines.append('')
lines.append(r'''static PFN_vkGetInstanceProcAddr backend_gipa() {
    return g_hal_gipa != nullptr ? g_hal_gipa : g_fallback_gipa;
}

static PFN_vkVoidFunction resolve(VkInstance, const char* name) {
    auto backend = backend_gipa();
    if (backend == nullptr) return nullptr;
    auto fn = backend(g_instance, name);
    if (!fn) fprintf(stderr, "[softisp-vulkan] unresolved %s\n", name);
    return fn;
}''')
lines.append('')
lines.append('extern "C" VKAPI_ATTR PFN_vkVoidFunction VKAPI_CALL vkGetInstanceProcAddr(VkInstance instance, const char* name);')
lines.append('')

registration = []
for name in sorted(funcs):
    if name in ('vkGetInstanceProcAddr',):
        continue
    ret, params_s = funcs[name]
    params = split_params(params_s)
    fixed_params = []
    names = []
    for idx, p in enumerate(params):
        n = param_name(p)
        if not n:
            n = f'unnamed{idx}'
            p = f'{p} {n}'
        fixed_params.append(p)
        names.append(n)
    params_s = ', '.join(fixed_params)
    handle = first_handle(fixed_params)
    resolve_arg = f'reinterpret_cast<VkInstance>({handle})' if handle else 'VK_NULL_HANDLE'
    lines.append(f'extern "C" VKAPI_ATTR {ret} VKAPI_CALL {name}({params_s}) {{')
    lines.append(f'    using Fn = {ret} (VKAPI_CALL *)({params_s});')
    lines.append(f'    static Fn fn = reinterpret_cast<Fn>(resolve({resolve_arg}, "{name}"));')
    call_args = ', '.join(names)
    if name == 'vkCreateInstance':
        lines.append(f'    VkResult rc = fn({call_args});')
        lines.append('    if (rc == VK_SUCCESS && pInstance != nullptr) g_instance = *pInstance;')
        lines.append('    return rc;')
    elif name == 'vkDestroyInstance':
        lines.append(f'    fn({call_args});')
        lines.append('    g_instance = VK_NULL_HANDLE;')
    else:
        lines.append(f'    return fn({call_args});')
    lines.append('}')
    registration.append((name, f'(PFN_vkVoidFunction)&{name}'))

lines.append('')
lines.append('struct NameEntry { const char* name; PFN_vkVoidFunction fn; };')
lines.append('static const NameEntry g_entries[] = {')
lines.append('    {"vkGetInstanceProcAddr", (PFN_vkVoidFunction)&vkGetInstanceProcAddr},')
for name, ptr in registration:
    lines.append(f'    {{"{name}", {ptr}}},')
lines.append('    {nullptr, nullptr},')
lines.append('};')
lines.append('')
lines.append('extern "C" VKAPI_ATTR PFN_vkVoidFunction VKAPI_CALL vkGetInstanceProcAddr(VkInstance instance, const char* name) {')
lines.append('    if (name == nullptr) return nullptr;')
lines.append('    for (const NameEntry* e = g_entries; e->name != nullptr; ++e) {')
lines.append('        if (std::strcmp(e->name, name) == 0) return e->fn;')
lines.append('    }')
lines.append('    auto backend = backend_gipa();')
lines.append('    return backend != nullptr ? backend(instance != VK_NULL_HANDLE ? instance : g_instance, name) : nullptr;')
lines.append('}')
lines.append('')

out = pathlib.Path(__file__).resolve().parents[1] / 'vulkan_loader/softisp_vulkan_loader.cpp'
out.parent.mkdir(parents=True, exist_ok=True)
out.write_text('\n'.join(lines) + '\n')
print(f'generated {len(registration)} wrappers -> {out}')
