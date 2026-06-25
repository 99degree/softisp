#include <stdio.h>
#include <dlfcn.h>
int main() {
    void* handle = dlopen("libMNN_Vulkan.so", RTLD_NOW | RTLD_GLOBAL);
    if (!handle) {
        printf("dlopen failed: %s\n", dlerror());
        return 1;
    }
    void* sym = dlsym(handle, "MNNVulkanFuseRegister");
    printf("dlsym result: %p\n", sym);
    if (!sym) printf("dlsym error: %s\n", dlerror());
    dlclose(handle);
    return 0;
}
