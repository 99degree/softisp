#include <stdio.h>
#include <dlfcn.h>
int main() {
    void* handle = dlopen("libMNN_Vulkan.so", RTLD_NOW | RTLD_LOCAL);
    printf("handle: %p\n", handle);
    if (!handle) { printf("dlopen error: %s\n", dlerror()); return 1; }
    
    dlerror();  // Clear errors
    
    // Try dlsym with the handle
    void* sym1 = dlsym(handle, "MNNVulkanFuseRegister");
    printf("dlsym1: %p\n", sym1);
    if (!sym1) printf(" err1: %s\n", dlerror());
    
    // Try dlsym with the handle for a known function in the Vulkan lib
    void* sym2 = dlsym(handle, "_GLOBAL__sub_I_VulkanFuse.cpp");
    printf("dlsym2: %p\n", sym2);
    if (!sym2) printf(" err2: %s\n", dlerror());
    
    // Try dlsym with RTLD_DEFAULT after loading the lib
    void* sym3 = dlsym(RTLD_DEFAULT, "MNNVulkanFuseRegister");
    printf("dlsym_RTLD_DEFAULT: %p\n", sym3);
    if (!sym3) printf(" err3: %s\n", dlerror());
    
    dlclose(handle);
    return 0;
}
