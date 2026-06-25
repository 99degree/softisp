#include <stdio.h>
#include <dlfcn.h>
int main() {
    void* handle = dlopen("libMNN_Vulkan.so", RTLD_GLOBAL | RTLD_NOW);
    printf("handle: %p\n", handle);
    if (!handle) { printf("dlopen error: %s\n", dlerror()); return 1; }
    
    dlerror();  // Clear errors
    
    void* sym = dlsym(handle, "MNNVulkanFuseRegister");
    printf("dlsym(handle): %p\n", sym);
    if (!sym) printf(" err: %s\n", dlerror());
    
    sym = dlsym(RTLD_DEFAULT, "MNNVulkanFuseRegister");
    printf("dlsym(RTLD_DEFAULT): %p\n", sym);
    if (!sym) printf(" err: %s\n", dlerror());
    
    dlclose(handle);
    return 0;
}
