#define _GNU_SOURCE
#include <stdio.h>
#include <dlfcn.h>
int main() {
    // Try RTLD_NOLOAD to get existing handle
    void* h1 = dlopen("libMNN_Vulkan.so", RTLD_NOLOAD | RTLD_LAZY);
    printf("RTLD_NOLOAD handle: %p\n", h1);
    
    void* h2 = dlopen("libMNN_Vulkan.so", RTLD_LAZY | RTLD_LOCAL);
    printf("RTLD_LAZY handle: %p\n", h2);
    
    // Try dlsym from RTLD_DEFAULT
    void* s1 = dlsym(RTLD_DEFAULT, "MNNVulkanFuseRegister");
    printf("RTLD_DEFAULT: %p\n", s1);
    if (!s1) printf(" error: %s\n", dlerror());
    
    // Try dlsym from RTLD_NEXT
    void* s2 = dlsym(RTLD_NEXT, "MNNVulkanFuseRegister");
    printf("RTLD_NEXT: %p\n", s2);
    if (!s2) printf(" error: %s\n", dlerror());
    
    // Try looking for any function in the Vulkan lib
    void* s3 = dlsym(RTLD_DEFAULT, "InitVulkan");
    printf("InitVulkan via RTLD_DEFAULT: %p\n", s3);
    if (!s3) printf(" error: %s\n", dlerror());
    
    return 0;
}
