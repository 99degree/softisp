#include <stdio.h>
extern void MNNVulkanFuseRegister();
int main() {
    printf("✅ MNNVulkanFuseRegister symbol resolves\n");
    MNNVulkanFuseRegister();
    return 0;
}
