// Generated passthrough loader for Android hwvulkan HAL drivers.
#define VK_USE_PLATFORM_ANDROID_KHR
#define VK_ENABLE_BETA_EXTENSIONS
#include <vulkan/vulkan.h>
#include <sys/system_properties.h>
#include <dlfcn.h>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

static PFN_vkGetInstanceProcAddr g_hal_gipa = nullptr;
static PFN_vkGetInstanceProcAddr g_fallback_gipa = nullptr;
static void* g_hal_handle = nullptr;
static void* g_fallback_handle = nullptr;
static VkInstance g_instance = VK_NULL_HANDLE;

static bool is_adreno_platform() {
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
}

static void init_fallback_loader() {
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
}

static void init_backend() {
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
}

__attribute__((constructor)) static void init_early() { init_backend(); }

static PFN_vkGetInstanceProcAddr backend_gipa() {
    return g_hal_gipa != nullptr ? g_hal_gipa : g_fallback_gipa;
}

static PFN_vkVoidFunction resolve(VkInstance, const char* name) {
    auto backend = backend_gipa();
    if (backend == nullptr) return nullptr;
    auto fn = backend(g_instance, name);
    if (!fn) fprintf(stderr, "[softisp-vulkan] unresolved %s\n", name);
    return fn;
}

extern "C" VKAPI_ATTR PFN_vkVoidFunction VKAPI_CALL vkGetInstanceProcAddr(VkInstance instance, const char* name);

extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkAcquireDrmDisplayEXT(VkPhysicalDevice                            physicalDevice, int32_t                                     drmFd, VkDisplayKHR                                display) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, int32_t                                     drmFd, VkDisplayKHR                                display);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkAcquireDrmDisplayEXT"));
    return fn(physicalDevice, drmFd, display);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkAcquireNextImage2KHR(VkDevice                                    device, const VkAcquireNextImageInfoKHR*            pAcquireInfo, uint32_t*                                   pImageIndex) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkAcquireNextImageInfoKHR*            pAcquireInfo, uint32_t*                                   pImageIndex);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkAcquireNextImage2KHR"));
    return fn(device, pAcquireInfo, pImageIndex);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkAcquireNextImageKHR(VkDevice                                    device, VkSwapchainKHR                              swapchain, uint64_t                                    timeout, VkSemaphore                                 semaphore, VkFence                                     fence, uint32_t*                                   pImageIndex) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkSwapchainKHR                              swapchain, uint64_t                                    timeout, VkSemaphore                                 semaphore, VkFence                                     fence, uint32_t*                                   pImageIndex);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkAcquireNextImageKHR"));
    return fn(device, swapchain, timeout, semaphore, fence, pImageIndex);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkAcquirePerformanceConfigurationINTEL(VkDevice                                    device, const VkPerformanceConfigurationAcquireInfoINTEL* pAcquireInfo, VkPerformanceConfigurationINTEL*            pConfiguration) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkPerformanceConfigurationAcquireInfoINTEL* pAcquireInfo, VkPerformanceConfigurationINTEL*            pConfiguration);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkAcquirePerformanceConfigurationINTEL"));
    return fn(device, pAcquireInfo, pConfiguration);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkAcquireProfilingLockKHR(VkDevice                                    device, const VkAcquireProfilingLockInfoKHR*        pInfo) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkAcquireProfilingLockInfoKHR*        pInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkAcquireProfilingLockKHR"));
    return fn(device, pInfo);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkAllocateCommandBuffers(VkDevice                                    device, const VkCommandBufferAllocateInfo*          pAllocateInfo, VkCommandBuffer*                            pCommandBuffers) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkCommandBufferAllocateInfo*          pAllocateInfo, VkCommandBuffer*                            pCommandBuffers);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkAllocateCommandBuffers"));
    return fn(device, pAllocateInfo, pCommandBuffers);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkAllocateDescriptorSets(VkDevice                                    device, const VkDescriptorSetAllocateInfo*          pAllocateInfo, VkDescriptorSet*                            pDescriptorSets) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkDescriptorSetAllocateInfo*          pAllocateInfo, VkDescriptorSet*                            pDescriptorSets);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkAllocateDescriptorSets"));
    return fn(device, pAllocateInfo, pDescriptorSets);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkAllocateMemory(VkDevice                                    device, const VkMemoryAllocateInfo*                 pAllocateInfo, const VkAllocationCallbacks*                pAllocator, VkDeviceMemory*                             pMemory) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkMemoryAllocateInfo*                 pAllocateInfo, const VkAllocationCallbacks*                pAllocator, VkDeviceMemory*                             pMemory);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkAllocateMemory"));
    return fn(device, pAllocateInfo, pAllocator, pMemory);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkBeginCommandBuffer(VkCommandBuffer                             commandBuffer, const VkCommandBufferBeginInfo*             pBeginInfo) {
    using Fn = VkResult (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkCommandBufferBeginInfo*             pBeginInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkBeginCommandBuffer"));
    return fn(commandBuffer, pBeginInfo);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkBindAccelerationStructureMemoryNV(VkDevice                                    device, uint32_t                                    bindInfoCount, const VkBindAccelerationStructureMemoryInfoNV* pBindInfos) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, uint32_t                                    bindInfoCount, const VkBindAccelerationStructureMemoryInfoNV* pBindInfos);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkBindAccelerationStructureMemoryNV"));
    return fn(device, bindInfoCount, pBindInfos);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkBindBufferMemory(VkDevice                                    device, VkBuffer                                    buffer, VkDeviceMemory                              memory, VkDeviceSize                                memoryOffset) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkBuffer                                    buffer, VkDeviceMemory                              memory, VkDeviceSize                                memoryOffset);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkBindBufferMemory"));
    return fn(device, buffer, memory, memoryOffset);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkBindBufferMemory2(VkDevice                                    device, uint32_t                                    bindInfoCount, const VkBindBufferMemoryInfo*               pBindInfos) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, uint32_t                                    bindInfoCount, const VkBindBufferMemoryInfo*               pBindInfos);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkBindBufferMemory2"));
    return fn(device, bindInfoCount, pBindInfos);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkBindBufferMemory2KHR(VkDevice                                    device, uint32_t                                    bindInfoCount, const VkBindBufferMemoryInfo*               pBindInfos) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, uint32_t                                    bindInfoCount, const VkBindBufferMemoryInfo*               pBindInfos);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkBindBufferMemory2KHR"));
    return fn(device, bindInfoCount, pBindInfos);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkBindImageMemory(VkDevice                                    device, VkImage                                     image, VkDeviceMemory                              memory, VkDeviceSize                                memoryOffset) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkImage                                     image, VkDeviceMemory                              memory, VkDeviceSize                                memoryOffset);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkBindImageMemory"));
    return fn(device, image, memory, memoryOffset);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkBindImageMemory2(VkDevice                                    device, uint32_t                                    bindInfoCount, const VkBindImageMemoryInfo*                pBindInfos) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, uint32_t                                    bindInfoCount, const VkBindImageMemoryInfo*                pBindInfos);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkBindImageMemory2"));
    return fn(device, bindInfoCount, pBindInfos);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkBindImageMemory2KHR(VkDevice                                    device, uint32_t                                    bindInfoCount, const VkBindImageMemoryInfo*                pBindInfos) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, uint32_t                                    bindInfoCount, const VkBindImageMemoryInfo*                pBindInfos);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkBindImageMemory2KHR"));
    return fn(device, bindInfoCount, pBindInfos);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkBindOpticalFlowSessionImageNV(VkDevice                                    device, VkOpticalFlowSessionNV                      session, VkOpticalFlowSessionBindingPointNV          bindingPoint, VkImageView                                 view, VkImageLayout                               layout) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkOpticalFlowSessionNV                      session, VkOpticalFlowSessionBindingPointNV          bindingPoint, VkImageView                                 view, VkImageLayout                               layout);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkBindOpticalFlowSessionImageNV"));
    return fn(device, session, bindingPoint, view, layout);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkBindVideoSessionMemoryKHR(VkDevice                                    device, VkVideoSessionKHR                           videoSession, uint32_t                                    bindSessionMemoryInfoCount, const VkBindVideoSessionMemoryInfoKHR*      pBindSessionMemoryInfos) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkVideoSessionKHR                           videoSession, uint32_t                                    bindSessionMemoryInfoCount, const VkBindVideoSessionMemoryInfoKHR*      pBindSessionMemoryInfos);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkBindVideoSessionMemoryKHR"));
    return fn(device, videoSession, bindSessionMemoryInfoCount, pBindSessionMemoryInfos);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkBuildAccelerationStructuresKHR(VkDevice                                    device, VkDeferredOperationKHR                      deferredOperation, uint32_t                                    infoCount, const VkAccelerationStructureBuildGeometryInfoKHR* pInfos, const VkAccelerationStructureBuildRangeInfoKHR* const* ppBuildRangeInfos) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkDeferredOperationKHR                      deferredOperation, uint32_t                                    infoCount, const VkAccelerationStructureBuildGeometryInfoKHR* pInfos, const VkAccelerationStructureBuildRangeInfoKHR* const* ppBuildRangeInfos);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkBuildAccelerationStructuresKHR"));
    return fn(device, deferredOperation, infoCount, pInfos, ppBuildRangeInfos);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkBuildMicromapsEXT(VkDevice                                    device, VkDeferredOperationKHR                      deferredOperation, uint32_t                                    infoCount, const VkMicromapBuildInfoEXT*               pInfos) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkDeferredOperationKHR                      deferredOperation, uint32_t                                    infoCount, const VkMicromapBuildInfoEXT*               pInfos);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkBuildMicromapsEXT"));
    return fn(device, deferredOperation, infoCount, pInfos);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBeginConditionalRenderingEXT(VkCommandBuffer                             commandBuffer, const VkConditionalRenderingBeginInfoEXT*   pConditionalRenderingBegin) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkConditionalRenderingBeginInfoEXT*   pConditionalRenderingBegin);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBeginConditionalRenderingEXT"));
    return fn(commandBuffer, pConditionalRenderingBegin);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBeginDebugUtilsLabelEXT(VkCommandBuffer                             commandBuffer, const VkDebugUtilsLabelEXT*                 pLabelInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkDebugUtilsLabelEXT*                 pLabelInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBeginDebugUtilsLabelEXT"));
    return fn(commandBuffer, pLabelInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBeginQuery(VkCommandBuffer                             commandBuffer, VkQueryPool                                 queryPool, uint32_t                                    query, VkQueryControlFlags                         flags) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkQueryPool                                 queryPool, uint32_t                                    query, VkQueryControlFlags                         flags);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBeginQuery"));
    return fn(commandBuffer, queryPool, query, flags);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBeginQueryIndexedEXT(VkCommandBuffer                             commandBuffer, VkQueryPool                                 queryPool, uint32_t                                    query, VkQueryControlFlags                         flags, uint32_t                                    index) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkQueryPool                                 queryPool, uint32_t                                    query, VkQueryControlFlags                         flags, uint32_t                                    index);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBeginQueryIndexedEXT"));
    return fn(commandBuffer, queryPool, query, flags, index);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBeginRenderPass(VkCommandBuffer                             commandBuffer, const VkRenderPassBeginInfo*                pRenderPassBegin, VkSubpassContents                           contents) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkRenderPassBeginInfo*                pRenderPassBegin, VkSubpassContents                           contents);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBeginRenderPass"));
    return fn(commandBuffer, pRenderPassBegin, contents);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBeginRenderPass2(VkCommandBuffer                             commandBuffer, const VkRenderPassBeginInfo*                pRenderPassBegin, const VkSubpassBeginInfo*                   pSubpassBeginInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkRenderPassBeginInfo*                pRenderPassBegin, const VkSubpassBeginInfo*                   pSubpassBeginInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBeginRenderPass2"));
    return fn(commandBuffer, pRenderPassBegin, pSubpassBeginInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBeginRenderPass2KHR(VkCommandBuffer                             commandBuffer, const VkRenderPassBeginInfo*                pRenderPassBegin, const VkSubpassBeginInfo*                   pSubpassBeginInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkRenderPassBeginInfo*                pRenderPassBegin, const VkSubpassBeginInfo*                   pSubpassBeginInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBeginRenderPass2KHR"));
    return fn(commandBuffer, pRenderPassBegin, pSubpassBeginInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBeginRendering(VkCommandBuffer                             commandBuffer, const VkRenderingInfo*                      pRenderingInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkRenderingInfo*                      pRenderingInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBeginRendering"));
    return fn(commandBuffer, pRenderingInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBeginRenderingKHR(VkCommandBuffer                             commandBuffer, const VkRenderingInfo*                      pRenderingInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkRenderingInfo*                      pRenderingInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBeginRenderingKHR"));
    return fn(commandBuffer, pRenderingInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBeginTransformFeedbackEXT(VkCommandBuffer                             commandBuffer, uint32_t                                    firstCounterBuffer, uint32_t                                    counterBufferCount, const VkBuffer*                             pCounterBuffers, const VkDeviceSize*                         pCounterBufferOffsets) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    firstCounterBuffer, uint32_t                                    counterBufferCount, const VkBuffer*                             pCounterBuffers, const VkDeviceSize*                         pCounterBufferOffsets);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBeginTransformFeedbackEXT"));
    return fn(commandBuffer, firstCounterBuffer, counterBufferCount, pCounterBuffers, pCounterBufferOffsets);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBeginVideoCodingKHR(VkCommandBuffer                             commandBuffer, const VkVideoBeginCodingInfoKHR*            pBeginInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkVideoBeginCodingInfoKHR*            pBeginInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBeginVideoCodingKHR"));
    return fn(commandBuffer, pBeginInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBindDescriptorBufferEmbeddedSamplers2EXT(VkCommandBuffer                             commandBuffer, const VkBindDescriptorBufferEmbeddedSamplersInfoEXT* pBindDescriptorBufferEmbeddedSamplersInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkBindDescriptorBufferEmbeddedSamplersInfoEXT* pBindDescriptorBufferEmbeddedSamplersInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBindDescriptorBufferEmbeddedSamplers2EXT"));
    return fn(commandBuffer, pBindDescriptorBufferEmbeddedSamplersInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBindDescriptorBufferEmbeddedSamplersEXT(VkCommandBuffer                             commandBuffer, VkPipelineBindPoint                         pipelineBindPoint, VkPipelineLayout                            layout, uint32_t                                    set) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkPipelineBindPoint                         pipelineBindPoint, VkPipelineLayout                            layout, uint32_t                                    set);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBindDescriptorBufferEmbeddedSamplersEXT"));
    return fn(commandBuffer, pipelineBindPoint, layout, set);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBindDescriptorBuffersEXT(VkCommandBuffer                             commandBuffer, uint32_t                                    bufferCount, const VkDescriptorBufferBindingInfoEXT*     pBindingInfos) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    bufferCount, const VkDescriptorBufferBindingInfoEXT*     pBindingInfos);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBindDescriptorBuffersEXT"));
    return fn(commandBuffer, bufferCount, pBindingInfos);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBindDescriptorSets(VkCommandBuffer                             commandBuffer, VkPipelineBindPoint                         pipelineBindPoint, VkPipelineLayout                            layout, uint32_t                                    firstSet, uint32_t                                    descriptorSetCount, const VkDescriptorSet*                      pDescriptorSets, uint32_t                                    dynamicOffsetCount, const uint32_t*                             pDynamicOffsets) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkPipelineBindPoint                         pipelineBindPoint, VkPipelineLayout                            layout, uint32_t                                    firstSet, uint32_t                                    descriptorSetCount, const VkDescriptorSet*                      pDescriptorSets, uint32_t                                    dynamicOffsetCount, const uint32_t*                             pDynamicOffsets);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBindDescriptorSets"));
    return fn(commandBuffer, pipelineBindPoint, layout, firstSet, descriptorSetCount, pDescriptorSets, dynamicOffsetCount, pDynamicOffsets);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBindDescriptorSets2KHR(VkCommandBuffer                             commandBuffer, const VkBindDescriptorSetsInfoKHR*          pBindDescriptorSetsInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkBindDescriptorSetsInfoKHR*          pBindDescriptorSetsInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBindDescriptorSets2KHR"));
    return fn(commandBuffer, pBindDescriptorSetsInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBindIndexBuffer(VkCommandBuffer                             commandBuffer, VkBuffer                                    buffer, VkDeviceSize                                offset, VkIndexType                                 indexType) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBuffer                                    buffer, VkDeviceSize                                offset, VkIndexType                                 indexType);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBindIndexBuffer"));
    return fn(commandBuffer, buffer, offset, indexType);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBindIndexBuffer2KHR(VkCommandBuffer                             commandBuffer, VkBuffer                                    buffer, VkDeviceSize                                offset, VkDeviceSize                                size, VkIndexType                                 indexType) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBuffer                                    buffer, VkDeviceSize                                offset, VkDeviceSize                                size, VkIndexType                                 indexType);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBindIndexBuffer2KHR"));
    return fn(commandBuffer, buffer, offset, size, indexType);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBindInvocationMaskHUAWEI(VkCommandBuffer                             commandBuffer, VkImageView                                 imageView, VkImageLayout                               imageLayout) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkImageView                                 imageView, VkImageLayout                               imageLayout);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBindInvocationMaskHUAWEI"));
    return fn(commandBuffer, imageView, imageLayout);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBindPipeline(VkCommandBuffer                             commandBuffer, VkPipelineBindPoint                         pipelineBindPoint, VkPipeline                                  pipeline) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkPipelineBindPoint                         pipelineBindPoint, VkPipeline                                  pipeline);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBindPipeline"));
    return fn(commandBuffer, pipelineBindPoint, pipeline);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBindPipelineShaderGroupNV(VkCommandBuffer                             commandBuffer, VkPipelineBindPoint                         pipelineBindPoint, VkPipeline                                  pipeline, uint32_t                                    groupIndex) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkPipelineBindPoint                         pipelineBindPoint, VkPipeline                                  pipeline, uint32_t                                    groupIndex);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBindPipelineShaderGroupNV"));
    return fn(commandBuffer, pipelineBindPoint, pipeline, groupIndex);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBindShadersEXT(VkCommandBuffer                             commandBuffer, uint32_t                                    stageCount, const VkShaderStageFlagBits*                pStages, const VkShaderEXT*                          pShaders) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    stageCount, const VkShaderStageFlagBits*                pStages, const VkShaderEXT*                          pShaders);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBindShadersEXT"));
    return fn(commandBuffer, stageCount, pStages, pShaders);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBindShadingRateImageNV(VkCommandBuffer                             commandBuffer, VkImageView                                 imageView, VkImageLayout                               imageLayout) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkImageView                                 imageView, VkImageLayout                               imageLayout);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBindShadingRateImageNV"));
    return fn(commandBuffer, imageView, imageLayout);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBindTransformFeedbackBuffersEXT(VkCommandBuffer                             commandBuffer, uint32_t                                    firstBinding, uint32_t                                    bindingCount, const VkBuffer*                             pBuffers, const VkDeviceSize*                         pOffsets, const VkDeviceSize*                         pSizes) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    firstBinding, uint32_t                                    bindingCount, const VkBuffer*                             pBuffers, const VkDeviceSize*                         pOffsets, const VkDeviceSize*                         pSizes);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBindTransformFeedbackBuffersEXT"));
    return fn(commandBuffer, firstBinding, bindingCount, pBuffers, pOffsets, pSizes);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBindVertexBuffers(VkCommandBuffer                             commandBuffer, uint32_t                                    firstBinding, uint32_t                                    bindingCount, const VkBuffer*                             pBuffers, const VkDeviceSize*                         pOffsets) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    firstBinding, uint32_t                                    bindingCount, const VkBuffer*                             pBuffers, const VkDeviceSize*                         pOffsets);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBindVertexBuffers"));
    return fn(commandBuffer, firstBinding, bindingCount, pBuffers, pOffsets);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBindVertexBuffers2(VkCommandBuffer                             commandBuffer, uint32_t                                    firstBinding, uint32_t                                    bindingCount, const VkBuffer*                             pBuffers, const VkDeviceSize*                         pOffsets, const VkDeviceSize*                         pSizes, const VkDeviceSize*                         pStrides) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    firstBinding, uint32_t                                    bindingCount, const VkBuffer*                             pBuffers, const VkDeviceSize*                         pOffsets, const VkDeviceSize*                         pSizes, const VkDeviceSize*                         pStrides);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBindVertexBuffers2"));
    return fn(commandBuffer, firstBinding, bindingCount, pBuffers, pOffsets, pSizes, pStrides);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBindVertexBuffers2EXT(VkCommandBuffer                             commandBuffer, uint32_t                                    firstBinding, uint32_t                                    bindingCount, const VkBuffer*                             pBuffers, const VkDeviceSize*                         pOffsets, const VkDeviceSize*                         pSizes, const VkDeviceSize*                         pStrides) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    firstBinding, uint32_t                                    bindingCount, const VkBuffer*                             pBuffers, const VkDeviceSize*                         pOffsets, const VkDeviceSize*                         pSizes, const VkDeviceSize*                         pStrides);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBindVertexBuffers2EXT"));
    return fn(commandBuffer, firstBinding, bindingCount, pBuffers, pOffsets, pSizes, pStrides);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBlitImage(VkCommandBuffer                             commandBuffer, VkImage                                     srcImage, VkImageLayout                               srcImageLayout, VkImage                                     dstImage, VkImageLayout                               dstImageLayout, uint32_t                                    regionCount, const VkImageBlit*                          pRegions, VkFilter                                    filter) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkImage                                     srcImage, VkImageLayout                               srcImageLayout, VkImage                                     dstImage, VkImageLayout                               dstImageLayout, uint32_t                                    regionCount, const VkImageBlit*                          pRegions, VkFilter                                    filter);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBlitImage"));
    return fn(commandBuffer, srcImage, srcImageLayout, dstImage, dstImageLayout, regionCount, pRegions, filter);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBlitImage2(VkCommandBuffer                             commandBuffer, const VkBlitImageInfo2*                     pBlitImageInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkBlitImageInfo2*                     pBlitImageInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBlitImage2"));
    return fn(commandBuffer, pBlitImageInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBlitImage2KHR(VkCommandBuffer                             commandBuffer, const VkBlitImageInfo2*                     pBlitImageInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkBlitImageInfo2*                     pBlitImageInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBlitImage2KHR"));
    return fn(commandBuffer, pBlitImageInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBuildAccelerationStructureNV(VkCommandBuffer                             commandBuffer, const VkAccelerationStructureInfoNV*        pInfo, VkBuffer                                    instanceData, VkDeviceSize                                instanceOffset, VkBool32                                    update, VkAccelerationStructureNV                   dst, VkAccelerationStructureNV                   src, VkBuffer                                    scratch, VkDeviceSize                                scratchOffset) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkAccelerationStructureInfoNV*        pInfo, VkBuffer                                    instanceData, VkDeviceSize                                instanceOffset, VkBool32                                    update, VkAccelerationStructureNV                   dst, VkAccelerationStructureNV                   src, VkBuffer                                    scratch, VkDeviceSize                                scratchOffset);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBuildAccelerationStructureNV"));
    return fn(commandBuffer, pInfo, instanceData, instanceOffset, update, dst, src, scratch, scratchOffset);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBuildAccelerationStructuresIndirectKHR(VkCommandBuffer                             commandBuffer, uint32_t                                    infoCount, const VkAccelerationStructureBuildGeometryInfoKHR* pInfos, const VkDeviceAddress*                      pIndirectDeviceAddresses, const uint32_t*                             pIndirectStrides, const uint32_t* const*                      ppMaxPrimitiveCounts) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    infoCount, const VkAccelerationStructureBuildGeometryInfoKHR* pInfos, const VkDeviceAddress*                      pIndirectDeviceAddresses, const uint32_t*                             pIndirectStrides, const uint32_t* const*                      ppMaxPrimitiveCounts);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBuildAccelerationStructuresIndirectKHR"));
    return fn(commandBuffer, infoCount, pInfos, pIndirectDeviceAddresses, pIndirectStrides, ppMaxPrimitiveCounts);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBuildAccelerationStructuresKHR(VkCommandBuffer                             commandBuffer, uint32_t                                    infoCount, const VkAccelerationStructureBuildGeometryInfoKHR* pInfos, const VkAccelerationStructureBuildRangeInfoKHR* const* ppBuildRangeInfos) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    infoCount, const VkAccelerationStructureBuildGeometryInfoKHR* pInfos, const VkAccelerationStructureBuildRangeInfoKHR* const* ppBuildRangeInfos);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBuildAccelerationStructuresKHR"));
    return fn(commandBuffer, infoCount, pInfos, ppBuildRangeInfos);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdBuildMicromapsEXT(VkCommandBuffer                             commandBuffer, uint32_t                                    infoCount, const VkMicromapBuildInfoEXT*               pInfos) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    infoCount, const VkMicromapBuildInfoEXT*               pInfos);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdBuildMicromapsEXT"));
    return fn(commandBuffer, infoCount, pInfos);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdClearAttachments(VkCommandBuffer                             commandBuffer, uint32_t                                    attachmentCount, const VkClearAttachment*                    pAttachments, uint32_t                                    rectCount, const VkClearRect*                          pRects) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    attachmentCount, const VkClearAttachment*                    pAttachments, uint32_t                                    rectCount, const VkClearRect*                          pRects);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdClearAttachments"));
    return fn(commandBuffer, attachmentCount, pAttachments, rectCount, pRects);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdClearColorImage(VkCommandBuffer                             commandBuffer, VkImage                                     image, VkImageLayout                               imageLayout, const VkClearColorValue*                    pColor, uint32_t                                    rangeCount, const VkImageSubresourceRange*              pRanges) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkImage                                     image, VkImageLayout                               imageLayout, const VkClearColorValue*                    pColor, uint32_t                                    rangeCount, const VkImageSubresourceRange*              pRanges);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdClearColorImage"));
    return fn(commandBuffer, image, imageLayout, pColor, rangeCount, pRanges);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdClearDepthStencilImage(VkCommandBuffer                             commandBuffer, VkImage                                     image, VkImageLayout                               imageLayout, const VkClearDepthStencilValue*             pDepthStencil, uint32_t                                    rangeCount, const VkImageSubresourceRange*              pRanges) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkImage                                     image, VkImageLayout                               imageLayout, const VkClearDepthStencilValue*             pDepthStencil, uint32_t                                    rangeCount, const VkImageSubresourceRange*              pRanges);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdClearDepthStencilImage"));
    return fn(commandBuffer, image, imageLayout, pDepthStencil, rangeCount, pRanges);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdControlVideoCodingKHR(VkCommandBuffer                             commandBuffer, const VkVideoCodingControlInfoKHR*          pCodingControlInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkVideoCodingControlInfoKHR*          pCodingControlInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdControlVideoCodingKHR"));
    return fn(commandBuffer, pCodingControlInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdCopyAccelerationStructureKHR(VkCommandBuffer                             commandBuffer, const VkCopyAccelerationStructureInfoKHR*   pInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkCopyAccelerationStructureInfoKHR*   pInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdCopyAccelerationStructureKHR"));
    return fn(commandBuffer, pInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdCopyAccelerationStructureNV(VkCommandBuffer                             commandBuffer, VkAccelerationStructureNV                   dst, VkAccelerationStructureNV                   src, VkCopyAccelerationStructureModeKHR          mode) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkAccelerationStructureNV                   dst, VkAccelerationStructureNV                   src, VkCopyAccelerationStructureModeKHR          mode);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdCopyAccelerationStructureNV"));
    return fn(commandBuffer, dst, src, mode);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdCopyAccelerationStructureToMemoryKHR(VkCommandBuffer                             commandBuffer, const VkCopyAccelerationStructureToMemoryInfoKHR* pInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkCopyAccelerationStructureToMemoryInfoKHR* pInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdCopyAccelerationStructureToMemoryKHR"));
    return fn(commandBuffer, pInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdCopyBuffer(VkCommandBuffer                             commandBuffer, VkBuffer                                    srcBuffer, VkBuffer                                    dstBuffer, uint32_t                                    regionCount, const VkBufferCopy*                         pRegions) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBuffer                                    srcBuffer, VkBuffer                                    dstBuffer, uint32_t                                    regionCount, const VkBufferCopy*                         pRegions);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdCopyBuffer"));
    return fn(commandBuffer, srcBuffer, dstBuffer, regionCount, pRegions);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdCopyBuffer2(VkCommandBuffer                             commandBuffer, const VkCopyBufferInfo2*                    pCopyBufferInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkCopyBufferInfo2*                    pCopyBufferInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdCopyBuffer2"));
    return fn(commandBuffer, pCopyBufferInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdCopyBuffer2KHR(VkCommandBuffer                             commandBuffer, const VkCopyBufferInfo2*                    pCopyBufferInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkCopyBufferInfo2*                    pCopyBufferInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdCopyBuffer2KHR"));
    return fn(commandBuffer, pCopyBufferInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdCopyBufferToImage(VkCommandBuffer                             commandBuffer, VkBuffer                                    srcBuffer, VkImage                                     dstImage, VkImageLayout                               dstImageLayout, uint32_t                                    regionCount, const VkBufferImageCopy*                    pRegions) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBuffer                                    srcBuffer, VkImage                                     dstImage, VkImageLayout                               dstImageLayout, uint32_t                                    regionCount, const VkBufferImageCopy*                    pRegions);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdCopyBufferToImage"));
    return fn(commandBuffer, srcBuffer, dstImage, dstImageLayout, regionCount, pRegions);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdCopyBufferToImage2(VkCommandBuffer                             commandBuffer, const VkCopyBufferToImageInfo2*             pCopyBufferToImageInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkCopyBufferToImageInfo2*             pCopyBufferToImageInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdCopyBufferToImage2"));
    return fn(commandBuffer, pCopyBufferToImageInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdCopyBufferToImage2KHR(VkCommandBuffer                             commandBuffer, const VkCopyBufferToImageInfo2*             pCopyBufferToImageInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkCopyBufferToImageInfo2*             pCopyBufferToImageInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdCopyBufferToImage2KHR"));
    return fn(commandBuffer, pCopyBufferToImageInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdCopyImage(VkCommandBuffer                             commandBuffer, VkImage                                     srcImage, VkImageLayout                               srcImageLayout, VkImage                                     dstImage, VkImageLayout                               dstImageLayout, uint32_t                                    regionCount, const VkImageCopy*                          pRegions) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkImage                                     srcImage, VkImageLayout                               srcImageLayout, VkImage                                     dstImage, VkImageLayout                               dstImageLayout, uint32_t                                    regionCount, const VkImageCopy*                          pRegions);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdCopyImage"));
    return fn(commandBuffer, srcImage, srcImageLayout, dstImage, dstImageLayout, regionCount, pRegions);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdCopyImage2(VkCommandBuffer                             commandBuffer, const VkCopyImageInfo2*                     pCopyImageInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkCopyImageInfo2*                     pCopyImageInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdCopyImage2"));
    return fn(commandBuffer, pCopyImageInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdCopyImage2KHR(VkCommandBuffer                             commandBuffer, const VkCopyImageInfo2*                     pCopyImageInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkCopyImageInfo2*                     pCopyImageInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdCopyImage2KHR"));
    return fn(commandBuffer, pCopyImageInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdCopyImageToBuffer(VkCommandBuffer                             commandBuffer, VkImage                                     srcImage, VkImageLayout                               srcImageLayout, VkBuffer                                    dstBuffer, uint32_t                                    regionCount, const VkBufferImageCopy*                    pRegions) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkImage                                     srcImage, VkImageLayout                               srcImageLayout, VkBuffer                                    dstBuffer, uint32_t                                    regionCount, const VkBufferImageCopy*                    pRegions);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdCopyImageToBuffer"));
    return fn(commandBuffer, srcImage, srcImageLayout, dstBuffer, regionCount, pRegions);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdCopyImageToBuffer2(VkCommandBuffer                             commandBuffer, const VkCopyImageToBufferInfo2*             pCopyImageToBufferInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkCopyImageToBufferInfo2*             pCopyImageToBufferInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdCopyImageToBuffer2"));
    return fn(commandBuffer, pCopyImageToBufferInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdCopyImageToBuffer2KHR(VkCommandBuffer                             commandBuffer, const VkCopyImageToBufferInfo2*             pCopyImageToBufferInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkCopyImageToBufferInfo2*             pCopyImageToBufferInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdCopyImageToBuffer2KHR"));
    return fn(commandBuffer, pCopyImageToBufferInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdCopyMemoryIndirectNV(VkCommandBuffer                             commandBuffer, VkDeviceAddress                             copyBufferAddress, uint32_t                                    copyCount, uint32_t                                    stride) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkDeviceAddress                             copyBufferAddress, uint32_t                                    copyCount, uint32_t                                    stride);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdCopyMemoryIndirectNV"));
    return fn(commandBuffer, copyBufferAddress, copyCount, stride);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdCopyMemoryToAccelerationStructureKHR(VkCommandBuffer                             commandBuffer, const VkCopyMemoryToAccelerationStructureInfoKHR* pInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkCopyMemoryToAccelerationStructureInfoKHR* pInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdCopyMemoryToAccelerationStructureKHR"));
    return fn(commandBuffer, pInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdCopyMemoryToImageIndirectNV(VkCommandBuffer                             commandBuffer, VkDeviceAddress                             copyBufferAddress, uint32_t                                    copyCount, uint32_t                                    stride, VkImage                                     dstImage, VkImageLayout                               dstImageLayout, const VkImageSubresourceLayers*             pImageSubresources) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkDeviceAddress                             copyBufferAddress, uint32_t                                    copyCount, uint32_t                                    stride, VkImage                                     dstImage, VkImageLayout                               dstImageLayout, const VkImageSubresourceLayers*             pImageSubresources);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdCopyMemoryToImageIndirectNV"));
    return fn(commandBuffer, copyBufferAddress, copyCount, stride, dstImage, dstImageLayout, pImageSubresources);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdCopyMemoryToMicromapEXT(VkCommandBuffer                             commandBuffer, const VkCopyMemoryToMicromapInfoEXT*        pInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkCopyMemoryToMicromapInfoEXT*        pInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdCopyMemoryToMicromapEXT"));
    return fn(commandBuffer, pInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdCopyMicromapEXT(VkCommandBuffer                             commandBuffer, const VkCopyMicromapInfoEXT*                pInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkCopyMicromapInfoEXT*                pInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdCopyMicromapEXT"));
    return fn(commandBuffer, pInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdCopyMicromapToMemoryEXT(VkCommandBuffer                             commandBuffer, const VkCopyMicromapToMemoryInfoEXT*        pInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkCopyMicromapToMemoryInfoEXT*        pInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdCopyMicromapToMemoryEXT"));
    return fn(commandBuffer, pInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdCopyQueryPoolResults(VkCommandBuffer                             commandBuffer, VkQueryPool                                 queryPool, uint32_t                                    firstQuery, uint32_t                                    queryCount, VkBuffer                                    dstBuffer, VkDeviceSize                                dstOffset, VkDeviceSize                                stride, VkQueryResultFlags                          flags) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkQueryPool                                 queryPool, uint32_t                                    firstQuery, uint32_t                                    queryCount, VkBuffer                                    dstBuffer, VkDeviceSize                                dstOffset, VkDeviceSize                                stride, VkQueryResultFlags                          flags);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdCopyQueryPoolResults"));
    return fn(commandBuffer, queryPool, firstQuery, queryCount, dstBuffer, dstOffset, stride, flags);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdCuLaunchKernelNVX(VkCommandBuffer                             commandBuffer, const VkCuLaunchInfoNVX*                    pLaunchInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkCuLaunchInfoNVX*                    pLaunchInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdCuLaunchKernelNVX"));
    return fn(commandBuffer, pLaunchInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdCudaLaunchKernelNV(VkCommandBuffer                             commandBuffer, const VkCudaLaunchInfoNV*                   pLaunchInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkCudaLaunchInfoNV*                   pLaunchInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdCudaLaunchKernelNV"));
    return fn(commandBuffer, pLaunchInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdDebugMarkerBeginEXT(VkCommandBuffer                             commandBuffer, const VkDebugMarkerMarkerInfoEXT*           pMarkerInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkDebugMarkerMarkerInfoEXT*           pMarkerInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdDebugMarkerBeginEXT"));
    return fn(commandBuffer, pMarkerInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdDebugMarkerEndEXT(VkCommandBuffer                             commandBuffer) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdDebugMarkerEndEXT"));
    return fn(commandBuffer);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdDebugMarkerInsertEXT(VkCommandBuffer                             commandBuffer, const VkDebugMarkerMarkerInfoEXT*           pMarkerInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkDebugMarkerMarkerInfoEXT*           pMarkerInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdDebugMarkerInsertEXT"));
    return fn(commandBuffer, pMarkerInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdDecodeVideoKHR(VkCommandBuffer                             commandBuffer, const VkVideoDecodeInfoKHR*                 pDecodeInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkVideoDecodeInfoKHR*                 pDecodeInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdDecodeVideoKHR"));
    return fn(commandBuffer, pDecodeInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdDecompressMemoryIndirectCountNV(VkCommandBuffer                             commandBuffer, VkDeviceAddress                             indirectCommandsAddress, VkDeviceAddress                             indirectCommandsCountAddress, uint32_t                                    stride) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkDeviceAddress                             indirectCommandsAddress, VkDeviceAddress                             indirectCommandsCountAddress, uint32_t                                    stride);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdDecompressMemoryIndirectCountNV"));
    return fn(commandBuffer, indirectCommandsAddress, indirectCommandsCountAddress, stride);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdDecompressMemoryNV(VkCommandBuffer                             commandBuffer, uint32_t                                    decompressRegionCount, const VkDecompressMemoryRegionNV*           pDecompressMemoryRegions) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    decompressRegionCount, const VkDecompressMemoryRegionNV*           pDecompressMemoryRegions);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdDecompressMemoryNV"));
    return fn(commandBuffer, decompressRegionCount, pDecompressMemoryRegions);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdDispatch(VkCommandBuffer                             commandBuffer, uint32_t                                    groupCountX, uint32_t                                    groupCountY, uint32_t                                    groupCountZ) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    groupCountX, uint32_t                                    groupCountY, uint32_t                                    groupCountZ);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdDispatch"));
    return fn(commandBuffer, groupCountX, groupCountY, groupCountZ);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdDispatchBase(VkCommandBuffer                             commandBuffer, uint32_t                                    baseGroupX, uint32_t                                    baseGroupY, uint32_t                                    baseGroupZ, uint32_t                                    groupCountX, uint32_t                                    groupCountY, uint32_t                                    groupCountZ) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    baseGroupX, uint32_t                                    baseGroupY, uint32_t                                    baseGroupZ, uint32_t                                    groupCountX, uint32_t                                    groupCountY, uint32_t                                    groupCountZ);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdDispatchBase"));
    return fn(commandBuffer, baseGroupX, baseGroupY, baseGroupZ, groupCountX, groupCountY, groupCountZ);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdDispatchBaseKHR(VkCommandBuffer                             commandBuffer, uint32_t                                    baseGroupX, uint32_t                                    baseGroupY, uint32_t                                    baseGroupZ, uint32_t                                    groupCountX, uint32_t                                    groupCountY, uint32_t                                    groupCountZ) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    baseGroupX, uint32_t                                    baseGroupY, uint32_t                                    baseGroupZ, uint32_t                                    groupCountX, uint32_t                                    groupCountY, uint32_t                                    groupCountZ);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdDispatchBaseKHR"));
    return fn(commandBuffer, baseGroupX, baseGroupY, baseGroupZ, groupCountX, groupCountY, groupCountZ);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdDispatchIndirect(VkCommandBuffer                             commandBuffer, VkBuffer                                    buffer, VkDeviceSize                                offset) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBuffer                                    buffer, VkDeviceSize                                offset);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdDispatchIndirect"));
    return fn(commandBuffer, buffer, offset);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdDraw(VkCommandBuffer                             commandBuffer, uint32_t                                    vertexCount, uint32_t                                    instanceCount, uint32_t                                    firstVertex, uint32_t                                    firstInstance) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    vertexCount, uint32_t                                    instanceCount, uint32_t                                    firstVertex, uint32_t                                    firstInstance);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdDraw"));
    return fn(commandBuffer, vertexCount, instanceCount, firstVertex, firstInstance);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdDrawClusterHUAWEI(VkCommandBuffer                             commandBuffer, uint32_t                                    groupCountX, uint32_t                                    groupCountY, uint32_t                                    groupCountZ) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    groupCountX, uint32_t                                    groupCountY, uint32_t                                    groupCountZ);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdDrawClusterHUAWEI"));
    return fn(commandBuffer, groupCountX, groupCountY, groupCountZ);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdDrawClusterIndirectHUAWEI(VkCommandBuffer                             commandBuffer, VkBuffer                                    buffer, VkDeviceSize                                offset) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBuffer                                    buffer, VkDeviceSize                                offset);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdDrawClusterIndirectHUAWEI"));
    return fn(commandBuffer, buffer, offset);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdDrawIndexed(VkCommandBuffer                             commandBuffer, uint32_t                                    indexCount, uint32_t                                    instanceCount, uint32_t                                    firstIndex, int32_t                                     vertexOffset, uint32_t                                    firstInstance) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    indexCount, uint32_t                                    instanceCount, uint32_t                                    firstIndex, int32_t                                     vertexOffset, uint32_t                                    firstInstance);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdDrawIndexed"));
    return fn(commandBuffer, indexCount, instanceCount, firstIndex, vertexOffset, firstInstance);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdDrawIndexedIndirect(VkCommandBuffer                             commandBuffer, VkBuffer                                    buffer, VkDeviceSize                                offset, uint32_t                                    drawCount, uint32_t                                    stride) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBuffer                                    buffer, VkDeviceSize                                offset, uint32_t                                    drawCount, uint32_t                                    stride);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdDrawIndexedIndirect"));
    return fn(commandBuffer, buffer, offset, drawCount, stride);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdDrawIndexedIndirectCount(VkCommandBuffer                             commandBuffer, VkBuffer                                    buffer, VkDeviceSize                                offset, VkBuffer                                    countBuffer, VkDeviceSize                                countBufferOffset, uint32_t                                    maxDrawCount, uint32_t                                    stride) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBuffer                                    buffer, VkDeviceSize                                offset, VkBuffer                                    countBuffer, VkDeviceSize                                countBufferOffset, uint32_t                                    maxDrawCount, uint32_t                                    stride);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdDrawIndexedIndirectCount"));
    return fn(commandBuffer, buffer, offset, countBuffer, countBufferOffset, maxDrawCount, stride);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdDrawIndexedIndirectCountAMD(VkCommandBuffer                             commandBuffer, VkBuffer                                    buffer, VkDeviceSize                                offset, VkBuffer                                    countBuffer, VkDeviceSize                                countBufferOffset, uint32_t                                    maxDrawCount, uint32_t                                    stride) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBuffer                                    buffer, VkDeviceSize                                offset, VkBuffer                                    countBuffer, VkDeviceSize                                countBufferOffset, uint32_t                                    maxDrawCount, uint32_t                                    stride);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdDrawIndexedIndirectCountAMD"));
    return fn(commandBuffer, buffer, offset, countBuffer, countBufferOffset, maxDrawCount, stride);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdDrawIndexedIndirectCountKHR(VkCommandBuffer                             commandBuffer, VkBuffer                                    buffer, VkDeviceSize                                offset, VkBuffer                                    countBuffer, VkDeviceSize                                countBufferOffset, uint32_t                                    maxDrawCount, uint32_t                                    stride) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBuffer                                    buffer, VkDeviceSize                                offset, VkBuffer                                    countBuffer, VkDeviceSize                                countBufferOffset, uint32_t                                    maxDrawCount, uint32_t                                    stride);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdDrawIndexedIndirectCountKHR"));
    return fn(commandBuffer, buffer, offset, countBuffer, countBufferOffset, maxDrawCount, stride);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdDrawIndirect(VkCommandBuffer                             commandBuffer, VkBuffer                                    buffer, VkDeviceSize                                offset, uint32_t                                    drawCount, uint32_t                                    stride) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBuffer                                    buffer, VkDeviceSize                                offset, uint32_t                                    drawCount, uint32_t                                    stride);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdDrawIndirect"));
    return fn(commandBuffer, buffer, offset, drawCount, stride);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdDrawIndirectByteCountEXT(VkCommandBuffer                             commandBuffer, uint32_t                                    instanceCount, uint32_t                                    firstInstance, VkBuffer                                    counterBuffer, VkDeviceSize                                counterBufferOffset, uint32_t                                    counterOffset, uint32_t                                    vertexStride) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    instanceCount, uint32_t                                    firstInstance, VkBuffer                                    counterBuffer, VkDeviceSize                                counterBufferOffset, uint32_t                                    counterOffset, uint32_t                                    vertexStride);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdDrawIndirectByteCountEXT"));
    return fn(commandBuffer, instanceCount, firstInstance, counterBuffer, counterBufferOffset, counterOffset, vertexStride);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdDrawIndirectCount(VkCommandBuffer                             commandBuffer, VkBuffer                                    buffer, VkDeviceSize                                offset, VkBuffer                                    countBuffer, VkDeviceSize                                countBufferOffset, uint32_t                                    maxDrawCount, uint32_t                                    stride) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBuffer                                    buffer, VkDeviceSize                                offset, VkBuffer                                    countBuffer, VkDeviceSize                                countBufferOffset, uint32_t                                    maxDrawCount, uint32_t                                    stride);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdDrawIndirectCount"));
    return fn(commandBuffer, buffer, offset, countBuffer, countBufferOffset, maxDrawCount, stride);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdDrawIndirectCountAMD(VkCommandBuffer                             commandBuffer, VkBuffer                                    buffer, VkDeviceSize                                offset, VkBuffer                                    countBuffer, VkDeviceSize                                countBufferOffset, uint32_t                                    maxDrawCount, uint32_t                                    stride) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBuffer                                    buffer, VkDeviceSize                                offset, VkBuffer                                    countBuffer, VkDeviceSize                                countBufferOffset, uint32_t                                    maxDrawCount, uint32_t                                    stride);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdDrawIndirectCountAMD"));
    return fn(commandBuffer, buffer, offset, countBuffer, countBufferOffset, maxDrawCount, stride);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdDrawIndirectCountKHR(VkCommandBuffer                             commandBuffer, VkBuffer                                    buffer, VkDeviceSize                                offset, VkBuffer                                    countBuffer, VkDeviceSize                                countBufferOffset, uint32_t                                    maxDrawCount, uint32_t                                    stride) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBuffer                                    buffer, VkDeviceSize                                offset, VkBuffer                                    countBuffer, VkDeviceSize                                countBufferOffset, uint32_t                                    maxDrawCount, uint32_t                                    stride);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdDrawIndirectCountKHR"));
    return fn(commandBuffer, buffer, offset, countBuffer, countBufferOffset, maxDrawCount, stride);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdDrawMeshTasksEXT(VkCommandBuffer                             commandBuffer, uint32_t                                    groupCountX, uint32_t                                    groupCountY, uint32_t                                    groupCountZ) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    groupCountX, uint32_t                                    groupCountY, uint32_t                                    groupCountZ);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdDrawMeshTasksEXT"));
    return fn(commandBuffer, groupCountX, groupCountY, groupCountZ);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdDrawMeshTasksIndirectCountEXT(VkCommandBuffer                             commandBuffer, VkBuffer                                    buffer, VkDeviceSize                                offset, VkBuffer                                    countBuffer, VkDeviceSize                                countBufferOffset, uint32_t                                    maxDrawCount, uint32_t                                    stride) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBuffer                                    buffer, VkDeviceSize                                offset, VkBuffer                                    countBuffer, VkDeviceSize                                countBufferOffset, uint32_t                                    maxDrawCount, uint32_t                                    stride);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdDrawMeshTasksIndirectCountEXT"));
    return fn(commandBuffer, buffer, offset, countBuffer, countBufferOffset, maxDrawCount, stride);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdDrawMeshTasksIndirectCountNV(VkCommandBuffer                             commandBuffer, VkBuffer                                    buffer, VkDeviceSize                                offset, VkBuffer                                    countBuffer, VkDeviceSize                                countBufferOffset, uint32_t                                    maxDrawCount, uint32_t                                    stride) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBuffer                                    buffer, VkDeviceSize                                offset, VkBuffer                                    countBuffer, VkDeviceSize                                countBufferOffset, uint32_t                                    maxDrawCount, uint32_t                                    stride);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdDrawMeshTasksIndirectCountNV"));
    return fn(commandBuffer, buffer, offset, countBuffer, countBufferOffset, maxDrawCount, stride);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdDrawMeshTasksIndirectEXT(VkCommandBuffer                             commandBuffer, VkBuffer                                    buffer, VkDeviceSize                                offset, uint32_t                                    drawCount, uint32_t                                    stride) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBuffer                                    buffer, VkDeviceSize                                offset, uint32_t                                    drawCount, uint32_t                                    stride);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdDrawMeshTasksIndirectEXT"));
    return fn(commandBuffer, buffer, offset, drawCount, stride);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdDrawMeshTasksIndirectNV(VkCommandBuffer                             commandBuffer, VkBuffer                                    buffer, VkDeviceSize                                offset, uint32_t                                    drawCount, uint32_t                                    stride) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBuffer                                    buffer, VkDeviceSize                                offset, uint32_t                                    drawCount, uint32_t                                    stride);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdDrawMeshTasksIndirectNV"));
    return fn(commandBuffer, buffer, offset, drawCount, stride);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdDrawMeshTasksNV(VkCommandBuffer                             commandBuffer, uint32_t                                    taskCount, uint32_t                                    firstTask) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    taskCount, uint32_t                                    firstTask);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdDrawMeshTasksNV"));
    return fn(commandBuffer, taskCount, firstTask);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdDrawMultiEXT(VkCommandBuffer                             commandBuffer, uint32_t                                    drawCount, const VkMultiDrawInfoEXT*                   pVertexInfo, uint32_t                                    instanceCount, uint32_t                                    firstInstance, uint32_t                                    stride) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    drawCount, const VkMultiDrawInfoEXT*                   pVertexInfo, uint32_t                                    instanceCount, uint32_t                                    firstInstance, uint32_t                                    stride);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdDrawMultiEXT"));
    return fn(commandBuffer, drawCount, pVertexInfo, instanceCount, firstInstance, stride);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdDrawMultiIndexedEXT(VkCommandBuffer                             commandBuffer, uint32_t                                    drawCount, const VkMultiDrawIndexedInfoEXT*            pIndexInfo, uint32_t                                    instanceCount, uint32_t                                    firstInstance, uint32_t                                    stride, const int32_t*                              pVertexOffset) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    drawCount, const VkMultiDrawIndexedInfoEXT*            pIndexInfo, uint32_t                                    instanceCount, uint32_t                                    firstInstance, uint32_t                                    stride, const int32_t*                              pVertexOffset);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdDrawMultiIndexedEXT"));
    return fn(commandBuffer, drawCount, pIndexInfo, instanceCount, firstInstance, stride, pVertexOffset);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdEncodeVideoKHR(VkCommandBuffer                             commandBuffer, const VkVideoEncodeInfoKHR*                 pEncodeInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkVideoEncodeInfoKHR*                 pEncodeInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdEncodeVideoKHR"));
    return fn(commandBuffer, pEncodeInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdEndConditionalRenderingEXT(VkCommandBuffer                             commandBuffer) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdEndConditionalRenderingEXT"));
    return fn(commandBuffer);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdEndDebugUtilsLabelEXT(VkCommandBuffer                             commandBuffer) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdEndDebugUtilsLabelEXT"));
    return fn(commandBuffer);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdEndQuery(VkCommandBuffer                             commandBuffer, VkQueryPool                                 queryPool, uint32_t                                    query) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkQueryPool                                 queryPool, uint32_t                                    query);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdEndQuery"));
    return fn(commandBuffer, queryPool, query);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdEndQueryIndexedEXT(VkCommandBuffer                             commandBuffer, VkQueryPool                                 queryPool, uint32_t                                    query, uint32_t                                    index) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkQueryPool                                 queryPool, uint32_t                                    query, uint32_t                                    index);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdEndQueryIndexedEXT"));
    return fn(commandBuffer, queryPool, query, index);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdEndRenderPass(VkCommandBuffer                             commandBuffer) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdEndRenderPass"));
    return fn(commandBuffer);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdEndRenderPass2(VkCommandBuffer                             commandBuffer, const VkSubpassEndInfo*                     pSubpassEndInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkSubpassEndInfo*                     pSubpassEndInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdEndRenderPass2"));
    return fn(commandBuffer, pSubpassEndInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdEndRenderPass2KHR(VkCommandBuffer                             commandBuffer, const VkSubpassEndInfo*                     pSubpassEndInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkSubpassEndInfo*                     pSubpassEndInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdEndRenderPass2KHR"));
    return fn(commandBuffer, pSubpassEndInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdEndRendering(VkCommandBuffer                             commandBuffer) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdEndRendering"));
    return fn(commandBuffer);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdEndRenderingKHR(VkCommandBuffer                             commandBuffer) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdEndRenderingKHR"));
    return fn(commandBuffer);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdEndTransformFeedbackEXT(VkCommandBuffer                             commandBuffer, uint32_t                                    firstCounterBuffer, uint32_t                                    counterBufferCount, const VkBuffer*                             pCounterBuffers, const VkDeviceSize*                         pCounterBufferOffsets) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    firstCounterBuffer, uint32_t                                    counterBufferCount, const VkBuffer*                             pCounterBuffers, const VkDeviceSize*                         pCounterBufferOffsets);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdEndTransformFeedbackEXT"));
    return fn(commandBuffer, firstCounterBuffer, counterBufferCount, pCounterBuffers, pCounterBufferOffsets);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdEndVideoCodingKHR(VkCommandBuffer                             commandBuffer, const VkVideoEndCodingInfoKHR*              pEndCodingInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkVideoEndCodingInfoKHR*              pEndCodingInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdEndVideoCodingKHR"));
    return fn(commandBuffer, pEndCodingInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdExecuteCommands(VkCommandBuffer                             commandBuffer, uint32_t                                    commandBufferCount, const VkCommandBuffer*                      pCommandBuffers) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    commandBufferCount, const VkCommandBuffer*                      pCommandBuffers);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdExecuteCommands"));
    return fn(commandBuffer, commandBufferCount, pCommandBuffers);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdExecuteGeneratedCommandsNV(VkCommandBuffer                             commandBuffer, VkBool32                                    isPreprocessed, const VkGeneratedCommandsInfoNV*            pGeneratedCommandsInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBool32                                    isPreprocessed, const VkGeneratedCommandsInfoNV*            pGeneratedCommandsInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdExecuteGeneratedCommandsNV"));
    return fn(commandBuffer, isPreprocessed, pGeneratedCommandsInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdFillBuffer(VkCommandBuffer                             commandBuffer, VkBuffer                                    dstBuffer, VkDeviceSize                                dstOffset, VkDeviceSize                                size, uint32_t                                    data) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBuffer                                    dstBuffer, VkDeviceSize                                dstOffset, VkDeviceSize                                size, uint32_t                                    data);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdFillBuffer"));
    return fn(commandBuffer, dstBuffer, dstOffset, size, data);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdInsertDebugUtilsLabelEXT(VkCommandBuffer                             commandBuffer, const VkDebugUtilsLabelEXT*                 pLabelInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkDebugUtilsLabelEXT*                 pLabelInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdInsertDebugUtilsLabelEXT"));
    return fn(commandBuffer, pLabelInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdNextSubpass(VkCommandBuffer                             commandBuffer, VkSubpassContents                           contents) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkSubpassContents                           contents);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdNextSubpass"));
    return fn(commandBuffer, contents);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdNextSubpass2(VkCommandBuffer                             commandBuffer, const VkSubpassBeginInfo*                   pSubpassBeginInfo, const VkSubpassEndInfo*                     pSubpassEndInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkSubpassBeginInfo*                   pSubpassBeginInfo, const VkSubpassEndInfo*                     pSubpassEndInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdNextSubpass2"));
    return fn(commandBuffer, pSubpassBeginInfo, pSubpassEndInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdNextSubpass2KHR(VkCommandBuffer                             commandBuffer, const VkSubpassBeginInfo*                   pSubpassBeginInfo, const VkSubpassEndInfo*                     pSubpassEndInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkSubpassBeginInfo*                   pSubpassBeginInfo, const VkSubpassEndInfo*                     pSubpassEndInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdNextSubpass2KHR"));
    return fn(commandBuffer, pSubpassBeginInfo, pSubpassEndInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdOpticalFlowExecuteNV(VkCommandBuffer                             commandBuffer, VkOpticalFlowSessionNV                      session, const VkOpticalFlowExecuteInfoNV*           pExecuteInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkOpticalFlowSessionNV                      session, const VkOpticalFlowExecuteInfoNV*           pExecuteInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdOpticalFlowExecuteNV"));
    return fn(commandBuffer, session, pExecuteInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdPipelineBarrier(VkCommandBuffer                             commandBuffer, VkPipelineStageFlags                        srcStageMask, VkPipelineStageFlags                        dstStageMask, VkDependencyFlags                           dependencyFlags, uint32_t                                    memoryBarrierCount, const VkMemoryBarrier*                      pMemoryBarriers, uint32_t                                    bufferMemoryBarrierCount, const VkBufferMemoryBarrier*                pBufferMemoryBarriers, uint32_t                                    imageMemoryBarrierCount, const VkImageMemoryBarrier*                 pImageMemoryBarriers) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkPipelineStageFlags                        srcStageMask, VkPipelineStageFlags                        dstStageMask, VkDependencyFlags                           dependencyFlags, uint32_t                                    memoryBarrierCount, const VkMemoryBarrier*                      pMemoryBarriers, uint32_t                                    bufferMemoryBarrierCount, const VkBufferMemoryBarrier*                pBufferMemoryBarriers, uint32_t                                    imageMemoryBarrierCount, const VkImageMemoryBarrier*                 pImageMemoryBarriers);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdPipelineBarrier"));
    return fn(commandBuffer, srcStageMask, dstStageMask, dependencyFlags, memoryBarrierCount, pMemoryBarriers, bufferMemoryBarrierCount, pBufferMemoryBarriers, imageMemoryBarrierCount, pImageMemoryBarriers);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdPipelineBarrier2(VkCommandBuffer                             commandBuffer, const VkDependencyInfo*                     pDependencyInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkDependencyInfo*                     pDependencyInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdPipelineBarrier2"));
    return fn(commandBuffer, pDependencyInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdPipelineBarrier2KHR(VkCommandBuffer                             commandBuffer, const VkDependencyInfo*                     pDependencyInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkDependencyInfo*                     pDependencyInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdPipelineBarrier2KHR"));
    return fn(commandBuffer, pDependencyInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdPreprocessGeneratedCommandsNV(VkCommandBuffer                             commandBuffer, const VkGeneratedCommandsInfoNV*            pGeneratedCommandsInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkGeneratedCommandsInfoNV*            pGeneratedCommandsInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdPreprocessGeneratedCommandsNV"));
    return fn(commandBuffer, pGeneratedCommandsInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdPushConstants(VkCommandBuffer                             commandBuffer, VkPipelineLayout                            layout, VkShaderStageFlags                          stageFlags, uint32_t                                    offset, uint32_t                                    size, const void*                                 pValues) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkPipelineLayout                            layout, VkShaderStageFlags                          stageFlags, uint32_t                                    offset, uint32_t                                    size, const void*                                 pValues);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdPushConstants"));
    return fn(commandBuffer, layout, stageFlags, offset, size, pValues);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdPushConstants2KHR(VkCommandBuffer                             commandBuffer, const VkPushConstantsInfoKHR*               pPushConstantsInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkPushConstantsInfoKHR*               pPushConstantsInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdPushConstants2KHR"));
    return fn(commandBuffer, pPushConstantsInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdPushDescriptorSet2KHR(VkCommandBuffer                             commandBuffer, const VkPushDescriptorSetInfoKHR*           pPushDescriptorSetInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkPushDescriptorSetInfoKHR*           pPushDescriptorSetInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdPushDescriptorSet2KHR"));
    return fn(commandBuffer, pPushDescriptorSetInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdPushDescriptorSetKHR(VkCommandBuffer                             commandBuffer, VkPipelineBindPoint                         pipelineBindPoint, VkPipelineLayout                            layout, uint32_t                                    set, uint32_t                                    descriptorWriteCount, const VkWriteDescriptorSet*                 pDescriptorWrites) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkPipelineBindPoint                         pipelineBindPoint, VkPipelineLayout                            layout, uint32_t                                    set, uint32_t                                    descriptorWriteCount, const VkWriteDescriptorSet*                 pDescriptorWrites);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdPushDescriptorSetKHR"));
    return fn(commandBuffer, pipelineBindPoint, layout, set, descriptorWriteCount, pDescriptorWrites);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdPushDescriptorSetWithTemplate2KHR(VkCommandBuffer                             commandBuffer, const VkPushDescriptorSetWithTemplateInfoKHR* pPushDescriptorSetWithTemplateInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkPushDescriptorSetWithTemplateInfoKHR* pPushDescriptorSetWithTemplateInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdPushDescriptorSetWithTemplate2KHR"));
    return fn(commandBuffer, pPushDescriptorSetWithTemplateInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdPushDescriptorSetWithTemplateKHR(VkCommandBuffer                             commandBuffer, VkDescriptorUpdateTemplate                  descriptorUpdateTemplate, VkPipelineLayout                            layout, uint32_t                                    set, const void*                                 pData) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkDescriptorUpdateTemplate                  descriptorUpdateTemplate, VkPipelineLayout                            layout, uint32_t                                    set, const void*                                 pData);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdPushDescriptorSetWithTemplateKHR"));
    return fn(commandBuffer, descriptorUpdateTemplate, layout, set, pData);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdResetEvent(VkCommandBuffer                             commandBuffer, VkEvent                                     event, VkPipelineStageFlags                        stageMask) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkEvent                                     event, VkPipelineStageFlags                        stageMask);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdResetEvent"));
    return fn(commandBuffer, event, stageMask);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdResetEvent2(VkCommandBuffer                             commandBuffer, VkEvent                                     event, VkPipelineStageFlags2                       stageMask) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkEvent                                     event, VkPipelineStageFlags2                       stageMask);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdResetEvent2"));
    return fn(commandBuffer, event, stageMask);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdResetEvent2KHR(VkCommandBuffer                             commandBuffer, VkEvent                                     event, VkPipelineStageFlags2                       stageMask) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkEvent                                     event, VkPipelineStageFlags2                       stageMask);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdResetEvent2KHR"));
    return fn(commandBuffer, event, stageMask);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdResetQueryPool(VkCommandBuffer                             commandBuffer, VkQueryPool                                 queryPool, uint32_t                                    firstQuery, uint32_t                                    queryCount) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkQueryPool                                 queryPool, uint32_t                                    firstQuery, uint32_t                                    queryCount);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdResetQueryPool"));
    return fn(commandBuffer, queryPool, firstQuery, queryCount);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdResolveImage(VkCommandBuffer                             commandBuffer, VkImage                                     srcImage, VkImageLayout                               srcImageLayout, VkImage                                     dstImage, VkImageLayout                               dstImageLayout, uint32_t                                    regionCount, const VkImageResolve*                       pRegions) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkImage                                     srcImage, VkImageLayout                               srcImageLayout, VkImage                                     dstImage, VkImageLayout                               dstImageLayout, uint32_t                                    regionCount, const VkImageResolve*                       pRegions);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdResolveImage"));
    return fn(commandBuffer, srcImage, srcImageLayout, dstImage, dstImageLayout, regionCount, pRegions);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdResolveImage2(VkCommandBuffer                             commandBuffer, const VkResolveImageInfo2*                  pResolveImageInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkResolveImageInfo2*                  pResolveImageInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdResolveImage2"));
    return fn(commandBuffer, pResolveImageInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdResolveImage2KHR(VkCommandBuffer                             commandBuffer, const VkResolveImageInfo2*                  pResolveImageInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkResolveImageInfo2*                  pResolveImageInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdResolveImage2KHR"));
    return fn(commandBuffer, pResolveImageInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetAlphaToCoverageEnableEXT(VkCommandBuffer                             commandBuffer, VkBool32                                    alphaToCoverageEnable) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBool32                                    alphaToCoverageEnable);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetAlphaToCoverageEnableEXT"));
    return fn(commandBuffer, alphaToCoverageEnable);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetAlphaToOneEnableEXT(VkCommandBuffer                             commandBuffer, VkBool32                                    alphaToOneEnable) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBool32                                    alphaToOneEnable);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetAlphaToOneEnableEXT"));
    return fn(commandBuffer, alphaToOneEnable);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetAttachmentFeedbackLoopEnableEXT(VkCommandBuffer                             commandBuffer, VkImageAspectFlags                          aspectMask) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkImageAspectFlags                          aspectMask);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetAttachmentFeedbackLoopEnableEXT"));
    return fn(commandBuffer, aspectMask);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetBlendConstants(VkCommandBuffer                             commandBuffer, const float                                 blendConstants[4]) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const float                                 blendConstants[4]);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetBlendConstants"));
    return fn(commandBuffer, blendConstants);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetCheckpointNV(VkCommandBuffer                             commandBuffer, const void*                                 pCheckpointMarker) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const void*                                 pCheckpointMarker);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetCheckpointNV"));
    return fn(commandBuffer, pCheckpointMarker);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetCoarseSampleOrderNV(VkCommandBuffer                             commandBuffer, VkCoarseSampleOrderTypeNV                   sampleOrderType, uint32_t                                    customSampleOrderCount, const VkCoarseSampleOrderCustomNV*          pCustomSampleOrders) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkCoarseSampleOrderTypeNV                   sampleOrderType, uint32_t                                    customSampleOrderCount, const VkCoarseSampleOrderCustomNV*          pCustomSampleOrders);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetCoarseSampleOrderNV"));
    return fn(commandBuffer, sampleOrderType, customSampleOrderCount, pCustomSampleOrders);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetColorBlendAdvancedEXT(VkCommandBuffer                             commandBuffer, uint32_t                                    firstAttachment, uint32_t                                    attachmentCount, const VkColorBlendAdvancedEXT*              pColorBlendAdvanced) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    firstAttachment, uint32_t                                    attachmentCount, const VkColorBlendAdvancedEXT*              pColorBlendAdvanced);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetColorBlendAdvancedEXT"));
    return fn(commandBuffer, firstAttachment, attachmentCount, pColorBlendAdvanced);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetColorBlendEnableEXT(VkCommandBuffer                             commandBuffer, uint32_t                                    firstAttachment, uint32_t                                    attachmentCount, const VkBool32*                             pColorBlendEnables) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    firstAttachment, uint32_t                                    attachmentCount, const VkBool32*                             pColorBlendEnables);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetColorBlendEnableEXT"));
    return fn(commandBuffer, firstAttachment, attachmentCount, pColorBlendEnables);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetColorBlendEquationEXT(VkCommandBuffer                             commandBuffer, uint32_t                                    firstAttachment, uint32_t                                    attachmentCount, const VkColorBlendEquationEXT*              pColorBlendEquations) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    firstAttachment, uint32_t                                    attachmentCount, const VkColorBlendEquationEXT*              pColorBlendEquations);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetColorBlendEquationEXT"));
    return fn(commandBuffer, firstAttachment, attachmentCount, pColorBlendEquations);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetColorWriteEnableEXT(VkCommandBuffer                             commandBuffer, uint32_t                                    attachmentCount, const VkBool32*                             pColorWriteEnables) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    attachmentCount, const VkBool32*                             pColorWriteEnables);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetColorWriteEnableEXT"));
    return fn(commandBuffer, attachmentCount, pColorWriteEnables);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetColorWriteMaskEXT(VkCommandBuffer                             commandBuffer, uint32_t                                    firstAttachment, uint32_t                                    attachmentCount, const VkColorComponentFlags*                pColorWriteMasks) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    firstAttachment, uint32_t                                    attachmentCount, const VkColorComponentFlags*                pColorWriteMasks);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetColorWriteMaskEXT"));
    return fn(commandBuffer, firstAttachment, attachmentCount, pColorWriteMasks);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetConservativeRasterizationModeEXT(VkCommandBuffer                             commandBuffer, VkConservativeRasterizationModeEXT          conservativeRasterizationMode) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkConservativeRasterizationModeEXT          conservativeRasterizationMode);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetConservativeRasterizationModeEXT"));
    return fn(commandBuffer, conservativeRasterizationMode);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetCoverageModulationModeNV(VkCommandBuffer                             commandBuffer, VkCoverageModulationModeNV                  coverageModulationMode) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkCoverageModulationModeNV                  coverageModulationMode);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetCoverageModulationModeNV"));
    return fn(commandBuffer, coverageModulationMode);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetCoverageModulationTableEnableNV(VkCommandBuffer                             commandBuffer, VkBool32                                    coverageModulationTableEnable) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBool32                                    coverageModulationTableEnable);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetCoverageModulationTableEnableNV"));
    return fn(commandBuffer, coverageModulationTableEnable);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetCoverageModulationTableNV(VkCommandBuffer                             commandBuffer, uint32_t                                    coverageModulationTableCount, const float*                                pCoverageModulationTable) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    coverageModulationTableCount, const float*                                pCoverageModulationTable);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetCoverageModulationTableNV"));
    return fn(commandBuffer, coverageModulationTableCount, pCoverageModulationTable);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetCoverageReductionModeNV(VkCommandBuffer                             commandBuffer, VkCoverageReductionModeNV                   coverageReductionMode) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkCoverageReductionModeNV                   coverageReductionMode);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetCoverageReductionModeNV"));
    return fn(commandBuffer, coverageReductionMode);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetCoverageToColorEnableNV(VkCommandBuffer                             commandBuffer, VkBool32                                    coverageToColorEnable) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBool32                                    coverageToColorEnable);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetCoverageToColorEnableNV"));
    return fn(commandBuffer, coverageToColorEnable);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetCoverageToColorLocationNV(VkCommandBuffer                             commandBuffer, uint32_t                                    coverageToColorLocation) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    coverageToColorLocation);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetCoverageToColorLocationNV"));
    return fn(commandBuffer, coverageToColorLocation);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetCullMode(VkCommandBuffer                             commandBuffer, VkCullModeFlags                             cullMode) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkCullModeFlags                             cullMode);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetCullMode"));
    return fn(commandBuffer, cullMode);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetCullModeEXT(VkCommandBuffer                             commandBuffer, VkCullModeFlags                             cullMode) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkCullModeFlags                             cullMode);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetCullModeEXT"));
    return fn(commandBuffer, cullMode);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetDepthBias(VkCommandBuffer                             commandBuffer, float                                       depthBiasConstantFactor, float                                       depthBiasClamp, float                                       depthBiasSlopeFactor) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, float                                       depthBiasConstantFactor, float                                       depthBiasClamp, float                                       depthBiasSlopeFactor);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetDepthBias"));
    return fn(commandBuffer, depthBiasConstantFactor, depthBiasClamp, depthBiasSlopeFactor);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetDepthBias2EXT(VkCommandBuffer                             commandBuffer, const VkDepthBiasInfoEXT*                   pDepthBiasInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkDepthBiasInfoEXT*                   pDepthBiasInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetDepthBias2EXT"));
    return fn(commandBuffer, pDepthBiasInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetDepthBiasEnable(VkCommandBuffer                             commandBuffer, VkBool32                                    depthBiasEnable) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBool32                                    depthBiasEnable);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetDepthBiasEnable"));
    return fn(commandBuffer, depthBiasEnable);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetDepthBiasEnableEXT(VkCommandBuffer                             commandBuffer, VkBool32                                    depthBiasEnable) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBool32                                    depthBiasEnable);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetDepthBiasEnableEXT"));
    return fn(commandBuffer, depthBiasEnable);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetDepthBounds(VkCommandBuffer                             commandBuffer, float                                       minDepthBounds, float                                       maxDepthBounds) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, float                                       minDepthBounds, float                                       maxDepthBounds);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetDepthBounds"));
    return fn(commandBuffer, minDepthBounds, maxDepthBounds);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetDepthBoundsTestEnable(VkCommandBuffer                             commandBuffer, VkBool32                                    depthBoundsTestEnable) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBool32                                    depthBoundsTestEnable);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetDepthBoundsTestEnable"));
    return fn(commandBuffer, depthBoundsTestEnable);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetDepthBoundsTestEnableEXT(VkCommandBuffer                             commandBuffer, VkBool32                                    depthBoundsTestEnable) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBool32                                    depthBoundsTestEnable);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetDepthBoundsTestEnableEXT"));
    return fn(commandBuffer, depthBoundsTestEnable);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetDepthClampEnableEXT(VkCommandBuffer                             commandBuffer, VkBool32                                    depthClampEnable) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBool32                                    depthClampEnable);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetDepthClampEnableEXT"));
    return fn(commandBuffer, depthClampEnable);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetDepthClipEnableEXT(VkCommandBuffer                             commandBuffer, VkBool32                                    depthClipEnable) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBool32                                    depthClipEnable);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetDepthClipEnableEXT"));
    return fn(commandBuffer, depthClipEnable);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetDepthClipNegativeOneToOneEXT(VkCommandBuffer                             commandBuffer, VkBool32                                    negativeOneToOne) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBool32                                    negativeOneToOne);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetDepthClipNegativeOneToOneEXT"));
    return fn(commandBuffer, negativeOneToOne);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetDepthCompareOp(VkCommandBuffer                             commandBuffer, VkCompareOp                                 depthCompareOp) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkCompareOp                                 depthCompareOp);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetDepthCompareOp"));
    return fn(commandBuffer, depthCompareOp);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetDepthCompareOpEXT(VkCommandBuffer                             commandBuffer, VkCompareOp                                 depthCompareOp) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkCompareOp                                 depthCompareOp);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetDepthCompareOpEXT"));
    return fn(commandBuffer, depthCompareOp);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetDepthTestEnable(VkCommandBuffer                             commandBuffer, VkBool32                                    depthTestEnable) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBool32                                    depthTestEnable);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetDepthTestEnable"));
    return fn(commandBuffer, depthTestEnable);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetDepthTestEnableEXT(VkCommandBuffer                             commandBuffer, VkBool32                                    depthTestEnable) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBool32                                    depthTestEnable);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetDepthTestEnableEXT"));
    return fn(commandBuffer, depthTestEnable);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetDepthWriteEnable(VkCommandBuffer                             commandBuffer, VkBool32                                    depthWriteEnable) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBool32                                    depthWriteEnable);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetDepthWriteEnable"));
    return fn(commandBuffer, depthWriteEnable);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetDepthWriteEnableEXT(VkCommandBuffer                             commandBuffer, VkBool32                                    depthWriteEnable) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBool32                                    depthWriteEnable);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetDepthWriteEnableEXT"));
    return fn(commandBuffer, depthWriteEnable);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetDescriptorBufferOffsets2EXT(VkCommandBuffer                             commandBuffer, const VkSetDescriptorBufferOffsetsInfoEXT*  pSetDescriptorBufferOffsetsInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkSetDescriptorBufferOffsetsInfoEXT*  pSetDescriptorBufferOffsetsInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetDescriptorBufferOffsets2EXT"));
    return fn(commandBuffer, pSetDescriptorBufferOffsetsInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetDescriptorBufferOffsetsEXT(VkCommandBuffer                             commandBuffer, VkPipelineBindPoint                         pipelineBindPoint, VkPipelineLayout                            layout, uint32_t                                    firstSet, uint32_t                                    setCount, const uint32_t*                             pBufferIndices, const VkDeviceSize*                         pOffsets) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkPipelineBindPoint                         pipelineBindPoint, VkPipelineLayout                            layout, uint32_t                                    firstSet, uint32_t                                    setCount, const uint32_t*                             pBufferIndices, const VkDeviceSize*                         pOffsets);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetDescriptorBufferOffsetsEXT"));
    return fn(commandBuffer, pipelineBindPoint, layout, firstSet, setCount, pBufferIndices, pOffsets);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetDeviceMask(VkCommandBuffer                             commandBuffer, uint32_t                                    deviceMask) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    deviceMask);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetDeviceMask"));
    return fn(commandBuffer, deviceMask);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetDeviceMaskKHR(VkCommandBuffer                             commandBuffer, uint32_t                                    deviceMask) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    deviceMask);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetDeviceMaskKHR"));
    return fn(commandBuffer, deviceMask);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetDiscardRectangleEXT(VkCommandBuffer                             commandBuffer, uint32_t                                    firstDiscardRectangle, uint32_t                                    discardRectangleCount, const VkRect2D*                             pDiscardRectangles) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    firstDiscardRectangle, uint32_t                                    discardRectangleCount, const VkRect2D*                             pDiscardRectangles);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetDiscardRectangleEXT"));
    return fn(commandBuffer, firstDiscardRectangle, discardRectangleCount, pDiscardRectangles);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetDiscardRectangleEnableEXT(VkCommandBuffer                             commandBuffer, VkBool32                                    discardRectangleEnable) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBool32                                    discardRectangleEnable);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetDiscardRectangleEnableEXT"));
    return fn(commandBuffer, discardRectangleEnable);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetDiscardRectangleModeEXT(VkCommandBuffer                             commandBuffer, VkDiscardRectangleModeEXT                   discardRectangleMode) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkDiscardRectangleModeEXT                   discardRectangleMode);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetDiscardRectangleModeEXT"));
    return fn(commandBuffer, discardRectangleMode);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetEvent(VkCommandBuffer                             commandBuffer, VkEvent                                     event, VkPipelineStageFlags                        stageMask) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkEvent                                     event, VkPipelineStageFlags                        stageMask);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetEvent"));
    return fn(commandBuffer, event, stageMask);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetEvent2(VkCommandBuffer                             commandBuffer, VkEvent                                     event, const VkDependencyInfo*                     pDependencyInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkEvent                                     event, const VkDependencyInfo*                     pDependencyInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetEvent2"));
    return fn(commandBuffer, event, pDependencyInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetEvent2KHR(VkCommandBuffer                             commandBuffer, VkEvent                                     event, const VkDependencyInfo*                     pDependencyInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkEvent                                     event, const VkDependencyInfo*                     pDependencyInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetEvent2KHR"));
    return fn(commandBuffer, event, pDependencyInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetExclusiveScissorEnableNV(VkCommandBuffer                             commandBuffer, uint32_t                                    firstExclusiveScissor, uint32_t                                    exclusiveScissorCount, const VkBool32*                             pExclusiveScissorEnables) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    firstExclusiveScissor, uint32_t                                    exclusiveScissorCount, const VkBool32*                             pExclusiveScissorEnables);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetExclusiveScissorEnableNV"));
    return fn(commandBuffer, firstExclusiveScissor, exclusiveScissorCount, pExclusiveScissorEnables);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetExclusiveScissorNV(VkCommandBuffer                             commandBuffer, uint32_t                                    firstExclusiveScissor, uint32_t                                    exclusiveScissorCount, const VkRect2D*                             pExclusiveScissors) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    firstExclusiveScissor, uint32_t                                    exclusiveScissorCount, const VkRect2D*                             pExclusiveScissors);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetExclusiveScissorNV"));
    return fn(commandBuffer, firstExclusiveScissor, exclusiveScissorCount, pExclusiveScissors);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetExtraPrimitiveOverestimationSizeEXT(VkCommandBuffer                             commandBuffer, float                                       extraPrimitiveOverestimationSize) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, float                                       extraPrimitiveOverestimationSize);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetExtraPrimitiveOverestimationSizeEXT"));
    return fn(commandBuffer, extraPrimitiveOverestimationSize);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetFragmentShadingRateEnumNV(VkCommandBuffer                             commandBuffer, VkFragmentShadingRateNV                     shadingRate, const VkFragmentShadingRateCombinerOpKHR    combinerOps[2]) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkFragmentShadingRateNV                     shadingRate, const VkFragmentShadingRateCombinerOpKHR    combinerOps[2]);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetFragmentShadingRateEnumNV"));
    return fn(commandBuffer, shadingRate, combinerOps);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetFragmentShadingRateKHR(VkCommandBuffer                             commandBuffer, const VkExtent2D*                           pFragmentSize, const VkFragmentShadingRateCombinerOpKHR    combinerOps[2]) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkExtent2D*                           pFragmentSize, const VkFragmentShadingRateCombinerOpKHR    combinerOps[2]);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetFragmentShadingRateKHR"));
    return fn(commandBuffer, pFragmentSize, combinerOps);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetFrontFace(VkCommandBuffer                             commandBuffer, VkFrontFace                                 frontFace) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkFrontFace                                 frontFace);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetFrontFace"));
    return fn(commandBuffer, frontFace);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetFrontFaceEXT(VkCommandBuffer                             commandBuffer, VkFrontFace                                 frontFace) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkFrontFace                                 frontFace);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetFrontFaceEXT"));
    return fn(commandBuffer, frontFace);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetLineRasterizationModeEXT(VkCommandBuffer                             commandBuffer, VkLineRasterizationModeEXT                  lineRasterizationMode) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkLineRasterizationModeEXT                  lineRasterizationMode);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetLineRasterizationModeEXT"));
    return fn(commandBuffer, lineRasterizationMode);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetLineStippleEXT(VkCommandBuffer                             commandBuffer, uint32_t                                    lineStippleFactor, uint16_t                                    lineStipplePattern) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    lineStippleFactor, uint16_t                                    lineStipplePattern);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetLineStippleEXT"));
    return fn(commandBuffer, lineStippleFactor, lineStipplePattern);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetLineStippleEnableEXT(VkCommandBuffer                             commandBuffer, VkBool32                                    stippledLineEnable) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBool32                                    stippledLineEnable);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetLineStippleEnableEXT"));
    return fn(commandBuffer, stippledLineEnable);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetLineWidth(VkCommandBuffer                             commandBuffer, float                                       lineWidth) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, float                                       lineWidth);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetLineWidth"));
    return fn(commandBuffer, lineWidth);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetLogicOpEXT(VkCommandBuffer                             commandBuffer, VkLogicOp                                   logicOp) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkLogicOp                                   logicOp);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetLogicOpEXT"));
    return fn(commandBuffer, logicOp);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetLogicOpEnableEXT(VkCommandBuffer                             commandBuffer, VkBool32                                    logicOpEnable) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBool32                                    logicOpEnable);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetLogicOpEnableEXT"));
    return fn(commandBuffer, logicOpEnable);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetPatchControlPointsEXT(VkCommandBuffer                             commandBuffer, uint32_t                                    patchControlPoints) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    patchControlPoints);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetPatchControlPointsEXT"));
    return fn(commandBuffer, patchControlPoints);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCmdSetPerformanceMarkerINTEL(VkCommandBuffer                             commandBuffer, const VkPerformanceMarkerInfoINTEL*         pMarkerInfo) {
    using Fn = VkResult (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkPerformanceMarkerInfoINTEL*         pMarkerInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetPerformanceMarkerINTEL"));
    return fn(commandBuffer, pMarkerInfo);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCmdSetPerformanceOverrideINTEL(VkCommandBuffer                             commandBuffer, const VkPerformanceOverrideInfoINTEL*       pOverrideInfo) {
    using Fn = VkResult (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkPerformanceOverrideInfoINTEL*       pOverrideInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetPerformanceOverrideINTEL"));
    return fn(commandBuffer, pOverrideInfo);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCmdSetPerformanceStreamMarkerINTEL(VkCommandBuffer                             commandBuffer, const VkPerformanceStreamMarkerInfoINTEL*   pMarkerInfo) {
    using Fn = VkResult (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkPerformanceStreamMarkerInfoINTEL*   pMarkerInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetPerformanceStreamMarkerINTEL"));
    return fn(commandBuffer, pMarkerInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetPolygonModeEXT(VkCommandBuffer                             commandBuffer, VkPolygonMode                               polygonMode) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkPolygonMode                               polygonMode);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetPolygonModeEXT"));
    return fn(commandBuffer, polygonMode);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetPrimitiveRestartEnable(VkCommandBuffer                             commandBuffer, VkBool32                                    primitiveRestartEnable) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBool32                                    primitiveRestartEnable);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetPrimitiveRestartEnable"));
    return fn(commandBuffer, primitiveRestartEnable);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetPrimitiveRestartEnableEXT(VkCommandBuffer                             commandBuffer, VkBool32                                    primitiveRestartEnable) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBool32                                    primitiveRestartEnable);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetPrimitiveRestartEnableEXT"));
    return fn(commandBuffer, primitiveRestartEnable);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetPrimitiveTopology(VkCommandBuffer                             commandBuffer, VkPrimitiveTopology                         primitiveTopology) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkPrimitiveTopology                         primitiveTopology);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetPrimitiveTopology"));
    return fn(commandBuffer, primitiveTopology);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetPrimitiveTopologyEXT(VkCommandBuffer                             commandBuffer, VkPrimitiveTopology                         primitiveTopology) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkPrimitiveTopology                         primitiveTopology);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetPrimitiveTopologyEXT"));
    return fn(commandBuffer, primitiveTopology);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetProvokingVertexModeEXT(VkCommandBuffer                             commandBuffer, VkProvokingVertexModeEXT                    provokingVertexMode) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkProvokingVertexModeEXT                    provokingVertexMode);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetProvokingVertexModeEXT"));
    return fn(commandBuffer, provokingVertexMode);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetRasterizationSamplesEXT(VkCommandBuffer                             commandBuffer, VkSampleCountFlagBits                       rasterizationSamples) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkSampleCountFlagBits                       rasterizationSamples);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetRasterizationSamplesEXT"));
    return fn(commandBuffer, rasterizationSamples);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetRasterizationStreamEXT(VkCommandBuffer                             commandBuffer, uint32_t                                    rasterizationStream) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    rasterizationStream);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetRasterizationStreamEXT"));
    return fn(commandBuffer, rasterizationStream);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetRasterizerDiscardEnable(VkCommandBuffer                             commandBuffer, VkBool32                                    rasterizerDiscardEnable) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBool32                                    rasterizerDiscardEnable);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetRasterizerDiscardEnable"));
    return fn(commandBuffer, rasterizerDiscardEnable);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetRasterizerDiscardEnableEXT(VkCommandBuffer                             commandBuffer, VkBool32                                    rasterizerDiscardEnable) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBool32                                    rasterizerDiscardEnable);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetRasterizerDiscardEnableEXT"));
    return fn(commandBuffer, rasterizerDiscardEnable);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetRayTracingPipelineStackSizeKHR(VkCommandBuffer                             commandBuffer, uint32_t                                    pipelineStackSize) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    pipelineStackSize);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetRayTracingPipelineStackSizeKHR"));
    return fn(commandBuffer, pipelineStackSize);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetRepresentativeFragmentTestEnableNV(VkCommandBuffer                             commandBuffer, VkBool32                                    representativeFragmentTestEnable) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBool32                                    representativeFragmentTestEnable);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetRepresentativeFragmentTestEnableNV"));
    return fn(commandBuffer, representativeFragmentTestEnable);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetSampleLocationsEXT(VkCommandBuffer                             commandBuffer, const VkSampleLocationsInfoEXT*             pSampleLocationsInfo) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkSampleLocationsInfoEXT*             pSampleLocationsInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetSampleLocationsEXT"));
    return fn(commandBuffer, pSampleLocationsInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetSampleLocationsEnableEXT(VkCommandBuffer                             commandBuffer, VkBool32                                    sampleLocationsEnable) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBool32                                    sampleLocationsEnable);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetSampleLocationsEnableEXT"));
    return fn(commandBuffer, sampleLocationsEnable);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetSampleMaskEXT(VkCommandBuffer                             commandBuffer, VkSampleCountFlagBits                       samples, const VkSampleMask*                         pSampleMask) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkSampleCountFlagBits                       samples, const VkSampleMask*                         pSampleMask);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetSampleMaskEXT"));
    return fn(commandBuffer, samples, pSampleMask);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetScissor(VkCommandBuffer                             commandBuffer, uint32_t                                    firstScissor, uint32_t                                    scissorCount, const VkRect2D*                             pScissors) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    firstScissor, uint32_t                                    scissorCount, const VkRect2D*                             pScissors);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetScissor"));
    return fn(commandBuffer, firstScissor, scissorCount, pScissors);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetScissorWithCount(VkCommandBuffer                             commandBuffer, uint32_t                                    scissorCount, const VkRect2D*                             pScissors) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    scissorCount, const VkRect2D*                             pScissors);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetScissorWithCount"));
    return fn(commandBuffer, scissorCount, pScissors);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetScissorWithCountEXT(VkCommandBuffer                             commandBuffer, uint32_t                                    scissorCount, const VkRect2D*                             pScissors) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    scissorCount, const VkRect2D*                             pScissors);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetScissorWithCountEXT"));
    return fn(commandBuffer, scissorCount, pScissors);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetShadingRateImageEnableNV(VkCommandBuffer                             commandBuffer, VkBool32                                    shadingRateImageEnable) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBool32                                    shadingRateImageEnable);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetShadingRateImageEnableNV"));
    return fn(commandBuffer, shadingRateImageEnable);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetStencilCompareMask(VkCommandBuffer                             commandBuffer, VkStencilFaceFlags                          faceMask, uint32_t                                    compareMask) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkStencilFaceFlags                          faceMask, uint32_t                                    compareMask);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetStencilCompareMask"));
    return fn(commandBuffer, faceMask, compareMask);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetStencilOp(VkCommandBuffer                             commandBuffer, VkStencilFaceFlags                          faceMask, VkStencilOp                                 failOp, VkStencilOp                                 passOp, VkStencilOp                                 depthFailOp, VkCompareOp                                 compareOp) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkStencilFaceFlags                          faceMask, VkStencilOp                                 failOp, VkStencilOp                                 passOp, VkStencilOp                                 depthFailOp, VkCompareOp                                 compareOp);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetStencilOp"));
    return fn(commandBuffer, faceMask, failOp, passOp, depthFailOp, compareOp);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetStencilOpEXT(VkCommandBuffer                             commandBuffer, VkStencilFaceFlags                          faceMask, VkStencilOp                                 failOp, VkStencilOp                                 passOp, VkStencilOp                                 depthFailOp, VkCompareOp                                 compareOp) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkStencilFaceFlags                          faceMask, VkStencilOp                                 failOp, VkStencilOp                                 passOp, VkStencilOp                                 depthFailOp, VkCompareOp                                 compareOp);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetStencilOpEXT"));
    return fn(commandBuffer, faceMask, failOp, passOp, depthFailOp, compareOp);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetStencilReference(VkCommandBuffer                             commandBuffer, VkStencilFaceFlags                          faceMask, uint32_t                                    reference) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkStencilFaceFlags                          faceMask, uint32_t                                    reference);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetStencilReference"));
    return fn(commandBuffer, faceMask, reference);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetStencilTestEnable(VkCommandBuffer                             commandBuffer, VkBool32                                    stencilTestEnable) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBool32                                    stencilTestEnable);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetStencilTestEnable"));
    return fn(commandBuffer, stencilTestEnable);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetStencilTestEnableEXT(VkCommandBuffer                             commandBuffer, VkBool32                                    stencilTestEnable) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBool32                                    stencilTestEnable);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetStencilTestEnableEXT"));
    return fn(commandBuffer, stencilTestEnable);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetStencilWriteMask(VkCommandBuffer                             commandBuffer, VkStencilFaceFlags                          faceMask, uint32_t                                    writeMask) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkStencilFaceFlags                          faceMask, uint32_t                                    writeMask);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetStencilWriteMask"));
    return fn(commandBuffer, faceMask, writeMask);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetTessellationDomainOriginEXT(VkCommandBuffer                             commandBuffer, VkTessellationDomainOrigin                  domainOrigin) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkTessellationDomainOrigin                  domainOrigin);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetTessellationDomainOriginEXT"));
    return fn(commandBuffer, domainOrigin);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetVertexInputEXT(VkCommandBuffer                             commandBuffer, uint32_t                                    vertexBindingDescriptionCount, const VkVertexInputBindingDescription2EXT*  pVertexBindingDescriptions, uint32_t                                    vertexAttributeDescriptionCount, const VkVertexInputAttributeDescription2EXT* pVertexAttributeDescriptions) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    vertexBindingDescriptionCount, const VkVertexInputBindingDescription2EXT*  pVertexBindingDescriptions, uint32_t                                    vertexAttributeDescriptionCount, const VkVertexInputAttributeDescription2EXT* pVertexAttributeDescriptions);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetVertexInputEXT"));
    return fn(commandBuffer, vertexBindingDescriptionCount, pVertexBindingDescriptions, vertexAttributeDescriptionCount, pVertexAttributeDescriptions);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetViewport(VkCommandBuffer                             commandBuffer, uint32_t                                    firstViewport, uint32_t                                    viewportCount, const VkViewport*                           pViewports) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    firstViewport, uint32_t                                    viewportCount, const VkViewport*                           pViewports);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetViewport"));
    return fn(commandBuffer, firstViewport, viewportCount, pViewports);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetViewportShadingRatePaletteNV(VkCommandBuffer                             commandBuffer, uint32_t                                    firstViewport, uint32_t                                    viewportCount, const VkShadingRatePaletteNV*               pShadingRatePalettes) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    firstViewport, uint32_t                                    viewportCount, const VkShadingRatePaletteNV*               pShadingRatePalettes);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetViewportShadingRatePaletteNV"));
    return fn(commandBuffer, firstViewport, viewportCount, pShadingRatePalettes);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetViewportSwizzleNV(VkCommandBuffer                             commandBuffer, uint32_t                                    firstViewport, uint32_t                                    viewportCount, const VkViewportSwizzleNV*                  pViewportSwizzles) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    firstViewport, uint32_t                                    viewportCount, const VkViewportSwizzleNV*                  pViewportSwizzles);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetViewportSwizzleNV"));
    return fn(commandBuffer, firstViewport, viewportCount, pViewportSwizzles);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetViewportWScalingEnableNV(VkCommandBuffer                             commandBuffer, VkBool32                                    viewportWScalingEnable) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBool32                                    viewportWScalingEnable);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetViewportWScalingEnableNV"));
    return fn(commandBuffer, viewportWScalingEnable);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetViewportWScalingNV(VkCommandBuffer                             commandBuffer, uint32_t                                    firstViewport, uint32_t                                    viewportCount, const VkViewportWScalingNV*                 pViewportWScalings) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    firstViewport, uint32_t                                    viewportCount, const VkViewportWScalingNV*                 pViewportWScalings);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetViewportWScalingNV"));
    return fn(commandBuffer, firstViewport, viewportCount, pViewportWScalings);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetViewportWithCount(VkCommandBuffer                             commandBuffer, uint32_t                                    viewportCount, const VkViewport*                           pViewports) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    viewportCount, const VkViewport*                           pViewports);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetViewportWithCount"));
    return fn(commandBuffer, viewportCount, pViewports);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSetViewportWithCountEXT(VkCommandBuffer                             commandBuffer, uint32_t                                    viewportCount, const VkViewport*                           pViewports) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    viewportCount, const VkViewport*                           pViewports);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSetViewportWithCountEXT"));
    return fn(commandBuffer, viewportCount, pViewports);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdSubpassShadingHUAWEI(VkCommandBuffer                             commandBuffer) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdSubpassShadingHUAWEI"));
    return fn(commandBuffer);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdTraceRaysIndirect2KHR(VkCommandBuffer                             commandBuffer, VkDeviceAddress                             indirectDeviceAddress) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkDeviceAddress                             indirectDeviceAddress);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdTraceRaysIndirect2KHR"));
    return fn(commandBuffer, indirectDeviceAddress);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdTraceRaysIndirectKHR(VkCommandBuffer                             commandBuffer, const VkStridedDeviceAddressRegionKHR*      pRaygenShaderBindingTable, const VkStridedDeviceAddressRegionKHR*      pMissShaderBindingTable, const VkStridedDeviceAddressRegionKHR*      pHitShaderBindingTable, const VkStridedDeviceAddressRegionKHR*      pCallableShaderBindingTable, VkDeviceAddress                             indirectDeviceAddress) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkStridedDeviceAddressRegionKHR*      pRaygenShaderBindingTable, const VkStridedDeviceAddressRegionKHR*      pMissShaderBindingTable, const VkStridedDeviceAddressRegionKHR*      pHitShaderBindingTable, const VkStridedDeviceAddressRegionKHR*      pCallableShaderBindingTable, VkDeviceAddress                             indirectDeviceAddress);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdTraceRaysIndirectKHR"));
    return fn(commandBuffer, pRaygenShaderBindingTable, pMissShaderBindingTable, pHitShaderBindingTable, pCallableShaderBindingTable, indirectDeviceAddress);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdTraceRaysKHR(VkCommandBuffer                             commandBuffer, const VkStridedDeviceAddressRegionKHR*      pRaygenShaderBindingTable, const VkStridedDeviceAddressRegionKHR*      pMissShaderBindingTable, const VkStridedDeviceAddressRegionKHR*      pHitShaderBindingTable, const VkStridedDeviceAddressRegionKHR*      pCallableShaderBindingTable, uint32_t                                    width, uint32_t                                    height, uint32_t                                    depth) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, const VkStridedDeviceAddressRegionKHR*      pRaygenShaderBindingTable, const VkStridedDeviceAddressRegionKHR*      pMissShaderBindingTable, const VkStridedDeviceAddressRegionKHR*      pHitShaderBindingTable, const VkStridedDeviceAddressRegionKHR*      pCallableShaderBindingTable, uint32_t                                    width, uint32_t                                    height, uint32_t                                    depth);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdTraceRaysKHR"));
    return fn(commandBuffer, pRaygenShaderBindingTable, pMissShaderBindingTable, pHitShaderBindingTable, pCallableShaderBindingTable, width, height, depth);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdTraceRaysNV(VkCommandBuffer                             commandBuffer, VkBuffer                                    raygenShaderBindingTableBuffer, VkDeviceSize                                raygenShaderBindingOffset, VkBuffer                                    missShaderBindingTableBuffer, VkDeviceSize                                missShaderBindingOffset, VkDeviceSize                                missShaderBindingStride, VkBuffer                                    hitShaderBindingTableBuffer, VkDeviceSize                                hitShaderBindingOffset, VkDeviceSize                                hitShaderBindingStride, VkBuffer                                    callableShaderBindingTableBuffer, VkDeviceSize                                callableShaderBindingOffset, VkDeviceSize                                callableShaderBindingStride, uint32_t                                    width, uint32_t                                    height, uint32_t                                    depth) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBuffer                                    raygenShaderBindingTableBuffer, VkDeviceSize                                raygenShaderBindingOffset, VkBuffer                                    missShaderBindingTableBuffer, VkDeviceSize                                missShaderBindingOffset, VkDeviceSize                                missShaderBindingStride, VkBuffer                                    hitShaderBindingTableBuffer, VkDeviceSize                                hitShaderBindingOffset, VkDeviceSize                                hitShaderBindingStride, VkBuffer                                    callableShaderBindingTableBuffer, VkDeviceSize                                callableShaderBindingOffset, VkDeviceSize                                callableShaderBindingStride, uint32_t                                    width, uint32_t                                    height, uint32_t                                    depth);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdTraceRaysNV"));
    return fn(commandBuffer, raygenShaderBindingTableBuffer, raygenShaderBindingOffset, missShaderBindingTableBuffer, missShaderBindingOffset, missShaderBindingStride, hitShaderBindingTableBuffer, hitShaderBindingOffset, hitShaderBindingStride, callableShaderBindingTableBuffer, callableShaderBindingOffset, callableShaderBindingStride, width, height, depth);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdUpdateBuffer(VkCommandBuffer                             commandBuffer, VkBuffer                                    dstBuffer, VkDeviceSize                                dstOffset, VkDeviceSize                                dataSize, const void*                                 pData) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkBuffer                                    dstBuffer, VkDeviceSize                                dstOffset, VkDeviceSize                                dataSize, const void*                                 pData);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdUpdateBuffer"));
    return fn(commandBuffer, dstBuffer, dstOffset, dataSize, pData);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdUpdatePipelineIndirectBufferNV(VkCommandBuffer                             commandBuffer, VkPipelineBindPoint                         pipelineBindPoint, VkPipeline                                  pipeline) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkPipelineBindPoint                         pipelineBindPoint, VkPipeline                                  pipeline);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdUpdatePipelineIndirectBufferNV"));
    return fn(commandBuffer, pipelineBindPoint, pipeline);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdWaitEvents(VkCommandBuffer                             commandBuffer, uint32_t                                    eventCount, const VkEvent*                              pEvents, VkPipelineStageFlags                        srcStageMask, VkPipelineStageFlags                        dstStageMask, uint32_t                                    memoryBarrierCount, const VkMemoryBarrier*                      pMemoryBarriers, uint32_t                                    bufferMemoryBarrierCount, const VkBufferMemoryBarrier*                pBufferMemoryBarriers, uint32_t                                    imageMemoryBarrierCount, const VkImageMemoryBarrier*                 pImageMemoryBarriers) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    eventCount, const VkEvent*                              pEvents, VkPipelineStageFlags                        srcStageMask, VkPipelineStageFlags                        dstStageMask, uint32_t                                    memoryBarrierCount, const VkMemoryBarrier*                      pMemoryBarriers, uint32_t                                    bufferMemoryBarrierCount, const VkBufferMemoryBarrier*                pBufferMemoryBarriers, uint32_t                                    imageMemoryBarrierCount, const VkImageMemoryBarrier*                 pImageMemoryBarriers);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdWaitEvents"));
    return fn(commandBuffer, eventCount, pEvents, srcStageMask, dstStageMask, memoryBarrierCount, pMemoryBarriers, bufferMemoryBarrierCount, pBufferMemoryBarriers, imageMemoryBarrierCount, pImageMemoryBarriers);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdWaitEvents2(VkCommandBuffer                             commandBuffer, uint32_t                                    eventCount, const VkEvent*                              pEvents, const VkDependencyInfo*                     pDependencyInfos) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    eventCount, const VkEvent*                              pEvents, const VkDependencyInfo*                     pDependencyInfos);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdWaitEvents2"));
    return fn(commandBuffer, eventCount, pEvents, pDependencyInfos);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdWaitEvents2KHR(VkCommandBuffer                             commandBuffer, uint32_t                                    eventCount, const VkEvent*                              pEvents, const VkDependencyInfo*                     pDependencyInfos) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    eventCount, const VkEvent*                              pEvents, const VkDependencyInfo*                     pDependencyInfos);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdWaitEvents2KHR"));
    return fn(commandBuffer, eventCount, pEvents, pDependencyInfos);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdWriteAccelerationStructuresPropertiesKHR(VkCommandBuffer                             commandBuffer, uint32_t                                    accelerationStructureCount, const VkAccelerationStructureKHR*           pAccelerationStructures, VkQueryType                                 queryType, VkQueryPool                                 queryPool, uint32_t                                    firstQuery) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    accelerationStructureCount, const VkAccelerationStructureKHR*           pAccelerationStructures, VkQueryType                                 queryType, VkQueryPool                                 queryPool, uint32_t                                    firstQuery);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdWriteAccelerationStructuresPropertiesKHR"));
    return fn(commandBuffer, accelerationStructureCount, pAccelerationStructures, queryType, queryPool, firstQuery);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdWriteAccelerationStructuresPropertiesNV(VkCommandBuffer                             commandBuffer, uint32_t                                    accelerationStructureCount, const VkAccelerationStructureNV*            pAccelerationStructures, VkQueryType                                 queryType, VkQueryPool                                 queryPool, uint32_t                                    firstQuery) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    accelerationStructureCount, const VkAccelerationStructureNV*            pAccelerationStructures, VkQueryType                                 queryType, VkQueryPool                                 queryPool, uint32_t                                    firstQuery);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdWriteAccelerationStructuresPropertiesNV"));
    return fn(commandBuffer, accelerationStructureCount, pAccelerationStructures, queryType, queryPool, firstQuery);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdWriteBufferMarker2AMD(VkCommandBuffer                             commandBuffer, VkPipelineStageFlags2                       stage, VkBuffer                                    dstBuffer, VkDeviceSize                                dstOffset, uint32_t                                    marker) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkPipelineStageFlags2                       stage, VkBuffer                                    dstBuffer, VkDeviceSize                                dstOffset, uint32_t                                    marker);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdWriteBufferMarker2AMD"));
    return fn(commandBuffer, stage, dstBuffer, dstOffset, marker);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdWriteBufferMarkerAMD(VkCommandBuffer                             commandBuffer, VkPipelineStageFlagBits                     pipelineStage, VkBuffer                                    dstBuffer, VkDeviceSize                                dstOffset, uint32_t                                    marker) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkPipelineStageFlagBits                     pipelineStage, VkBuffer                                    dstBuffer, VkDeviceSize                                dstOffset, uint32_t                                    marker);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdWriteBufferMarkerAMD"));
    return fn(commandBuffer, pipelineStage, dstBuffer, dstOffset, marker);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdWriteMicromapsPropertiesEXT(VkCommandBuffer                             commandBuffer, uint32_t                                    micromapCount, const VkMicromapEXT*                        pMicromaps, VkQueryType                                 queryType, VkQueryPool                                 queryPool, uint32_t                                    firstQuery) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, uint32_t                                    micromapCount, const VkMicromapEXT*                        pMicromaps, VkQueryType                                 queryType, VkQueryPool                                 queryPool, uint32_t                                    firstQuery);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdWriteMicromapsPropertiesEXT"));
    return fn(commandBuffer, micromapCount, pMicromaps, queryType, queryPool, firstQuery);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdWriteTimestamp(VkCommandBuffer                             commandBuffer, VkPipelineStageFlagBits                     pipelineStage, VkQueryPool                                 queryPool, uint32_t                                    query) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkPipelineStageFlagBits                     pipelineStage, VkQueryPool                                 queryPool, uint32_t                                    query);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdWriteTimestamp"));
    return fn(commandBuffer, pipelineStage, queryPool, query);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdWriteTimestamp2(VkCommandBuffer                             commandBuffer, VkPipelineStageFlags2                       stage, VkQueryPool                                 queryPool, uint32_t                                    query) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkPipelineStageFlags2                       stage, VkQueryPool                                 queryPool, uint32_t                                    query);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdWriteTimestamp2"));
    return fn(commandBuffer, stage, queryPool, query);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkCmdWriteTimestamp2KHR(VkCommandBuffer                             commandBuffer, VkPipelineStageFlags2                       stage, VkQueryPool                                 queryPool, uint32_t                                    query) {
    using Fn = void (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkPipelineStageFlags2                       stage, VkQueryPool                                 queryPool, uint32_t                                    query);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkCmdWriteTimestamp2KHR"));
    return fn(commandBuffer, stage, queryPool, query);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCompileDeferredNV(VkDevice                                    device, VkPipeline                                  pipeline, uint32_t                                    shader) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkPipeline                                  pipeline, uint32_t                                    shader);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCompileDeferredNV"));
    return fn(device, pipeline, shader);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCopyAccelerationStructureKHR(VkDevice                                    device, VkDeferredOperationKHR                      deferredOperation, const VkCopyAccelerationStructureInfoKHR*   pInfo) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkDeferredOperationKHR                      deferredOperation, const VkCopyAccelerationStructureInfoKHR*   pInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCopyAccelerationStructureKHR"));
    return fn(device, deferredOperation, pInfo);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCopyAccelerationStructureToMemoryKHR(VkDevice                                    device, VkDeferredOperationKHR                      deferredOperation, const VkCopyAccelerationStructureToMemoryInfoKHR* pInfo) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkDeferredOperationKHR                      deferredOperation, const VkCopyAccelerationStructureToMemoryInfoKHR* pInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCopyAccelerationStructureToMemoryKHR"));
    return fn(device, deferredOperation, pInfo);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCopyImageToImageEXT(VkDevice                                    device, const VkCopyImageToImageInfoEXT*            pCopyImageToImageInfo) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkCopyImageToImageInfoEXT*            pCopyImageToImageInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCopyImageToImageEXT"));
    return fn(device, pCopyImageToImageInfo);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCopyImageToMemoryEXT(VkDevice                                    device, const VkCopyImageToMemoryInfoEXT*           pCopyImageToMemoryInfo) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkCopyImageToMemoryInfoEXT*           pCopyImageToMemoryInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCopyImageToMemoryEXT"));
    return fn(device, pCopyImageToMemoryInfo);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCopyMemoryToAccelerationStructureKHR(VkDevice                                    device, VkDeferredOperationKHR                      deferredOperation, const VkCopyMemoryToAccelerationStructureInfoKHR* pInfo) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkDeferredOperationKHR                      deferredOperation, const VkCopyMemoryToAccelerationStructureInfoKHR* pInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCopyMemoryToAccelerationStructureKHR"));
    return fn(device, deferredOperation, pInfo);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCopyMemoryToImageEXT(VkDevice                                    device, const VkCopyMemoryToImageInfoEXT*           pCopyMemoryToImageInfo) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkCopyMemoryToImageInfoEXT*           pCopyMemoryToImageInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCopyMemoryToImageEXT"));
    return fn(device, pCopyMemoryToImageInfo);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCopyMemoryToMicromapEXT(VkDevice                                    device, VkDeferredOperationKHR                      deferredOperation, const VkCopyMemoryToMicromapInfoEXT*        pInfo) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkDeferredOperationKHR                      deferredOperation, const VkCopyMemoryToMicromapInfoEXT*        pInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCopyMemoryToMicromapEXT"));
    return fn(device, deferredOperation, pInfo);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCopyMicromapEXT(VkDevice                                    device, VkDeferredOperationKHR                      deferredOperation, const VkCopyMicromapInfoEXT*                pInfo) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkDeferredOperationKHR                      deferredOperation, const VkCopyMicromapInfoEXT*                pInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCopyMicromapEXT"));
    return fn(device, deferredOperation, pInfo);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCopyMicromapToMemoryEXT(VkDevice                                    device, VkDeferredOperationKHR                      deferredOperation, const VkCopyMicromapToMemoryInfoEXT*        pInfo) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkDeferredOperationKHR                      deferredOperation, const VkCopyMicromapToMemoryInfoEXT*        pInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCopyMicromapToMemoryEXT"));
    return fn(device, deferredOperation, pInfo);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateAccelerationStructureKHR(VkDevice                                    device, const VkAccelerationStructureCreateInfoKHR* pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkAccelerationStructureKHR*                 pAccelerationStructure) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkAccelerationStructureCreateInfoKHR* pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkAccelerationStructureKHR*                 pAccelerationStructure);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateAccelerationStructureKHR"));
    return fn(device, pCreateInfo, pAllocator, pAccelerationStructure);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateAccelerationStructureNV(VkDevice                                    device, const VkAccelerationStructureCreateInfoNV*  pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkAccelerationStructureNV*                  pAccelerationStructure) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkAccelerationStructureCreateInfoNV*  pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkAccelerationStructureNV*                  pAccelerationStructure);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateAccelerationStructureNV"));
    return fn(device, pCreateInfo, pAllocator, pAccelerationStructure);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateAndroidSurfaceKHR(VkInstance                                  instance, const VkAndroidSurfaceCreateInfoKHR*        pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkSurfaceKHR*                               pSurface) {
    using Fn = VkResult (VKAPI_CALL *)(VkInstance                                  instance, const VkAndroidSurfaceCreateInfoKHR*        pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkSurfaceKHR*                               pSurface);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(instance), "vkCreateAndroidSurfaceKHR"));
    return fn(instance, pCreateInfo, pAllocator, pSurface);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateBuffer(VkDevice                                    device, const VkBufferCreateInfo*                   pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkBuffer*                                   pBuffer) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkBufferCreateInfo*                   pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkBuffer*                                   pBuffer);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateBuffer"));
    return fn(device, pCreateInfo, pAllocator, pBuffer);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateBufferView(VkDevice                                    device, const VkBufferViewCreateInfo*               pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkBufferView*                               pView) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkBufferViewCreateInfo*               pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkBufferView*                               pView);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateBufferView"));
    return fn(device, pCreateInfo, pAllocator, pView);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateCommandPool(VkDevice                                    device, const VkCommandPoolCreateInfo*              pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkCommandPool*                              pCommandPool) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkCommandPoolCreateInfo*              pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkCommandPool*                              pCommandPool);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateCommandPool"));
    return fn(device, pCreateInfo, pAllocator, pCommandPool);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateComputePipelines(VkDevice                                    device, VkPipelineCache                             pipelineCache, uint32_t                                    createInfoCount, const VkComputePipelineCreateInfo*          pCreateInfos, const VkAllocationCallbacks*                pAllocator, VkPipeline*                                 pPipelines) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkPipelineCache                             pipelineCache, uint32_t                                    createInfoCount, const VkComputePipelineCreateInfo*          pCreateInfos, const VkAllocationCallbacks*                pAllocator, VkPipeline*                                 pPipelines);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateComputePipelines"));
    return fn(device, pipelineCache, createInfoCount, pCreateInfos, pAllocator, pPipelines);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateCuFunctionNVX(VkDevice                                    device, const VkCuFunctionCreateInfoNVX*            pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkCuFunctionNVX*                            pFunction) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkCuFunctionCreateInfoNVX*            pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkCuFunctionNVX*                            pFunction);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateCuFunctionNVX"));
    return fn(device, pCreateInfo, pAllocator, pFunction);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateCuModuleNVX(VkDevice                                    device, const VkCuModuleCreateInfoNVX*              pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkCuModuleNVX*                              pModule) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkCuModuleCreateInfoNVX*              pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkCuModuleNVX*                              pModule);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateCuModuleNVX"));
    return fn(device, pCreateInfo, pAllocator, pModule);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateCudaFunctionNV(VkDevice                                    device, const VkCudaFunctionCreateInfoNV*           pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkCudaFunctionNV*                           pFunction) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkCudaFunctionCreateInfoNV*           pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkCudaFunctionNV*                           pFunction);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateCudaFunctionNV"));
    return fn(device, pCreateInfo, pAllocator, pFunction);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateCudaModuleNV(VkDevice                                    device, const VkCudaModuleCreateInfoNV*             pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkCudaModuleNV*                             pModule) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkCudaModuleCreateInfoNV*             pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkCudaModuleNV*                             pModule);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateCudaModuleNV"));
    return fn(device, pCreateInfo, pAllocator, pModule);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateDebugReportCallbackEXT(VkInstance                                  instance, const VkDebugReportCallbackCreateInfoEXT*   pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkDebugReportCallbackEXT*                   pCallback) {
    using Fn = VkResult (VKAPI_CALL *)(VkInstance                                  instance, const VkDebugReportCallbackCreateInfoEXT*   pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkDebugReportCallbackEXT*                   pCallback);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(instance), "vkCreateDebugReportCallbackEXT"));
    return fn(instance, pCreateInfo, pAllocator, pCallback);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateDebugUtilsMessengerEXT(VkInstance                                  instance, const VkDebugUtilsMessengerCreateInfoEXT*   pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkDebugUtilsMessengerEXT*                   pMessenger) {
    using Fn = VkResult (VKAPI_CALL *)(VkInstance                                  instance, const VkDebugUtilsMessengerCreateInfoEXT*   pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkDebugUtilsMessengerEXT*                   pMessenger);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(instance), "vkCreateDebugUtilsMessengerEXT"));
    return fn(instance, pCreateInfo, pAllocator, pMessenger);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateDeferredOperationKHR(VkDevice                                    device, const VkAllocationCallbacks*                pAllocator, VkDeferredOperationKHR*                     pDeferredOperation) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkAllocationCallbacks*                pAllocator, VkDeferredOperationKHR*                     pDeferredOperation);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateDeferredOperationKHR"));
    return fn(device, pAllocator, pDeferredOperation);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateDescriptorPool(VkDevice                                    device, const VkDescriptorPoolCreateInfo*           pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkDescriptorPool*                           pDescriptorPool) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkDescriptorPoolCreateInfo*           pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkDescriptorPool*                           pDescriptorPool);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateDescriptorPool"));
    return fn(device, pCreateInfo, pAllocator, pDescriptorPool);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateDescriptorSetLayout(VkDevice                                    device, const VkDescriptorSetLayoutCreateInfo*      pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkDescriptorSetLayout*                      pSetLayout) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkDescriptorSetLayoutCreateInfo*      pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkDescriptorSetLayout*                      pSetLayout);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateDescriptorSetLayout"));
    return fn(device, pCreateInfo, pAllocator, pSetLayout);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateDescriptorUpdateTemplate(VkDevice                                    device, const VkDescriptorUpdateTemplateCreateInfo* pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkDescriptorUpdateTemplate*                 pDescriptorUpdateTemplate) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkDescriptorUpdateTemplateCreateInfo* pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkDescriptorUpdateTemplate*                 pDescriptorUpdateTemplate);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateDescriptorUpdateTemplate"));
    return fn(device, pCreateInfo, pAllocator, pDescriptorUpdateTemplate);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateDescriptorUpdateTemplateKHR(VkDevice                                    device, const VkDescriptorUpdateTemplateCreateInfo* pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkDescriptorUpdateTemplate*                 pDescriptorUpdateTemplate) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkDescriptorUpdateTemplateCreateInfo* pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkDescriptorUpdateTemplate*                 pDescriptorUpdateTemplate);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateDescriptorUpdateTemplateKHR"));
    return fn(device, pCreateInfo, pAllocator, pDescriptorUpdateTemplate);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateDevice(VkPhysicalDevice                            physicalDevice, const VkDeviceCreateInfo*                   pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkDevice*                                   pDevice) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, const VkDeviceCreateInfo*                   pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkDevice*                                   pDevice);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkCreateDevice"));
    return fn(physicalDevice, pCreateInfo, pAllocator, pDevice);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateDisplayModeKHR(VkPhysicalDevice                            physicalDevice, VkDisplayKHR                                display, const VkDisplayModeCreateInfoKHR*           pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkDisplayModeKHR*                           pMode) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, VkDisplayKHR                                display, const VkDisplayModeCreateInfoKHR*           pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkDisplayModeKHR*                           pMode);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkCreateDisplayModeKHR"));
    return fn(physicalDevice, display, pCreateInfo, pAllocator, pMode);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateDisplayPlaneSurfaceKHR(VkInstance                                  instance, const VkDisplaySurfaceCreateInfoKHR*        pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkSurfaceKHR*                               pSurface) {
    using Fn = VkResult (VKAPI_CALL *)(VkInstance                                  instance, const VkDisplaySurfaceCreateInfoKHR*        pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkSurfaceKHR*                               pSurface);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(instance), "vkCreateDisplayPlaneSurfaceKHR"));
    return fn(instance, pCreateInfo, pAllocator, pSurface);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateEvent(VkDevice                                    device, const VkEventCreateInfo*                    pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkEvent*                                    pEvent) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkEventCreateInfo*                    pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkEvent*                                    pEvent);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateEvent"));
    return fn(device, pCreateInfo, pAllocator, pEvent);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateFence(VkDevice                                    device, const VkFenceCreateInfo*                    pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkFence*                                    pFence) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkFenceCreateInfo*                    pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkFence*                                    pFence);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateFence"));
    return fn(device, pCreateInfo, pAllocator, pFence);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateFramebuffer(VkDevice                                    device, const VkFramebufferCreateInfo*              pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkFramebuffer*                              pFramebuffer) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkFramebufferCreateInfo*              pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkFramebuffer*                              pFramebuffer);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateFramebuffer"));
    return fn(device, pCreateInfo, pAllocator, pFramebuffer);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateGraphicsPipelines(VkDevice                                    device, VkPipelineCache                             pipelineCache, uint32_t                                    createInfoCount, const VkGraphicsPipelineCreateInfo*         pCreateInfos, const VkAllocationCallbacks*                pAllocator, VkPipeline*                                 pPipelines) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkPipelineCache                             pipelineCache, uint32_t                                    createInfoCount, const VkGraphicsPipelineCreateInfo*         pCreateInfos, const VkAllocationCallbacks*                pAllocator, VkPipeline*                                 pPipelines);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateGraphicsPipelines"));
    return fn(device, pipelineCache, createInfoCount, pCreateInfos, pAllocator, pPipelines);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateHeadlessSurfaceEXT(VkInstance                                  instance, const VkHeadlessSurfaceCreateInfoEXT*       pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkSurfaceKHR*                               pSurface) {
    using Fn = VkResult (VKAPI_CALL *)(VkInstance                                  instance, const VkHeadlessSurfaceCreateInfoEXT*       pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkSurfaceKHR*                               pSurface);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(instance), "vkCreateHeadlessSurfaceEXT"));
    return fn(instance, pCreateInfo, pAllocator, pSurface);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateImage(VkDevice                                    device, const VkImageCreateInfo*                    pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkImage*                                    pImage) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkImageCreateInfo*                    pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkImage*                                    pImage);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateImage"));
    return fn(device, pCreateInfo, pAllocator, pImage);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateImageView(VkDevice                                    device, const VkImageViewCreateInfo*                pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkImageView*                                pView) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkImageViewCreateInfo*                pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkImageView*                                pView);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateImageView"));
    return fn(device, pCreateInfo, pAllocator, pView);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateIndirectCommandsLayoutNV(VkDevice                                    device, const VkIndirectCommandsLayoutCreateInfoNV* pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkIndirectCommandsLayoutNV*                 pIndirectCommandsLayout) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkIndirectCommandsLayoutCreateInfoNV* pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkIndirectCommandsLayoutNV*                 pIndirectCommandsLayout);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateIndirectCommandsLayoutNV"));
    return fn(device, pCreateInfo, pAllocator, pIndirectCommandsLayout);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateInstance(const VkInstanceCreateInfo*                 pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkInstance*                                 pInstance) {
    using Fn = VkResult (VKAPI_CALL *)(const VkInstanceCreateInfo*                 pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkInstance*                                 pInstance);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(pInstance), "vkCreateInstance"));
    VkResult rc = fn(pCreateInfo, pAllocator, pInstance);
    if (rc == VK_SUCCESS && pInstance != nullptr) g_instance = *pInstance;
    return rc;
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateMicromapEXT(VkDevice                                    device, const VkMicromapCreateInfoEXT*              pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkMicromapEXT*                              pMicromap) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkMicromapCreateInfoEXT*              pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkMicromapEXT*                              pMicromap);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateMicromapEXT"));
    return fn(device, pCreateInfo, pAllocator, pMicromap);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateOpticalFlowSessionNV(VkDevice                                    device, const VkOpticalFlowSessionCreateInfoNV*     pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkOpticalFlowSessionNV*                     pSession) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkOpticalFlowSessionCreateInfoNV*     pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkOpticalFlowSessionNV*                     pSession);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateOpticalFlowSessionNV"));
    return fn(device, pCreateInfo, pAllocator, pSession);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreatePipelineCache(VkDevice                                    device, const VkPipelineCacheCreateInfo*            pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkPipelineCache*                            pPipelineCache) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkPipelineCacheCreateInfo*            pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkPipelineCache*                            pPipelineCache);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreatePipelineCache"));
    return fn(device, pCreateInfo, pAllocator, pPipelineCache);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreatePipelineLayout(VkDevice                                    device, const VkPipelineLayoutCreateInfo*           pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkPipelineLayout*                           pPipelineLayout) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkPipelineLayoutCreateInfo*           pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkPipelineLayout*                           pPipelineLayout);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreatePipelineLayout"));
    return fn(device, pCreateInfo, pAllocator, pPipelineLayout);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreatePrivateDataSlot(VkDevice                                    device, const VkPrivateDataSlotCreateInfo*          pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkPrivateDataSlot*                          pPrivateDataSlot) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkPrivateDataSlotCreateInfo*          pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkPrivateDataSlot*                          pPrivateDataSlot);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreatePrivateDataSlot"));
    return fn(device, pCreateInfo, pAllocator, pPrivateDataSlot);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreatePrivateDataSlotEXT(VkDevice                                    device, const VkPrivateDataSlotCreateInfo*          pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkPrivateDataSlot*                          pPrivateDataSlot) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkPrivateDataSlotCreateInfo*          pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkPrivateDataSlot*                          pPrivateDataSlot);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreatePrivateDataSlotEXT"));
    return fn(device, pCreateInfo, pAllocator, pPrivateDataSlot);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateQueryPool(VkDevice                                    device, const VkQueryPoolCreateInfo*                pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkQueryPool*                                pQueryPool) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkQueryPoolCreateInfo*                pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkQueryPool*                                pQueryPool);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateQueryPool"));
    return fn(device, pCreateInfo, pAllocator, pQueryPool);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateRayTracingPipelinesKHR(VkDevice                                    device, VkDeferredOperationKHR                      deferredOperation, VkPipelineCache                             pipelineCache, uint32_t                                    createInfoCount, const VkRayTracingPipelineCreateInfoKHR*    pCreateInfos, const VkAllocationCallbacks*                pAllocator, VkPipeline*                                 pPipelines) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkDeferredOperationKHR                      deferredOperation, VkPipelineCache                             pipelineCache, uint32_t                                    createInfoCount, const VkRayTracingPipelineCreateInfoKHR*    pCreateInfos, const VkAllocationCallbacks*                pAllocator, VkPipeline*                                 pPipelines);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateRayTracingPipelinesKHR"));
    return fn(device, deferredOperation, pipelineCache, createInfoCount, pCreateInfos, pAllocator, pPipelines);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateRayTracingPipelinesNV(VkDevice                                    device, VkPipelineCache                             pipelineCache, uint32_t                                    createInfoCount, const VkRayTracingPipelineCreateInfoNV*     pCreateInfos, const VkAllocationCallbacks*                pAllocator, VkPipeline*                                 pPipelines) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkPipelineCache                             pipelineCache, uint32_t                                    createInfoCount, const VkRayTracingPipelineCreateInfoNV*     pCreateInfos, const VkAllocationCallbacks*                pAllocator, VkPipeline*                                 pPipelines);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateRayTracingPipelinesNV"));
    return fn(device, pipelineCache, createInfoCount, pCreateInfos, pAllocator, pPipelines);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateRenderPass(VkDevice                                    device, const VkRenderPassCreateInfo*               pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkRenderPass*                               pRenderPass) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkRenderPassCreateInfo*               pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkRenderPass*                               pRenderPass);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateRenderPass"));
    return fn(device, pCreateInfo, pAllocator, pRenderPass);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateRenderPass2(VkDevice                                    device, const VkRenderPassCreateInfo2*              pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkRenderPass*                               pRenderPass) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkRenderPassCreateInfo2*              pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkRenderPass*                               pRenderPass);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateRenderPass2"));
    return fn(device, pCreateInfo, pAllocator, pRenderPass);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateRenderPass2KHR(VkDevice                                    device, const VkRenderPassCreateInfo2*              pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkRenderPass*                               pRenderPass) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkRenderPassCreateInfo2*              pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkRenderPass*                               pRenderPass);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateRenderPass2KHR"));
    return fn(device, pCreateInfo, pAllocator, pRenderPass);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateSampler(VkDevice                                    device, const VkSamplerCreateInfo*                  pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkSampler*                                  pSampler) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkSamplerCreateInfo*                  pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkSampler*                                  pSampler);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateSampler"));
    return fn(device, pCreateInfo, pAllocator, pSampler);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateSamplerYcbcrConversion(VkDevice                                    device, const VkSamplerYcbcrConversionCreateInfo*   pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkSamplerYcbcrConversion*                   pYcbcrConversion) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkSamplerYcbcrConversionCreateInfo*   pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkSamplerYcbcrConversion*                   pYcbcrConversion);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateSamplerYcbcrConversion"));
    return fn(device, pCreateInfo, pAllocator, pYcbcrConversion);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateSamplerYcbcrConversionKHR(VkDevice                                    device, const VkSamplerYcbcrConversionCreateInfo*   pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkSamplerYcbcrConversion*                   pYcbcrConversion) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkSamplerYcbcrConversionCreateInfo*   pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkSamplerYcbcrConversion*                   pYcbcrConversion);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateSamplerYcbcrConversionKHR"));
    return fn(device, pCreateInfo, pAllocator, pYcbcrConversion);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateSemaphore(VkDevice                                    device, const VkSemaphoreCreateInfo*                pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkSemaphore*                                pSemaphore) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkSemaphoreCreateInfo*                pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkSemaphore*                                pSemaphore);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateSemaphore"));
    return fn(device, pCreateInfo, pAllocator, pSemaphore);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateShaderModule(VkDevice                                    device, const VkShaderModuleCreateInfo*             pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkShaderModule*                             pShaderModule) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkShaderModuleCreateInfo*             pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkShaderModule*                             pShaderModule);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateShaderModule"));
    return fn(device, pCreateInfo, pAllocator, pShaderModule);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateShadersEXT(VkDevice                                    device, uint32_t                                    createInfoCount, const VkShaderCreateInfoEXT*                pCreateInfos, const VkAllocationCallbacks*                pAllocator, VkShaderEXT*                                pShaders) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, uint32_t                                    createInfoCount, const VkShaderCreateInfoEXT*                pCreateInfos, const VkAllocationCallbacks*                pAllocator, VkShaderEXT*                                pShaders);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateShadersEXT"));
    return fn(device, createInfoCount, pCreateInfos, pAllocator, pShaders);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateSharedSwapchainsKHR(VkDevice                                    device, uint32_t                                    swapchainCount, const VkSwapchainCreateInfoKHR*             pCreateInfos, const VkAllocationCallbacks*                pAllocator, VkSwapchainKHR*                             pSwapchains) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, uint32_t                                    swapchainCount, const VkSwapchainCreateInfoKHR*             pCreateInfos, const VkAllocationCallbacks*                pAllocator, VkSwapchainKHR*                             pSwapchains);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateSharedSwapchainsKHR"));
    return fn(device, swapchainCount, pCreateInfos, pAllocator, pSwapchains);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateSwapchainKHR(VkDevice                                    device, const VkSwapchainCreateInfoKHR*             pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkSwapchainKHR*                             pSwapchain) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkSwapchainCreateInfoKHR*             pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkSwapchainKHR*                             pSwapchain);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateSwapchainKHR"));
    return fn(device, pCreateInfo, pAllocator, pSwapchain);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateValidationCacheEXT(VkDevice                                    device, const VkValidationCacheCreateInfoEXT*       pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkValidationCacheEXT*                       pValidationCache) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkValidationCacheCreateInfoEXT*       pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkValidationCacheEXT*                       pValidationCache);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateValidationCacheEXT"));
    return fn(device, pCreateInfo, pAllocator, pValidationCache);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateVideoSessionKHR(VkDevice                                    device, const VkVideoSessionCreateInfoKHR*          pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkVideoSessionKHR*                          pVideoSession) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkVideoSessionCreateInfoKHR*          pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkVideoSessionKHR*                          pVideoSession);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateVideoSessionKHR"));
    return fn(device, pCreateInfo, pAllocator, pVideoSession);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkCreateVideoSessionParametersKHR(VkDevice                                    device, const VkVideoSessionParametersCreateInfoKHR* pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkVideoSessionParametersKHR*                pVideoSessionParameters) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkVideoSessionParametersCreateInfoKHR* pCreateInfo, const VkAllocationCallbacks*                pAllocator, VkVideoSessionParametersKHR*                pVideoSessionParameters);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkCreateVideoSessionParametersKHR"));
    return fn(device, pCreateInfo, pAllocator, pVideoSessionParameters);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkDebugMarkerSetObjectNameEXT(VkDevice                                    device, const VkDebugMarkerObjectNameInfoEXT*       pNameInfo) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkDebugMarkerObjectNameInfoEXT*       pNameInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDebugMarkerSetObjectNameEXT"));
    return fn(device, pNameInfo);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkDebugMarkerSetObjectTagEXT(VkDevice                                    device, const VkDebugMarkerObjectTagInfoEXT*        pTagInfo) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkDebugMarkerObjectTagInfoEXT*        pTagInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDebugMarkerSetObjectTagEXT"));
    return fn(device, pTagInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDebugReportMessageEXT(VkInstance                                  instance, VkDebugReportFlagsEXT                       flags, VkDebugReportObjectTypeEXT                  objectType, uint64_t                                    object, size_t                                      location, int32_t                                     messageCode, const char*                                 pLayerPrefix, const char*                                 pMessage) {
    using Fn = void (VKAPI_CALL *)(VkInstance                                  instance, VkDebugReportFlagsEXT                       flags, VkDebugReportObjectTypeEXT                  objectType, uint64_t                                    object, size_t                                      location, int32_t                                     messageCode, const char*                                 pLayerPrefix, const char*                                 pMessage);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(instance), "vkDebugReportMessageEXT"));
    return fn(instance, flags, objectType, object, location, messageCode, pLayerPrefix, pMessage);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkDeferredOperationJoinKHR(VkDevice                                    device, VkDeferredOperationKHR                      operation) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkDeferredOperationKHR                      operation);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDeferredOperationJoinKHR"));
    return fn(device, operation);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyAccelerationStructureKHR(VkDevice                                    device, VkAccelerationStructureKHR                  accelerationStructure, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkAccelerationStructureKHR                  accelerationStructure, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyAccelerationStructureKHR"));
    return fn(device, accelerationStructure, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyAccelerationStructureNV(VkDevice                                    device, VkAccelerationStructureNV                   accelerationStructure, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkAccelerationStructureNV                   accelerationStructure, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyAccelerationStructureNV"));
    return fn(device, accelerationStructure, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyBuffer(VkDevice                                    device, VkBuffer                                    buffer, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkBuffer                                    buffer, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyBuffer"));
    return fn(device, buffer, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyBufferView(VkDevice                                    device, VkBufferView                                bufferView, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkBufferView                                bufferView, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyBufferView"));
    return fn(device, bufferView, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyCommandPool(VkDevice                                    device, VkCommandPool                               commandPool, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkCommandPool                               commandPool, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyCommandPool"));
    return fn(device, commandPool, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyCuFunctionNVX(VkDevice                                    device, VkCuFunctionNVX                             function, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkCuFunctionNVX                             function, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyCuFunctionNVX"));
    return fn(device, function, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyCuModuleNVX(VkDevice                                    device, VkCuModuleNVX                               module, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkCuModuleNVX                               module, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyCuModuleNVX"));
    return fn(device, module, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyCudaFunctionNV(VkDevice                                    device, VkCudaFunctionNV                            function, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkCudaFunctionNV                            function, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyCudaFunctionNV"));
    return fn(device, function, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyCudaModuleNV(VkDevice                                    device, VkCudaModuleNV                              module, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkCudaModuleNV                              module, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyCudaModuleNV"));
    return fn(device, module, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyDebugReportCallbackEXT(VkInstance                                  instance, VkDebugReportCallbackEXT                    callback, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkInstance                                  instance, VkDebugReportCallbackEXT                    callback, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(instance), "vkDestroyDebugReportCallbackEXT"));
    return fn(instance, callback, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyDebugUtilsMessengerEXT(VkInstance                                  instance, VkDebugUtilsMessengerEXT                    messenger, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkInstance                                  instance, VkDebugUtilsMessengerEXT                    messenger, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(instance), "vkDestroyDebugUtilsMessengerEXT"));
    return fn(instance, messenger, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyDeferredOperationKHR(VkDevice                                    device, VkDeferredOperationKHR                      operation, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkDeferredOperationKHR                      operation, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyDeferredOperationKHR"));
    return fn(device, operation, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyDescriptorPool(VkDevice                                    device, VkDescriptorPool                            descriptorPool, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkDescriptorPool                            descriptorPool, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyDescriptorPool"));
    return fn(device, descriptorPool, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyDescriptorSetLayout(VkDevice                                    device, VkDescriptorSetLayout                       descriptorSetLayout, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkDescriptorSetLayout                       descriptorSetLayout, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyDescriptorSetLayout"));
    return fn(device, descriptorSetLayout, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyDescriptorUpdateTemplate(VkDevice                                    device, VkDescriptorUpdateTemplate                  descriptorUpdateTemplate, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkDescriptorUpdateTemplate                  descriptorUpdateTemplate, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyDescriptorUpdateTemplate"));
    return fn(device, descriptorUpdateTemplate, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyDescriptorUpdateTemplateKHR(VkDevice                                    device, VkDescriptorUpdateTemplate                  descriptorUpdateTemplate, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkDescriptorUpdateTemplate                  descriptorUpdateTemplate, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyDescriptorUpdateTemplateKHR"));
    return fn(device, descriptorUpdateTemplate, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyDevice(VkDevice                                    device, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyDevice"));
    return fn(device, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyEvent(VkDevice                                    device, VkEvent                                     event, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkEvent                                     event, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyEvent"));
    return fn(device, event, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyFence(VkDevice                                    device, VkFence                                     fence, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkFence                                     fence, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyFence"));
    return fn(device, fence, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyFramebuffer(VkDevice                                    device, VkFramebuffer                               framebuffer, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkFramebuffer                               framebuffer, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyFramebuffer"));
    return fn(device, framebuffer, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyImage(VkDevice                                    device, VkImage                                     image, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkImage                                     image, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyImage"));
    return fn(device, image, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyImageView(VkDevice                                    device, VkImageView                                 imageView, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkImageView                                 imageView, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyImageView"));
    return fn(device, imageView, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyIndirectCommandsLayoutNV(VkDevice                                    device, VkIndirectCommandsLayoutNV                  indirectCommandsLayout, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkIndirectCommandsLayoutNV                  indirectCommandsLayout, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyIndirectCommandsLayoutNV"));
    return fn(device, indirectCommandsLayout, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyInstance(VkInstance                                  instance, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkInstance                                  instance, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(instance), "vkDestroyInstance"));
    fn(instance, pAllocator);
    g_instance = VK_NULL_HANDLE;
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyMicromapEXT(VkDevice                                    device, VkMicromapEXT                               micromap, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkMicromapEXT                               micromap, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyMicromapEXT"));
    return fn(device, micromap, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyOpticalFlowSessionNV(VkDevice                                    device, VkOpticalFlowSessionNV                      session, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkOpticalFlowSessionNV                      session, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyOpticalFlowSessionNV"));
    return fn(device, session, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyPipeline(VkDevice                                    device, VkPipeline                                  pipeline, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkPipeline                                  pipeline, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyPipeline"));
    return fn(device, pipeline, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyPipelineCache(VkDevice                                    device, VkPipelineCache                             pipelineCache, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkPipelineCache                             pipelineCache, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyPipelineCache"));
    return fn(device, pipelineCache, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyPipelineLayout(VkDevice                                    device, VkPipelineLayout                            pipelineLayout, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkPipelineLayout                            pipelineLayout, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyPipelineLayout"));
    return fn(device, pipelineLayout, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyPrivateDataSlot(VkDevice                                    device, VkPrivateDataSlot                           privateDataSlot, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkPrivateDataSlot                           privateDataSlot, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyPrivateDataSlot"));
    return fn(device, privateDataSlot, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyPrivateDataSlotEXT(VkDevice                                    device, VkPrivateDataSlot                           privateDataSlot, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkPrivateDataSlot                           privateDataSlot, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyPrivateDataSlotEXT"));
    return fn(device, privateDataSlot, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyQueryPool(VkDevice                                    device, VkQueryPool                                 queryPool, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkQueryPool                                 queryPool, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyQueryPool"));
    return fn(device, queryPool, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyRenderPass(VkDevice                                    device, VkRenderPass                                renderPass, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkRenderPass                                renderPass, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyRenderPass"));
    return fn(device, renderPass, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroySampler(VkDevice                                    device, VkSampler                                   sampler, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkSampler                                   sampler, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroySampler"));
    return fn(device, sampler, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroySamplerYcbcrConversion(VkDevice                                    device, VkSamplerYcbcrConversion                    ycbcrConversion, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkSamplerYcbcrConversion                    ycbcrConversion, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroySamplerYcbcrConversion"));
    return fn(device, ycbcrConversion, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroySamplerYcbcrConversionKHR(VkDevice                                    device, VkSamplerYcbcrConversion                    ycbcrConversion, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkSamplerYcbcrConversion                    ycbcrConversion, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroySamplerYcbcrConversionKHR"));
    return fn(device, ycbcrConversion, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroySemaphore(VkDevice                                    device, VkSemaphore                                 semaphore, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkSemaphore                                 semaphore, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroySemaphore"));
    return fn(device, semaphore, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyShaderEXT(VkDevice                                    device, VkShaderEXT                                 shader, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkShaderEXT                                 shader, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyShaderEXT"));
    return fn(device, shader, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyShaderModule(VkDevice                                    device, VkShaderModule                              shaderModule, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkShaderModule                              shaderModule, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyShaderModule"));
    return fn(device, shaderModule, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroySurfaceKHR(VkInstance                                  instance, VkSurfaceKHR                                surface, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkInstance                                  instance, VkSurfaceKHR                                surface, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(instance), "vkDestroySurfaceKHR"));
    return fn(instance, surface, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroySwapchainKHR(VkDevice                                    device, VkSwapchainKHR                              swapchain, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkSwapchainKHR                              swapchain, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroySwapchainKHR"));
    return fn(device, swapchain, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyValidationCacheEXT(VkDevice                                    device, VkValidationCacheEXT                        validationCache, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkValidationCacheEXT                        validationCache, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyValidationCacheEXT"));
    return fn(device, validationCache, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyVideoSessionKHR(VkDevice                                    device, VkVideoSessionKHR                           videoSession, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkVideoSessionKHR                           videoSession, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyVideoSessionKHR"));
    return fn(device, videoSession, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkDestroyVideoSessionParametersKHR(VkDevice                                    device, VkVideoSessionParametersKHR                 videoSessionParameters, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkVideoSessionParametersKHR                 videoSessionParameters, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDestroyVideoSessionParametersKHR"));
    return fn(device, videoSessionParameters, pAllocator);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkDeviceWaitIdle(VkDevice                                    device) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDeviceWaitIdle"));
    return fn(device);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkDisplayPowerControlEXT(VkDevice                                    device, VkDisplayKHR                                display, const VkDisplayPowerInfoEXT*                pDisplayPowerInfo) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkDisplayKHR                                display, const VkDisplayPowerInfoEXT*                pDisplayPowerInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkDisplayPowerControlEXT"));
    return fn(device, display, pDisplayPowerInfo);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkEndCommandBuffer(VkCommandBuffer                             commandBuffer) {
    using Fn = VkResult (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkEndCommandBuffer"));
    return fn(commandBuffer);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkEnumerateDeviceExtensionProperties(VkPhysicalDevice                            physicalDevice, const char*                                 pLayerName, uint32_t*                                   pPropertyCount, VkExtensionProperties*                      pProperties) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, const char*                                 pLayerName, uint32_t*                                   pPropertyCount, VkExtensionProperties*                      pProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkEnumerateDeviceExtensionProperties"));
    return fn(physicalDevice, pLayerName, pPropertyCount, pProperties);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkEnumerateDeviceLayerProperties(VkPhysicalDevice                            physicalDevice, uint32_t*                                   pPropertyCount, VkLayerProperties*                          pProperties) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, uint32_t*                                   pPropertyCount, VkLayerProperties*                          pProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkEnumerateDeviceLayerProperties"));
    return fn(physicalDevice, pPropertyCount, pProperties);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkEnumerateInstanceExtensionProperties(const char*                                 pLayerName, uint32_t*                                   pPropertyCount, VkExtensionProperties*                      pProperties) {
    using Fn = VkResult (VKAPI_CALL *)(const char*                                 pLayerName, uint32_t*                                   pPropertyCount, VkExtensionProperties*                      pProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(pProperties), "vkEnumerateInstanceExtensionProperties"));
    return fn(pLayerName, pPropertyCount, pProperties);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkEnumerateInstanceLayerProperties(uint32_t*                                   pPropertyCount, VkLayerProperties*                          pProperties) {
    using Fn = VkResult (VKAPI_CALL *)(uint32_t*                                   pPropertyCount, VkLayerProperties*                          pProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(pProperties), "vkEnumerateInstanceLayerProperties"));
    return fn(pPropertyCount, pProperties);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkEnumerateInstanceVersion(uint32_t*                                   pApiVersion) {
    using Fn = VkResult (VKAPI_CALL *)(uint32_t*                                   pApiVersion);
    static Fn fn = reinterpret_cast<Fn>(resolve(VK_NULL_HANDLE, "vkEnumerateInstanceVersion"));
    return fn(pApiVersion);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkEnumeratePhysicalDeviceGroups(VkInstance                                  instance, uint32_t*                                   pPhysicalDeviceGroupCount, VkPhysicalDeviceGroupProperties*            pPhysicalDeviceGroupProperties) {
    using Fn = VkResult (VKAPI_CALL *)(VkInstance                                  instance, uint32_t*                                   pPhysicalDeviceGroupCount, VkPhysicalDeviceGroupProperties*            pPhysicalDeviceGroupProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(instance), "vkEnumeratePhysicalDeviceGroups"));
    return fn(instance, pPhysicalDeviceGroupCount, pPhysicalDeviceGroupProperties);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkEnumeratePhysicalDeviceGroupsKHR(VkInstance                                  instance, uint32_t*                                   pPhysicalDeviceGroupCount, VkPhysicalDeviceGroupProperties*            pPhysicalDeviceGroupProperties) {
    using Fn = VkResult (VKAPI_CALL *)(VkInstance                                  instance, uint32_t*                                   pPhysicalDeviceGroupCount, VkPhysicalDeviceGroupProperties*            pPhysicalDeviceGroupProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(instance), "vkEnumeratePhysicalDeviceGroupsKHR"));
    return fn(instance, pPhysicalDeviceGroupCount, pPhysicalDeviceGroupProperties);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkEnumeratePhysicalDeviceQueueFamilyPerformanceQueryCountersKHR(VkPhysicalDevice                            physicalDevice, uint32_t                                    queueFamilyIndex, uint32_t*                                   pCounterCount, VkPerformanceCounterKHR*                    pCounters, VkPerformanceCounterDescriptionKHR*         pCounterDescriptions) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, uint32_t                                    queueFamilyIndex, uint32_t*                                   pCounterCount, VkPerformanceCounterKHR*                    pCounters, VkPerformanceCounterDescriptionKHR*         pCounterDescriptions);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkEnumeratePhysicalDeviceQueueFamilyPerformanceQueryCountersKHR"));
    return fn(physicalDevice, queueFamilyIndex, pCounterCount, pCounters, pCounterDescriptions);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkEnumeratePhysicalDevices(VkInstance                                  instance, uint32_t*                                   pPhysicalDeviceCount, VkPhysicalDevice*                           pPhysicalDevices) {
    using Fn = VkResult (VKAPI_CALL *)(VkInstance                                  instance, uint32_t*                                   pPhysicalDeviceCount, VkPhysicalDevice*                           pPhysicalDevices);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(instance), "vkEnumeratePhysicalDevices"));
    return fn(instance, pPhysicalDeviceCount, pPhysicalDevices);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkFlushMappedMemoryRanges(VkDevice                                    device, uint32_t                                    memoryRangeCount, const VkMappedMemoryRange*                  pMemoryRanges) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, uint32_t                                    memoryRangeCount, const VkMappedMemoryRange*                  pMemoryRanges);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkFlushMappedMemoryRanges"));
    return fn(device, memoryRangeCount, pMemoryRanges);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkFreeCommandBuffers(VkDevice                                    device, VkCommandPool                               commandPool, uint32_t                                    commandBufferCount, const VkCommandBuffer*                      pCommandBuffers) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkCommandPool                               commandPool, uint32_t                                    commandBufferCount, const VkCommandBuffer*                      pCommandBuffers);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkFreeCommandBuffers"));
    return fn(device, commandPool, commandBufferCount, pCommandBuffers);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkFreeDescriptorSets(VkDevice                                    device, VkDescriptorPool                            descriptorPool, uint32_t                                    descriptorSetCount, const VkDescriptorSet*                      pDescriptorSets) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkDescriptorPool                            descriptorPool, uint32_t                                    descriptorSetCount, const VkDescriptorSet*                      pDescriptorSets);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkFreeDescriptorSets"));
    return fn(device, descriptorPool, descriptorSetCount, pDescriptorSets);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkFreeMemory(VkDevice                                    device, VkDeviceMemory                              memory, const VkAllocationCallbacks*                pAllocator) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkDeviceMemory                              memory, const VkAllocationCallbacks*                pAllocator);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkFreeMemory"));
    return fn(device, memory, pAllocator);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetAccelerationStructureBuildSizesKHR(VkDevice                                    device, VkAccelerationStructureBuildTypeKHR         buildType, const VkAccelerationStructureBuildGeometryInfoKHR* pBuildInfo, const uint32_t*                             pMaxPrimitiveCounts, VkAccelerationStructureBuildSizesInfoKHR*   pSizeInfo) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkAccelerationStructureBuildTypeKHR         buildType, const VkAccelerationStructureBuildGeometryInfoKHR* pBuildInfo, const uint32_t*                             pMaxPrimitiveCounts, VkAccelerationStructureBuildSizesInfoKHR*   pSizeInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetAccelerationStructureBuildSizesKHR"));
    return fn(device, buildType, pBuildInfo, pMaxPrimitiveCounts, pSizeInfo);
}
extern "C" VKAPI_ATTR VkDeviceAddress VKAPI_CALL vkGetAccelerationStructureDeviceAddressKHR(VkDevice                                    device, const VkAccelerationStructureDeviceAddressInfoKHR* pInfo) {
    using Fn = VkDeviceAddress (VKAPI_CALL *)(VkDevice                                    device, const VkAccelerationStructureDeviceAddressInfoKHR* pInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetAccelerationStructureDeviceAddressKHR"));
    return fn(device, pInfo);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetAccelerationStructureHandleNV(VkDevice                                    device, VkAccelerationStructureNV                   accelerationStructure, size_t                                      dataSize, void*                                       pData) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkAccelerationStructureNV                   accelerationStructure, size_t                                      dataSize, void*                                       pData);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetAccelerationStructureHandleNV"));
    return fn(device, accelerationStructure, dataSize, pData);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetAccelerationStructureMemoryRequirementsNV(VkDevice                                    device, const VkAccelerationStructureMemoryRequirementsInfoNV* pInfo, VkMemoryRequirements2KHR*                   pMemoryRequirements) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, const VkAccelerationStructureMemoryRequirementsInfoNV* pInfo, VkMemoryRequirements2KHR*                   pMemoryRequirements);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetAccelerationStructureMemoryRequirementsNV"));
    return fn(device, pInfo, pMemoryRequirements);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetAccelerationStructureOpaqueCaptureDescriptorDataEXT(VkDevice                                    device, const VkAccelerationStructureCaptureDescriptorDataInfoEXT* pInfo, void*                                       pData) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkAccelerationStructureCaptureDescriptorDataInfoEXT* pInfo, void*                                       pData);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetAccelerationStructureOpaqueCaptureDescriptorDataEXT"));
    return fn(device, pInfo, pData);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetAndroidHardwareBufferPropertiesANDROID(VkDevice                                    device, const struct AHardwareBuffer*               buffer, VkAndroidHardwareBufferPropertiesANDROID*   pProperties) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const struct AHardwareBuffer*               buffer, VkAndroidHardwareBufferPropertiesANDROID*   pProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetAndroidHardwareBufferPropertiesANDROID"));
    return fn(device, buffer, pProperties);
}
extern "C" VKAPI_ATTR VkDeviceAddress VKAPI_CALL vkGetBufferDeviceAddress(VkDevice                                    device, const VkBufferDeviceAddressInfo*            pInfo) {
    using Fn = VkDeviceAddress (VKAPI_CALL *)(VkDevice                                    device, const VkBufferDeviceAddressInfo*            pInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetBufferDeviceAddress"));
    return fn(device, pInfo);
}
extern "C" VKAPI_ATTR VkDeviceAddress VKAPI_CALL vkGetBufferDeviceAddressEXT(VkDevice                                    device, const VkBufferDeviceAddressInfo*            pInfo) {
    using Fn = VkDeviceAddress (VKAPI_CALL *)(VkDevice                                    device, const VkBufferDeviceAddressInfo*            pInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetBufferDeviceAddressEXT"));
    return fn(device, pInfo);
}
extern "C" VKAPI_ATTR VkDeviceAddress VKAPI_CALL vkGetBufferDeviceAddressKHR(VkDevice                                    device, const VkBufferDeviceAddressInfo*            pInfo) {
    using Fn = VkDeviceAddress (VKAPI_CALL *)(VkDevice                                    device, const VkBufferDeviceAddressInfo*            pInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetBufferDeviceAddressKHR"));
    return fn(device, pInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetBufferMemoryRequirements(VkDevice                                    device, VkBuffer                                    buffer, VkMemoryRequirements*                       pMemoryRequirements) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkBuffer                                    buffer, VkMemoryRequirements*                       pMemoryRequirements);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetBufferMemoryRequirements"));
    return fn(device, buffer, pMemoryRequirements);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetBufferMemoryRequirements2(VkDevice                                    device, const VkBufferMemoryRequirementsInfo2*      pInfo, VkMemoryRequirements2*                      pMemoryRequirements) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, const VkBufferMemoryRequirementsInfo2*      pInfo, VkMemoryRequirements2*                      pMemoryRequirements);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetBufferMemoryRequirements2"));
    return fn(device, pInfo, pMemoryRequirements);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetBufferMemoryRequirements2KHR(VkDevice                                    device, const VkBufferMemoryRequirementsInfo2*      pInfo, VkMemoryRequirements2*                      pMemoryRequirements) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, const VkBufferMemoryRequirementsInfo2*      pInfo, VkMemoryRequirements2*                      pMemoryRequirements);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetBufferMemoryRequirements2KHR"));
    return fn(device, pInfo, pMemoryRequirements);
}
extern "C" VKAPI_ATTR uint64_t VKAPI_CALL vkGetBufferOpaqueCaptureAddress(VkDevice                                    device, const VkBufferDeviceAddressInfo*            pInfo) {
    using Fn = uint64_t (VKAPI_CALL *)(VkDevice                                    device, const VkBufferDeviceAddressInfo*            pInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetBufferOpaqueCaptureAddress"));
    return fn(device, pInfo);
}
extern "C" VKAPI_ATTR uint64_t VKAPI_CALL vkGetBufferOpaqueCaptureAddressKHR(VkDevice                                    device, const VkBufferDeviceAddressInfo*            pInfo) {
    using Fn = uint64_t (VKAPI_CALL *)(VkDevice                                    device, const VkBufferDeviceAddressInfo*            pInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetBufferOpaqueCaptureAddressKHR"));
    return fn(device, pInfo);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetBufferOpaqueCaptureDescriptorDataEXT(VkDevice                                    device, const VkBufferCaptureDescriptorDataInfoEXT* pInfo, void*                                       pData) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkBufferCaptureDescriptorDataInfoEXT* pInfo, void*                                       pData);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetBufferOpaqueCaptureDescriptorDataEXT"));
    return fn(device, pInfo, pData);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetCalibratedTimestampsEXT(VkDevice                                    device, uint32_t                                    timestampCount, const VkCalibratedTimestampInfoKHR*         pTimestampInfos, uint64_t*                                   pTimestamps, uint64_t*                                   pMaxDeviation) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, uint32_t                                    timestampCount, const VkCalibratedTimestampInfoKHR*         pTimestampInfos, uint64_t*                                   pTimestamps, uint64_t*                                   pMaxDeviation);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetCalibratedTimestampsEXT"));
    return fn(device, timestampCount, pTimestampInfos, pTimestamps, pMaxDeviation);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetCalibratedTimestampsKHR(VkDevice                                    device, uint32_t                                    timestampCount, const VkCalibratedTimestampInfoKHR*         pTimestampInfos, uint64_t*                                   pTimestamps, uint64_t*                                   pMaxDeviation) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, uint32_t                                    timestampCount, const VkCalibratedTimestampInfoKHR*         pTimestampInfos, uint64_t*                                   pTimestamps, uint64_t*                                   pMaxDeviation);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetCalibratedTimestampsKHR"));
    return fn(device, timestampCount, pTimestampInfos, pTimestamps, pMaxDeviation);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetCudaModuleCacheNV(VkDevice                                    device, VkCudaModuleNV                              module, size_t*                                     pCacheSize, void*                                       pCacheData) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkCudaModuleNV                              module, size_t*                                     pCacheSize, void*                                       pCacheData);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetCudaModuleCacheNV"));
    return fn(device, module, pCacheSize, pCacheData);
}
extern "C" VKAPI_ATTR uint32_t VKAPI_CALL vkGetDeferredOperationMaxConcurrencyKHR(VkDevice                                    device, VkDeferredOperationKHR                      operation) {
    using Fn = uint32_t (VKAPI_CALL *)(VkDevice                                    device, VkDeferredOperationKHR                      operation);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetDeferredOperationMaxConcurrencyKHR"));
    return fn(device, operation);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetDeferredOperationResultKHR(VkDevice                                    device, VkDeferredOperationKHR                      operation) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkDeferredOperationKHR                      operation);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetDeferredOperationResultKHR"));
    return fn(device, operation);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetDescriptorEXT(VkDevice                                    device, const VkDescriptorGetInfoEXT*               pDescriptorInfo, size_t                                      dataSize, void*                                       pDescriptor) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, const VkDescriptorGetInfoEXT*               pDescriptorInfo, size_t                                      dataSize, void*                                       pDescriptor);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetDescriptorEXT"));
    return fn(device, pDescriptorInfo, dataSize, pDescriptor);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetDescriptorSetHostMappingVALVE(VkDevice                                    device, VkDescriptorSet                             descriptorSet, void**                                      ppData) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkDescriptorSet                             descriptorSet, void**                                      ppData);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetDescriptorSetHostMappingVALVE"));
    return fn(device, descriptorSet, ppData);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetDescriptorSetLayoutBindingOffsetEXT(VkDevice                                    device, VkDescriptorSetLayout                       layout, uint32_t                                    binding, VkDeviceSize*                               pOffset) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkDescriptorSetLayout                       layout, uint32_t                                    binding, VkDeviceSize*                               pOffset);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetDescriptorSetLayoutBindingOffsetEXT"));
    return fn(device, layout, binding, pOffset);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetDescriptorSetLayoutHostMappingInfoVALVE(VkDevice                                    device, const VkDescriptorSetBindingReferenceVALVE* pBindingReference, VkDescriptorSetLayoutHostMappingInfoVALVE*  pHostMapping) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, const VkDescriptorSetBindingReferenceVALVE* pBindingReference, VkDescriptorSetLayoutHostMappingInfoVALVE*  pHostMapping);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetDescriptorSetLayoutHostMappingInfoVALVE"));
    return fn(device, pBindingReference, pHostMapping);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetDescriptorSetLayoutSizeEXT(VkDevice                                    device, VkDescriptorSetLayout                       layout, VkDeviceSize*                               pLayoutSizeInBytes) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkDescriptorSetLayout                       layout, VkDeviceSize*                               pLayoutSizeInBytes);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetDescriptorSetLayoutSizeEXT"));
    return fn(device, layout, pLayoutSizeInBytes);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetDescriptorSetLayoutSupport(VkDevice                                    device, const VkDescriptorSetLayoutCreateInfo*      pCreateInfo, VkDescriptorSetLayoutSupport*               pSupport) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, const VkDescriptorSetLayoutCreateInfo*      pCreateInfo, VkDescriptorSetLayoutSupport*               pSupport);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetDescriptorSetLayoutSupport"));
    return fn(device, pCreateInfo, pSupport);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetDescriptorSetLayoutSupportKHR(VkDevice                                    device, const VkDescriptorSetLayoutCreateInfo*      pCreateInfo, VkDescriptorSetLayoutSupport*               pSupport) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, const VkDescriptorSetLayoutCreateInfo*      pCreateInfo, VkDescriptorSetLayoutSupport*               pSupport);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetDescriptorSetLayoutSupportKHR"));
    return fn(device, pCreateInfo, pSupport);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetDeviceAccelerationStructureCompatibilityKHR(VkDevice                                    device, const VkAccelerationStructureVersionInfoKHR* pVersionInfo, VkAccelerationStructureCompatibilityKHR*    pCompatibility) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, const VkAccelerationStructureVersionInfoKHR* pVersionInfo, VkAccelerationStructureCompatibilityKHR*    pCompatibility);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetDeviceAccelerationStructureCompatibilityKHR"));
    return fn(device, pVersionInfo, pCompatibility);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetDeviceBufferMemoryRequirements(VkDevice                                    device, const VkDeviceBufferMemoryRequirements*     pInfo, VkMemoryRequirements2*                      pMemoryRequirements) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, const VkDeviceBufferMemoryRequirements*     pInfo, VkMemoryRequirements2*                      pMemoryRequirements);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetDeviceBufferMemoryRequirements"));
    return fn(device, pInfo, pMemoryRequirements);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetDeviceBufferMemoryRequirementsKHR(VkDevice                                    device, const VkDeviceBufferMemoryRequirements*     pInfo, VkMemoryRequirements2*                      pMemoryRequirements) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, const VkDeviceBufferMemoryRequirements*     pInfo, VkMemoryRequirements2*                      pMemoryRequirements);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetDeviceBufferMemoryRequirementsKHR"));
    return fn(device, pInfo, pMemoryRequirements);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetDeviceFaultInfoEXT(VkDevice                                    device, VkDeviceFaultCountsEXT*                     pFaultCounts, VkDeviceFaultInfoEXT*                       pFaultInfo) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkDeviceFaultCountsEXT*                     pFaultCounts, VkDeviceFaultInfoEXT*                       pFaultInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetDeviceFaultInfoEXT"));
    return fn(device, pFaultCounts, pFaultInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetDeviceGroupPeerMemoryFeatures(VkDevice                                    device, uint32_t                                    heapIndex, uint32_t                                    localDeviceIndex, uint32_t                                    remoteDeviceIndex, VkPeerMemoryFeatureFlags*                   pPeerMemoryFeatures) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, uint32_t                                    heapIndex, uint32_t                                    localDeviceIndex, uint32_t                                    remoteDeviceIndex, VkPeerMemoryFeatureFlags*                   pPeerMemoryFeatures);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetDeviceGroupPeerMemoryFeatures"));
    return fn(device, heapIndex, localDeviceIndex, remoteDeviceIndex, pPeerMemoryFeatures);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetDeviceGroupPeerMemoryFeaturesKHR(VkDevice                                    device, uint32_t                                    heapIndex, uint32_t                                    localDeviceIndex, uint32_t                                    remoteDeviceIndex, VkPeerMemoryFeatureFlags*                   pPeerMemoryFeatures) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, uint32_t                                    heapIndex, uint32_t                                    localDeviceIndex, uint32_t                                    remoteDeviceIndex, VkPeerMemoryFeatureFlags*                   pPeerMemoryFeatures);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetDeviceGroupPeerMemoryFeaturesKHR"));
    return fn(device, heapIndex, localDeviceIndex, remoteDeviceIndex, pPeerMemoryFeatures);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetDeviceGroupPresentCapabilitiesKHR(VkDevice                                    device, VkDeviceGroupPresentCapabilitiesKHR*        pDeviceGroupPresentCapabilities) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkDeviceGroupPresentCapabilitiesKHR*        pDeviceGroupPresentCapabilities);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetDeviceGroupPresentCapabilitiesKHR"));
    return fn(device, pDeviceGroupPresentCapabilities);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetDeviceGroupSurfacePresentModesKHR(VkDevice                                    device, VkSurfaceKHR                                surface, VkDeviceGroupPresentModeFlagsKHR*           pModes) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkSurfaceKHR                                surface, VkDeviceGroupPresentModeFlagsKHR*           pModes);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetDeviceGroupSurfacePresentModesKHR"));
    return fn(device, surface, pModes);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetDeviceImageMemoryRequirements(VkDevice                                    device, const VkDeviceImageMemoryRequirements*      pInfo, VkMemoryRequirements2*                      pMemoryRequirements) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, const VkDeviceImageMemoryRequirements*      pInfo, VkMemoryRequirements2*                      pMemoryRequirements);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetDeviceImageMemoryRequirements"));
    return fn(device, pInfo, pMemoryRequirements);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetDeviceImageMemoryRequirementsKHR(VkDevice                                    device, const VkDeviceImageMemoryRequirements*      pInfo, VkMemoryRequirements2*                      pMemoryRequirements) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, const VkDeviceImageMemoryRequirements*      pInfo, VkMemoryRequirements2*                      pMemoryRequirements);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetDeviceImageMemoryRequirementsKHR"));
    return fn(device, pInfo, pMemoryRequirements);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetDeviceImageSparseMemoryRequirements(VkDevice                                    device, const VkDeviceImageMemoryRequirements*      pInfo, uint32_t*                                   pSparseMemoryRequirementCount, VkSparseImageMemoryRequirements2*           pSparseMemoryRequirements) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, const VkDeviceImageMemoryRequirements*      pInfo, uint32_t*                                   pSparseMemoryRequirementCount, VkSparseImageMemoryRequirements2*           pSparseMemoryRequirements);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetDeviceImageSparseMemoryRequirements"));
    return fn(device, pInfo, pSparseMemoryRequirementCount, pSparseMemoryRequirements);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetDeviceImageSparseMemoryRequirementsKHR(VkDevice                                    device, const VkDeviceImageMemoryRequirements*      pInfo, uint32_t*                                   pSparseMemoryRequirementCount, VkSparseImageMemoryRequirements2*           pSparseMemoryRequirements) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, const VkDeviceImageMemoryRequirements*      pInfo, uint32_t*                                   pSparseMemoryRequirementCount, VkSparseImageMemoryRequirements2*           pSparseMemoryRequirements);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetDeviceImageSparseMemoryRequirementsKHR"));
    return fn(device, pInfo, pSparseMemoryRequirementCount, pSparseMemoryRequirements);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetDeviceImageSubresourceLayoutKHR(VkDevice                                    device, const VkDeviceImageSubresourceInfoKHR*      pInfo, VkSubresourceLayout2KHR*                    pLayout) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, const VkDeviceImageSubresourceInfoKHR*      pInfo, VkSubresourceLayout2KHR*                    pLayout);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetDeviceImageSubresourceLayoutKHR"));
    return fn(device, pInfo, pLayout);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetDeviceMemoryCommitment(VkDevice                                    device, VkDeviceMemory                              memory, VkDeviceSize*                               pCommittedMemoryInBytes) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkDeviceMemory                              memory, VkDeviceSize*                               pCommittedMemoryInBytes);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetDeviceMemoryCommitment"));
    return fn(device, memory, pCommittedMemoryInBytes);
}
extern "C" VKAPI_ATTR uint64_t VKAPI_CALL vkGetDeviceMemoryOpaqueCaptureAddress(VkDevice                                    device, const VkDeviceMemoryOpaqueCaptureAddressInfo* pInfo) {
    using Fn = uint64_t (VKAPI_CALL *)(VkDevice                                    device, const VkDeviceMemoryOpaqueCaptureAddressInfo* pInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetDeviceMemoryOpaqueCaptureAddress"));
    return fn(device, pInfo);
}
extern "C" VKAPI_ATTR uint64_t VKAPI_CALL vkGetDeviceMemoryOpaqueCaptureAddressKHR(VkDevice                                    device, const VkDeviceMemoryOpaqueCaptureAddressInfo* pInfo) {
    using Fn = uint64_t (VKAPI_CALL *)(VkDevice                                    device, const VkDeviceMemoryOpaqueCaptureAddressInfo* pInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetDeviceMemoryOpaqueCaptureAddressKHR"));
    return fn(device, pInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetDeviceMicromapCompatibilityEXT(VkDevice                                    device, const VkMicromapVersionInfoEXT*             pVersionInfo, VkAccelerationStructureCompatibilityKHR*    pCompatibility) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, const VkMicromapVersionInfoEXT*             pVersionInfo, VkAccelerationStructureCompatibilityKHR*    pCompatibility);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetDeviceMicromapCompatibilityEXT"));
    return fn(device, pVersionInfo, pCompatibility);
}
extern "C" VKAPI_ATTR PFN_vkVoidFunction VKAPI_CALL vkGetDeviceProcAddr(VkDevice                                    device, const char*                                 pName) {
    using Fn = PFN_vkVoidFunction (VKAPI_CALL *)(VkDevice                                    device, const char*                                 pName);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetDeviceProcAddr"));
    return fn(device, pName);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetDeviceQueue(VkDevice                                    device, uint32_t                                    queueFamilyIndex, uint32_t                                    queueIndex, VkQueue*                                    pQueue) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, uint32_t                                    queueFamilyIndex, uint32_t                                    queueIndex, VkQueue*                                    pQueue);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetDeviceQueue"));
    return fn(device, queueFamilyIndex, queueIndex, pQueue);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetDeviceQueue2(VkDevice                                    device, const VkDeviceQueueInfo2*                   pQueueInfo, VkQueue*                                    pQueue) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, const VkDeviceQueueInfo2*                   pQueueInfo, VkQueue*                                    pQueue);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetDeviceQueue2"));
    return fn(device, pQueueInfo, pQueue);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetDeviceSubpassShadingMaxWorkgroupSizeHUAWEI(VkDevice                                    device, VkRenderPass                                renderpass, VkExtent2D*                                 pMaxWorkgroupSize) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkRenderPass                                renderpass, VkExtent2D*                                 pMaxWorkgroupSize);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetDeviceSubpassShadingMaxWorkgroupSizeHUAWEI"));
    return fn(device, renderpass, pMaxWorkgroupSize);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetDisplayModeProperties2KHR(VkPhysicalDevice                            physicalDevice, VkDisplayKHR                                display, uint32_t*                                   pPropertyCount, VkDisplayModeProperties2KHR*                pProperties) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, VkDisplayKHR                                display, uint32_t*                                   pPropertyCount, VkDisplayModeProperties2KHR*                pProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetDisplayModeProperties2KHR"));
    return fn(physicalDevice, display, pPropertyCount, pProperties);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetDisplayModePropertiesKHR(VkPhysicalDevice                            physicalDevice, VkDisplayKHR                                display, uint32_t*                                   pPropertyCount, VkDisplayModePropertiesKHR*                 pProperties) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, VkDisplayKHR                                display, uint32_t*                                   pPropertyCount, VkDisplayModePropertiesKHR*                 pProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetDisplayModePropertiesKHR"));
    return fn(physicalDevice, display, pPropertyCount, pProperties);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetDisplayPlaneCapabilities2KHR(VkPhysicalDevice                            physicalDevice, const VkDisplayPlaneInfo2KHR*               pDisplayPlaneInfo, VkDisplayPlaneCapabilities2KHR*             pCapabilities) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, const VkDisplayPlaneInfo2KHR*               pDisplayPlaneInfo, VkDisplayPlaneCapabilities2KHR*             pCapabilities);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetDisplayPlaneCapabilities2KHR"));
    return fn(physicalDevice, pDisplayPlaneInfo, pCapabilities);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetDisplayPlaneCapabilitiesKHR(VkPhysicalDevice                            physicalDevice, VkDisplayModeKHR                            mode, uint32_t                                    planeIndex, VkDisplayPlaneCapabilitiesKHR*              pCapabilities) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, VkDisplayModeKHR                            mode, uint32_t                                    planeIndex, VkDisplayPlaneCapabilitiesKHR*              pCapabilities);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetDisplayPlaneCapabilitiesKHR"));
    return fn(physicalDevice, mode, planeIndex, pCapabilities);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetDisplayPlaneSupportedDisplaysKHR(VkPhysicalDevice                            physicalDevice, uint32_t                                    planeIndex, uint32_t*                                   pDisplayCount, VkDisplayKHR*                               pDisplays) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, uint32_t                                    planeIndex, uint32_t*                                   pDisplayCount, VkDisplayKHR*                               pDisplays);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetDisplayPlaneSupportedDisplaysKHR"));
    return fn(physicalDevice, planeIndex, pDisplayCount, pDisplays);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetDrmDisplayEXT(VkPhysicalDevice                            physicalDevice, int32_t                                     drmFd, uint32_t                                    connectorId, VkDisplayKHR*                               display) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, int32_t                                     drmFd, uint32_t                                    connectorId, VkDisplayKHR*                               display);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetDrmDisplayEXT"));
    return fn(physicalDevice, drmFd, connectorId, display);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetDynamicRenderingTilePropertiesQCOM(VkDevice                                    device, const VkRenderingInfo*                      pRenderingInfo, VkTilePropertiesQCOM*                       pProperties) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkRenderingInfo*                      pRenderingInfo, VkTilePropertiesQCOM*                       pProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetDynamicRenderingTilePropertiesQCOM"));
    return fn(device, pRenderingInfo, pProperties);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetEncodedVideoSessionParametersKHR(VkDevice                                    device, const VkVideoEncodeSessionParametersGetInfoKHR* pVideoSessionParametersInfo, VkVideoEncodeSessionParametersFeedbackInfoKHR* pFeedbackInfo, size_t*                                     pDataSize, void*                                       pData) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkVideoEncodeSessionParametersGetInfoKHR* pVideoSessionParametersInfo, VkVideoEncodeSessionParametersFeedbackInfoKHR* pFeedbackInfo, size_t*                                     pDataSize, void*                                       pData);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetEncodedVideoSessionParametersKHR"));
    return fn(device, pVideoSessionParametersInfo, pFeedbackInfo, pDataSize, pData);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetEventStatus(VkDevice                                    device, VkEvent                                     event) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkEvent                                     event);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetEventStatus"));
    return fn(device, event);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetFenceFdKHR(VkDevice                                    device, const VkFenceGetFdInfoKHR*                  pGetFdInfo, int*                                        pFd) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkFenceGetFdInfoKHR*                  pGetFdInfo, int*                                        pFd);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetFenceFdKHR"));
    return fn(device, pGetFdInfo, pFd);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetFenceStatus(VkDevice                                    device, VkFence                                     fence) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkFence                                     fence);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetFenceStatus"));
    return fn(device, fence);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetFramebufferTilePropertiesQCOM(VkDevice                                    device, VkFramebuffer                               framebuffer, uint32_t*                                   pPropertiesCount, VkTilePropertiesQCOM*                       pProperties) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkFramebuffer                               framebuffer, uint32_t*                                   pPropertiesCount, VkTilePropertiesQCOM*                       pProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetFramebufferTilePropertiesQCOM"));
    return fn(device, framebuffer, pPropertiesCount, pProperties);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetGeneratedCommandsMemoryRequirementsNV(VkDevice                                    device, const VkGeneratedCommandsMemoryRequirementsInfoNV* pInfo, VkMemoryRequirements2*                      pMemoryRequirements) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, const VkGeneratedCommandsMemoryRequirementsInfoNV* pInfo, VkMemoryRequirements2*                      pMemoryRequirements);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetGeneratedCommandsMemoryRequirementsNV"));
    return fn(device, pInfo, pMemoryRequirements);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetImageDrmFormatModifierPropertiesEXT(VkDevice                                    device, VkImage                                     image, VkImageDrmFormatModifierPropertiesEXT*      pProperties) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkImage                                     image, VkImageDrmFormatModifierPropertiesEXT*      pProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetImageDrmFormatModifierPropertiesEXT"));
    return fn(device, image, pProperties);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetImageMemoryRequirements(VkDevice                                    device, VkImage                                     image, VkMemoryRequirements*                       pMemoryRequirements) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkImage                                     image, VkMemoryRequirements*                       pMemoryRequirements);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetImageMemoryRequirements"));
    return fn(device, image, pMemoryRequirements);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetImageMemoryRequirements2(VkDevice                                    device, const VkImageMemoryRequirementsInfo2*       pInfo, VkMemoryRequirements2*                      pMemoryRequirements) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, const VkImageMemoryRequirementsInfo2*       pInfo, VkMemoryRequirements2*                      pMemoryRequirements);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetImageMemoryRequirements2"));
    return fn(device, pInfo, pMemoryRequirements);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetImageMemoryRequirements2KHR(VkDevice                                    device, const VkImageMemoryRequirementsInfo2*       pInfo, VkMemoryRequirements2*                      pMemoryRequirements) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, const VkImageMemoryRequirementsInfo2*       pInfo, VkMemoryRequirements2*                      pMemoryRequirements);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetImageMemoryRequirements2KHR"));
    return fn(device, pInfo, pMemoryRequirements);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetImageOpaqueCaptureDescriptorDataEXT(VkDevice                                    device, const VkImageCaptureDescriptorDataInfoEXT*  pInfo, void*                                       pData) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkImageCaptureDescriptorDataInfoEXT*  pInfo, void*                                       pData);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetImageOpaqueCaptureDescriptorDataEXT"));
    return fn(device, pInfo, pData);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetImageSparseMemoryRequirements(VkDevice                                    device, VkImage                                     image, uint32_t*                                   pSparseMemoryRequirementCount, VkSparseImageMemoryRequirements*            pSparseMemoryRequirements) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkImage                                     image, uint32_t*                                   pSparseMemoryRequirementCount, VkSparseImageMemoryRequirements*            pSparseMemoryRequirements);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetImageSparseMemoryRequirements"));
    return fn(device, image, pSparseMemoryRequirementCount, pSparseMemoryRequirements);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetImageSparseMemoryRequirements2(VkDevice                                    device, const VkImageSparseMemoryRequirementsInfo2* pInfo, uint32_t*                                   pSparseMemoryRequirementCount, VkSparseImageMemoryRequirements2*           pSparseMemoryRequirements) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, const VkImageSparseMemoryRequirementsInfo2* pInfo, uint32_t*                                   pSparseMemoryRequirementCount, VkSparseImageMemoryRequirements2*           pSparseMemoryRequirements);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetImageSparseMemoryRequirements2"));
    return fn(device, pInfo, pSparseMemoryRequirementCount, pSparseMemoryRequirements);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetImageSparseMemoryRequirements2KHR(VkDevice                                    device, const VkImageSparseMemoryRequirementsInfo2* pInfo, uint32_t*                                   pSparseMemoryRequirementCount, VkSparseImageMemoryRequirements2*           pSparseMemoryRequirements) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, const VkImageSparseMemoryRequirementsInfo2* pInfo, uint32_t*                                   pSparseMemoryRequirementCount, VkSparseImageMemoryRequirements2*           pSparseMemoryRequirements);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetImageSparseMemoryRequirements2KHR"));
    return fn(device, pInfo, pSparseMemoryRequirementCount, pSparseMemoryRequirements);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetImageSubresourceLayout(VkDevice                                    device, VkImage                                     image, const VkImageSubresource*                   pSubresource, VkSubresourceLayout*                        pLayout) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkImage                                     image, const VkImageSubresource*                   pSubresource, VkSubresourceLayout*                        pLayout);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetImageSubresourceLayout"));
    return fn(device, image, pSubresource, pLayout);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetImageSubresourceLayout2EXT(VkDevice                                    device, VkImage                                     image, const VkImageSubresource2KHR*               pSubresource, VkSubresourceLayout2KHR*                    pLayout) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkImage                                     image, const VkImageSubresource2KHR*               pSubresource, VkSubresourceLayout2KHR*                    pLayout);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetImageSubresourceLayout2EXT"));
    return fn(device, image, pSubresource, pLayout);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetImageSubresourceLayout2KHR(VkDevice                                    device, VkImage                                     image, const VkImageSubresource2KHR*               pSubresource, VkSubresourceLayout2KHR*                    pLayout) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkImage                                     image, const VkImageSubresource2KHR*               pSubresource, VkSubresourceLayout2KHR*                    pLayout);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetImageSubresourceLayout2KHR"));
    return fn(device, image, pSubresource, pLayout);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetImageViewAddressNVX(VkDevice                                    device, VkImageView                                 imageView, VkImageViewAddressPropertiesNVX*            pProperties) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkImageView                                 imageView, VkImageViewAddressPropertiesNVX*            pProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetImageViewAddressNVX"));
    return fn(device, imageView, pProperties);
}
extern "C" VKAPI_ATTR uint32_t VKAPI_CALL vkGetImageViewHandleNVX(VkDevice                                    device, const VkImageViewHandleInfoNVX*             pInfo) {
    using Fn = uint32_t (VKAPI_CALL *)(VkDevice                                    device, const VkImageViewHandleInfoNVX*             pInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetImageViewHandleNVX"));
    return fn(device, pInfo);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetImageViewOpaqueCaptureDescriptorDataEXT(VkDevice                                    device, const VkImageViewCaptureDescriptorDataInfoEXT* pInfo, void*                                       pData) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkImageViewCaptureDescriptorDataInfoEXT* pInfo, void*                                       pData);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetImageViewOpaqueCaptureDescriptorDataEXT"));
    return fn(device, pInfo, pData);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetLatencyTimingsNV(VkDevice                                    device, VkSwapchainKHR                              swapchain, VkGetLatencyMarkerInfoNV*                   pLatencyMarkerInfo) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkSwapchainKHR                              swapchain, VkGetLatencyMarkerInfoNV*                   pLatencyMarkerInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetLatencyTimingsNV"));
    return fn(device, swapchain, pLatencyMarkerInfo);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetMemoryAndroidHardwareBufferANDROID(VkDevice                                    device, const VkMemoryGetAndroidHardwareBufferInfoANDROID* pInfo, struct AHardwareBuffer**                    pBuffer) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkMemoryGetAndroidHardwareBufferInfoANDROID* pInfo, struct AHardwareBuffer**                    pBuffer);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetMemoryAndroidHardwareBufferANDROID"));
    return fn(device, pInfo, pBuffer);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetMemoryFdKHR(VkDevice                                    device, const VkMemoryGetFdInfoKHR*                 pGetFdInfo, int*                                        pFd) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkMemoryGetFdInfoKHR*                 pGetFdInfo, int*                                        pFd);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetMemoryFdKHR"));
    return fn(device, pGetFdInfo, pFd);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetMemoryFdPropertiesKHR(VkDevice                                    device, VkExternalMemoryHandleTypeFlagBits          handleType, int                                         fd, VkMemoryFdPropertiesKHR*                    pMemoryFdProperties) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkExternalMemoryHandleTypeFlagBits          handleType, int                                         fd, VkMemoryFdPropertiesKHR*                    pMemoryFdProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetMemoryFdPropertiesKHR"));
    return fn(device, handleType, fd, pMemoryFdProperties);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetMemoryHostPointerPropertiesEXT(VkDevice                                    device, VkExternalMemoryHandleTypeFlagBits          handleType, const void*                                 pHostPointer, VkMemoryHostPointerPropertiesEXT*           pMemoryHostPointerProperties) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkExternalMemoryHandleTypeFlagBits          handleType, const void*                                 pHostPointer, VkMemoryHostPointerPropertiesEXT*           pMemoryHostPointerProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetMemoryHostPointerPropertiesEXT"));
    return fn(device, handleType, pHostPointer, pMemoryHostPointerProperties);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetMemoryRemoteAddressNV(VkDevice                                    device, const VkMemoryGetRemoteAddressInfoNV*       pMemoryGetRemoteAddressInfo, VkRemoteAddressNV*                          pAddress) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkMemoryGetRemoteAddressInfoNV*       pMemoryGetRemoteAddressInfo, VkRemoteAddressNV*                          pAddress);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetMemoryRemoteAddressNV"));
    return fn(device, pMemoryGetRemoteAddressInfo, pAddress);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetMicromapBuildSizesEXT(VkDevice                                    device, VkAccelerationStructureBuildTypeKHR         buildType, const VkMicromapBuildInfoEXT*               pBuildInfo, VkMicromapBuildSizesInfoEXT*                pSizeInfo) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkAccelerationStructureBuildTypeKHR         buildType, const VkMicromapBuildInfoEXT*               pBuildInfo, VkMicromapBuildSizesInfoEXT*                pSizeInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetMicromapBuildSizesEXT"));
    return fn(device, buildType, pBuildInfo, pSizeInfo);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPastPresentationTimingGOOGLE(VkDevice                                    device, VkSwapchainKHR                              swapchain, uint32_t*                                   pPresentationTimingCount, VkPastPresentationTimingGOOGLE*             pPresentationTimings) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkSwapchainKHR                              swapchain, uint32_t*                                   pPresentationTimingCount, VkPastPresentationTimingGOOGLE*             pPresentationTimings);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetPastPresentationTimingGOOGLE"));
    return fn(device, swapchain, pPresentationTimingCount, pPresentationTimings);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPerformanceParameterINTEL(VkDevice                                    device, VkPerformanceParameterTypeINTEL             parameter, VkPerformanceValueINTEL*                    pValue) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkPerformanceParameterTypeINTEL             parameter, VkPerformanceValueINTEL*                    pValue);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetPerformanceParameterINTEL"));
    return fn(device, parameter, pValue);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPhysicalDeviceCalibrateableTimeDomainsEXT(VkPhysicalDevice                            physicalDevice, uint32_t*                                   pTimeDomainCount, VkTimeDomainKHR*                            pTimeDomains) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, uint32_t*                                   pTimeDomainCount, VkTimeDomainKHR*                            pTimeDomains);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceCalibrateableTimeDomainsEXT"));
    return fn(physicalDevice, pTimeDomainCount, pTimeDomains);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPhysicalDeviceCalibrateableTimeDomainsKHR(VkPhysicalDevice                            physicalDevice, uint32_t*                                   pTimeDomainCount, VkTimeDomainKHR*                            pTimeDomains) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, uint32_t*                                   pTimeDomainCount, VkTimeDomainKHR*                            pTimeDomains);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceCalibrateableTimeDomainsKHR"));
    return fn(physicalDevice, pTimeDomainCount, pTimeDomains);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPhysicalDeviceCooperativeMatrixPropertiesKHR(VkPhysicalDevice                            physicalDevice, uint32_t*                                   pPropertyCount, VkCooperativeMatrixPropertiesKHR*           pProperties) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, uint32_t*                                   pPropertyCount, VkCooperativeMatrixPropertiesKHR*           pProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceCooperativeMatrixPropertiesKHR"));
    return fn(physicalDevice, pPropertyCount, pProperties);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPhysicalDeviceCooperativeMatrixPropertiesNV(VkPhysicalDevice                            physicalDevice, uint32_t*                                   pPropertyCount, VkCooperativeMatrixPropertiesNV*            pProperties) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, uint32_t*                                   pPropertyCount, VkCooperativeMatrixPropertiesNV*            pProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceCooperativeMatrixPropertiesNV"));
    return fn(physicalDevice, pPropertyCount, pProperties);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPhysicalDeviceDisplayPlaneProperties2KHR(VkPhysicalDevice                            physicalDevice, uint32_t*                                   pPropertyCount, VkDisplayPlaneProperties2KHR*               pProperties) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, uint32_t*                                   pPropertyCount, VkDisplayPlaneProperties2KHR*               pProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceDisplayPlaneProperties2KHR"));
    return fn(physicalDevice, pPropertyCount, pProperties);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPhysicalDeviceDisplayPlanePropertiesKHR(VkPhysicalDevice                            physicalDevice, uint32_t*                                   pPropertyCount, VkDisplayPlanePropertiesKHR*                pProperties) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, uint32_t*                                   pPropertyCount, VkDisplayPlanePropertiesKHR*                pProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceDisplayPlanePropertiesKHR"));
    return fn(physicalDevice, pPropertyCount, pProperties);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPhysicalDeviceDisplayProperties2KHR(VkPhysicalDevice                            physicalDevice, uint32_t*                                   pPropertyCount, VkDisplayProperties2KHR*                    pProperties) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, uint32_t*                                   pPropertyCount, VkDisplayProperties2KHR*                    pProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceDisplayProperties2KHR"));
    return fn(physicalDevice, pPropertyCount, pProperties);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPhysicalDeviceDisplayPropertiesKHR(VkPhysicalDevice                            physicalDevice, uint32_t*                                   pPropertyCount, VkDisplayPropertiesKHR*                     pProperties) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, uint32_t*                                   pPropertyCount, VkDisplayPropertiesKHR*                     pProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceDisplayPropertiesKHR"));
    return fn(physicalDevice, pPropertyCount, pProperties);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetPhysicalDeviceExternalBufferProperties(VkPhysicalDevice                            physicalDevice, const VkPhysicalDeviceExternalBufferInfo*   pExternalBufferInfo, VkExternalBufferProperties*                 pExternalBufferProperties) {
    using Fn = void (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, const VkPhysicalDeviceExternalBufferInfo*   pExternalBufferInfo, VkExternalBufferProperties*                 pExternalBufferProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceExternalBufferProperties"));
    return fn(physicalDevice, pExternalBufferInfo, pExternalBufferProperties);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetPhysicalDeviceExternalBufferPropertiesKHR(VkPhysicalDevice                            physicalDevice, const VkPhysicalDeviceExternalBufferInfo*   pExternalBufferInfo, VkExternalBufferProperties*                 pExternalBufferProperties) {
    using Fn = void (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, const VkPhysicalDeviceExternalBufferInfo*   pExternalBufferInfo, VkExternalBufferProperties*                 pExternalBufferProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceExternalBufferPropertiesKHR"));
    return fn(physicalDevice, pExternalBufferInfo, pExternalBufferProperties);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetPhysicalDeviceExternalFenceProperties(VkPhysicalDevice                            physicalDevice, const VkPhysicalDeviceExternalFenceInfo*    pExternalFenceInfo, VkExternalFenceProperties*                  pExternalFenceProperties) {
    using Fn = void (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, const VkPhysicalDeviceExternalFenceInfo*    pExternalFenceInfo, VkExternalFenceProperties*                  pExternalFenceProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceExternalFenceProperties"));
    return fn(physicalDevice, pExternalFenceInfo, pExternalFenceProperties);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetPhysicalDeviceExternalFencePropertiesKHR(VkPhysicalDevice                            physicalDevice, const VkPhysicalDeviceExternalFenceInfo*    pExternalFenceInfo, VkExternalFenceProperties*                  pExternalFenceProperties) {
    using Fn = void (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, const VkPhysicalDeviceExternalFenceInfo*    pExternalFenceInfo, VkExternalFenceProperties*                  pExternalFenceProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceExternalFencePropertiesKHR"));
    return fn(physicalDevice, pExternalFenceInfo, pExternalFenceProperties);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPhysicalDeviceExternalImageFormatPropertiesNV(VkPhysicalDevice                            physicalDevice, VkFormat                                    format, VkImageType                                 type, VkImageTiling                               tiling, VkImageUsageFlags                           usage, VkImageCreateFlags                          flags, VkExternalMemoryHandleTypeFlagsNV           externalHandleType, VkExternalImageFormatPropertiesNV*          pExternalImageFormatProperties) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, VkFormat                                    format, VkImageType                                 type, VkImageTiling                               tiling, VkImageUsageFlags                           usage, VkImageCreateFlags                          flags, VkExternalMemoryHandleTypeFlagsNV           externalHandleType, VkExternalImageFormatPropertiesNV*          pExternalImageFormatProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceExternalImageFormatPropertiesNV"));
    return fn(physicalDevice, format, type, tiling, usage, flags, externalHandleType, pExternalImageFormatProperties);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetPhysicalDeviceExternalSemaphoreProperties(VkPhysicalDevice                            physicalDevice, const VkPhysicalDeviceExternalSemaphoreInfo* pExternalSemaphoreInfo, VkExternalSemaphoreProperties*              pExternalSemaphoreProperties) {
    using Fn = void (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, const VkPhysicalDeviceExternalSemaphoreInfo* pExternalSemaphoreInfo, VkExternalSemaphoreProperties*              pExternalSemaphoreProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceExternalSemaphoreProperties"));
    return fn(physicalDevice, pExternalSemaphoreInfo, pExternalSemaphoreProperties);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetPhysicalDeviceExternalSemaphorePropertiesKHR(VkPhysicalDevice                            physicalDevice, const VkPhysicalDeviceExternalSemaphoreInfo* pExternalSemaphoreInfo, VkExternalSemaphoreProperties*              pExternalSemaphoreProperties) {
    using Fn = void (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, const VkPhysicalDeviceExternalSemaphoreInfo* pExternalSemaphoreInfo, VkExternalSemaphoreProperties*              pExternalSemaphoreProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceExternalSemaphorePropertiesKHR"));
    return fn(physicalDevice, pExternalSemaphoreInfo, pExternalSemaphoreProperties);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetPhysicalDeviceFeatures(VkPhysicalDevice                            physicalDevice, VkPhysicalDeviceFeatures*                   pFeatures) {
    using Fn = void (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, VkPhysicalDeviceFeatures*                   pFeatures);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceFeatures"));
    return fn(physicalDevice, pFeatures);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetPhysicalDeviceFeatures2(VkPhysicalDevice                            physicalDevice, VkPhysicalDeviceFeatures2*                  pFeatures) {
    using Fn = void (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, VkPhysicalDeviceFeatures2*                  pFeatures);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceFeatures2"));
    return fn(physicalDevice, pFeatures);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetPhysicalDeviceFeatures2KHR(VkPhysicalDevice                            physicalDevice, VkPhysicalDeviceFeatures2*                  pFeatures) {
    using Fn = void (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, VkPhysicalDeviceFeatures2*                  pFeatures);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceFeatures2KHR"));
    return fn(physicalDevice, pFeatures);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetPhysicalDeviceFormatProperties(VkPhysicalDevice                            physicalDevice, VkFormat                                    format, VkFormatProperties*                         pFormatProperties) {
    using Fn = void (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, VkFormat                                    format, VkFormatProperties*                         pFormatProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceFormatProperties"));
    return fn(physicalDevice, format, pFormatProperties);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetPhysicalDeviceFormatProperties2(VkPhysicalDevice                            physicalDevice, VkFormat                                    format, VkFormatProperties2*                        pFormatProperties) {
    using Fn = void (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, VkFormat                                    format, VkFormatProperties2*                        pFormatProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceFormatProperties2"));
    return fn(physicalDevice, format, pFormatProperties);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetPhysicalDeviceFormatProperties2KHR(VkPhysicalDevice                            physicalDevice, VkFormat                                    format, VkFormatProperties2*                        pFormatProperties) {
    using Fn = void (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, VkFormat                                    format, VkFormatProperties2*                        pFormatProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceFormatProperties2KHR"));
    return fn(physicalDevice, format, pFormatProperties);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPhysicalDeviceFragmentShadingRatesKHR(VkPhysicalDevice                            physicalDevice, uint32_t*                                   pFragmentShadingRateCount, VkPhysicalDeviceFragmentShadingRateKHR*     pFragmentShadingRates) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, uint32_t*                                   pFragmentShadingRateCount, VkPhysicalDeviceFragmentShadingRateKHR*     pFragmentShadingRates);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceFragmentShadingRatesKHR"));
    return fn(physicalDevice, pFragmentShadingRateCount, pFragmentShadingRates);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPhysicalDeviceImageFormatProperties(VkPhysicalDevice                            physicalDevice, VkFormat                                    format, VkImageType                                 type, VkImageTiling                               tiling, VkImageUsageFlags                           usage, VkImageCreateFlags                          flags, VkImageFormatProperties*                    pImageFormatProperties) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, VkFormat                                    format, VkImageType                                 type, VkImageTiling                               tiling, VkImageUsageFlags                           usage, VkImageCreateFlags                          flags, VkImageFormatProperties*                    pImageFormatProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceImageFormatProperties"));
    return fn(physicalDevice, format, type, tiling, usage, flags, pImageFormatProperties);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPhysicalDeviceImageFormatProperties2(VkPhysicalDevice                            physicalDevice, const VkPhysicalDeviceImageFormatInfo2*     pImageFormatInfo, VkImageFormatProperties2*                   pImageFormatProperties) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, const VkPhysicalDeviceImageFormatInfo2*     pImageFormatInfo, VkImageFormatProperties2*                   pImageFormatProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceImageFormatProperties2"));
    return fn(physicalDevice, pImageFormatInfo, pImageFormatProperties);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPhysicalDeviceImageFormatProperties2KHR(VkPhysicalDevice                            physicalDevice, const VkPhysicalDeviceImageFormatInfo2*     pImageFormatInfo, VkImageFormatProperties2*                   pImageFormatProperties) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, const VkPhysicalDeviceImageFormatInfo2*     pImageFormatInfo, VkImageFormatProperties2*                   pImageFormatProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceImageFormatProperties2KHR"));
    return fn(physicalDevice, pImageFormatInfo, pImageFormatProperties);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetPhysicalDeviceMemoryProperties(VkPhysicalDevice                            physicalDevice, VkPhysicalDeviceMemoryProperties*           pMemoryProperties) {
    using Fn = void (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, VkPhysicalDeviceMemoryProperties*           pMemoryProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceMemoryProperties"));
    return fn(physicalDevice, pMemoryProperties);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetPhysicalDeviceMemoryProperties2(VkPhysicalDevice                            physicalDevice, VkPhysicalDeviceMemoryProperties2*          pMemoryProperties) {
    using Fn = void (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, VkPhysicalDeviceMemoryProperties2*          pMemoryProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceMemoryProperties2"));
    return fn(physicalDevice, pMemoryProperties);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetPhysicalDeviceMemoryProperties2KHR(VkPhysicalDevice                            physicalDevice, VkPhysicalDeviceMemoryProperties2*          pMemoryProperties) {
    using Fn = void (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, VkPhysicalDeviceMemoryProperties2*          pMemoryProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceMemoryProperties2KHR"));
    return fn(physicalDevice, pMemoryProperties);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetPhysicalDeviceMultisamplePropertiesEXT(VkPhysicalDevice                            physicalDevice, VkSampleCountFlagBits                       samples, VkMultisamplePropertiesEXT*                 pMultisampleProperties) {
    using Fn = void (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, VkSampleCountFlagBits                       samples, VkMultisamplePropertiesEXT*                 pMultisampleProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceMultisamplePropertiesEXT"));
    return fn(physicalDevice, samples, pMultisampleProperties);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPhysicalDeviceOpticalFlowImageFormatsNV(VkPhysicalDevice                            physicalDevice, const VkOpticalFlowImageFormatInfoNV*       pOpticalFlowImageFormatInfo, uint32_t*                                   pFormatCount, VkOpticalFlowImageFormatPropertiesNV*       pImageFormatProperties) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, const VkOpticalFlowImageFormatInfoNV*       pOpticalFlowImageFormatInfo, uint32_t*                                   pFormatCount, VkOpticalFlowImageFormatPropertiesNV*       pImageFormatProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceOpticalFlowImageFormatsNV"));
    return fn(physicalDevice, pOpticalFlowImageFormatInfo, pFormatCount, pImageFormatProperties);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPhysicalDevicePresentRectanglesKHR(VkPhysicalDevice                            physicalDevice, VkSurfaceKHR                                surface, uint32_t*                                   pRectCount, VkRect2D*                                   pRects) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, VkSurfaceKHR                                surface, uint32_t*                                   pRectCount, VkRect2D*                                   pRects);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDevicePresentRectanglesKHR"));
    return fn(physicalDevice, surface, pRectCount, pRects);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetPhysicalDeviceProperties(VkPhysicalDevice                            physicalDevice, VkPhysicalDeviceProperties*                 pProperties) {
    using Fn = void (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, VkPhysicalDeviceProperties*                 pProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceProperties"));
    return fn(physicalDevice, pProperties);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetPhysicalDeviceProperties2(VkPhysicalDevice                            physicalDevice, VkPhysicalDeviceProperties2*                pProperties) {
    using Fn = void (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, VkPhysicalDeviceProperties2*                pProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceProperties2"));
    return fn(physicalDevice, pProperties);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetPhysicalDeviceProperties2KHR(VkPhysicalDevice                            physicalDevice, VkPhysicalDeviceProperties2*                pProperties) {
    using Fn = void (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, VkPhysicalDeviceProperties2*                pProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceProperties2KHR"));
    return fn(physicalDevice, pProperties);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetPhysicalDeviceQueueFamilyPerformanceQueryPassesKHR(VkPhysicalDevice                            physicalDevice, const VkQueryPoolPerformanceCreateInfoKHR*  pPerformanceQueryCreateInfo, uint32_t*                                   pNumPasses) {
    using Fn = void (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, const VkQueryPoolPerformanceCreateInfoKHR*  pPerformanceQueryCreateInfo, uint32_t*                                   pNumPasses);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceQueueFamilyPerformanceQueryPassesKHR"));
    return fn(physicalDevice, pPerformanceQueryCreateInfo, pNumPasses);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetPhysicalDeviceQueueFamilyProperties(VkPhysicalDevice                            physicalDevice, uint32_t*                                   pQueueFamilyPropertyCount, VkQueueFamilyProperties*                    pQueueFamilyProperties) {
    using Fn = void (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, uint32_t*                                   pQueueFamilyPropertyCount, VkQueueFamilyProperties*                    pQueueFamilyProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceQueueFamilyProperties"));
    return fn(physicalDevice, pQueueFamilyPropertyCount, pQueueFamilyProperties);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetPhysicalDeviceQueueFamilyProperties2(VkPhysicalDevice                            physicalDevice, uint32_t*                                   pQueueFamilyPropertyCount, VkQueueFamilyProperties2*                   pQueueFamilyProperties) {
    using Fn = void (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, uint32_t*                                   pQueueFamilyPropertyCount, VkQueueFamilyProperties2*                   pQueueFamilyProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceQueueFamilyProperties2"));
    return fn(physicalDevice, pQueueFamilyPropertyCount, pQueueFamilyProperties);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetPhysicalDeviceQueueFamilyProperties2KHR(VkPhysicalDevice                            physicalDevice, uint32_t*                                   pQueueFamilyPropertyCount, VkQueueFamilyProperties2*                   pQueueFamilyProperties) {
    using Fn = void (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, uint32_t*                                   pQueueFamilyPropertyCount, VkQueueFamilyProperties2*                   pQueueFamilyProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceQueueFamilyProperties2KHR"));
    return fn(physicalDevice, pQueueFamilyPropertyCount, pQueueFamilyProperties);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetPhysicalDeviceSparseImageFormatProperties(VkPhysicalDevice                            physicalDevice, VkFormat                                    format, VkImageType                                 type, VkSampleCountFlagBits                       samples, VkImageUsageFlags                           usage, VkImageTiling                               tiling, uint32_t*                                   pPropertyCount, VkSparseImageFormatProperties*              pProperties) {
    using Fn = void (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, VkFormat                                    format, VkImageType                                 type, VkSampleCountFlagBits                       samples, VkImageUsageFlags                           usage, VkImageTiling                               tiling, uint32_t*                                   pPropertyCount, VkSparseImageFormatProperties*              pProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceSparseImageFormatProperties"));
    return fn(physicalDevice, format, type, samples, usage, tiling, pPropertyCount, pProperties);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetPhysicalDeviceSparseImageFormatProperties2(VkPhysicalDevice                            physicalDevice, const VkPhysicalDeviceSparseImageFormatInfo2* pFormatInfo, uint32_t*                                   pPropertyCount, VkSparseImageFormatProperties2*             pProperties) {
    using Fn = void (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, const VkPhysicalDeviceSparseImageFormatInfo2* pFormatInfo, uint32_t*                                   pPropertyCount, VkSparseImageFormatProperties2*             pProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceSparseImageFormatProperties2"));
    return fn(physicalDevice, pFormatInfo, pPropertyCount, pProperties);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetPhysicalDeviceSparseImageFormatProperties2KHR(VkPhysicalDevice                            physicalDevice, const VkPhysicalDeviceSparseImageFormatInfo2* pFormatInfo, uint32_t*                                   pPropertyCount, VkSparseImageFormatProperties2*             pProperties) {
    using Fn = void (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, const VkPhysicalDeviceSparseImageFormatInfo2* pFormatInfo, uint32_t*                                   pPropertyCount, VkSparseImageFormatProperties2*             pProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceSparseImageFormatProperties2KHR"));
    return fn(physicalDevice, pFormatInfo, pPropertyCount, pProperties);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPhysicalDeviceSupportedFramebufferMixedSamplesCombinationsNV(VkPhysicalDevice                            physicalDevice, uint32_t*                                   pCombinationCount, VkFramebufferMixedSamplesCombinationNV*     pCombinations) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, uint32_t*                                   pCombinationCount, VkFramebufferMixedSamplesCombinationNV*     pCombinations);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceSupportedFramebufferMixedSamplesCombinationsNV"));
    return fn(physicalDevice, pCombinationCount, pCombinations);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPhysicalDeviceSurfaceCapabilities2EXT(VkPhysicalDevice                            physicalDevice, VkSurfaceKHR                                surface, VkSurfaceCapabilities2EXT*                  pSurfaceCapabilities) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, VkSurfaceKHR                                surface, VkSurfaceCapabilities2EXT*                  pSurfaceCapabilities);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceSurfaceCapabilities2EXT"));
    return fn(physicalDevice, surface, pSurfaceCapabilities);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPhysicalDeviceSurfaceCapabilities2KHR(VkPhysicalDevice                            physicalDevice, const VkPhysicalDeviceSurfaceInfo2KHR*      pSurfaceInfo, VkSurfaceCapabilities2KHR*                  pSurfaceCapabilities) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, const VkPhysicalDeviceSurfaceInfo2KHR*      pSurfaceInfo, VkSurfaceCapabilities2KHR*                  pSurfaceCapabilities);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceSurfaceCapabilities2KHR"));
    return fn(physicalDevice, pSurfaceInfo, pSurfaceCapabilities);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPhysicalDeviceSurfaceCapabilitiesKHR(VkPhysicalDevice                            physicalDevice, VkSurfaceKHR                                surface, VkSurfaceCapabilitiesKHR*                   pSurfaceCapabilities) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, VkSurfaceKHR                                surface, VkSurfaceCapabilitiesKHR*                   pSurfaceCapabilities);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceSurfaceCapabilitiesKHR"));
    return fn(physicalDevice, surface, pSurfaceCapabilities);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPhysicalDeviceSurfaceFormats2KHR(VkPhysicalDevice                            physicalDevice, const VkPhysicalDeviceSurfaceInfo2KHR*      pSurfaceInfo, uint32_t*                                   pSurfaceFormatCount, VkSurfaceFormat2KHR*                        pSurfaceFormats) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, const VkPhysicalDeviceSurfaceInfo2KHR*      pSurfaceInfo, uint32_t*                                   pSurfaceFormatCount, VkSurfaceFormat2KHR*                        pSurfaceFormats);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceSurfaceFormats2KHR"));
    return fn(physicalDevice, pSurfaceInfo, pSurfaceFormatCount, pSurfaceFormats);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPhysicalDeviceSurfaceFormatsKHR(VkPhysicalDevice                            physicalDevice, VkSurfaceKHR                                surface, uint32_t*                                   pSurfaceFormatCount, VkSurfaceFormatKHR*                         pSurfaceFormats) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, VkSurfaceKHR                                surface, uint32_t*                                   pSurfaceFormatCount, VkSurfaceFormatKHR*                         pSurfaceFormats);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceSurfaceFormatsKHR"));
    return fn(physicalDevice, surface, pSurfaceFormatCount, pSurfaceFormats);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPhysicalDeviceSurfacePresentModesKHR(VkPhysicalDevice                            physicalDevice, VkSurfaceKHR                                surface, uint32_t*                                   pPresentModeCount, VkPresentModeKHR*                           pPresentModes) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, VkSurfaceKHR                                surface, uint32_t*                                   pPresentModeCount, VkPresentModeKHR*                           pPresentModes);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceSurfacePresentModesKHR"));
    return fn(physicalDevice, surface, pPresentModeCount, pPresentModes);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPhysicalDeviceSurfaceSupportKHR(VkPhysicalDevice                            physicalDevice, uint32_t                                    queueFamilyIndex, VkSurfaceKHR                                surface, VkBool32*                                   pSupported) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, uint32_t                                    queueFamilyIndex, VkSurfaceKHR                                surface, VkBool32*                                   pSupported);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceSurfaceSupportKHR"));
    return fn(physicalDevice, queueFamilyIndex, surface, pSupported);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPhysicalDeviceToolProperties(VkPhysicalDevice                            physicalDevice, uint32_t*                                   pToolCount, VkPhysicalDeviceToolProperties*             pToolProperties) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, uint32_t*                                   pToolCount, VkPhysicalDeviceToolProperties*             pToolProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceToolProperties"));
    return fn(physicalDevice, pToolCount, pToolProperties);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPhysicalDeviceToolPropertiesEXT(VkPhysicalDevice                            physicalDevice, uint32_t*                                   pToolCount, VkPhysicalDeviceToolProperties*             pToolProperties) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, uint32_t*                                   pToolCount, VkPhysicalDeviceToolProperties*             pToolProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceToolPropertiesEXT"));
    return fn(physicalDevice, pToolCount, pToolProperties);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPhysicalDeviceVideoCapabilitiesKHR(VkPhysicalDevice                            physicalDevice, const VkVideoProfileInfoKHR*                pVideoProfile, VkVideoCapabilitiesKHR*                     pCapabilities) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, const VkVideoProfileInfoKHR*                pVideoProfile, VkVideoCapabilitiesKHR*                     pCapabilities);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceVideoCapabilitiesKHR"));
    return fn(physicalDevice, pVideoProfile, pCapabilities);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPhysicalDeviceVideoEncodeQualityLevelPropertiesKHR(VkPhysicalDevice                            physicalDevice, const VkPhysicalDeviceVideoEncodeQualityLevelInfoKHR* pQualityLevelInfo, VkVideoEncodeQualityLevelPropertiesKHR*     pQualityLevelProperties) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, const VkPhysicalDeviceVideoEncodeQualityLevelInfoKHR* pQualityLevelInfo, VkVideoEncodeQualityLevelPropertiesKHR*     pQualityLevelProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceVideoEncodeQualityLevelPropertiesKHR"));
    return fn(physicalDevice, pQualityLevelInfo, pQualityLevelProperties);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPhysicalDeviceVideoFormatPropertiesKHR(VkPhysicalDevice                            physicalDevice, const VkPhysicalDeviceVideoFormatInfoKHR*   pVideoFormatInfo, uint32_t*                                   pVideoFormatPropertyCount, VkVideoFormatPropertiesKHR*                 pVideoFormatProperties) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, const VkPhysicalDeviceVideoFormatInfoKHR*   pVideoFormatInfo, uint32_t*                                   pVideoFormatPropertyCount, VkVideoFormatPropertiesKHR*                 pVideoFormatProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkGetPhysicalDeviceVideoFormatPropertiesKHR"));
    return fn(physicalDevice, pVideoFormatInfo, pVideoFormatPropertyCount, pVideoFormatProperties);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPipelineCacheData(VkDevice                                    device, VkPipelineCache                             pipelineCache, size_t*                                     pDataSize, void*                                       pData) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkPipelineCache                             pipelineCache, size_t*                                     pDataSize, void*                                       pData);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetPipelineCacheData"));
    return fn(device, pipelineCache, pDataSize, pData);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPipelineExecutableInternalRepresentationsKHR(VkDevice                                    device, const VkPipelineExecutableInfoKHR*          pExecutableInfo, uint32_t*                                   pInternalRepresentationCount, VkPipelineExecutableInternalRepresentationKHR* pInternalRepresentations) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkPipelineExecutableInfoKHR*          pExecutableInfo, uint32_t*                                   pInternalRepresentationCount, VkPipelineExecutableInternalRepresentationKHR* pInternalRepresentations);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetPipelineExecutableInternalRepresentationsKHR"));
    return fn(device, pExecutableInfo, pInternalRepresentationCount, pInternalRepresentations);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPipelineExecutablePropertiesKHR(VkDevice                                    device, const VkPipelineInfoKHR*                    pPipelineInfo, uint32_t*                                   pExecutableCount, VkPipelineExecutablePropertiesKHR*          pProperties) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkPipelineInfoKHR*                    pPipelineInfo, uint32_t*                                   pExecutableCount, VkPipelineExecutablePropertiesKHR*          pProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetPipelineExecutablePropertiesKHR"));
    return fn(device, pPipelineInfo, pExecutableCount, pProperties);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPipelineExecutableStatisticsKHR(VkDevice                                    device, const VkPipelineExecutableInfoKHR*          pExecutableInfo, uint32_t*                                   pStatisticCount, VkPipelineExecutableStatisticKHR*           pStatistics) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkPipelineExecutableInfoKHR*          pExecutableInfo, uint32_t*                                   pStatisticCount, VkPipelineExecutableStatisticKHR*           pStatistics);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetPipelineExecutableStatisticsKHR"));
    return fn(device, pExecutableInfo, pStatisticCount, pStatistics);
}
extern "C" VKAPI_ATTR VkDeviceAddress VKAPI_CALL vkGetPipelineIndirectDeviceAddressNV(VkDevice                                    device, const VkPipelineIndirectDeviceAddressInfoNV* pInfo) {
    using Fn = VkDeviceAddress (VKAPI_CALL *)(VkDevice                                    device, const VkPipelineIndirectDeviceAddressInfoNV* pInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetPipelineIndirectDeviceAddressNV"));
    return fn(device, pInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetPipelineIndirectMemoryRequirementsNV(VkDevice                                    device, const VkComputePipelineCreateInfo*          pCreateInfo, VkMemoryRequirements2*                      pMemoryRequirements) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, const VkComputePipelineCreateInfo*          pCreateInfo, VkMemoryRequirements2*                      pMemoryRequirements);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetPipelineIndirectMemoryRequirementsNV"));
    return fn(device, pCreateInfo, pMemoryRequirements);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetPipelinePropertiesEXT(VkDevice                                    device, const VkPipelineInfoEXT*                    pPipelineInfo, VkBaseOutStructure*                         pPipelineProperties) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkPipelineInfoEXT*                    pPipelineInfo, VkBaseOutStructure*                         pPipelineProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetPipelinePropertiesEXT"));
    return fn(device, pPipelineInfo, pPipelineProperties);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetPrivateData(VkDevice                                    device, VkObjectType                                objectType, uint64_t                                    objectHandle, VkPrivateDataSlot                           privateDataSlot, uint64_t*                                   pData) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkObjectType                                objectType, uint64_t                                    objectHandle, VkPrivateDataSlot                           privateDataSlot, uint64_t*                                   pData);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetPrivateData"));
    return fn(device, objectType, objectHandle, privateDataSlot, pData);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetPrivateDataEXT(VkDevice                                    device, VkObjectType                                objectType, uint64_t                                    objectHandle, VkPrivateDataSlot                           privateDataSlot, uint64_t*                                   pData) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkObjectType                                objectType, uint64_t                                    objectHandle, VkPrivateDataSlot                           privateDataSlot, uint64_t*                                   pData);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetPrivateDataEXT"));
    return fn(device, objectType, objectHandle, privateDataSlot, pData);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetQueryPoolResults(VkDevice                                    device, VkQueryPool                                 queryPool, uint32_t                                    firstQuery, uint32_t                                    queryCount, size_t                                      dataSize, void*                                       pData, VkDeviceSize                                stride, VkQueryResultFlags                          flags) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkQueryPool                                 queryPool, uint32_t                                    firstQuery, uint32_t                                    queryCount, size_t                                      dataSize, void*                                       pData, VkDeviceSize                                stride, VkQueryResultFlags                          flags);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetQueryPoolResults"));
    return fn(device, queryPool, firstQuery, queryCount, dataSize, pData, stride, flags);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetQueueCheckpointData2NV(VkQueue                                     queue, uint32_t*                                   pCheckpointDataCount, VkCheckpointData2NV*                        pCheckpointData) {
    using Fn = void (VKAPI_CALL *)(VkQueue                                     queue, uint32_t*                                   pCheckpointDataCount, VkCheckpointData2NV*                        pCheckpointData);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(queue), "vkGetQueueCheckpointData2NV"));
    return fn(queue, pCheckpointDataCount, pCheckpointData);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetQueueCheckpointDataNV(VkQueue                                     queue, uint32_t*                                   pCheckpointDataCount, VkCheckpointDataNV*                         pCheckpointData) {
    using Fn = void (VKAPI_CALL *)(VkQueue                                     queue, uint32_t*                                   pCheckpointDataCount, VkCheckpointDataNV*                         pCheckpointData);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(queue), "vkGetQueueCheckpointDataNV"));
    return fn(queue, pCheckpointDataCount, pCheckpointData);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetRayTracingCaptureReplayShaderGroupHandlesKHR(VkDevice                                    device, VkPipeline                                  pipeline, uint32_t                                    firstGroup, uint32_t                                    groupCount, size_t                                      dataSize, void*                                       pData) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkPipeline                                  pipeline, uint32_t                                    firstGroup, uint32_t                                    groupCount, size_t                                      dataSize, void*                                       pData);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetRayTracingCaptureReplayShaderGroupHandlesKHR"));
    return fn(device, pipeline, firstGroup, groupCount, dataSize, pData);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetRayTracingShaderGroupHandlesKHR(VkDevice                                    device, VkPipeline                                  pipeline, uint32_t                                    firstGroup, uint32_t                                    groupCount, size_t                                      dataSize, void*                                       pData) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkPipeline                                  pipeline, uint32_t                                    firstGroup, uint32_t                                    groupCount, size_t                                      dataSize, void*                                       pData);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetRayTracingShaderGroupHandlesKHR"));
    return fn(device, pipeline, firstGroup, groupCount, dataSize, pData);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetRayTracingShaderGroupHandlesNV(VkDevice                                    device, VkPipeline                                  pipeline, uint32_t                                    firstGroup, uint32_t                                    groupCount, size_t                                      dataSize, void*                                       pData) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkPipeline                                  pipeline, uint32_t                                    firstGroup, uint32_t                                    groupCount, size_t                                      dataSize, void*                                       pData);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetRayTracingShaderGroupHandlesNV"));
    return fn(device, pipeline, firstGroup, groupCount, dataSize, pData);
}
extern "C" VKAPI_ATTR VkDeviceSize VKAPI_CALL vkGetRayTracingShaderGroupStackSizeKHR(VkDevice                                    device, VkPipeline                                  pipeline, uint32_t                                    group, VkShaderGroupShaderKHR                      groupShader) {
    using Fn = VkDeviceSize (VKAPI_CALL *)(VkDevice                                    device, VkPipeline                                  pipeline, uint32_t                                    group, VkShaderGroupShaderKHR                      groupShader);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetRayTracingShaderGroupStackSizeKHR"));
    return fn(device, pipeline, group, groupShader);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetRefreshCycleDurationGOOGLE(VkDevice                                    device, VkSwapchainKHR                              swapchain, VkRefreshCycleDurationGOOGLE*               pDisplayTimingProperties) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkSwapchainKHR                              swapchain, VkRefreshCycleDurationGOOGLE*               pDisplayTimingProperties);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetRefreshCycleDurationGOOGLE"));
    return fn(device, swapchain, pDisplayTimingProperties);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetRenderAreaGranularity(VkDevice                                    device, VkRenderPass                                renderPass, VkExtent2D*                                 pGranularity) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkRenderPass                                renderPass, VkExtent2D*                                 pGranularity);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetRenderAreaGranularity"));
    return fn(device, renderPass, pGranularity);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetRenderingAreaGranularityKHR(VkDevice                                    device, const VkRenderingAreaInfoKHR*               pRenderingAreaInfo, VkExtent2D*                                 pGranularity) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, const VkRenderingAreaInfoKHR*               pRenderingAreaInfo, VkExtent2D*                                 pGranularity);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetRenderingAreaGranularityKHR"));
    return fn(device, pRenderingAreaInfo, pGranularity);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetSamplerOpaqueCaptureDescriptorDataEXT(VkDevice                                    device, const VkSamplerCaptureDescriptorDataInfoEXT* pInfo, void*                                       pData) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkSamplerCaptureDescriptorDataInfoEXT* pInfo, void*                                       pData);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetSamplerOpaqueCaptureDescriptorDataEXT"));
    return fn(device, pInfo, pData);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetSemaphoreCounterValue(VkDevice                                    device, VkSemaphore                                 semaphore, uint64_t*                                   pValue) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkSemaphore                                 semaphore, uint64_t*                                   pValue);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetSemaphoreCounterValue"));
    return fn(device, semaphore, pValue);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetSemaphoreCounterValueKHR(VkDevice                                    device, VkSemaphore                                 semaphore, uint64_t*                                   pValue) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkSemaphore                                 semaphore, uint64_t*                                   pValue);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetSemaphoreCounterValueKHR"));
    return fn(device, semaphore, pValue);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetSemaphoreFdKHR(VkDevice                                    device, const VkSemaphoreGetFdInfoKHR*              pGetFdInfo, int*                                        pFd) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkSemaphoreGetFdInfoKHR*              pGetFdInfo, int*                                        pFd);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetSemaphoreFdKHR"));
    return fn(device, pGetFdInfo, pFd);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetShaderBinaryDataEXT(VkDevice                                    device, VkShaderEXT                                 shader, size_t*                                     pDataSize, void*                                       pData) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkShaderEXT                                 shader, size_t*                                     pDataSize, void*                                       pData);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetShaderBinaryDataEXT"));
    return fn(device, shader, pDataSize, pData);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetShaderInfoAMD(VkDevice                                    device, VkPipeline                                  pipeline, VkShaderStageFlagBits                       shaderStage, VkShaderInfoTypeAMD                         infoType, size_t*                                     pInfoSize, void*                                       pInfo) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkPipeline                                  pipeline, VkShaderStageFlagBits                       shaderStage, VkShaderInfoTypeAMD                         infoType, size_t*                                     pInfoSize, void*                                       pInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetShaderInfoAMD"));
    return fn(device, pipeline, shaderStage, infoType, pInfoSize, pInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetShaderModuleCreateInfoIdentifierEXT(VkDevice                                    device, const VkShaderModuleCreateInfo*             pCreateInfo, VkShaderModuleIdentifierEXT*                pIdentifier) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, const VkShaderModuleCreateInfo*             pCreateInfo, VkShaderModuleIdentifierEXT*                pIdentifier);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetShaderModuleCreateInfoIdentifierEXT"));
    return fn(device, pCreateInfo, pIdentifier);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkGetShaderModuleIdentifierEXT(VkDevice                                    device, VkShaderModule                              shaderModule, VkShaderModuleIdentifierEXT*                pIdentifier) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkShaderModule                              shaderModule, VkShaderModuleIdentifierEXT*                pIdentifier);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetShaderModuleIdentifierEXT"));
    return fn(device, shaderModule, pIdentifier);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetSwapchainCounterEXT(VkDevice                                    device, VkSwapchainKHR                              swapchain, VkSurfaceCounterFlagBitsEXT                 counter, uint64_t*                                   pCounterValue) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkSwapchainKHR                              swapchain, VkSurfaceCounterFlagBitsEXT                 counter, uint64_t*                                   pCounterValue);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetSwapchainCounterEXT"));
    return fn(device, swapchain, counter, pCounterValue);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetSwapchainImagesKHR(VkDevice                                    device, VkSwapchainKHR                              swapchain, uint32_t*                                   pSwapchainImageCount, VkImage*                                    pSwapchainImages) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkSwapchainKHR                              swapchain, uint32_t*                                   pSwapchainImageCount, VkImage*                                    pSwapchainImages);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetSwapchainImagesKHR"));
    return fn(device, swapchain, pSwapchainImageCount, pSwapchainImages);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetSwapchainStatusKHR(VkDevice                                    device, VkSwapchainKHR                              swapchain) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkSwapchainKHR                              swapchain);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetSwapchainStatusKHR"));
    return fn(device, swapchain);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetValidationCacheDataEXT(VkDevice                                    device, VkValidationCacheEXT                        validationCache, size_t*                                     pDataSize, void*                                       pData) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkValidationCacheEXT                        validationCache, size_t*                                     pDataSize, void*                                       pData);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetValidationCacheDataEXT"));
    return fn(device, validationCache, pDataSize, pData);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkGetVideoSessionMemoryRequirementsKHR(VkDevice                                    device, VkVideoSessionKHR                           videoSession, uint32_t*                                   pMemoryRequirementsCount, VkVideoSessionMemoryRequirementsKHR*        pMemoryRequirements) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkVideoSessionKHR                           videoSession, uint32_t*                                   pMemoryRequirementsCount, VkVideoSessionMemoryRequirementsKHR*        pMemoryRequirements);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkGetVideoSessionMemoryRequirementsKHR"));
    return fn(device, videoSession, pMemoryRequirementsCount, pMemoryRequirements);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkImportFenceFdKHR(VkDevice                                    device, const VkImportFenceFdInfoKHR*               pImportFenceFdInfo) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkImportFenceFdInfoKHR*               pImportFenceFdInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkImportFenceFdKHR"));
    return fn(device, pImportFenceFdInfo);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkImportSemaphoreFdKHR(VkDevice                                    device, const VkImportSemaphoreFdInfoKHR*           pImportSemaphoreFdInfo) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkImportSemaphoreFdInfoKHR*           pImportSemaphoreFdInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkImportSemaphoreFdKHR"));
    return fn(device, pImportSemaphoreFdInfo);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkInitializePerformanceApiINTEL(VkDevice                                    device, const VkInitializePerformanceApiInfoINTEL*  pInitializeInfo) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkInitializePerformanceApiInfoINTEL*  pInitializeInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkInitializePerformanceApiINTEL"));
    return fn(device, pInitializeInfo);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkInvalidateMappedMemoryRanges(VkDevice                                    device, uint32_t                                    memoryRangeCount, const VkMappedMemoryRange*                  pMemoryRanges) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, uint32_t                                    memoryRangeCount, const VkMappedMemoryRange*                  pMemoryRanges);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkInvalidateMappedMemoryRanges"));
    return fn(device, memoryRangeCount, pMemoryRanges);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkLatencySleepNV(VkDevice                                    device, VkSwapchainKHR                              swapchain, const VkLatencySleepInfoNV*                 pSleepInfo) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkSwapchainKHR                              swapchain, const VkLatencySleepInfoNV*                 pSleepInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkLatencySleepNV"));
    return fn(device, swapchain, pSleepInfo);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkMapMemory(VkDevice                                    device, VkDeviceMemory                              memory, VkDeviceSize                                offset, VkDeviceSize                                size, VkMemoryMapFlags                            flags, void**                                      ppData) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkDeviceMemory                              memory, VkDeviceSize                                offset, VkDeviceSize                                size, VkMemoryMapFlags                            flags, void**                                      ppData);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkMapMemory"));
    return fn(device, memory, offset, size, flags, ppData);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkMapMemory2KHR(VkDevice                                    device, const VkMemoryMapInfoKHR*                   pMemoryMapInfo, void**                                      ppData) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkMemoryMapInfoKHR*                   pMemoryMapInfo, void**                                      ppData);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkMapMemory2KHR"));
    return fn(device, pMemoryMapInfo, ppData);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkMergePipelineCaches(VkDevice                                    device, VkPipelineCache                             dstCache, uint32_t                                    srcCacheCount, const VkPipelineCache*                      pSrcCaches) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkPipelineCache                             dstCache, uint32_t                                    srcCacheCount, const VkPipelineCache*                      pSrcCaches);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkMergePipelineCaches"));
    return fn(device, dstCache, srcCacheCount, pSrcCaches);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkMergeValidationCachesEXT(VkDevice                                    device, VkValidationCacheEXT                        dstCache, uint32_t                                    srcCacheCount, const VkValidationCacheEXT*                 pSrcCaches) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkValidationCacheEXT                        dstCache, uint32_t                                    srcCacheCount, const VkValidationCacheEXT*                 pSrcCaches);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkMergeValidationCachesEXT"));
    return fn(device, dstCache, srcCacheCount, pSrcCaches);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkQueueBeginDebugUtilsLabelEXT(VkQueue                                     queue, const VkDebugUtilsLabelEXT*                 pLabelInfo) {
    using Fn = void (VKAPI_CALL *)(VkQueue                                     queue, const VkDebugUtilsLabelEXT*                 pLabelInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(queue), "vkQueueBeginDebugUtilsLabelEXT"));
    return fn(queue, pLabelInfo);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkQueueBindSparse(VkQueue                                     queue, uint32_t                                    bindInfoCount, const VkBindSparseInfo*                     pBindInfo, VkFence                                     fence) {
    using Fn = VkResult (VKAPI_CALL *)(VkQueue                                     queue, uint32_t                                    bindInfoCount, const VkBindSparseInfo*                     pBindInfo, VkFence                                     fence);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(queue), "vkQueueBindSparse"));
    return fn(queue, bindInfoCount, pBindInfo, fence);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkQueueEndDebugUtilsLabelEXT(VkQueue                                     queue) {
    using Fn = void (VKAPI_CALL *)(VkQueue                                     queue);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(queue), "vkQueueEndDebugUtilsLabelEXT"));
    return fn(queue);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkQueueInsertDebugUtilsLabelEXT(VkQueue                                     queue, const VkDebugUtilsLabelEXT*                 pLabelInfo) {
    using Fn = void (VKAPI_CALL *)(VkQueue                                     queue, const VkDebugUtilsLabelEXT*                 pLabelInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(queue), "vkQueueInsertDebugUtilsLabelEXT"));
    return fn(queue, pLabelInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkQueueNotifyOutOfBandNV(VkQueue                                     queue, const VkOutOfBandQueueTypeInfoNV*           pQueueTypeInfo) {
    using Fn = void (VKAPI_CALL *)(VkQueue                                     queue, const VkOutOfBandQueueTypeInfoNV*           pQueueTypeInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(queue), "vkQueueNotifyOutOfBandNV"));
    return fn(queue, pQueueTypeInfo);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkQueuePresentKHR(VkQueue                                     queue, const VkPresentInfoKHR*                     pPresentInfo) {
    using Fn = VkResult (VKAPI_CALL *)(VkQueue                                     queue, const VkPresentInfoKHR*                     pPresentInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(queue), "vkQueuePresentKHR"));
    return fn(queue, pPresentInfo);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkQueueSetPerformanceConfigurationINTEL(VkQueue                                     queue, VkPerformanceConfigurationINTEL             configuration) {
    using Fn = VkResult (VKAPI_CALL *)(VkQueue                                     queue, VkPerformanceConfigurationINTEL             configuration);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(queue), "vkQueueSetPerformanceConfigurationINTEL"));
    return fn(queue, configuration);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkQueueSubmit(VkQueue                                     queue, uint32_t                                    submitCount, const VkSubmitInfo*                         pSubmits, VkFence                                     fence) {
    using Fn = VkResult (VKAPI_CALL *)(VkQueue                                     queue, uint32_t                                    submitCount, const VkSubmitInfo*                         pSubmits, VkFence                                     fence);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(queue), "vkQueueSubmit"));
    return fn(queue, submitCount, pSubmits, fence);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkQueueSubmit2(VkQueue                                     queue, uint32_t                                    submitCount, const VkSubmitInfo2*                        pSubmits, VkFence                                     fence) {
    using Fn = VkResult (VKAPI_CALL *)(VkQueue                                     queue, uint32_t                                    submitCount, const VkSubmitInfo2*                        pSubmits, VkFence                                     fence);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(queue), "vkQueueSubmit2"));
    return fn(queue, submitCount, pSubmits, fence);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkQueueSubmit2KHR(VkQueue                                     queue, uint32_t                                    submitCount, const VkSubmitInfo2*                        pSubmits, VkFence                                     fence) {
    using Fn = VkResult (VKAPI_CALL *)(VkQueue                                     queue, uint32_t                                    submitCount, const VkSubmitInfo2*                        pSubmits, VkFence                                     fence);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(queue), "vkQueueSubmit2KHR"));
    return fn(queue, submitCount, pSubmits, fence);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkQueueWaitIdle(VkQueue                                     queue) {
    using Fn = VkResult (VKAPI_CALL *)(VkQueue                                     queue);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(queue), "vkQueueWaitIdle"));
    return fn(queue);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkRegisterDeviceEventEXT(VkDevice                                    device, const VkDeviceEventInfoEXT*                 pDeviceEventInfo, const VkAllocationCallbacks*                pAllocator, VkFence*                                    pFence) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkDeviceEventInfoEXT*                 pDeviceEventInfo, const VkAllocationCallbacks*                pAllocator, VkFence*                                    pFence);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkRegisterDeviceEventEXT"));
    return fn(device, pDeviceEventInfo, pAllocator, pFence);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkRegisterDisplayEventEXT(VkDevice                                    device, VkDisplayKHR                                display, const VkDisplayEventInfoEXT*                pDisplayEventInfo, const VkAllocationCallbacks*                pAllocator, VkFence*                                    pFence) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkDisplayKHR                                display, const VkDisplayEventInfoEXT*                pDisplayEventInfo, const VkAllocationCallbacks*                pAllocator, VkFence*                                    pFence);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkRegisterDisplayEventEXT"));
    return fn(device, display, pDisplayEventInfo, pAllocator, pFence);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkReleaseDisplayEXT(VkPhysicalDevice                            physicalDevice, VkDisplayKHR                                display) {
    using Fn = VkResult (VKAPI_CALL *)(VkPhysicalDevice                            physicalDevice, VkDisplayKHR                                display);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(physicalDevice), "vkReleaseDisplayEXT"));
    return fn(physicalDevice, display);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkReleasePerformanceConfigurationINTEL(VkDevice                                    device, VkPerformanceConfigurationINTEL             configuration) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkPerformanceConfigurationINTEL             configuration);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkReleasePerformanceConfigurationINTEL"));
    return fn(device, configuration);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkReleaseProfilingLockKHR(VkDevice                                    device) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkReleaseProfilingLockKHR"));
    return fn(device);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkReleaseSwapchainImagesEXT(VkDevice                                    device, const VkReleaseSwapchainImagesInfoEXT*      pReleaseInfo) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkReleaseSwapchainImagesInfoEXT*      pReleaseInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkReleaseSwapchainImagesEXT"));
    return fn(device, pReleaseInfo);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkResetCommandBuffer(VkCommandBuffer                             commandBuffer, VkCommandBufferResetFlags                   flags) {
    using Fn = VkResult (VKAPI_CALL *)(VkCommandBuffer                             commandBuffer, VkCommandBufferResetFlags                   flags);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(commandBuffer), "vkResetCommandBuffer"));
    return fn(commandBuffer, flags);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkResetCommandPool(VkDevice                                    device, VkCommandPool                               commandPool, VkCommandPoolResetFlags                     flags) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkCommandPool                               commandPool, VkCommandPoolResetFlags                     flags);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkResetCommandPool"));
    return fn(device, commandPool, flags);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkResetDescriptorPool(VkDevice                                    device, VkDescriptorPool                            descriptorPool, VkDescriptorPoolResetFlags                  flags) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkDescriptorPool                            descriptorPool, VkDescriptorPoolResetFlags                  flags);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkResetDescriptorPool"));
    return fn(device, descriptorPool, flags);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkResetEvent(VkDevice                                    device, VkEvent                                     event) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkEvent                                     event);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkResetEvent"));
    return fn(device, event);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkResetFences(VkDevice                                    device, uint32_t                                    fenceCount, const VkFence*                              pFences) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, uint32_t                                    fenceCount, const VkFence*                              pFences);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkResetFences"));
    return fn(device, fenceCount, pFences);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkResetQueryPool(VkDevice                                    device, VkQueryPool                                 queryPool, uint32_t                                    firstQuery, uint32_t                                    queryCount) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkQueryPool                                 queryPool, uint32_t                                    firstQuery, uint32_t                                    queryCount);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkResetQueryPool"));
    return fn(device, queryPool, firstQuery, queryCount);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkResetQueryPoolEXT(VkDevice                                    device, VkQueryPool                                 queryPool, uint32_t                                    firstQuery, uint32_t                                    queryCount) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkQueryPool                                 queryPool, uint32_t                                    firstQuery, uint32_t                                    queryCount);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkResetQueryPoolEXT"));
    return fn(device, queryPool, firstQuery, queryCount);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkSetDebugUtilsObjectNameEXT(VkDevice                                    device, const VkDebugUtilsObjectNameInfoEXT*        pNameInfo) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkDebugUtilsObjectNameInfoEXT*        pNameInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkSetDebugUtilsObjectNameEXT"));
    return fn(device, pNameInfo);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkSetDebugUtilsObjectTagEXT(VkDevice                                    device, const VkDebugUtilsObjectTagInfoEXT*         pTagInfo) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkDebugUtilsObjectTagInfoEXT*         pTagInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkSetDebugUtilsObjectTagEXT"));
    return fn(device, pTagInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkSetDeviceMemoryPriorityEXT(VkDevice                                    device, VkDeviceMemory                              memory, float                                       priority) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkDeviceMemory                              memory, float                                       priority);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkSetDeviceMemoryPriorityEXT"));
    return fn(device, memory, priority);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkSetEvent(VkDevice                                    device, VkEvent                                     event) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkEvent                                     event);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkSetEvent"));
    return fn(device, event);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkSetHdrMetadataEXT(VkDevice                                    device, uint32_t                                    swapchainCount, const VkSwapchainKHR*                       pSwapchains, const VkHdrMetadataEXT*                     pMetadata) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, uint32_t                                    swapchainCount, const VkSwapchainKHR*                       pSwapchains, const VkHdrMetadataEXT*                     pMetadata);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkSetHdrMetadataEXT"));
    return fn(device, swapchainCount, pSwapchains, pMetadata);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkSetLatencyMarkerNV(VkDevice                                    device, VkSwapchainKHR                              swapchain, const VkSetLatencyMarkerInfoNV*             pLatencyMarkerInfo) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkSwapchainKHR                              swapchain, const VkSetLatencyMarkerInfoNV*             pLatencyMarkerInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkSetLatencyMarkerNV"));
    return fn(device, swapchain, pLatencyMarkerInfo);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkSetLatencySleepModeNV(VkDevice                                    device, VkSwapchainKHR                              swapchain, const VkLatencySleepModeInfoNV*             pSleepModeInfo) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkSwapchainKHR                              swapchain, const VkLatencySleepModeInfoNV*             pSleepModeInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkSetLatencySleepModeNV"));
    return fn(device, swapchain, pSleepModeInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkSetLocalDimmingAMD(VkDevice                                    device, VkSwapchainKHR                              swapChain, VkBool32                                    localDimmingEnable) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkSwapchainKHR                              swapChain, VkBool32                                    localDimmingEnable);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkSetLocalDimmingAMD"));
    return fn(device, swapChain, localDimmingEnable);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkSetPrivateData(VkDevice                                    device, VkObjectType                                objectType, uint64_t                                    objectHandle, VkPrivateDataSlot                           privateDataSlot, uint64_t                                    data) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkObjectType                                objectType, uint64_t                                    objectHandle, VkPrivateDataSlot                           privateDataSlot, uint64_t                                    data);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkSetPrivateData"));
    return fn(device, objectType, objectHandle, privateDataSlot, data);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkSetPrivateDataEXT(VkDevice                                    device, VkObjectType                                objectType, uint64_t                                    objectHandle, VkPrivateDataSlot                           privateDataSlot, uint64_t                                    data) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkObjectType                                objectType, uint64_t                                    objectHandle, VkPrivateDataSlot                           privateDataSlot, uint64_t                                    data);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkSetPrivateDataEXT"));
    return fn(device, objectType, objectHandle, privateDataSlot, data);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkSignalSemaphore(VkDevice                                    device, const VkSemaphoreSignalInfo*                pSignalInfo) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkSemaphoreSignalInfo*                pSignalInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkSignalSemaphore"));
    return fn(device, pSignalInfo);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkSignalSemaphoreKHR(VkDevice                                    device, const VkSemaphoreSignalInfo*                pSignalInfo) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkSemaphoreSignalInfo*                pSignalInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkSignalSemaphoreKHR"));
    return fn(device, pSignalInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkSubmitDebugUtilsMessageEXT(VkInstance                                  instance, VkDebugUtilsMessageSeverityFlagBitsEXT      messageSeverity, VkDebugUtilsMessageTypeFlagsEXT             messageTypes, const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData) {
    using Fn = void (VKAPI_CALL *)(VkInstance                                  instance, VkDebugUtilsMessageSeverityFlagBitsEXT      messageSeverity, VkDebugUtilsMessageTypeFlagsEXT             messageTypes, const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(instance), "vkSubmitDebugUtilsMessageEXT"));
    return fn(instance, messageSeverity, messageTypes, pCallbackData);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkTransitionImageLayoutEXT(VkDevice                                    device, uint32_t                                    transitionCount, const VkHostImageLayoutTransitionInfoEXT*   pTransitions) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, uint32_t                                    transitionCount, const VkHostImageLayoutTransitionInfoEXT*   pTransitions);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkTransitionImageLayoutEXT"));
    return fn(device, transitionCount, pTransitions);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkTrimCommandPool(VkDevice                                    device, VkCommandPool                               commandPool, VkCommandPoolTrimFlags                      flags) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkCommandPool                               commandPool, VkCommandPoolTrimFlags                      flags);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkTrimCommandPool"));
    return fn(device, commandPool, flags);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkTrimCommandPoolKHR(VkDevice                                    device, VkCommandPool                               commandPool, VkCommandPoolTrimFlags                      flags) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkCommandPool                               commandPool, VkCommandPoolTrimFlags                      flags);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkTrimCommandPoolKHR"));
    return fn(device, commandPool, flags);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkUninitializePerformanceApiINTEL(VkDevice                                    device) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkUninitializePerformanceApiINTEL"));
    return fn(device);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkUnmapMemory(VkDevice                                    device, VkDeviceMemory                              memory) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkDeviceMemory                              memory);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkUnmapMemory"));
    return fn(device, memory);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkUnmapMemory2KHR(VkDevice                                    device, const VkMemoryUnmapInfoKHR*                 pMemoryUnmapInfo) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkMemoryUnmapInfoKHR*                 pMemoryUnmapInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkUnmapMemory2KHR"));
    return fn(device, pMemoryUnmapInfo);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkUpdateDescriptorSetWithTemplate(VkDevice                                    device, VkDescriptorSet                             descriptorSet, VkDescriptorUpdateTemplate                  descriptorUpdateTemplate, const void*                                 pData) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkDescriptorSet                             descriptorSet, VkDescriptorUpdateTemplate                  descriptorUpdateTemplate, const void*                                 pData);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkUpdateDescriptorSetWithTemplate"));
    return fn(device, descriptorSet, descriptorUpdateTemplate, pData);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkUpdateDescriptorSetWithTemplateKHR(VkDevice                                    device, VkDescriptorSet                             descriptorSet, VkDescriptorUpdateTemplate                  descriptorUpdateTemplate, const void*                                 pData) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, VkDescriptorSet                             descriptorSet, VkDescriptorUpdateTemplate                  descriptorUpdateTemplate, const void*                                 pData);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkUpdateDescriptorSetWithTemplateKHR"));
    return fn(device, descriptorSet, descriptorUpdateTemplate, pData);
}
extern "C" VKAPI_ATTR void VKAPI_CALL vkUpdateDescriptorSets(VkDevice                                    device, uint32_t                                    descriptorWriteCount, const VkWriteDescriptorSet*                 pDescriptorWrites, uint32_t                                    descriptorCopyCount, const VkCopyDescriptorSet*                  pDescriptorCopies) {
    using Fn = void (VKAPI_CALL *)(VkDevice                                    device, uint32_t                                    descriptorWriteCount, const VkWriteDescriptorSet*                 pDescriptorWrites, uint32_t                                    descriptorCopyCount, const VkCopyDescriptorSet*                  pDescriptorCopies);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkUpdateDescriptorSets"));
    return fn(device, descriptorWriteCount, pDescriptorWrites, descriptorCopyCount, pDescriptorCopies);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkUpdateVideoSessionParametersKHR(VkDevice                                    device, VkVideoSessionParametersKHR                 videoSessionParameters, const VkVideoSessionParametersUpdateInfoKHR* pUpdateInfo) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkVideoSessionParametersKHR                 videoSessionParameters, const VkVideoSessionParametersUpdateInfoKHR* pUpdateInfo);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkUpdateVideoSessionParametersKHR"));
    return fn(device, videoSessionParameters, pUpdateInfo);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkWaitForFences(VkDevice                                    device, uint32_t                                    fenceCount, const VkFence*                              pFences, VkBool32                                    waitAll, uint64_t                                    timeout) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, uint32_t                                    fenceCount, const VkFence*                              pFences, VkBool32                                    waitAll, uint64_t                                    timeout);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkWaitForFences"));
    return fn(device, fenceCount, pFences, waitAll, timeout);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkWaitForPresentKHR(VkDevice                                    device, VkSwapchainKHR                              swapchain, uint64_t                                    presentId, uint64_t                                    timeout) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, VkSwapchainKHR                              swapchain, uint64_t                                    presentId, uint64_t                                    timeout);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkWaitForPresentKHR"));
    return fn(device, swapchain, presentId, timeout);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkWaitSemaphores(VkDevice                                    device, const VkSemaphoreWaitInfo*                  pWaitInfo, uint64_t                                    timeout) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkSemaphoreWaitInfo*                  pWaitInfo, uint64_t                                    timeout);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkWaitSemaphores"));
    return fn(device, pWaitInfo, timeout);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkWaitSemaphoresKHR(VkDevice                                    device, const VkSemaphoreWaitInfo*                  pWaitInfo, uint64_t                                    timeout) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, const VkSemaphoreWaitInfo*                  pWaitInfo, uint64_t                                    timeout);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkWaitSemaphoresKHR"));
    return fn(device, pWaitInfo, timeout);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkWriteAccelerationStructuresPropertiesKHR(VkDevice                                    device, uint32_t                                    accelerationStructureCount, const VkAccelerationStructureKHR*           pAccelerationStructures, VkQueryType                                 queryType, size_t                                      dataSize, void*                                       pData, size_t                                      stride) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, uint32_t                                    accelerationStructureCount, const VkAccelerationStructureKHR*           pAccelerationStructures, VkQueryType                                 queryType, size_t                                      dataSize, void*                                       pData, size_t                                      stride);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkWriteAccelerationStructuresPropertiesKHR"));
    return fn(device, accelerationStructureCount, pAccelerationStructures, queryType, dataSize, pData, stride);
}
extern "C" VKAPI_ATTR VkResult VKAPI_CALL vkWriteMicromapsPropertiesEXT(VkDevice                                    device, uint32_t                                    micromapCount, const VkMicromapEXT*                        pMicromaps, VkQueryType                                 queryType, size_t                                      dataSize, void*                                       pData, size_t                                      stride) {
    using Fn = VkResult (VKAPI_CALL *)(VkDevice                                    device, uint32_t                                    micromapCount, const VkMicromapEXT*                        pMicromaps, VkQueryType                                 queryType, size_t                                      dataSize, void*                                       pData, size_t                                      stride);
    static Fn fn = reinterpret_cast<Fn>(resolve(reinterpret_cast<VkInstance>(device), "vkWriteMicromapsPropertiesEXT"));
    return fn(device, micromapCount, pMicromaps, queryType, dataSize, pData, stride);
}

struct NameEntry { const char* name; PFN_vkVoidFunction fn; };
static const NameEntry g_entries[] = {
    {"vkGetInstanceProcAddr", (PFN_vkVoidFunction)&vkGetInstanceProcAddr},
    {"vkAcquireDrmDisplayEXT", (PFN_vkVoidFunction)&vkAcquireDrmDisplayEXT},
    {"vkAcquireNextImage2KHR", (PFN_vkVoidFunction)&vkAcquireNextImage2KHR},
    {"vkAcquireNextImageKHR", (PFN_vkVoidFunction)&vkAcquireNextImageKHR},
    {"vkAcquirePerformanceConfigurationINTEL", (PFN_vkVoidFunction)&vkAcquirePerformanceConfigurationINTEL},
    {"vkAcquireProfilingLockKHR", (PFN_vkVoidFunction)&vkAcquireProfilingLockKHR},
    {"vkAllocateCommandBuffers", (PFN_vkVoidFunction)&vkAllocateCommandBuffers},
    {"vkAllocateDescriptorSets", (PFN_vkVoidFunction)&vkAllocateDescriptorSets},
    {"vkAllocateMemory", (PFN_vkVoidFunction)&vkAllocateMemory},
    {"vkBeginCommandBuffer", (PFN_vkVoidFunction)&vkBeginCommandBuffer},
    {"vkBindAccelerationStructureMemoryNV", (PFN_vkVoidFunction)&vkBindAccelerationStructureMemoryNV},
    {"vkBindBufferMemory", (PFN_vkVoidFunction)&vkBindBufferMemory},
    {"vkBindBufferMemory2", (PFN_vkVoidFunction)&vkBindBufferMemory2},
    {"vkBindBufferMemory2KHR", (PFN_vkVoidFunction)&vkBindBufferMemory2KHR},
    {"vkBindImageMemory", (PFN_vkVoidFunction)&vkBindImageMemory},
    {"vkBindImageMemory2", (PFN_vkVoidFunction)&vkBindImageMemory2},
    {"vkBindImageMemory2KHR", (PFN_vkVoidFunction)&vkBindImageMemory2KHR},
    {"vkBindOpticalFlowSessionImageNV", (PFN_vkVoidFunction)&vkBindOpticalFlowSessionImageNV},
    {"vkBindVideoSessionMemoryKHR", (PFN_vkVoidFunction)&vkBindVideoSessionMemoryKHR},
    {"vkBuildAccelerationStructuresKHR", (PFN_vkVoidFunction)&vkBuildAccelerationStructuresKHR},
    {"vkBuildMicromapsEXT", (PFN_vkVoidFunction)&vkBuildMicromapsEXT},
    {"vkCmdBeginConditionalRenderingEXT", (PFN_vkVoidFunction)&vkCmdBeginConditionalRenderingEXT},
    {"vkCmdBeginDebugUtilsLabelEXT", (PFN_vkVoidFunction)&vkCmdBeginDebugUtilsLabelEXT},
    {"vkCmdBeginQuery", (PFN_vkVoidFunction)&vkCmdBeginQuery},
    {"vkCmdBeginQueryIndexedEXT", (PFN_vkVoidFunction)&vkCmdBeginQueryIndexedEXT},
    {"vkCmdBeginRenderPass", (PFN_vkVoidFunction)&vkCmdBeginRenderPass},
    {"vkCmdBeginRenderPass2", (PFN_vkVoidFunction)&vkCmdBeginRenderPass2},
    {"vkCmdBeginRenderPass2KHR", (PFN_vkVoidFunction)&vkCmdBeginRenderPass2KHR},
    {"vkCmdBeginRendering", (PFN_vkVoidFunction)&vkCmdBeginRendering},
    {"vkCmdBeginRenderingKHR", (PFN_vkVoidFunction)&vkCmdBeginRenderingKHR},
    {"vkCmdBeginTransformFeedbackEXT", (PFN_vkVoidFunction)&vkCmdBeginTransformFeedbackEXT},
    {"vkCmdBeginVideoCodingKHR", (PFN_vkVoidFunction)&vkCmdBeginVideoCodingKHR},
    {"vkCmdBindDescriptorBufferEmbeddedSamplers2EXT", (PFN_vkVoidFunction)&vkCmdBindDescriptorBufferEmbeddedSamplers2EXT},
    {"vkCmdBindDescriptorBufferEmbeddedSamplersEXT", (PFN_vkVoidFunction)&vkCmdBindDescriptorBufferEmbeddedSamplersEXT},
    {"vkCmdBindDescriptorBuffersEXT", (PFN_vkVoidFunction)&vkCmdBindDescriptorBuffersEXT},
    {"vkCmdBindDescriptorSets", (PFN_vkVoidFunction)&vkCmdBindDescriptorSets},
    {"vkCmdBindDescriptorSets2KHR", (PFN_vkVoidFunction)&vkCmdBindDescriptorSets2KHR},
    {"vkCmdBindIndexBuffer", (PFN_vkVoidFunction)&vkCmdBindIndexBuffer},
    {"vkCmdBindIndexBuffer2KHR", (PFN_vkVoidFunction)&vkCmdBindIndexBuffer2KHR},
    {"vkCmdBindInvocationMaskHUAWEI", (PFN_vkVoidFunction)&vkCmdBindInvocationMaskHUAWEI},
    {"vkCmdBindPipeline", (PFN_vkVoidFunction)&vkCmdBindPipeline},
    {"vkCmdBindPipelineShaderGroupNV", (PFN_vkVoidFunction)&vkCmdBindPipelineShaderGroupNV},
    {"vkCmdBindShadersEXT", (PFN_vkVoidFunction)&vkCmdBindShadersEXT},
    {"vkCmdBindShadingRateImageNV", (PFN_vkVoidFunction)&vkCmdBindShadingRateImageNV},
    {"vkCmdBindTransformFeedbackBuffersEXT", (PFN_vkVoidFunction)&vkCmdBindTransformFeedbackBuffersEXT},
    {"vkCmdBindVertexBuffers", (PFN_vkVoidFunction)&vkCmdBindVertexBuffers},
    {"vkCmdBindVertexBuffers2", (PFN_vkVoidFunction)&vkCmdBindVertexBuffers2},
    {"vkCmdBindVertexBuffers2EXT", (PFN_vkVoidFunction)&vkCmdBindVertexBuffers2EXT},
    {"vkCmdBlitImage", (PFN_vkVoidFunction)&vkCmdBlitImage},
    {"vkCmdBlitImage2", (PFN_vkVoidFunction)&vkCmdBlitImage2},
    {"vkCmdBlitImage2KHR", (PFN_vkVoidFunction)&vkCmdBlitImage2KHR},
    {"vkCmdBuildAccelerationStructureNV", (PFN_vkVoidFunction)&vkCmdBuildAccelerationStructureNV},
    {"vkCmdBuildAccelerationStructuresIndirectKHR", (PFN_vkVoidFunction)&vkCmdBuildAccelerationStructuresIndirectKHR},
    {"vkCmdBuildAccelerationStructuresKHR", (PFN_vkVoidFunction)&vkCmdBuildAccelerationStructuresKHR},
    {"vkCmdBuildMicromapsEXT", (PFN_vkVoidFunction)&vkCmdBuildMicromapsEXT},
    {"vkCmdClearAttachments", (PFN_vkVoidFunction)&vkCmdClearAttachments},
    {"vkCmdClearColorImage", (PFN_vkVoidFunction)&vkCmdClearColorImage},
    {"vkCmdClearDepthStencilImage", (PFN_vkVoidFunction)&vkCmdClearDepthStencilImage},
    {"vkCmdControlVideoCodingKHR", (PFN_vkVoidFunction)&vkCmdControlVideoCodingKHR},
    {"vkCmdCopyAccelerationStructureKHR", (PFN_vkVoidFunction)&vkCmdCopyAccelerationStructureKHR},
    {"vkCmdCopyAccelerationStructureNV", (PFN_vkVoidFunction)&vkCmdCopyAccelerationStructureNV},
    {"vkCmdCopyAccelerationStructureToMemoryKHR", (PFN_vkVoidFunction)&vkCmdCopyAccelerationStructureToMemoryKHR},
    {"vkCmdCopyBuffer", (PFN_vkVoidFunction)&vkCmdCopyBuffer},
    {"vkCmdCopyBuffer2", (PFN_vkVoidFunction)&vkCmdCopyBuffer2},
    {"vkCmdCopyBuffer2KHR", (PFN_vkVoidFunction)&vkCmdCopyBuffer2KHR},
    {"vkCmdCopyBufferToImage", (PFN_vkVoidFunction)&vkCmdCopyBufferToImage},
    {"vkCmdCopyBufferToImage2", (PFN_vkVoidFunction)&vkCmdCopyBufferToImage2},
    {"vkCmdCopyBufferToImage2KHR", (PFN_vkVoidFunction)&vkCmdCopyBufferToImage2KHR},
    {"vkCmdCopyImage", (PFN_vkVoidFunction)&vkCmdCopyImage},
    {"vkCmdCopyImage2", (PFN_vkVoidFunction)&vkCmdCopyImage2},
    {"vkCmdCopyImage2KHR", (PFN_vkVoidFunction)&vkCmdCopyImage2KHR},
    {"vkCmdCopyImageToBuffer", (PFN_vkVoidFunction)&vkCmdCopyImageToBuffer},
    {"vkCmdCopyImageToBuffer2", (PFN_vkVoidFunction)&vkCmdCopyImageToBuffer2},
    {"vkCmdCopyImageToBuffer2KHR", (PFN_vkVoidFunction)&vkCmdCopyImageToBuffer2KHR},
    {"vkCmdCopyMemoryIndirectNV", (PFN_vkVoidFunction)&vkCmdCopyMemoryIndirectNV},
    {"vkCmdCopyMemoryToAccelerationStructureKHR", (PFN_vkVoidFunction)&vkCmdCopyMemoryToAccelerationStructureKHR},
    {"vkCmdCopyMemoryToImageIndirectNV", (PFN_vkVoidFunction)&vkCmdCopyMemoryToImageIndirectNV},
    {"vkCmdCopyMemoryToMicromapEXT", (PFN_vkVoidFunction)&vkCmdCopyMemoryToMicromapEXT},
    {"vkCmdCopyMicromapEXT", (PFN_vkVoidFunction)&vkCmdCopyMicromapEXT},
    {"vkCmdCopyMicromapToMemoryEXT", (PFN_vkVoidFunction)&vkCmdCopyMicromapToMemoryEXT},
    {"vkCmdCopyQueryPoolResults", (PFN_vkVoidFunction)&vkCmdCopyQueryPoolResults},
    {"vkCmdCuLaunchKernelNVX", (PFN_vkVoidFunction)&vkCmdCuLaunchKernelNVX},
    {"vkCmdCudaLaunchKernelNV", (PFN_vkVoidFunction)&vkCmdCudaLaunchKernelNV},
    {"vkCmdDebugMarkerBeginEXT", (PFN_vkVoidFunction)&vkCmdDebugMarkerBeginEXT},
    {"vkCmdDebugMarkerEndEXT", (PFN_vkVoidFunction)&vkCmdDebugMarkerEndEXT},
    {"vkCmdDebugMarkerInsertEXT", (PFN_vkVoidFunction)&vkCmdDebugMarkerInsertEXT},
    {"vkCmdDecodeVideoKHR", (PFN_vkVoidFunction)&vkCmdDecodeVideoKHR},
    {"vkCmdDecompressMemoryIndirectCountNV", (PFN_vkVoidFunction)&vkCmdDecompressMemoryIndirectCountNV},
    {"vkCmdDecompressMemoryNV", (PFN_vkVoidFunction)&vkCmdDecompressMemoryNV},
    {"vkCmdDispatch", (PFN_vkVoidFunction)&vkCmdDispatch},
    {"vkCmdDispatchBase", (PFN_vkVoidFunction)&vkCmdDispatchBase},
    {"vkCmdDispatchBaseKHR", (PFN_vkVoidFunction)&vkCmdDispatchBaseKHR},
    {"vkCmdDispatchIndirect", (PFN_vkVoidFunction)&vkCmdDispatchIndirect},
    {"vkCmdDraw", (PFN_vkVoidFunction)&vkCmdDraw},
    {"vkCmdDrawClusterHUAWEI", (PFN_vkVoidFunction)&vkCmdDrawClusterHUAWEI},
    {"vkCmdDrawClusterIndirectHUAWEI", (PFN_vkVoidFunction)&vkCmdDrawClusterIndirectHUAWEI},
    {"vkCmdDrawIndexed", (PFN_vkVoidFunction)&vkCmdDrawIndexed},
    {"vkCmdDrawIndexedIndirect", (PFN_vkVoidFunction)&vkCmdDrawIndexedIndirect},
    {"vkCmdDrawIndexedIndirectCount", (PFN_vkVoidFunction)&vkCmdDrawIndexedIndirectCount},
    {"vkCmdDrawIndexedIndirectCountAMD", (PFN_vkVoidFunction)&vkCmdDrawIndexedIndirectCountAMD},
    {"vkCmdDrawIndexedIndirectCountKHR", (PFN_vkVoidFunction)&vkCmdDrawIndexedIndirectCountKHR},
    {"vkCmdDrawIndirect", (PFN_vkVoidFunction)&vkCmdDrawIndirect},
    {"vkCmdDrawIndirectByteCountEXT", (PFN_vkVoidFunction)&vkCmdDrawIndirectByteCountEXT},
    {"vkCmdDrawIndirectCount", (PFN_vkVoidFunction)&vkCmdDrawIndirectCount},
    {"vkCmdDrawIndirectCountAMD", (PFN_vkVoidFunction)&vkCmdDrawIndirectCountAMD},
    {"vkCmdDrawIndirectCountKHR", (PFN_vkVoidFunction)&vkCmdDrawIndirectCountKHR},
    {"vkCmdDrawMeshTasksEXT", (PFN_vkVoidFunction)&vkCmdDrawMeshTasksEXT},
    {"vkCmdDrawMeshTasksIndirectCountEXT", (PFN_vkVoidFunction)&vkCmdDrawMeshTasksIndirectCountEXT},
    {"vkCmdDrawMeshTasksIndirectCountNV", (PFN_vkVoidFunction)&vkCmdDrawMeshTasksIndirectCountNV},
    {"vkCmdDrawMeshTasksIndirectEXT", (PFN_vkVoidFunction)&vkCmdDrawMeshTasksIndirectEXT},
    {"vkCmdDrawMeshTasksIndirectNV", (PFN_vkVoidFunction)&vkCmdDrawMeshTasksIndirectNV},
    {"vkCmdDrawMeshTasksNV", (PFN_vkVoidFunction)&vkCmdDrawMeshTasksNV},
    {"vkCmdDrawMultiEXT", (PFN_vkVoidFunction)&vkCmdDrawMultiEXT},
    {"vkCmdDrawMultiIndexedEXT", (PFN_vkVoidFunction)&vkCmdDrawMultiIndexedEXT},
    {"vkCmdEncodeVideoKHR", (PFN_vkVoidFunction)&vkCmdEncodeVideoKHR},
    {"vkCmdEndConditionalRenderingEXT", (PFN_vkVoidFunction)&vkCmdEndConditionalRenderingEXT},
    {"vkCmdEndDebugUtilsLabelEXT", (PFN_vkVoidFunction)&vkCmdEndDebugUtilsLabelEXT},
    {"vkCmdEndQuery", (PFN_vkVoidFunction)&vkCmdEndQuery},
    {"vkCmdEndQueryIndexedEXT", (PFN_vkVoidFunction)&vkCmdEndQueryIndexedEXT},
    {"vkCmdEndRenderPass", (PFN_vkVoidFunction)&vkCmdEndRenderPass},
    {"vkCmdEndRenderPass2", (PFN_vkVoidFunction)&vkCmdEndRenderPass2},
    {"vkCmdEndRenderPass2KHR", (PFN_vkVoidFunction)&vkCmdEndRenderPass2KHR},
    {"vkCmdEndRendering", (PFN_vkVoidFunction)&vkCmdEndRendering},
    {"vkCmdEndRenderingKHR", (PFN_vkVoidFunction)&vkCmdEndRenderingKHR},
    {"vkCmdEndTransformFeedbackEXT", (PFN_vkVoidFunction)&vkCmdEndTransformFeedbackEXT},
    {"vkCmdEndVideoCodingKHR", (PFN_vkVoidFunction)&vkCmdEndVideoCodingKHR},
    {"vkCmdExecuteCommands", (PFN_vkVoidFunction)&vkCmdExecuteCommands},
    {"vkCmdExecuteGeneratedCommandsNV", (PFN_vkVoidFunction)&vkCmdExecuteGeneratedCommandsNV},
    {"vkCmdFillBuffer", (PFN_vkVoidFunction)&vkCmdFillBuffer},
    {"vkCmdInsertDebugUtilsLabelEXT", (PFN_vkVoidFunction)&vkCmdInsertDebugUtilsLabelEXT},
    {"vkCmdNextSubpass", (PFN_vkVoidFunction)&vkCmdNextSubpass},
    {"vkCmdNextSubpass2", (PFN_vkVoidFunction)&vkCmdNextSubpass2},
    {"vkCmdNextSubpass2KHR", (PFN_vkVoidFunction)&vkCmdNextSubpass2KHR},
    {"vkCmdOpticalFlowExecuteNV", (PFN_vkVoidFunction)&vkCmdOpticalFlowExecuteNV},
    {"vkCmdPipelineBarrier", (PFN_vkVoidFunction)&vkCmdPipelineBarrier},
    {"vkCmdPipelineBarrier2", (PFN_vkVoidFunction)&vkCmdPipelineBarrier2},
    {"vkCmdPipelineBarrier2KHR", (PFN_vkVoidFunction)&vkCmdPipelineBarrier2KHR},
    {"vkCmdPreprocessGeneratedCommandsNV", (PFN_vkVoidFunction)&vkCmdPreprocessGeneratedCommandsNV},
    {"vkCmdPushConstants", (PFN_vkVoidFunction)&vkCmdPushConstants},
    {"vkCmdPushConstants2KHR", (PFN_vkVoidFunction)&vkCmdPushConstants2KHR},
    {"vkCmdPushDescriptorSet2KHR", (PFN_vkVoidFunction)&vkCmdPushDescriptorSet2KHR},
    {"vkCmdPushDescriptorSetKHR", (PFN_vkVoidFunction)&vkCmdPushDescriptorSetKHR},
    {"vkCmdPushDescriptorSetWithTemplate2KHR", (PFN_vkVoidFunction)&vkCmdPushDescriptorSetWithTemplate2KHR},
    {"vkCmdPushDescriptorSetWithTemplateKHR", (PFN_vkVoidFunction)&vkCmdPushDescriptorSetWithTemplateKHR},
    {"vkCmdResetEvent", (PFN_vkVoidFunction)&vkCmdResetEvent},
    {"vkCmdResetEvent2", (PFN_vkVoidFunction)&vkCmdResetEvent2},
    {"vkCmdResetEvent2KHR", (PFN_vkVoidFunction)&vkCmdResetEvent2KHR},
    {"vkCmdResetQueryPool", (PFN_vkVoidFunction)&vkCmdResetQueryPool},
    {"vkCmdResolveImage", (PFN_vkVoidFunction)&vkCmdResolveImage},
    {"vkCmdResolveImage2", (PFN_vkVoidFunction)&vkCmdResolveImage2},
    {"vkCmdResolveImage2KHR", (PFN_vkVoidFunction)&vkCmdResolveImage2KHR},
    {"vkCmdSetAlphaToCoverageEnableEXT", (PFN_vkVoidFunction)&vkCmdSetAlphaToCoverageEnableEXT},
    {"vkCmdSetAlphaToOneEnableEXT", (PFN_vkVoidFunction)&vkCmdSetAlphaToOneEnableEXT},
    {"vkCmdSetAttachmentFeedbackLoopEnableEXT", (PFN_vkVoidFunction)&vkCmdSetAttachmentFeedbackLoopEnableEXT},
    {"vkCmdSetBlendConstants", (PFN_vkVoidFunction)&vkCmdSetBlendConstants},
    {"vkCmdSetCheckpointNV", (PFN_vkVoidFunction)&vkCmdSetCheckpointNV},
    {"vkCmdSetCoarseSampleOrderNV", (PFN_vkVoidFunction)&vkCmdSetCoarseSampleOrderNV},
    {"vkCmdSetColorBlendAdvancedEXT", (PFN_vkVoidFunction)&vkCmdSetColorBlendAdvancedEXT},
    {"vkCmdSetColorBlendEnableEXT", (PFN_vkVoidFunction)&vkCmdSetColorBlendEnableEXT},
    {"vkCmdSetColorBlendEquationEXT", (PFN_vkVoidFunction)&vkCmdSetColorBlendEquationEXT},
    {"vkCmdSetColorWriteEnableEXT", (PFN_vkVoidFunction)&vkCmdSetColorWriteEnableEXT},
    {"vkCmdSetColorWriteMaskEXT", (PFN_vkVoidFunction)&vkCmdSetColorWriteMaskEXT},
    {"vkCmdSetConservativeRasterizationModeEXT", (PFN_vkVoidFunction)&vkCmdSetConservativeRasterizationModeEXT},
    {"vkCmdSetCoverageModulationModeNV", (PFN_vkVoidFunction)&vkCmdSetCoverageModulationModeNV},
    {"vkCmdSetCoverageModulationTableEnableNV", (PFN_vkVoidFunction)&vkCmdSetCoverageModulationTableEnableNV},
    {"vkCmdSetCoverageModulationTableNV", (PFN_vkVoidFunction)&vkCmdSetCoverageModulationTableNV},
    {"vkCmdSetCoverageReductionModeNV", (PFN_vkVoidFunction)&vkCmdSetCoverageReductionModeNV},
    {"vkCmdSetCoverageToColorEnableNV", (PFN_vkVoidFunction)&vkCmdSetCoverageToColorEnableNV},
    {"vkCmdSetCoverageToColorLocationNV", (PFN_vkVoidFunction)&vkCmdSetCoverageToColorLocationNV},
    {"vkCmdSetCullMode", (PFN_vkVoidFunction)&vkCmdSetCullMode},
    {"vkCmdSetCullModeEXT", (PFN_vkVoidFunction)&vkCmdSetCullModeEXT},
    {"vkCmdSetDepthBias", (PFN_vkVoidFunction)&vkCmdSetDepthBias},
    {"vkCmdSetDepthBias2EXT", (PFN_vkVoidFunction)&vkCmdSetDepthBias2EXT},
    {"vkCmdSetDepthBiasEnable", (PFN_vkVoidFunction)&vkCmdSetDepthBiasEnable},
    {"vkCmdSetDepthBiasEnableEXT", (PFN_vkVoidFunction)&vkCmdSetDepthBiasEnableEXT},
    {"vkCmdSetDepthBounds", (PFN_vkVoidFunction)&vkCmdSetDepthBounds},
    {"vkCmdSetDepthBoundsTestEnable", (PFN_vkVoidFunction)&vkCmdSetDepthBoundsTestEnable},
    {"vkCmdSetDepthBoundsTestEnableEXT", (PFN_vkVoidFunction)&vkCmdSetDepthBoundsTestEnableEXT},
    {"vkCmdSetDepthClampEnableEXT", (PFN_vkVoidFunction)&vkCmdSetDepthClampEnableEXT},
    {"vkCmdSetDepthClipEnableEXT", (PFN_vkVoidFunction)&vkCmdSetDepthClipEnableEXT},
    {"vkCmdSetDepthClipNegativeOneToOneEXT", (PFN_vkVoidFunction)&vkCmdSetDepthClipNegativeOneToOneEXT},
    {"vkCmdSetDepthCompareOp", (PFN_vkVoidFunction)&vkCmdSetDepthCompareOp},
    {"vkCmdSetDepthCompareOpEXT", (PFN_vkVoidFunction)&vkCmdSetDepthCompareOpEXT},
    {"vkCmdSetDepthTestEnable", (PFN_vkVoidFunction)&vkCmdSetDepthTestEnable},
    {"vkCmdSetDepthTestEnableEXT", (PFN_vkVoidFunction)&vkCmdSetDepthTestEnableEXT},
    {"vkCmdSetDepthWriteEnable", (PFN_vkVoidFunction)&vkCmdSetDepthWriteEnable},
    {"vkCmdSetDepthWriteEnableEXT", (PFN_vkVoidFunction)&vkCmdSetDepthWriteEnableEXT},
    {"vkCmdSetDescriptorBufferOffsets2EXT", (PFN_vkVoidFunction)&vkCmdSetDescriptorBufferOffsets2EXT},
    {"vkCmdSetDescriptorBufferOffsetsEXT", (PFN_vkVoidFunction)&vkCmdSetDescriptorBufferOffsetsEXT},
    {"vkCmdSetDeviceMask", (PFN_vkVoidFunction)&vkCmdSetDeviceMask},
    {"vkCmdSetDeviceMaskKHR", (PFN_vkVoidFunction)&vkCmdSetDeviceMaskKHR},
    {"vkCmdSetDiscardRectangleEXT", (PFN_vkVoidFunction)&vkCmdSetDiscardRectangleEXT},
    {"vkCmdSetDiscardRectangleEnableEXT", (PFN_vkVoidFunction)&vkCmdSetDiscardRectangleEnableEXT},
    {"vkCmdSetDiscardRectangleModeEXT", (PFN_vkVoidFunction)&vkCmdSetDiscardRectangleModeEXT},
    {"vkCmdSetEvent", (PFN_vkVoidFunction)&vkCmdSetEvent},
    {"vkCmdSetEvent2", (PFN_vkVoidFunction)&vkCmdSetEvent2},
    {"vkCmdSetEvent2KHR", (PFN_vkVoidFunction)&vkCmdSetEvent2KHR},
    {"vkCmdSetExclusiveScissorEnableNV", (PFN_vkVoidFunction)&vkCmdSetExclusiveScissorEnableNV},
    {"vkCmdSetExclusiveScissorNV", (PFN_vkVoidFunction)&vkCmdSetExclusiveScissorNV},
    {"vkCmdSetExtraPrimitiveOverestimationSizeEXT", (PFN_vkVoidFunction)&vkCmdSetExtraPrimitiveOverestimationSizeEXT},
    {"vkCmdSetFragmentShadingRateEnumNV", (PFN_vkVoidFunction)&vkCmdSetFragmentShadingRateEnumNV},
    {"vkCmdSetFragmentShadingRateKHR", (PFN_vkVoidFunction)&vkCmdSetFragmentShadingRateKHR},
    {"vkCmdSetFrontFace", (PFN_vkVoidFunction)&vkCmdSetFrontFace},
    {"vkCmdSetFrontFaceEXT", (PFN_vkVoidFunction)&vkCmdSetFrontFaceEXT},
    {"vkCmdSetLineRasterizationModeEXT", (PFN_vkVoidFunction)&vkCmdSetLineRasterizationModeEXT},
    {"vkCmdSetLineStippleEXT", (PFN_vkVoidFunction)&vkCmdSetLineStippleEXT},
    {"vkCmdSetLineStippleEnableEXT", (PFN_vkVoidFunction)&vkCmdSetLineStippleEnableEXT},
    {"vkCmdSetLineWidth", (PFN_vkVoidFunction)&vkCmdSetLineWidth},
    {"vkCmdSetLogicOpEXT", (PFN_vkVoidFunction)&vkCmdSetLogicOpEXT},
    {"vkCmdSetLogicOpEnableEXT", (PFN_vkVoidFunction)&vkCmdSetLogicOpEnableEXT},
    {"vkCmdSetPatchControlPointsEXT", (PFN_vkVoidFunction)&vkCmdSetPatchControlPointsEXT},
    {"vkCmdSetPerformanceMarkerINTEL", (PFN_vkVoidFunction)&vkCmdSetPerformanceMarkerINTEL},
    {"vkCmdSetPerformanceOverrideINTEL", (PFN_vkVoidFunction)&vkCmdSetPerformanceOverrideINTEL},
    {"vkCmdSetPerformanceStreamMarkerINTEL", (PFN_vkVoidFunction)&vkCmdSetPerformanceStreamMarkerINTEL},
    {"vkCmdSetPolygonModeEXT", (PFN_vkVoidFunction)&vkCmdSetPolygonModeEXT},
    {"vkCmdSetPrimitiveRestartEnable", (PFN_vkVoidFunction)&vkCmdSetPrimitiveRestartEnable},
    {"vkCmdSetPrimitiveRestartEnableEXT", (PFN_vkVoidFunction)&vkCmdSetPrimitiveRestartEnableEXT},
    {"vkCmdSetPrimitiveTopology", (PFN_vkVoidFunction)&vkCmdSetPrimitiveTopology},
    {"vkCmdSetPrimitiveTopologyEXT", (PFN_vkVoidFunction)&vkCmdSetPrimitiveTopologyEXT},
    {"vkCmdSetProvokingVertexModeEXT", (PFN_vkVoidFunction)&vkCmdSetProvokingVertexModeEXT},
    {"vkCmdSetRasterizationSamplesEXT", (PFN_vkVoidFunction)&vkCmdSetRasterizationSamplesEXT},
    {"vkCmdSetRasterizationStreamEXT", (PFN_vkVoidFunction)&vkCmdSetRasterizationStreamEXT},
    {"vkCmdSetRasterizerDiscardEnable", (PFN_vkVoidFunction)&vkCmdSetRasterizerDiscardEnable},
    {"vkCmdSetRasterizerDiscardEnableEXT", (PFN_vkVoidFunction)&vkCmdSetRasterizerDiscardEnableEXT},
    {"vkCmdSetRayTracingPipelineStackSizeKHR", (PFN_vkVoidFunction)&vkCmdSetRayTracingPipelineStackSizeKHR},
    {"vkCmdSetRepresentativeFragmentTestEnableNV", (PFN_vkVoidFunction)&vkCmdSetRepresentativeFragmentTestEnableNV},
    {"vkCmdSetSampleLocationsEXT", (PFN_vkVoidFunction)&vkCmdSetSampleLocationsEXT},
    {"vkCmdSetSampleLocationsEnableEXT", (PFN_vkVoidFunction)&vkCmdSetSampleLocationsEnableEXT},
    {"vkCmdSetSampleMaskEXT", (PFN_vkVoidFunction)&vkCmdSetSampleMaskEXT},
    {"vkCmdSetScissor", (PFN_vkVoidFunction)&vkCmdSetScissor},
    {"vkCmdSetScissorWithCount", (PFN_vkVoidFunction)&vkCmdSetScissorWithCount},
    {"vkCmdSetScissorWithCountEXT", (PFN_vkVoidFunction)&vkCmdSetScissorWithCountEXT},
    {"vkCmdSetShadingRateImageEnableNV", (PFN_vkVoidFunction)&vkCmdSetShadingRateImageEnableNV},
    {"vkCmdSetStencilCompareMask", (PFN_vkVoidFunction)&vkCmdSetStencilCompareMask},
    {"vkCmdSetStencilOp", (PFN_vkVoidFunction)&vkCmdSetStencilOp},
    {"vkCmdSetStencilOpEXT", (PFN_vkVoidFunction)&vkCmdSetStencilOpEXT},
    {"vkCmdSetStencilReference", (PFN_vkVoidFunction)&vkCmdSetStencilReference},
    {"vkCmdSetStencilTestEnable", (PFN_vkVoidFunction)&vkCmdSetStencilTestEnable},
    {"vkCmdSetStencilTestEnableEXT", (PFN_vkVoidFunction)&vkCmdSetStencilTestEnableEXT},
    {"vkCmdSetStencilWriteMask", (PFN_vkVoidFunction)&vkCmdSetStencilWriteMask},
    {"vkCmdSetTessellationDomainOriginEXT", (PFN_vkVoidFunction)&vkCmdSetTessellationDomainOriginEXT},
    {"vkCmdSetVertexInputEXT", (PFN_vkVoidFunction)&vkCmdSetVertexInputEXT},
    {"vkCmdSetViewport", (PFN_vkVoidFunction)&vkCmdSetViewport},
    {"vkCmdSetViewportShadingRatePaletteNV", (PFN_vkVoidFunction)&vkCmdSetViewportShadingRatePaletteNV},
    {"vkCmdSetViewportSwizzleNV", (PFN_vkVoidFunction)&vkCmdSetViewportSwizzleNV},
    {"vkCmdSetViewportWScalingEnableNV", (PFN_vkVoidFunction)&vkCmdSetViewportWScalingEnableNV},
    {"vkCmdSetViewportWScalingNV", (PFN_vkVoidFunction)&vkCmdSetViewportWScalingNV},
    {"vkCmdSetViewportWithCount", (PFN_vkVoidFunction)&vkCmdSetViewportWithCount},
    {"vkCmdSetViewportWithCountEXT", (PFN_vkVoidFunction)&vkCmdSetViewportWithCountEXT},
    {"vkCmdSubpassShadingHUAWEI", (PFN_vkVoidFunction)&vkCmdSubpassShadingHUAWEI},
    {"vkCmdTraceRaysIndirect2KHR", (PFN_vkVoidFunction)&vkCmdTraceRaysIndirect2KHR},
    {"vkCmdTraceRaysIndirectKHR", (PFN_vkVoidFunction)&vkCmdTraceRaysIndirectKHR},
    {"vkCmdTraceRaysKHR", (PFN_vkVoidFunction)&vkCmdTraceRaysKHR},
    {"vkCmdTraceRaysNV", (PFN_vkVoidFunction)&vkCmdTraceRaysNV},
    {"vkCmdUpdateBuffer", (PFN_vkVoidFunction)&vkCmdUpdateBuffer},
    {"vkCmdUpdatePipelineIndirectBufferNV", (PFN_vkVoidFunction)&vkCmdUpdatePipelineIndirectBufferNV},
    {"vkCmdWaitEvents", (PFN_vkVoidFunction)&vkCmdWaitEvents},
    {"vkCmdWaitEvents2", (PFN_vkVoidFunction)&vkCmdWaitEvents2},
    {"vkCmdWaitEvents2KHR", (PFN_vkVoidFunction)&vkCmdWaitEvents2KHR},
    {"vkCmdWriteAccelerationStructuresPropertiesKHR", (PFN_vkVoidFunction)&vkCmdWriteAccelerationStructuresPropertiesKHR},
    {"vkCmdWriteAccelerationStructuresPropertiesNV", (PFN_vkVoidFunction)&vkCmdWriteAccelerationStructuresPropertiesNV},
    {"vkCmdWriteBufferMarker2AMD", (PFN_vkVoidFunction)&vkCmdWriteBufferMarker2AMD},
    {"vkCmdWriteBufferMarkerAMD", (PFN_vkVoidFunction)&vkCmdWriteBufferMarkerAMD},
    {"vkCmdWriteMicromapsPropertiesEXT", (PFN_vkVoidFunction)&vkCmdWriteMicromapsPropertiesEXT},
    {"vkCmdWriteTimestamp", (PFN_vkVoidFunction)&vkCmdWriteTimestamp},
    {"vkCmdWriteTimestamp2", (PFN_vkVoidFunction)&vkCmdWriteTimestamp2},
    {"vkCmdWriteTimestamp2KHR", (PFN_vkVoidFunction)&vkCmdWriteTimestamp2KHR},
    {"vkCompileDeferredNV", (PFN_vkVoidFunction)&vkCompileDeferredNV},
    {"vkCopyAccelerationStructureKHR", (PFN_vkVoidFunction)&vkCopyAccelerationStructureKHR},
    {"vkCopyAccelerationStructureToMemoryKHR", (PFN_vkVoidFunction)&vkCopyAccelerationStructureToMemoryKHR},
    {"vkCopyImageToImageEXT", (PFN_vkVoidFunction)&vkCopyImageToImageEXT},
    {"vkCopyImageToMemoryEXT", (PFN_vkVoidFunction)&vkCopyImageToMemoryEXT},
    {"vkCopyMemoryToAccelerationStructureKHR", (PFN_vkVoidFunction)&vkCopyMemoryToAccelerationStructureKHR},
    {"vkCopyMemoryToImageEXT", (PFN_vkVoidFunction)&vkCopyMemoryToImageEXT},
    {"vkCopyMemoryToMicromapEXT", (PFN_vkVoidFunction)&vkCopyMemoryToMicromapEXT},
    {"vkCopyMicromapEXT", (PFN_vkVoidFunction)&vkCopyMicromapEXT},
    {"vkCopyMicromapToMemoryEXT", (PFN_vkVoidFunction)&vkCopyMicromapToMemoryEXT},
    {"vkCreateAccelerationStructureKHR", (PFN_vkVoidFunction)&vkCreateAccelerationStructureKHR},
    {"vkCreateAccelerationStructureNV", (PFN_vkVoidFunction)&vkCreateAccelerationStructureNV},
    {"vkCreateAndroidSurfaceKHR", (PFN_vkVoidFunction)&vkCreateAndroidSurfaceKHR},
    {"vkCreateBuffer", (PFN_vkVoidFunction)&vkCreateBuffer},
    {"vkCreateBufferView", (PFN_vkVoidFunction)&vkCreateBufferView},
    {"vkCreateCommandPool", (PFN_vkVoidFunction)&vkCreateCommandPool},
    {"vkCreateComputePipelines", (PFN_vkVoidFunction)&vkCreateComputePipelines},
    {"vkCreateCuFunctionNVX", (PFN_vkVoidFunction)&vkCreateCuFunctionNVX},
    {"vkCreateCuModuleNVX", (PFN_vkVoidFunction)&vkCreateCuModuleNVX},
    {"vkCreateCudaFunctionNV", (PFN_vkVoidFunction)&vkCreateCudaFunctionNV},
    {"vkCreateCudaModuleNV", (PFN_vkVoidFunction)&vkCreateCudaModuleNV},
    {"vkCreateDebugReportCallbackEXT", (PFN_vkVoidFunction)&vkCreateDebugReportCallbackEXT},
    {"vkCreateDebugUtilsMessengerEXT", (PFN_vkVoidFunction)&vkCreateDebugUtilsMessengerEXT},
    {"vkCreateDeferredOperationKHR", (PFN_vkVoidFunction)&vkCreateDeferredOperationKHR},
    {"vkCreateDescriptorPool", (PFN_vkVoidFunction)&vkCreateDescriptorPool},
    {"vkCreateDescriptorSetLayout", (PFN_vkVoidFunction)&vkCreateDescriptorSetLayout},
    {"vkCreateDescriptorUpdateTemplate", (PFN_vkVoidFunction)&vkCreateDescriptorUpdateTemplate},
    {"vkCreateDescriptorUpdateTemplateKHR", (PFN_vkVoidFunction)&vkCreateDescriptorUpdateTemplateKHR},
    {"vkCreateDevice", (PFN_vkVoidFunction)&vkCreateDevice},
    {"vkCreateDisplayModeKHR", (PFN_vkVoidFunction)&vkCreateDisplayModeKHR},
    {"vkCreateDisplayPlaneSurfaceKHR", (PFN_vkVoidFunction)&vkCreateDisplayPlaneSurfaceKHR},
    {"vkCreateEvent", (PFN_vkVoidFunction)&vkCreateEvent},
    {"vkCreateFence", (PFN_vkVoidFunction)&vkCreateFence},
    {"vkCreateFramebuffer", (PFN_vkVoidFunction)&vkCreateFramebuffer},
    {"vkCreateGraphicsPipelines", (PFN_vkVoidFunction)&vkCreateGraphicsPipelines},
    {"vkCreateHeadlessSurfaceEXT", (PFN_vkVoidFunction)&vkCreateHeadlessSurfaceEXT},
    {"vkCreateImage", (PFN_vkVoidFunction)&vkCreateImage},
    {"vkCreateImageView", (PFN_vkVoidFunction)&vkCreateImageView},
    {"vkCreateIndirectCommandsLayoutNV", (PFN_vkVoidFunction)&vkCreateIndirectCommandsLayoutNV},
    {"vkCreateInstance", (PFN_vkVoidFunction)&vkCreateInstance},
    {"vkCreateMicromapEXT", (PFN_vkVoidFunction)&vkCreateMicromapEXT},
    {"vkCreateOpticalFlowSessionNV", (PFN_vkVoidFunction)&vkCreateOpticalFlowSessionNV},
    {"vkCreatePipelineCache", (PFN_vkVoidFunction)&vkCreatePipelineCache},
    {"vkCreatePipelineLayout", (PFN_vkVoidFunction)&vkCreatePipelineLayout},
    {"vkCreatePrivateDataSlot", (PFN_vkVoidFunction)&vkCreatePrivateDataSlot},
    {"vkCreatePrivateDataSlotEXT", (PFN_vkVoidFunction)&vkCreatePrivateDataSlotEXT},
    {"vkCreateQueryPool", (PFN_vkVoidFunction)&vkCreateQueryPool},
    {"vkCreateRayTracingPipelinesKHR", (PFN_vkVoidFunction)&vkCreateRayTracingPipelinesKHR},
    {"vkCreateRayTracingPipelinesNV", (PFN_vkVoidFunction)&vkCreateRayTracingPipelinesNV},
    {"vkCreateRenderPass", (PFN_vkVoidFunction)&vkCreateRenderPass},
    {"vkCreateRenderPass2", (PFN_vkVoidFunction)&vkCreateRenderPass2},
    {"vkCreateRenderPass2KHR", (PFN_vkVoidFunction)&vkCreateRenderPass2KHR},
    {"vkCreateSampler", (PFN_vkVoidFunction)&vkCreateSampler},
    {"vkCreateSamplerYcbcrConversion", (PFN_vkVoidFunction)&vkCreateSamplerYcbcrConversion},
    {"vkCreateSamplerYcbcrConversionKHR", (PFN_vkVoidFunction)&vkCreateSamplerYcbcrConversionKHR},
    {"vkCreateSemaphore", (PFN_vkVoidFunction)&vkCreateSemaphore},
    {"vkCreateShaderModule", (PFN_vkVoidFunction)&vkCreateShaderModule},
    {"vkCreateShadersEXT", (PFN_vkVoidFunction)&vkCreateShadersEXT},
    {"vkCreateSharedSwapchainsKHR", (PFN_vkVoidFunction)&vkCreateSharedSwapchainsKHR},
    {"vkCreateSwapchainKHR", (PFN_vkVoidFunction)&vkCreateSwapchainKHR},
    {"vkCreateValidationCacheEXT", (PFN_vkVoidFunction)&vkCreateValidationCacheEXT},
    {"vkCreateVideoSessionKHR", (PFN_vkVoidFunction)&vkCreateVideoSessionKHR},
    {"vkCreateVideoSessionParametersKHR", (PFN_vkVoidFunction)&vkCreateVideoSessionParametersKHR},
    {"vkDebugMarkerSetObjectNameEXT", (PFN_vkVoidFunction)&vkDebugMarkerSetObjectNameEXT},
    {"vkDebugMarkerSetObjectTagEXT", (PFN_vkVoidFunction)&vkDebugMarkerSetObjectTagEXT},
    {"vkDebugReportMessageEXT", (PFN_vkVoidFunction)&vkDebugReportMessageEXT},
    {"vkDeferredOperationJoinKHR", (PFN_vkVoidFunction)&vkDeferredOperationJoinKHR},
    {"vkDestroyAccelerationStructureKHR", (PFN_vkVoidFunction)&vkDestroyAccelerationStructureKHR},
    {"vkDestroyAccelerationStructureNV", (PFN_vkVoidFunction)&vkDestroyAccelerationStructureNV},
    {"vkDestroyBuffer", (PFN_vkVoidFunction)&vkDestroyBuffer},
    {"vkDestroyBufferView", (PFN_vkVoidFunction)&vkDestroyBufferView},
    {"vkDestroyCommandPool", (PFN_vkVoidFunction)&vkDestroyCommandPool},
    {"vkDestroyCuFunctionNVX", (PFN_vkVoidFunction)&vkDestroyCuFunctionNVX},
    {"vkDestroyCuModuleNVX", (PFN_vkVoidFunction)&vkDestroyCuModuleNVX},
    {"vkDestroyCudaFunctionNV", (PFN_vkVoidFunction)&vkDestroyCudaFunctionNV},
    {"vkDestroyCudaModuleNV", (PFN_vkVoidFunction)&vkDestroyCudaModuleNV},
    {"vkDestroyDebugReportCallbackEXT", (PFN_vkVoidFunction)&vkDestroyDebugReportCallbackEXT},
    {"vkDestroyDebugUtilsMessengerEXT", (PFN_vkVoidFunction)&vkDestroyDebugUtilsMessengerEXT},
    {"vkDestroyDeferredOperationKHR", (PFN_vkVoidFunction)&vkDestroyDeferredOperationKHR},
    {"vkDestroyDescriptorPool", (PFN_vkVoidFunction)&vkDestroyDescriptorPool},
    {"vkDestroyDescriptorSetLayout", (PFN_vkVoidFunction)&vkDestroyDescriptorSetLayout},
    {"vkDestroyDescriptorUpdateTemplate", (PFN_vkVoidFunction)&vkDestroyDescriptorUpdateTemplate},
    {"vkDestroyDescriptorUpdateTemplateKHR", (PFN_vkVoidFunction)&vkDestroyDescriptorUpdateTemplateKHR},
    {"vkDestroyDevice", (PFN_vkVoidFunction)&vkDestroyDevice},
    {"vkDestroyEvent", (PFN_vkVoidFunction)&vkDestroyEvent},
    {"vkDestroyFence", (PFN_vkVoidFunction)&vkDestroyFence},
    {"vkDestroyFramebuffer", (PFN_vkVoidFunction)&vkDestroyFramebuffer},
    {"vkDestroyImage", (PFN_vkVoidFunction)&vkDestroyImage},
    {"vkDestroyImageView", (PFN_vkVoidFunction)&vkDestroyImageView},
    {"vkDestroyIndirectCommandsLayoutNV", (PFN_vkVoidFunction)&vkDestroyIndirectCommandsLayoutNV},
    {"vkDestroyInstance", (PFN_vkVoidFunction)&vkDestroyInstance},
    {"vkDestroyMicromapEXT", (PFN_vkVoidFunction)&vkDestroyMicromapEXT},
    {"vkDestroyOpticalFlowSessionNV", (PFN_vkVoidFunction)&vkDestroyOpticalFlowSessionNV},
    {"vkDestroyPipeline", (PFN_vkVoidFunction)&vkDestroyPipeline},
    {"vkDestroyPipelineCache", (PFN_vkVoidFunction)&vkDestroyPipelineCache},
    {"vkDestroyPipelineLayout", (PFN_vkVoidFunction)&vkDestroyPipelineLayout},
    {"vkDestroyPrivateDataSlot", (PFN_vkVoidFunction)&vkDestroyPrivateDataSlot},
    {"vkDestroyPrivateDataSlotEXT", (PFN_vkVoidFunction)&vkDestroyPrivateDataSlotEXT},
    {"vkDestroyQueryPool", (PFN_vkVoidFunction)&vkDestroyQueryPool},
    {"vkDestroyRenderPass", (PFN_vkVoidFunction)&vkDestroyRenderPass},
    {"vkDestroySampler", (PFN_vkVoidFunction)&vkDestroySampler},
    {"vkDestroySamplerYcbcrConversion", (PFN_vkVoidFunction)&vkDestroySamplerYcbcrConversion},
    {"vkDestroySamplerYcbcrConversionKHR", (PFN_vkVoidFunction)&vkDestroySamplerYcbcrConversionKHR},
    {"vkDestroySemaphore", (PFN_vkVoidFunction)&vkDestroySemaphore},
    {"vkDestroyShaderEXT", (PFN_vkVoidFunction)&vkDestroyShaderEXT},
    {"vkDestroyShaderModule", (PFN_vkVoidFunction)&vkDestroyShaderModule},
    {"vkDestroySurfaceKHR", (PFN_vkVoidFunction)&vkDestroySurfaceKHR},
    {"vkDestroySwapchainKHR", (PFN_vkVoidFunction)&vkDestroySwapchainKHR},
    {"vkDestroyValidationCacheEXT", (PFN_vkVoidFunction)&vkDestroyValidationCacheEXT},
    {"vkDestroyVideoSessionKHR", (PFN_vkVoidFunction)&vkDestroyVideoSessionKHR},
    {"vkDestroyVideoSessionParametersKHR", (PFN_vkVoidFunction)&vkDestroyVideoSessionParametersKHR},
    {"vkDeviceWaitIdle", (PFN_vkVoidFunction)&vkDeviceWaitIdle},
    {"vkDisplayPowerControlEXT", (PFN_vkVoidFunction)&vkDisplayPowerControlEXT},
    {"vkEndCommandBuffer", (PFN_vkVoidFunction)&vkEndCommandBuffer},
    {"vkEnumerateDeviceExtensionProperties", (PFN_vkVoidFunction)&vkEnumerateDeviceExtensionProperties},
    {"vkEnumerateDeviceLayerProperties", (PFN_vkVoidFunction)&vkEnumerateDeviceLayerProperties},
    {"vkEnumerateInstanceExtensionProperties", (PFN_vkVoidFunction)&vkEnumerateInstanceExtensionProperties},
    {"vkEnumerateInstanceLayerProperties", (PFN_vkVoidFunction)&vkEnumerateInstanceLayerProperties},
    {"vkEnumerateInstanceVersion", (PFN_vkVoidFunction)&vkEnumerateInstanceVersion},
    {"vkEnumeratePhysicalDeviceGroups", (PFN_vkVoidFunction)&vkEnumeratePhysicalDeviceGroups},
    {"vkEnumeratePhysicalDeviceGroupsKHR", (PFN_vkVoidFunction)&vkEnumeratePhysicalDeviceGroupsKHR},
    {"vkEnumeratePhysicalDeviceQueueFamilyPerformanceQueryCountersKHR", (PFN_vkVoidFunction)&vkEnumeratePhysicalDeviceQueueFamilyPerformanceQueryCountersKHR},
    {"vkEnumeratePhysicalDevices", (PFN_vkVoidFunction)&vkEnumeratePhysicalDevices},
    {"vkFlushMappedMemoryRanges", (PFN_vkVoidFunction)&vkFlushMappedMemoryRanges},
    {"vkFreeCommandBuffers", (PFN_vkVoidFunction)&vkFreeCommandBuffers},
    {"vkFreeDescriptorSets", (PFN_vkVoidFunction)&vkFreeDescriptorSets},
    {"vkFreeMemory", (PFN_vkVoidFunction)&vkFreeMemory},
    {"vkGetAccelerationStructureBuildSizesKHR", (PFN_vkVoidFunction)&vkGetAccelerationStructureBuildSizesKHR},
    {"vkGetAccelerationStructureDeviceAddressKHR", (PFN_vkVoidFunction)&vkGetAccelerationStructureDeviceAddressKHR},
    {"vkGetAccelerationStructureHandleNV", (PFN_vkVoidFunction)&vkGetAccelerationStructureHandleNV},
    {"vkGetAccelerationStructureMemoryRequirementsNV", (PFN_vkVoidFunction)&vkGetAccelerationStructureMemoryRequirementsNV},
    {"vkGetAccelerationStructureOpaqueCaptureDescriptorDataEXT", (PFN_vkVoidFunction)&vkGetAccelerationStructureOpaqueCaptureDescriptorDataEXT},
    {"vkGetAndroidHardwareBufferPropertiesANDROID", (PFN_vkVoidFunction)&vkGetAndroidHardwareBufferPropertiesANDROID},
    {"vkGetBufferDeviceAddress", (PFN_vkVoidFunction)&vkGetBufferDeviceAddress},
    {"vkGetBufferDeviceAddressEXT", (PFN_vkVoidFunction)&vkGetBufferDeviceAddressEXT},
    {"vkGetBufferDeviceAddressKHR", (PFN_vkVoidFunction)&vkGetBufferDeviceAddressKHR},
    {"vkGetBufferMemoryRequirements", (PFN_vkVoidFunction)&vkGetBufferMemoryRequirements},
    {"vkGetBufferMemoryRequirements2", (PFN_vkVoidFunction)&vkGetBufferMemoryRequirements2},
    {"vkGetBufferMemoryRequirements2KHR", (PFN_vkVoidFunction)&vkGetBufferMemoryRequirements2KHR},
    {"vkGetBufferOpaqueCaptureAddress", (PFN_vkVoidFunction)&vkGetBufferOpaqueCaptureAddress},
    {"vkGetBufferOpaqueCaptureAddressKHR", (PFN_vkVoidFunction)&vkGetBufferOpaqueCaptureAddressKHR},
    {"vkGetBufferOpaqueCaptureDescriptorDataEXT", (PFN_vkVoidFunction)&vkGetBufferOpaqueCaptureDescriptorDataEXT},
    {"vkGetCalibratedTimestampsEXT", (PFN_vkVoidFunction)&vkGetCalibratedTimestampsEXT},
    {"vkGetCalibratedTimestampsKHR", (PFN_vkVoidFunction)&vkGetCalibratedTimestampsKHR},
    {"vkGetCudaModuleCacheNV", (PFN_vkVoidFunction)&vkGetCudaModuleCacheNV},
    {"vkGetDeferredOperationMaxConcurrencyKHR", (PFN_vkVoidFunction)&vkGetDeferredOperationMaxConcurrencyKHR},
    {"vkGetDeferredOperationResultKHR", (PFN_vkVoidFunction)&vkGetDeferredOperationResultKHR},
    {"vkGetDescriptorEXT", (PFN_vkVoidFunction)&vkGetDescriptorEXT},
    {"vkGetDescriptorSetHostMappingVALVE", (PFN_vkVoidFunction)&vkGetDescriptorSetHostMappingVALVE},
    {"vkGetDescriptorSetLayoutBindingOffsetEXT", (PFN_vkVoidFunction)&vkGetDescriptorSetLayoutBindingOffsetEXT},
    {"vkGetDescriptorSetLayoutHostMappingInfoVALVE", (PFN_vkVoidFunction)&vkGetDescriptorSetLayoutHostMappingInfoVALVE},
    {"vkGetDescriptorSetLayoutSizeEXT", (PFN_vkVoidFunction)&vkGetDescriptorSetLayoutSizeEXT},
    {"vkGetDescriptorSetLayoutSupport", (PFN_vkVoidFunction)&vkGetDescriptorSetLayoutSupport},
    {"vkGetDescriptorSetLayoutSupportKHR", (PFN_vkVoidFunction)&vkGetDescriptorSetLayoutSupportKHR},
    {"vkGetDeviceAccelerationStructureCompatibilityKHR", (PFN_vkVoidFunction)&vkGetDeviceAccelerationStructureCompatibilityKHR},
    {"vkGetDeviceBufferMemoryRequirements", (PFN_vkVoidFunction)&vkGetDeviceBufferMemoryRequirements},
    {"vkGetDeviceBufferMemoryRequirementsKHR", (PFN_vkVoidFunction)&vkGetDeviceBufferMemoryRequirementsKHR},
    {"vkGetDeviceFaultInfoEXT", (PFN_vkVoidFunction)&vkGetDeviceFaultInfoEXT},
    {"vkGetDeviceGroupPeerMemoryFeatures", (PFN_vkVoidFunction)&vkGetDeviceGroupPeerMemoryFeatures},
    {"vkGetDeviceGroupPeerMemoryFeaturesKHR", (PFN_vkVoidFunction)&vkGetDeviceGroupPeerMemoryFeaturesKHR},
    {"vkGetDeviceGroupPresentCapabilitiesKHR", (PFN_vkVoidFunction)&vkGetDeviceGroupPresentCapabilitiesKHR},
    {"vkGetDeviceGroupSurfacePresentModesKHR", (PFN_vkVoidFunction)&vkGetDeviceGroupSurfacePresentModesKHR},
    {"vkGetDeviceImageMemoryRequirements", (PFN_vkVoidFunction)&vkGetDeviceImageMemoryRequirements},
    {"vkGetDeviceImageMemoryRequirementsKHR", (PFN_vkVoidFunction)&vkGetDeviceImageMemoryRequirementsKHR},
    {"vkGetDeviceImageSparseMemoryRequirements", (PFN_vkVoidFunction)&vkGetDeviceImageSparseMemoryRequirements},
    {"vkGetDeviceImageSparseMemoryRequirementsKHR", (PFN_vkVoidFunction)&vkGetDeviceImageSparseMemoryRequirementsKHR},
    {"vkGetDeviceImageSubresourceLayoutKHR", (PFN_vkVoidFunction)&vkGetDeviceImageSubresourceLayoutKHR},
    {"vkGetDeviceMemoryCommitment", (PFN_vkVoidFunction)&vkGetDeviceMemoryCommitment},
    {"vkGetDeviceMemoryOpaqueCaptureAddress", (PFN_vkVoidFunction)&vkGetDeviceMemoryOpaqueCaptureAddress},
    {"vkGetDeviceMemoryOpaqueCaptureAddressKHR", (PFN_vkVoidFunction)&vkGetDeviceMemoryOpaqueCaptureAddressKHR},
    {"vkGetDeviceMicromapCompatibilityEXT", (PFN_vkVoidFunction)&vkGetDeviceMicromapCompatibilityEXT},
    {"vkGetDeviceProcAddr", (PFN_vkVoidFunction)&vkGetDeviceProcAddr},
    {"vkGetDeviceQueue", (PFN_vkVoidFunction)&vkGetDeviceQueue},
    {"vkGetDeviceQueue2", (PFN_vkVoidFunction)&vkGetDeviceQueue2},
    {"vkGetDeviceSubpassShadingMaxWorkgroupSizeHUAWEI", (PFN_vkVoidFunction)&vkGetDeviceSubpassShadingMaxWorkgroupSizeHUAWEI},
    {"vkGetDisplayModeProperties2KHR", (PFN_vkVoidFunction)&vkGetDisplayModeProperties2KHR},
    {"vkGetDisplayModePropertiesKHR", (PFN_vkVoidFunction)&vkGetDisplayModePropertiesKHR},
    {"vkGetDisplayPlaneCapabilities2KHR", (PFN_vkVoidFunction)&vkGetDisplayPlaneCapabilities2KHR},
    {"vkGetDisplayPlaneCapabilitiesKHR", (PFN_vkVoidFunction)&vkGetDisplayPlaneCapabilitiesKHR},
    {"vkGetDisplayPlaneSupportedDisplaysKHR", (PFN_vkVoidFunction)&vkGetDisplayPlaneSupportedDisplaysKHR},
    {"vkGetDrmDisplayEXT", (PFN_vkVoidFunction)&vkGetDrmDisplayEXT},
    {"vkGetDynamicRenderingTilePropertiesQCOM", (PFN_vkVoidFunction)&vkGetDynamicRenderingTilePropertiesQCOM},
    {"vkGetEncodedVideoSessionParametersKHR", (PFN_vkVoidFunction)&vkGetEncodedVideoSessionParametersKHR},
    {"vkGetEventStatus", (PFN_vkVoidFunction)&vkGetEventStatus},
    {"vkGetFenceFdKHR", (PFN_vkVoidFunction)&vkGetFenceFdKHR},
    {"vkGetFenceStatus", (PFN_vkVoidFunction)&vkGetFenceStatus},
    {"vkGetFramebufferTilePropertiesQCOM", (PFN_vkVoidFunction)&vkGetFramebufferTilePropertiesQCOM},
    {"vkGetGeneratedCommandsMemoryRequirementsNV", (PFN_vkVoidFunction)&vkGetGeneratedCommandsMemoryRequirementsNV},
    {"vkGetImageDrmFormatModifierPropertiesEXT", (PFN_vkVoidFunction)&vkGetImageDrmFormatModifierPropertiesEXT},
    {"vkGetImageMemoryRequirements", (PFN_vkVoidFunction)&vkGetImageMemoryRequirements},
    {"vkGetImageMemoryRequirements2", (PFN_vkVoidFunction)&vkGetImageMemoryRequirements2},
    {"vkGetImageMemoryRequirements2KHR", (PFN_vkVoidFunction)&vkGetImageMemoryRequirements2KHR},
    {"vkGetImageOpaqueCaptureDescriptorDataEXT", (PFN_vkVoidFunction)&vkGetImageOpaqueCaptureDescriptorDataEXT},
    {"vkGetImageSparseMemoryRequirements", (PFN_vkVoidFunction)&vkGetImageSparseMemoryRequirements},
    {"vkGetImageSparseMemoryRequirements2", (PFN_vkVoidFunction)&vkGetImageSparseMemoryRequirements2},
    {"vkGetImageSparseMemoryRequirements2KHR", (PFN_vkVoidFunction)&vkGetImageSparseMemoryRequirements2KHR},
    {"vkGetImageSubresourceLayout", (PFN_vkVoidFunction)&vkGetImageSubresourceLayout},
    {"vkGetImageSubresourceLayout2EXT", (PFN_vkVoidFunction)&vkGetImageSubresourceLayout2EXT},
    {"vkGetImageSubresourceLayout2KHR", (PFN_vkVoidFunction)&vkGetImageSubresourceLayout2KHR},
    {"vkGetImageViewAddressNVX", (PFN_vkVoidFunction)&vkGetImageViewAddressNVX},
    {"vkGetImageViewHandleNVX", (PFN_vkVoidFunction)&vkGetImageViewHandleNVX},
    {"vkGetImageViewOpaqueCaptureDescriptorDataEXT", (PFN_vkVoidFunction)&vkGetImageViewOpaqueCaptureDescriptorDataEXT},
    {"vkGetLatencyTimingsNV", (PFN_vkVoidFunction)&vkGetLatencyTimingsNV},
    {"vkGetMemoryAndroidHardwareBufferANDROID", (PFN_vkVoidFunction)&vkGetMemoryAndroidHardwareBufferANDROID},
    {"vkGetMemoryFdKHR", (PFN_vkVoidFunction)&vkGetMemoryFdKHR},
    {"vkGetMemoryFdPropertiesKHR", (PFN_vkVoidFunction)&vkGetMemoryFdPropertiesKHR},
    {"vkGetMemoryHostPointerPropertiesEXT", (PFN_vkVoidFunction)&vkGetMemoryHostPointerPropertiesEXT},
    {"vkGetMemoryRemoteAddressNV", (PFN_vkVoidFunction)&vkGetMemoryRemoteAddressNV},
    {"vkGetMicromapBuildSizesEXT", (PFN_vkVoidFunction)&vkGetMicromapBuildSizesEXT},
    {"vkGetPastPresentationTimingGOOGLE", (PFN_vkVoidFunction)&vkGetPastPresentationTimingGOOGLE},
    {"vkGetPerformanceParameterINTEL", (PFN_vkVoidFunction)&vkGetPerformanceParameterINTEL},
    {"vkGetPhysicalDeviceCalibrateableTimeDomainsEXT", (PFN_vkVoidFunction)&vkGetPhysicalDeviceCalibrateableTimeDomainsEXT},
    {"vkGetPhysicalDeviceCalibrateableTimeDomainsKHR", (PFN_vkVoidFunction)&vkGetPhysicalDeviceCalibrateableTimeDomainsKHR},
    {"vkGetPhysicalDeviceCooperativeMatrixPropertiesKHR", (PFN_vkVoidFunction)&vkGetPhysicalDeviceCooperativeMatrixPropertiesKHR},
    {"vkGetPhysicalDeviceCooperativeMatrixPropertiesNV", (PFN_vkVoidFunction)&vkGetPhysicalDeviceCooperativeMatrixPropertiesNV},
    {"vkGetPhysicalDeviceDisplayPlaneProperties2KHR", (PFN_vkVoidFunction)&vkGetPhysicalDeviceDisplayPlaneProperties2KHR},
    {"vkGetPhysicalDeviceDisplayPlanePropertiesKHR", (PFN_vkVoidFunction)&vkGetPhysicalDeviceDisplayPlanePropertiesKHR},
    {"vkGetPhysicalDeviceDisplayProperties2KHR", (PFN_vkVoidFunction)&vkGetPhysicalDeviceDisplayProperties2KHR},
    {"vkGetPhysicalDeviceDisplayPropertiesKHR", (PFN_vkVoidFunction)&vkGetPhysicalDeviceDisplayPropertiesKHR},
    {"vkGetPhysicalDeviceExternalBufferProperties", (PFN_vkVoidFunction)&vkGetPhysicalDeviceExternalBufferProperties},
    {"vkGetPhysicalDeviceExternalBufferPropertiesKHR", (PFN_vkVoidFunction)&vkGetPhysicalDeviceExternalBufferPropertiesKHR},
    {"vkGetPhysicalDeviceExternalFenceProperties", (PFN_vkVoidFunction)&vkGetPhysicalDeviceExternalFenceProperties},
    {"vkGetPhysicalDeviceExternalFencePropertiesKHR", (PFN_vkVoidFunction)&vkGetPhysicalDeviceExternalFencePropertiesKHR},
    {"vkGetPhysicalDeviceExternalImageFormatPropertiesNV", (PFN_vkVoidFunction)&vkGetPhysicalDeviceExternalImageFormatPropertiesNV},
    {"vkGetPhysicalDeviceExternalSemaphoreProperties", (PFN_vkVoidFunction)&vkGetPhysicalDeviceExternalSemaphoreProperties},
    {"vkGetPhysicalDeviceExternalSemaphorePropertiesKHR", (PFN_vkVoidFunction)&vkGetPhysicalDeviceExternalSemaphorePropertiesKHR},
    {"vkGetPhysicalDeviceFeatures", (PFN_vkVoidFunction)&vkGetPhysicalDeviceFeatures},
    {"vkGetPhysicalDeviceFeatures2", (PFN_vkVoidFunction)&vkGetPhysicalDeviceFeatures2},
    {"vkGetPhysicalDeviceFeatures2KHR", (PFN_vkVoidFunction)&vkGetPhysicalDeviceFeatures2KHR},
    {"vkGetPhysicalDeviceFormatProperties", (PFN_vkVoidFunction)&vkGetPhysicalDeviceFormatProperties},
    {"vkGetPhysicalDeviceFormatProperties2", (PFN_vkVoidFunction)&vkGetPhysicalDeviceFormatProperties2},
    {"vkGetPhysicalDeviceFormatProperties2KHR", (PFN_vkVoidFunction)&vkGetPhysicalDeviceFormatProperties2KHR},
    {"vkGetPhysicalDeviceFragmentShadingRatesKHR", (PFN_vkVoidFunction)&vkGetPhysicalDeviceFragmentShadingRatesKHR},
    {"vkGetPhysicalDeviceImageFormatProperties", (PFN_vkVoidFunction)&vkGetPhysicalDeviceImageFormatProperties},
    {"vkGetPhysicalDeviceImageFormatProperties2", (PFN_vkVoidFunction)&vkGetPhysicalDeviceImageFormatProperties2},
    {"vkGetPhysicalDeviceImageFormatProperties2KHR", (PFN_vkVoidFunction)&vkGetPhysicalDeviceImageFormatProperties2KHR},
    {"vkGetPhysicalDeviceMemoryProperties", (PFN_vkVoidFunction)&vkGetPhysicalDeviceMemoryProperties},
    {"vkGetPhysicalDeviceMemoryProperties2", (PFN_vkVoidFunction)&vkGetPhysicalDeviceMemoryProperties2},
    {"vkGetPhysicalDeviceMemoryProperties2KHR", (PFN_vkVoidFunction)&vkGetPhysicalDeviceMemoryProperties2KHR},
    {"vkGetPhysicalDeviceMultisamplePropertiesEXT", (PFN_vkVoidFunction)&vkGetPhysicalDeviceMultisamplePropertiesEXT},
    {"vkGetPhysicalDeviceOpticalFlowImageFormatsNV", (PFN_vkVoidFunction)&vkGetPhysicalDeviceOpticalFlowImageFormatsNV},
    {"vkGetPhysicalDevicePresentRectanglesKHR", (PFN_vkVoidFunction)&vkGetPhysicalDevicePresentRectanglesKHR},
    {"vkGetPhysicalDeviceProperties", (PFN_vkVoidFunction)&vkGetPhysicalDeviceProperties},
    {"vkGetPhysicalDeviceProperties2", (PFN_vkVoidFunction)&vkGetPhysicalDeviceProperties2},
    {"vkGetPhysicalDeviceProperties2KHR", (PFN_vkVoidFunction)&vkGetPhysicalDeviceProperties2KHR},
    {"vkGetPhysicalDeviceQueueFamilyPerformanceQueryPassesKHR", (PFN_vkVoidFunction)&vkGetPhysicalDeviceQueueFamilyPerformanceQueryPassesKHR},
    {"vkGetPhysicalDeviceQueueFamilyProperties", (PFN_vkVoidFunction)&vkGetPhysicalDeviceQueueFamilyProperties},
    {"vkGetPhysicalDeviceQueueFamilyProperties2", (PFN_vkVoidFunction)&vkGetPhysicalDeviceQueueFamilyProperties2},
    {"vkGetPhysicalDeviceQueueFamilyProperties2KHR", (PFN_vkVoidFunction)&vkGetPhysicalDeviceQueueFamilyProperties2KHR},
    {"vkGetPhysicalDeviceSparseImageFormatProperties", (PFN_vkVoidFunction)&vkGetPhysicalDeviceSparseImageFormatProperties},
    {"vkGetPhysicalDeviceSparseImageFormatProperties2", (PFN_vkVoidFunction)&vkGetPhysicalDeviceSparseImageFormatProperties2},
    {"vkGetPhysicalDeviceSparseImageFormatProperties2KHR", (PFN_vkVoidFunction)&vkGetPhysicalDeviceSparseImageFormatProperties2KHR},
    {"vkGetPhysicalDeviceSupportedFramebufferMixedSamplesCombinationsNV", (PFN_vkVoidFunction)&vkGetPhysicalDeviceSupportedFramebufferMixedSamplesCombinationsNV},
    {"vkGetPhysicalDeviceSurfaceCapabilities2EXT", (PFN_vkVoidFunction)&vkGetPhysicalDeviceSurfaceCapabilities2EXT},
    {"vkGetPhysicalDeviceSurfaceCapabilities2KHR", (PFN_vkVoidFunction)&vkGetPhysicalDeviceSurfaceCapabilities2KHR},
    {"vkGetPhysicalDeviceSurfaceCapabilitiesKHR", (PFN_vkVoidFunction)&vkGetPhysicalDeviceSurfaceCapabilitiesKHR},
    {"vkGetPhysicalDeviceSurfaceFormats2KHR", (PFN_vkVoidFunction)&vkGetPhysicalDeviceSurfaceFormats2KHR},
    {"vkGetPhysicalDeviceSurfaceFormatsKHR", (PFN_vkVoidFunction)&vkGetPhysicalDeviceSurfaceFormatsKHR},
    {"vkGetPhysicalDeviceSurfacePresentModesKHR", (PFN_vkVoidFunction)&vkGetPhysicalDeviceSurfacePresentModesKHR},
    {"vkGetPhysicalDeviceSurfaceSupportKHR", (PFN_vkVoidFunction)&vkGetPhysicalDeviceSurfaceSupportKHR},
    {"vkGetPhysicalDeviceToolProperties", (PFN_vkVoidFunction)&vkGetPhysicalDeviceToolProperties},
    {"vkGetPhysicalDeviceToolPropertiesEXT", (PFN_vkVoidFunction)&vkGetPhysicalDeviceToolPropertiesEXT},
    {"vkGetPhysicalDeviceVideoCapabilitiesKHR", (PFN_vkVoidFunction)&vkGetPhysicalDeviceVideoCapabilitiesKHR},
    {"vkGetPhysicalDeviceVideoEncodeQualityLevelPropertiesKHR", (PFN_vkVoidFunction)&vkGetPhysicalDeviceVideoEncodeQualityLevelPropertiesKHR},
    {"vkGetPhysicalDeviceVideoFormatPropertiesKHR", (PFN_vkVoidFunction)&vkGetPhysicalDeviceVideoFormatPropertiesKHR},
    {"vkGetPipelineCacheData", (PFN_vkVoidFunction)&vkGetPipelineCacheData},
    {"vkGetPipelineExecutableInternalRepresentationsKHR", (PFN_vkVoidFunction)&vkGetPipelineExecutableInternalRepresentationsKHR},
    {"vkGetPipelineExecutablePropertiesKHR", (PFN_vkVoidFunction)&vkGetPipelineExecutablePropertiesKHR},
    {"vkGetPipelineExecutableStatisticsKHR", (PFN_vkVoidFunction)&vkGetPipelineExecutableStatisticsKHR},
    {"vkGetPipelineIndirectDeviceAddressNV", (PFN_vkVoidFunction)&vkGetPipelineIndirectDeviceAddressNV},
    {"vkGetPipelineIndirectMemoryRequirementsNV", (PFN_vkVoidFunction)&vkGetPipelineIndirectMemoryRequirementsNV},
    {"vkGetPipelinePropertiesEXT", (PFN_vkVoidFunction)&vkGetPipelinePropertiesEXT},
    {"vkGetPrivateData", (PFN_vkVoidFunction)&vkGetPrivateData},
    {"vkGetPrivateDataEXT", (PFN_vkVoidFunction)&vkGetPrivateDataEXT},
    {"vkGetQueryPoolResults", (PFN_vkVoidFunction)&vkGetQueryPoolResults},
    {"vkGetQueueCheckpointData2NV", (PFN_vkVoidFunction)&vkGetQueueCheckpointData2NV},
    {"vkGetQueueCheckpointDataNV", (PFN_vkVoidFunction)&vkGetQueueCheckpointDataNV},
    {"vkGetRayTracingCaptureReplayShaderGroupHandlesKHR", (PFN_vkVoidFunction)&vkGetRayTracingCaptureReplayShaderGroupHandlesKHR},
    {"vkGetRayTracingShaderGroupHandlesKHR", (PFN_vkVoidFunction)&vkGetRayTracingShaderGroupHandlesKHR},
    {"vkGetRayTracingShaderGroupHandlesNV", (PFN_vkVoidFunction)&vkGetRayTracingShaderGroupHandlesNV},
    {"vkGetRayTracingShaderGroupStackSizeKHR", (PFN_vkVoidFunction)&vkGetRayTracingShaderGroupStackSizeKHR},
    {"vkGetRefreshCycleDurationGOOGLE", (PFN_vkVoidFunction)&vkGetRefreshCycleDurationGOOGLE},
    {"vkGetRenderAreaGranularity", (PFN_vkVoidFunction)&vkGetRenderAreaGranularity},
    {"vkGetRenderingAreaGranularityKHR", (PFN_vkVoidFunction)&vkGetRenderingAreaGranularityKHR},
    {"vkGetSamplerOpaqueCaptureDescriptorDataEXT", (PFN_vkVoidFunction)&vkGetSamplerOpaqueCaptureDescriptorDataEXT},
    {"vkGetSemaphoreCounterValue", (PFN_vkVoidFunction)&vkGetSemaphoreCounterValue},
    {"vkGetSemaphoreCounterValueKHR", (PFN_vkVoidFunction)&vkGetSemaphoreCounterValueKHR},
    {"vkGetSemaphoreFdKHR", (PFN_vkVoidFunction)&vkGetSemaphoreFdKHR},
    {"vkGetShaderBinaryDataEXT", (PFN_vkVoidFunction)&vkGetShaderBinaryDataEXT},
    {"vkGetShaderInfoAMD", (PFN_vkVoidFunction)&vkGetShaderInfoAMD},
    {"vkGetShaderModuleCreateInfoIdentifierEXT", (PFN_vkVoidFunction)&vkGetShaderModuleCreateInfoIdentifierEXT},
    {"vkGetShaderModuleIdentifierEXT", (PFN_vkVoidFunction)&vkGetShaderModuleIdentifierEXT},
    {"vkGetSwapchainCounterEXT", (PFN_vkVoidFunction)&vkGetSwapchainCounterEXT},
    {"vkGetSwapchainImagesKHR", (PFN_vkVoidFunction)&vkGetSwapchainImagesKHR},
    {"vkGetSwapchainStatusKHR", (PFN_vkVoidFunction)&vkGetSwapchainStatusKHR},
    {"vkGetValidationCacheDataEXT", (PFN_vkVoidFunction)&vkGetValidationCacheDataEXT},
    {"vkGetVideoSessionMemoryRequirementsKHR", (PFN_vkVoidFunction)&vkGetVideoSessionMemoryRequirementsKHR},
    {"vkImportFenceFdKHR", (PFN_vkVoidFunction)&vkImportFenceFdKHR},
    {"vkImportSemaphoreFdKHR", (PFN_vkVoidFunction)&vkImportSemaphoreFdKHR},
    {"vkInitializePerformanceApiINTEL", (PFN_vkVoidFunction)&vkInitializePerformanceApiINTEL},
    {"vkInvalidateMappedMemoryRanges", (PFN_vkVoidFunction)&vkInvalidateMappedMemoryRanges},
    {"vkLatencySleepNV", (PFN_vkVoidFunction)&vkLatencySleepNV},
    {"vkMapMemory", (PFN_vkVoidFunction)&vkMapMemory},
    {"vkMapMemory2KHR", (PFN_vkVoidFunction)&vkMapMemory2KHR},
    {"vkMergePipelineCaches", (PFN_vkVoidFunction)&vkMergePipelineCaches},
    {"vkMergeValidationCachesEXT", (PFN_vkVoidFunction)&vkMergeValidationCachesEXT},
    {"vkQueueBeginDebugUtilsLabelEXT", (PFN_vkVoidFunction)&vkQueueBeginDebugUtilsLabelEXT},
    {"vkQueueBindSparse", (PFN_vkVoidFunction)&vkQueueBindSparse},
    {"vkQueueEndDebugUtilsLabelEXT", (PFN_vkVoidFunction)&vkQueueEndDebugUtilsLabelEXT},
    {"vkQueueInsertDebugUtilsLabelEXT", (PFN_vkVoidFunction)&vkQueueInsertDebugUtilsLabelEXT},
    {"vkQueueNotifyOutOfBandNV", (PFN_vkVoidFunction)&vkQueueNotifyOutOfBandNV},
    {"vkQueuePresentKHR", (PFN_vkVoidFunction)&vkQueuePresentKHR},
    {"vkQueueSetPerformanceConfigurationINTEL", (PFN_vkVoidFunction)&vkQueueSetPerformanceConfigurationINTEL},
    {"vkQueueSubmit", (PFN_vkVoidFunction)&vkQueueSubmit},
    {"vkQueueSubmit2", (PFN_vkVoidFunction)&vkQueueSubmit2},
    {"vkQueueSubmit2KHR", (PFN_vkVoidFunction)&vkQueueSubmit2KHR},
    {"vkQueueWaitIdle", (PFN_vkVoidFunction)&vkQueueWaitIdle},
    {"vkRegisterDeviceEventEXT", (PFN_vkVoidFunction)&vkRegisterDeviceEventEXT},
    {"vkRegisterDisplayEventEXT", (PFN_vkVoidFunction)&vkRegisterDisplayEventEXT},
    {"vkReleaseDisplayEXT", (PFN_vkVoidFunction)&vkReleaseDisplayEXT},
    {"vkReleasePerformanceConfigurationINTEL", (PFN_vkVoidFunction)&vkReleasePerformanceConfigurationINTEL},
    {"vkReleaseProfilingLockKHR", (PFN_vkVoidFunction)&vkReleaseProfilingLockKHR},
    {"vkReleaseSwapchainImagesEXT", (PFN_vkVoidFunction)&vkReleaseSwapchainImagesEXT},
    {"vkResetCommandBuffer", (PFN_vkVoidFunction)&vkResetCommandBuffer},
    {"vkResetCommandPool", (PFN_vkVoidFunction)&vkResetCommandPool},
    {"vkResetDescriptorPool", (PFN_vkVoidFunction)&vkResetDescriptorPool},
    {"vkResetEvent", (PFN_vkVoidFunction)&vkResetEvent},
    {"vkResetFences", (PFN_vkVoidFunction)&vkResetFences},
    {"vkResetQueryPool", (PFN_vkVoidFunction)&vkResetQueryPool},
    {"vkResetQueryPoolEXT", (PFN_vkVoidFunction)&vkResetQueryPoolEXT},
    {"vkSetDebugUtilsObjectNameEXT", (PFN_vkVoidFunction)&vkSetDebugUtilsObjectNameEXT},
    {"vkSetDebugUtilsObjectTagEXT", (PFN_vkVoidFunction)&vkSetDebugUtilsObjectTagEXT},
    {"vkSetDeviceMemoryPriorityEXT", (PFN_vkVoidFunction)&vkSetDeviceMemoryPriorityEXT},
    {"vkSetEvent", (PFN_vkVoidFunction)&vkSetEvent},
    {"vkSetHdrMetadataEXT", (PFN_vkVoidFunction)&vkSetHdrMetadataEXT},
    {"vkSetLatencyMarkerNV", (PFN_vkVoidFunction)&vkSetLatencyMarkerNV},
    {"vkSetLatencySleepModeNV", (PFN_vkVoidFunction)&vkSetLatencySleepModeNV},
    {"vkSetLocalDimmingAMD", (PFN_vkVoidFunction)&vkSetLocalDimmingAMD},
    {"vkSetPrivateData", (PFN_vkVoidFunction)&vkSetPrivateData},
    {"vkSetPrivateDataEXT", (PFN_vkVoidFunction)&vkSetPrivateDataEXT},
    {"vkSignalSemaphore", (PFN_vkVoidFunction)&vkSignalSemaphore},
    {"vkSignalSemaphoreKHR", (PFN_vkVoidFunction)&vkSignalSemaphoreKHR},
    {"vkSubmitDebugUtilsMessageEXT", (PFN_vkVoidFunction)&vkSubmitDebugUtilsMessageEXT},
    {"vkTransitionImageLayoutEXT", (PFN_vkVoidFunction)&vkTransitionImageLayoutEXT},
    {"vkTrimCommandPool", (PFN_vkVoidFunction)&vkTrimCommandPool},
    {"vkTrimCommandPoolKHR", (PFN_vkVoidFunction)&vkTrimCommandPoolKHR},
    {"vkUninitializePerformanceApiINTEL", (PFN_vkVoidFunction)&vkUninitializePerformanceApiINTEL},
    {"vkUnmapMemory", (PFN_vkVoidFunction)&vkUnmapMemory},
    {"vkUnmapMemory2KHR", (PFN_vkVoidFunction)&vkUnmapMemory2KHR},
    {"vkUpdateDescriptorSetWithTemplate", (PFN_vkVoidFunction)&vkUpdateDescriptorSetWithTemplate},
    {"vkUpdateDescriptorSetWithTemplateKHR", (PFN_vkVoidFunction)&vkUpdateDescriptorSetWithTemplateKHR},
    {"vkUpdateDescriptorSets", (PFN_vkVoidFunction)&vkUpdateDescriptorSets},
    {"vkUpdateVideoSessionParametersKHR", (PFN_vkVoidFunction)&vkUpdateVideoSessionParametersKHR},
    {"vkWaitForFences", (PFN_vkVoidFunction)&vkWaitForFences},
    {"vkWaitForPresentKHR", (PFN_vkVoidFunction)&vkWaitForPresentKHR},
    {"vkWaitSemaphores", (PFN_vkVoidFunction)&vkWaitSemaphores},
    {"vkWaitSemaphoresKHR", (PFN_vkVoidFunction)&vkWaitSemaphoresKHR},
    {"vkWriteAccelerationStructuresPropertiesKHR", (PFN_vkVoidFunction)&vkWriteAccelerationStructuresPropertiesKHR},
    {"vkWriteMicromapsPropertiesEXT", (PFN_vkVoidFunction)&vkWriteMicromapsPropertiesEXT},
    {nullptr, nullptr},
};

extern "C" VKAPI_ATTR PFN_vkVoidFunction VKAPI_CALL vkGetInstanceProcAddr(VkInstance instance, const char* name) {
    if (name == nullptr) return nullptr;
    for (const NameEntry* e = g_entries; e->name != nullptr; ++e) {
        if (std::strcmp(e->name, name) == 0) return e->fn;
    }
    auto backend = backend_gipa();
    return backend != nullptr ? backend(instance != VK_NULL_HANDLE ? instance : g_instance, name) : nullptr;
}

