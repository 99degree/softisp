#include "vulkan/vulkan.h"
#include <chrono>
#include <vector>
#include <iostream>
#include <cstdio>
#include <cassert>
#include <cstdlib>

// Vulkan memory helpers
typedef struct { VkBuffer buf; VkDeviceMemory mem; } GpuBuffer;

// Global Vulkan handles
static VkDevice device = 0;
static VkPhysicalDevice physical = 0;
static VkQueue queue = 0;
static VkCommandPool cmd_pool = 0;
static VkDescriptorPool desc_pool = 0;
static uint32_t queue_index = 0;

static const uint32_t BUFFER_WIDTH = 3840*2;
static const uint32_t BUFFER_HEIGHT = 2160;

// SPIR-V shader variants:
struct Workgroup {
    const char* name;
    uint32_t size_x, size_y, size_z;
    GpuBuffer shader;
};

std::vector<Workgroup> variants;

GpuBuffer load_spv(const char* path, uint64_t elem_size) {
    FILE* fp = fopen(path, "rb");
    fseek(fp, 0, SEEK_END);
    size_t len = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    GpuBuffer res;
    void* mapped;
    VkBufferCreateInfo info = {VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    info.size = len * elem_size;
    info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC;
    info.queueFamilyIndexCount = 1;
    info.pQueueFamilyIndices = &queue_index;
    vkCreateBuffer(device, &info, 0, &res.buf);

    VkMemoryRequirements req;
    vkGetBufferMemoryRequirements(device, res.buf, &req);
    VkMemoryAllocateInfo alloc = {VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    alloc.allocationSize = req.size;
    alloc.memoryTypeIndex = 0; // Assume memory type 0 is visible & coherent
    vkAllocateMemory(device, &alloc, 0, &res.mem);
    vkBindBufferMemory(device, res.buf, res.mem, 0);
    vkMapMemory(device, res.mem, 0, len * elem_size, 0, &mapped);
    fread(mapped, elem_size, len, fp);
    vkUnmapMemory(device, res.mem);
    fclose(fp);
    return res;
}

void bench_workgroup(const Workgroup& wg) {
    // ... Vulkan setup: SSBO, descriptor set, pipeline, dispatch ...
    uint32_t batch = (BUFFER_WIDTH + wg.size_x - 1) / wg.size_x;
    uint32_t batch_y = (BUFFER_HEIGHT + wg.size_y - 1) / wg.size_y;

    auto start = std::chrono::steady_clock::now();
    vkCmdDispatch(cmd, batch, batch_y, 1);
    vk_end_cmd_fence(cmd);
    auto finish = std::chrono::steady_clock::now();
    auto ms = std::chrono::duration<double, std::milli>(finish - start).count();

    std::cout << wg.name << ": batch=" << batch << ", " << ms << " ms\n";
}

int main() {
    // Vulkan init
    VkInstance instance;
    VkResult res;
    const char* layers[] = { "VK_LAYER_KHRONOS_validation" };
    VkInstanceCreateInfo info = {VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
    info.enabledLayerCount = 1;
    info.ppEnabledLayerNames = layers;
    res = vkCreateInstance(&info, 0, &instance);
    assert(res == VK_SUCCESS);

    uint32_t phys_count;
    vkEnumeratePhysicalDevices(instance, &phys_count, 0);
    std::vector<VkPhysicalDevice> phys(phys_count);
    vkEnumeratePhysicalDevices(instance, &phys_count, phys.data());
    physical = phys[0];
    printf("Physical device: %p\n", physical);

    VkDeviceCreateInfo devinfo = {VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
    float prio = 0.f;
    VkDeviceQueueCreateInfo queueinfo = {VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
    queueinfo.queueCount = 1;
    queueinfo.pQueuePriorities = &prio;
    queueinfo.queueFamilyIndex = queue_index;
    devinfo.queueCreateInfoCount = 1;
    devinfo.pQueueCreateInfos = &queueinfo;
    vkCreateDevice(physical, &devinfo, 0, &device);
    vkGetDeviceQueue(device, queue_index, 0, &queue);

    // Load shaders
    variants.push_back({"16x16", 16, 16, 1, {}});
    variants.push_back({"16x32", 16, 32, 1, {}});
    variants.push_back({"32x16", 32, 16, 1, {}});
    variants.push_back({"32x32", 32, 32, 1, {}});
    variants.push_back({"64x4", 64, 4, 1, {}});
    variants.push_back({"4x64", 4, 64, 1, {}});
    variants.push_back({"8x8", 8, 8, 1, {}});

    for (auto& wg : variants) {
        char path[128];
        snprintf(path, sizeof(path), "shader_unpack_demosaic_%s.spv", wg.name);
        wg.shader = load_spv(path, 4);
    }

    // Benchmark each
    for (auto& wg : variants) {
        bench_workgroup(wg);
    }
    return 0;
}