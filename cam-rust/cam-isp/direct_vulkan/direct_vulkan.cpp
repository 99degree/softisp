// Direct Vulkan compute path for RAW16 AHB byte containers.
//
// This module deliberately does not use MNN. It imports AHardwareBuffer objects
// as VkBuffer-backed external memory, runs one compute shader, and waits on a
// fence. Imports are cached per AHardwareBuffer pointer so the per-frame path
// is descriptor writes + one dispatch, with no host upload and no AHB lock.

#include <vulkan/vulkan.h>
#include <vulkan/vulkan_android.h>

#include <algorithm>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "raw16_bgra_spv.inc"

namespace {

thread_local std::string g_last_error;

bool fail(const char* fmt, ...) {
    char buf[512];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    g_last_error = buf;
    return false;
}

#define DV_CHECK(call, what)                                                     \
    do {                                                                         \
        VkResult dv_check_rc = (call);                                           \
        if (dv_check_rc != VK_SUCCESS) {                                         \
            return fail("%s: VkResult=%d", (what), (int)dv_check_rc);            \
        }                                                                        \
    } while (0)

struct Buffer {
    VkBuffer buffer = VK_NULL_HANDLE;
    VkDeviceMemory memory = VK_NULL_HANDLE;
    VkDeviceSize logical_size = 0;
    VkDeviceSize alloc_size = 0;
};

struct AhbCacheEntry {
    void* ahb = nullptr;
    Buffer buffer;
};

struct Context {
    VkInstance instance = VK_NULL_HANDLE;
    VkPhysicalDevice physical = VK_NULL_HANDLE;
    VkDevice device = VK_NULL_HANDLE;
    VkQueue queue = VK_NULL_HANDLE;
    uint32_t queue_family = 0;
    bool ahb_supported = false;

    VkCommandPool command_pool = VK_NULL_HANDLE;
    VkCommandBuffer command_buffer = VK_NULL_HANDLE;
    VkFence fence = VK_NULL_HANDLE;
    VkDescriptorPool descriptor_pool = VK_NULL_HANDLE;
    VkDescriptorSet descriptor_set = VK_NULL_HANDLE;
    VkDescriptorSetLayout descriptor_layout = VK_NULL_HANDLE;
    VkPipelineLayout pipeline_layout = VK_NULL_HANDLE;
    VkPipeline pipeline = VK_NULL_HANDLE;

    VkBuffer bound_input = VK_NULL_HANDLE;
    VkBuffer bound_output = VK_NULL_HANDLE;
    std::vector<AhbCacheEntry> ahb_cache;

    PFN_vkGetAndroidHardwareBufferPropertiesANDROID get_ahb_properties = nullptr;
};

const char* required_device_extensions[] = {
    VK_KHR_EXTERNAL_MEMORY_EXTENSION_NAME,
    VK_ANDROID_EXTERNAL_MEMORY_ANDROID_HARDWARE_BUFFER_EXTENSION_NAME,
    VK_KHR_GET_MEMORY_REQUIREMENTS_2_EXTENSION_NAME,
    VK_KHR_DEDICATED_ALLOCATION_EXTENSION_NAME,
    VK_KHR_BIND_MEMORY_2_EXTENSION_NAME,
};

bool has_device_extension(VkPhysicalDevice physical, const char* name) {
    uint32_t count = 0;
    vkEnumerateDeviceExtensionProperties(physical, nullptr, &count, nullptr);
    std::vector<VkExtensionProperties> props(count);
    vkEnumerateDeviceExtensionProperties(physical, nullptr, &count, props.data());
    return std::any_of(props.begin(), props.end(), [name](const VkExtensionProperties& p) {
        return std::strcmp(p.extensionName, name) == 0;
    });
}

uint32_t compute_queue_family(VkPhysicalDevice physical) {
    uint32_t count = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(physical, &count, nullptr);
    std::vector<VkQueueFamilyProperties> families(count);
    vkGetPhysicalDeviceQueueFamilyProperties(physical, &count, families.data());
    for (uint32_t i = 0; i < count; ++i) {
        if ((families[i].queueFlags & VK_QUEUE_COMPUTE_BIT) != 0) {
            return i;
        }
    }
    return UINT32_MAX;
}

uint32_t device_score(VkPhysicalDevice physical) {
    VkPhysicalDeviceProperties props{};
    vkGetPhysicalDeviceProperties(physical, &props);
    if (compute_queue_family(physical) == UINT32_MAX) {
        return 0;
    }
    for (const char* name : required_device_extensions) {
        if (!has_device_extension(physical, name)) {
            return 0;
        }
    }
    switch (props.deviceType) {
        case VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU:
            return 3;
        case VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU:
            return 2;
        case VK_PHYSICAL_DEVICE_TYPE_CPU:
            return 1;
        default:
            return 2;
    }
}

void destroy_buffer(Context* ctx, Buffer* buf) {
    if (!ctx || !ctx->device || !buf) {
        return;
    }
    if (buf->buffer != VK_NULL_HANDLE) {
        vkDestroyBuffer(ctx->device, buf->buffer, nullptr);
        buf->buffer = VK_NULL_HANDLE;
    }
    if (buf->memory != VK_NULL_HANDLE) {
        vkFreeMemory(ctx->device, buf->memory, nullptr);
        buf->memory = VK_NULL_HANDLE;
    }
    buf->logical_size = 0;
    buf->alloc_size = 0;
}

bool create_ahb_buffer(Context* ctx, void* ahb, VkDeviceSize logical_size, Buffer* out) {
    if (!ctx->ahb_supported || !ctx->get_ahb_properties) {
        return fail("AHB external-memory Vulkan extensions unavailable");
    }
    if (!ahb || logical_size == 0) {
        return fail("invalid AHardwareBuffer or zero size");
    }

    VkAndroidHardwareBufferPropertiesANDROID props{};
    props.sType = VK_STRUCTURE_TYPE_ANDROID_HARDWARE_BUFFER_PROPERTIES_ANDROID;
    DV_CHECK(ctx->get_ahb_properties(ctx->device, static_cast<struct AHardwareBuffer*>(ahb), &props),
             "vkGetAndroidHardwareBufferPropertiesANDROID");

    VkExternalMemoryBufferCreateInfo external{};
    external.sType = VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_BUFFER_CREATE_INFO;
    external.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_ANDROID_HARDWARE_BUFFER_BIT_ANDROID;

    VkBufferCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    info.pNext = &external;
    info.size = logical_size;
    info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                 VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
    info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    DV_CHECK(vkCreateBuffer(ctx->device, &info, nullptr, &out->buffer), "vkCreateBuffer(AHB)");

    VkMemoryRequirements req{};
    vkGetBufferMemoryRequirements(ctx->device, out->buffer, &req);

    VkPhysicalDeviceMemoryProperties mem_props{};
    vkGetPhysicalDeviceMemoryProperties(ctx->physical, &mem_props);

    VkImportAndroidHardwareBufferInfoANDROID import{};
    import.sType = VK_STRUCTURE_TYPE_IMPORT_ANDROID_HARDWARE_BUFFER_INFO_ANDROID;
    import.buffer = static_cast<struct AHardwareBuffer*>(ahb);

    VkMemoryDedicatedAllocateInfo dedicated{};
    dedicated.sType = VK_STRUCTURE_TYPE_MEMORY_DEDICATED_ALLOCATE_INFO;
    dedicated.pNext = &import;
    dedicated.buffer = out->buffer;

    VkDeviceSize allocation_size = std::max<VkDeviceSize>(req.size, props.allocationSize);
    uint32_t candidates = req.memoryTypeBits & props.memoryTypeBits;
    bool allocated = false;
    for (uint32_t i = 0; i < 32 && !allocated; ++i) {
        if ((candidates & (1u << i)) == 0 || i >= mem_props.memoryTypeCount) {
            continue;
        }
        VkMemoryAllocateInfo alloc{};
        alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        alloc.pNext = &dedicated;
        alloc.allocationSize = allocation_size;
        alloc.memoryTypeIndex = i;
        VkResult rc = vkAllocateMemory(ctx->device, &alloc, nullptr, &out->memory);
        allocated = rc == VK_SUCCESS;
    }
    if (!allocated) {
        vkDestroyBuffer(ctx->device, out->buffer, nullptr);
        out->buffer = VK_NULL_HANDLE;
        return fail("AHB import failed: req=%llu ahb=%llu types=0x%x",
                    (unsigned long long)req.size, (unsigned long long)props.allocationSize,
                    (unsigned)candidates);
    }

    DV_CHECK(vkBindBufferMemory(ctx->device, out->buffer, out->memory, 0),
             "vkBindBufferMemory(AHB)");
    out->logical_size = logical_size;
    out->alloc_size = allocation_size;
    return true;
}

Buffer* get_ahb_buffer(Context* ctx, void* ahb, VkDeviceSize logical_size) {
    for (auto& entry : ctx->ahb_cache) {
        if (entry.ahb == ahb && entry.buffer.alloc_size >= logical_size) {
            entry.buffer.logical_size = logical_size;
            return &entry.buffer;
        }
    }
    Buffer fresh;
    if (!create_ahb_buffer(ctx, ahb, logical_size, &fresh)) {
        return nullptr;
    }
    for (auto& entry : ctx->ahb_cache) {
        if (entry.ahb == ahb) {
            destroy_buffer(ctx, &entry.buffer);
            entry.buffer = fresh;
            return &entry.buffer;
        }
    }
    ctx->ahb_cache.push_back(AhbCacheEntry{ahb, fresh});
    return &ctx->ahb_cache.back().buffer;
}

bool submit(Context* ctx, const Buffer& input, const Buffer& output, uint32_t width,
            uint32_t height, uint32_t mode) {
    VkWriteDescriptorSet writes[2]{};
    uint32_t write_count = 0;
    VkDescriptorBufferInfo in_info{};
    VkDescriptorBufferInfo out_info{};

    if (ctx->bound_input != input.buffer) {
        in_info.buffer = input.buffer;
        in_info.range = input.logical_size;
        writes[write_count].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[write_count].dstSet = ctx->descriptor_set;
        writes[write_count].dstBinding = 0;
        writes[write_count].descriptorCount = 1;
        writes[write_count].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[write_count].pBufferInfo = &in_info;
        ++write_count;
        ctx->bound_input = input.buffer;
    }
    if (ctx->bound_output != output.buffer) {
        out_info.buffer = output.buffer;
        out_info.range = output.logical_size;
        writes[write_count].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[write_count].dstSet = ctx->descriptor_set;
        writes[write_count].dstBinding = 1;
        writes[write_count].descriptorCount = 1;
        writes[write_count].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[write_count].pBufferInfo = &out_info;
        ++write_count;
        ctx->bound_output = output.buffer;
    }
    if (write_count != 0) {
        vkUpdateDescriptorSets(ctx->device, write_count, writes, 0, nullptr);
    }

    uint32_t params[3] = {width, height, mode};
    DV_CHECK(vkResetFences(ctx->device, 1, &ctx->fence), "vkResetFences");
    DV_CHECK(vkResetCommandBuffer(ctx->command_buffer, 0), "vkResetCommandBuffer");

    VkCommandBufferBeginInfo begin{};
    begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    DV_CHECK(vkBeginCommandBuffer(ctx->command_buffer, &begin), "vkBeginCommandBuffer");

    vkCmdBindPipeline(ctx->command_buffer, VK_PIPELINE_BIND_POINT_COMPUTE, ctx->pipeline);
    vkCmdPushConstants(ctx->command_buffer, ctx->pipeline_layout, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                       sizeof(params), params);
    vkCmdBindDescriptorSets(ctx->command_buffer, VK_PIPELINE_BIND_POINT_COMPUTE,
                            ctx->pipeline_layout, 0, 1, &ctx->descriptor_set, 0, nullptr);
    uint32_t words = (width * height + 1u) / 2u;
    vkCmdDispatch(ctx->command_buffer, (words + 255u) / 256u, 1, 1);
    DV_CHECK(vkEndCommandBuffer(ctx->command_buffer), "vkEndCommandBuffer");

    VkSubmitInfo submit_info{};
    submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submit_info.commandBufferCount = 1;
    submit_info.pCommandBuffers = &ctx->command_buffer;
    DV_CHECK(vkQueueSubmit(ctx->queue, 1, &submit_info, ctx->fence), "vkQueueSubmit");
    DV_CHECK(vkWaitForFences(ctx->device, 1, &ctx->fence, VK_TRUE, UINT64_MAX),
             "vkWaitForFences");
    return true;
}

bool create_pipeline(Context* ctx) {
    VkShaderModuleCreateInfo shader_info{};
    shader_info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    shader_info.codeSize = RAW16_BGRA_SPV_LEN;
    shader_info.pCode = reinterpret_cast<const uint32_t*>(RAW16_BGRA_SPV);
    VkShaderModule shader = VK_NULL_HANDLE;
    if (vkCreateShaderModule(ctx->device, &shader_info, nullptr, &shader) != VK_SUCCESS) {
        return fail("vkCreateShaderModule failed");
    }

    VkDescriptorSetLayoutBinding bindings[2]{};
    for (uint32_t i = 0; i < 2; ++i) {
        bindings[i].binding = i;
        bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        bindings[i].descriptorCount = 1;
        bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    }
    VkDescriptorSetLayoutCreateInfo layout_info{};
    layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layout_info.bindingCount = 2;
    layout_info.pBindings = bindings;
    if (vkCreateDescriptorSetLayout(ctx->device, &layout_info, nullptr, &ctx->descriptor_layout) !=
        VK_SUCCESS) {
        vkDestroyShaderModule(ctx->device, shader, nullptr);
        return fail("vkCreateDescriptorSetLayout failed");
    }

    VkDescriptorPoolSize pool_size{};
    pool_size.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    pool_size.descriptorCount = 2;
    VkDescriptorPoolCreateInfo pool_info{};
    pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    pool_info.maxSets = 1;
    pool_info.poolSizeCount = 1;
    pool_info.pPoolSizes = &pool_size;
    if (vkCreateDescriptorPool(ctx->device, &pool_info, nullptr, &ctx->descriptor_pool) !=
        VK_SUCCESS) {
        vkDestroyShaderModule(ctx->device, shader, nullptr);
        return fail("vkCreateDescriptorPool failed");
    }
    VkDescriptorSetAllocateInfo alloc_info{};
    alloc_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    alloc_info.descriptorPool = ctx->descriptor_pool;
    alloc_info.descriptorSetCount = 1;
    alloc_info.pSetLayouts = &ctx->descriptor_layout;
    if (vkAllocateDescriptorSets(ctx->device, &alloc_info, &ctx->descriptor_set) != VK_SUCCESS) {
        vkDestroyShaderModule(ctx->device, shader, nullptr);
        return fail("vkAllocateDescriptorSets failed");
    }

    VkPushConstantRange range{};
    range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    range.offset = 0;
    range.size = 12;
    VkPipelineLayoutCreateInfo pipeline_layout_info{};
    pipeline_layout_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipeline_layout_info.setLayoutCount = 1;
    pipeline_layout_info.pSetLayouts = &ctx->descriptor_layout;
    pipeline_layout_info.pushConstantRangeCount = 1;
    pipeline_layout_info.pPushConstantRanges = &range;
    if (vkCreatePipelineLayout(ctx->device, &pipeline_layout_info, nullptr, &ctx->pipeline_layout) !=
        VK_SUCCESS) {
        vkDestroyShaderModule(ctx->device, shader, nullptr);
        return fail("vkCreatePipelineLayout failed");
    }

    VkPipelineShaderStageCreateInfo stage{};
    stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    stage.module = shader;
    stage.pName = "main";
    VkComputePipelineCreateInfo pipeline_info{};
    pipeline_info.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    pipeline_info.stage = stage;
    pipeline_info.layout = ctx->pipeline_layout;
    if (vkCreateComputePipelines(ctx->device, VK_NULL_HANDLE, 1, &pipeline_info, nullptr,
                                 &ctx->pipeline) != VK_SUCCESS) {
        vkDestroyShaderModule(ctx->device, shader, nullptr);
        return fail("vkCreateComputePipelines failed");
    }
    vkDestroyShaderModule(ctx->device, shader, nullptr);
    return true;
}

} // namespace

extern "C" {

const char* dv_last_error(void) {
    return g_last_error.c_str();
}

void dv_context_destroy(void* handle) {
    auto* ctx = static_cast<Context*>(handle);
    if (!ctx) {
        return;
    }
    if (ctx->device != VK_NULL_HANDLE) {
        if (ctx->fence != VK_NULL_HANDLE) {
            vkWaitForFences(ctx->device, 1, &ctx->fence, VK_TRUE, UINT64_MAX);
        }
        for (auto& entry : ctx->ahb_cache) {
            destroy_buffer(ctx, &entry.buffer);
        }
        ctx->ahb_cache.clear();
        if (ctx->fence != VK_NULL_HANDLE) {
            vkDestroyFence(ctx->device, ctx->fence, nullptr);
        }
        if (ctx->pipeline != VK_NULL_HANDLE) {
            vkDestroyPipeline(ctx->device, ctx->pipeline, nullptr);
        }
        if (ctx->pipeline_layout != VK_NULL_HANDLE) {
            vkDestroyPipelineLayout(ctx->device, ctx->pipeline_layout, nullptr);
        }
        if (ctx->descriptor_pool != VK_NULL_HANDLE) {
            vkDestroyDescriptorPool(ctx->device, ctx->descriptor_pool, nullptr);
        }
        if (ctx->descriptor_layout != VK_NULL_HANDLE) {
            vkDestroyDescriptorSetLayout(ctx->device, ctx->descriptor_layout, nullptr);
        }
        if (ctx->command_pool != VK_NULL_HANDLE) {
            vkDestroyCommandPool(ctx->device, ctx->command_pool, nullptr);
        }
        vkDestroyDevice(ctx->device, nullptr);
    }
    if (ctx->instance != VK_NULL_HANDLE) {
        vkDestroyInstance(ctx->instance, nullptr);
    }
    delete ctx;
}

void* dv_context_create(void) {
    g_last_error.clear();
    auto* ctx = new Context();

    uint32_t api_version = VK_API_VERSION_1_0;
    auto enumerate_version = reinterpret_cast<PFN_vkEnumerateInstanceVersion>(
        vkGetInstanceProcAddr(nullptr, "vkEnumerateInstanceVersion"));
    if (enumerate_version != nullptr && enumerate_version(&api_version) != VK_SUCCESS) {
        api_version = VK_API_VERSION_1_0;
    }

    VkApplicationInfo app{};
    app.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    app.pApplicationName = "softisp-direct-vulkan";
    app.applicationVersion = 1;
    app.pEngineName = "softisp";
    app.engineVersion = 1;
    app.apiVersion = api_version;

    std::vector<VkExtensionProperties> instance_props;
    uint32_t instance_extension_count = 0;
    vkEnumerateInstanceExtensionProperties(nullptr, &instance_extension_count, nullptr);
    instance_props.resize(instance_extension_count);
    vkEnumerateInstanceExtensionProperties(nullptr, &instance_extension_count,
                                           instance_props.data());
    auto has_instance_extension = [&instance_props](const char* name) {
        return std::any_of(instance_props.begin(), instance_props.end(),
                           [name](const VkExtensionProperties& p) {
                               return std::strcmp(p.extensionName, name) == 0;
                           });
    };
    std::vector<const char*> instance_extensions;
    if (has_instance_extension(VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME)) {
        instance_extensions.push_back(VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME);
    }
    if (has_instance_extension(VK_KHR_EXTERNAL_MEMORY_CAPABILITIES_EXTENSION_NAME)) {
        instance_extensions.push_back(VK_KHR_EXTERNAL_MEMORY_CAPABILITIES_EXTENSION_NAME);
    }
    VkInstanceCreateFlags instance_flags = 0;
    if (has_instance_extension(VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME)) {
        instance_extensions.push_back(VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME);
        instance_flags |= VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR;
    }

    VkInstanceCreateInfo instance_info{};
    instance_info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    instance_info.flags = instance_flags;
    instance_info.pApplicationInfo = &app;
    instance_info.enabledExtensionCount = (uint32_t)instance_extensions.size();
    instance_info.ppEnabledExtensionNames =
        instance_extensions.empty() ? nullptr : instance_extensions.data();
    if (vkCreateInstance(&instance_info, nullptr, &ctx->instance) != VK_SUCCESS) {
        fail("vkCreateInstance failed");
        delete ctx;
        return nullptr;
    }

    uint32_t device_count = 0;
    vkEnumeratePhysicalDevices(ctx->instance, &device_count, nullptr);
    std::vector<VkPhysicalDevice> devices(device_count);
    vkEnumeratePhysicalDevices(ctx->instance, &device_count, devices.data());
    uint32_t best_score = 0;
    for (VkPhysicalDevice device : devices) {
        uint32_t score = device_score(device);
        if (score > best_score) {
            best_score = score;
            ctx->physical = device;
        }
    }
    if (ctx->physical == VK_NULL_HANDLE) {
        fail("no Vulkan device with AHB external memory and a compute queue");
        vkDestroyInstance(ctx->instance, nullptr);
        delete ctx;
        return nullptr;
    }
    ctx->queue_family = compute_queue_family(ctx->physical);
    ctx->ahb_supported = true;

    std::vector<const char*> enabled;
    for (const char* name : required_device_extensions) {
        enabled.push_back(name);
    }
    if (has_device_extension(ctx->physical, "VK_KHR_portability_subset")) {
        enabled.push_back("VK_KHR_portability_subset");
    }

    float priority = 1.0f;
    VkDeviceQueueCreateInfo queue_info{};
    queue_info.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
    queue_info.queueFamilyIndex = ctx->queue_family;
    queue_info.queueCount = 1;
    queue_info.pQueuePriorities = &priority;

    VkDeviceCreateInfo device_info{};
    device_info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    device_info.queueCreateInfoCount = 1;
    device_info.pQueueCreateInfos = &queue_info;
    device_info.enabledExtensionCount = (uint32_t)enabled.size();
    device_info.ppEnabledExtensionNames = enabled.data();
    if (vkCreateDevice(ctx->physical, &device_info, nullptr, &ctx->device) != VK_SUCCESS) {
        fail("vkCreateDevice failed");
        vkDestroyInstance(ctx->instance, nullptr);
        delete ctx;
        return nullptr;
    }
    vkGetDeviceQueue(ctx->device, ctx->queue_family, 0, &ctx->queue);

    ctx->get_ahb_properties = reinterpret_cast<PFN_vkGetAndroidHardwareBufferPropertiesANDROID>(
        vkGetDeviceProcAddr(ctx->device, "vkGetAndroidHardwareBufferPropertiesANDROID"));
    ctx->ahb_supported = ctx->get_ahb_properties != nullptr;

    if (!create_pipeline(ctx)) {
        dv_context_destroy(ctx);
        return nullptr;
    }

    VkCommandPoolCreateInfo command_pool_info{};
    command_pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    command_pool_info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    command_pool_info.queueFamilyIndex = ctx->queue_family;
    if (vkCreateCommandPool(ctx->device, &command_pool_info, nullptr, &ctx->command_pool) !=
        VK_SUCCESS) {
        fail("vkCreateCommandPool failed");
        dv_context_destroy(ctx);
        return nullptr;
    }
    VkCommandBufferAllocateInfo command_info{};
    command_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    command_info.commandPool = ctx->command_pool;
    command_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    command_info.commandBufferCount = 1;
    if (vkAllocateCommandBuffers(ctx->device, &command_info, &ctx->command_buffer) != VK_SUCCESS) {
        fail("vkAllocateCommandBuffers failed");
        dv_context_destroy(ctx);
        return nullptr;
    }

    VkFenceCreateInfo fence_info{};
    fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    if (vkCreateFence(ctx->device, &fence_info, nullptr, &ctx->fence) != VK_SUCCESS) {
        fail("vkCreateFence failed");
        dv_context_destroy(ctx);
        return nullptr;
    }

    return ctx;
}

int dv_run_ahb(void* handle, void* input_ahb, size_t input_len, void* output_ahb,
               size_t output_len, uint32_t width, uint32_t height, uint32_t mode) {
    g_last_error.clear();
    auto* ctx = static_cast<Context*>(handle);
    if (!ctx || !ctx->device) {
        fail("null direct Vulkan context");
        return -1;
    }
    VkDeviceSize need_in = ((VkDeviceSize)width * height + 1u) / 2u * 4u;
    VkDeviceSize need_out = (VkDeviceSize)width * height * 4u;
    if (!input_ahb || !output_ahb || input_len < (size_t)need_in ||
        output_len < (size_t)need_out) {
        fail("AHB too small: in=%zu need=%llu out=%zu need=%llu", input_len,
             (unsigned long long)need_in, output_len, (unsigned long long)need_out);
        return -1;
    }

    Buffer* input = get_ahb_buffer(ctx, input_ahb, need_in);
    if (input == nullptr) {
        return -1;
    }
    Buffer* output = get_ahb_buffer(ctx, output_ahb, need_out);
    if (output == nullptr) {
        return -1;
    }
    if (!submit(ctx, *input, *output, width, height, mode)) {
        return -1;
    }
    return 0;
}

} // extern "C"
