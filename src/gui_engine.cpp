#include <GLFW/glfw3.h>
#include <algorithm>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_vulkan.h>
#include <iostream>
#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <stdexcept>
#include <string>
#include <vulkan/vulkan.hpp>

#include "gui_node/gui_engine.hpp"

namespace gui_node
{

bool GuiEngine::checkValidationLayerSupport()
{
    uint32_t layer_count;
    vkEnumerateInstanceLayerProperties(&layer_count, nullptr);
    std::vector<VkLayerProperties> available_layers(layer_count);
    vkEnumerateInstanceLayerProperties(&layer_count, available_layers.data());

    for (const std::string &layer_name : validation_layers)
    {
        bool layer_found = false;
        for (const VkLayerProperties &layer_properties : available_layers)
        {
            if (layer_name == layer_properties.layerName)
            {
                layer_found = true;
                break;
            }
        }

        if (!layer_found)
        {
            return false;
        }
    }
    return true;
}

static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
                                                    VkDebugUtilsMessageTypeFlagsEXT messageType,
                                                    const VkDebugUtilsMessengerCallbackDataEXT *pCallbackData,
                                                    __attribute__((unused)) void *pUserData)
{
    if (messageType != VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT)
    {
        if (messageSeverity >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT &&
            messageSeverity < VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT)
        {
            RCLCPP_WARN(rclcpp::get_logger("GuiEngine"), "Validation layer: %s", pCallbackData->pMessage);
        }
        else if (messageSeverity >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT)
        {
            RCLCPP_ERROR(rclcpp::get_logger("GuiEngine"), "Validation layer: %s", pCallbackData->pMessage);
        }
    }
    return VK_FALSE;
}

void GuiEngine::populateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT &createInfo)
{
    createInfo = {};
    createInfo.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;

    // The debug callback will be called with the most verbose output for all messages with severity >= WARNING
    createInfo.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT |
                                 VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
                                 VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
    // The debug callback will be called for all message types
    createInfo.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
                             VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
                             VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
    createInfo.pfnUserCallback = debugCallback;
}

void GuiEngine::createInstance()
{
    if (enable_validation_layers && !checkValidationLayerSupport())
    {
        RCLCPP_FATAL(node->get_logger(), "Validation layers requested, but not available!");
        throw std::runtime_error("Validation layers requested, but not available!");
    }
    uint32_t glfw_extension_count = 0;
    const char **glfw_extensions = glfwGetRequiredInstanceExtensions(&glfw_extension_count);
    std::vector<const char *> extensions(glfw_extensions, glfw_extensions + glfw_extension_count);
    if (enable_validation_layers)
    {
        extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
    }

    VkInstanceCreateInfo create_info = {};
    create_info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    create_info.pApplicationInfo = nullptr;
    create_info.enabledExtensionCount = static_cast<uint32_t>(extensions.size());
    create_info.ppEnabledExtensionNames = extensions.data();

    VkDebugUtilsMessengerCreateInfoEXT debug_create_info;
    if (enable_validation_layers)
    {
        create_info.enabledLayerCount = static_cast<uint32_t>(validation_layers.size());
        create_info.ppEnabledLayerNames = validation_layers.data();

        populateDebugMessengerCreateInfo(debug_create_info);
        create_info.pNext = reinterpret_cast<VkDebugUtilsMessengerCreateInfoEXT *>(&debug_create_info);
    }
    else
    {
        create_info.enabledLayerCount = 0;
        create_info.pNext = nullptr;
    }

    std::function<void(VkInstance *)> deleter = [](VkInstance *instance) { vkDestroyInstance(*instance, nullptr); };
    instance = std::unique_ptr<VkInstance, std::function<void(VkInstance *)>>(new VkInstance, deleter);
    if (vkCreateInstance(&create_info, nullptr, instance.get()) != VK_SUCCESS)
    {
        RCLCPP_FATAL(node->get_logger(), "Failed to create instance");
        throw std::runtime_error("Failed to create instance");
    }
}

void GuiEngine::setupDebugMessenger()
{
    if (!enable_validation_layers)
    {
        return;
    }
    VkDebugUtilsMessengerCreateInfoEXT create_info;

    std::function<void(VkDebugUtilsMessengerEXT *)> deleter = [this](VkDebugUtilsMessengerEXT *messenger)
    {
        PFN_vkDestroyDebugUtilsMessengerEXT func = reinterpret_cast<PFN_vkDestroyDebugUtilsMessengerEXT>(
            vkGetInstanceProcAddr(getInstance(), "vkDestroyDebugUtilsMessengerEXT"));
        if (func != nullptr)
        {
            func(getInstance(), *messenger, nullptr);
        }
    };

    debug_messenger = std::unique_ptr<VkDebugUtilsMessengerEXT, std::function<void(VkDebugUtilsMessengerEXT *)>>(
        new VkDebugUtilsMessengerEXT, deleter);

    populateDebugMessengerCreateInfo(create_info);

    PFN_vkCreateDebugUtilsMessengerEXT func = reinterpret_cast<PFN_vkCreateDebugUtilsMessengerEXT>(
        vkGetInstanceProcAddr(getInstance(), "vkCreateDebugUtilsMessengerEXT"));

    if (func != nullptr)
    {
        if (func(getInstance(), &create_info, nullptr, debug_messenger.get()) != VK_SUCCESS)
        {
            RCLCPP_FATAL(node->get_logger(), "Failed to set up debug messenger");
            throw std::runtime_error("Failed to set up debug messenger");
        }
    }
    else
    {
        RCLCPP_FATAL(node->get_logger(), "Failed to set up debug messenger, extension is not present");
        throw std::runtime_error("Failed to set up debug messenger, extension is not present");
    }
}

void GuiEngine::createSurface()
{
    std::function<void(VkSurfaceKHR *)> deleter = [this](VkSurfaceKHR *surface)
    { vkDestroySurfaceKHR(getInstance(), *surface, nullptr); };
    surface = std::unique_ptr<VkSurfaceKHR, std::function<void(VkSurfaceKHR *)>>(new VkSurfaceKHR, deleter);
    if (glfwCreateWindowSurface(getInstance(), getWindow(), nullptr, surface.get()) != VK_SUCCESS)
    {
        RCLCPP_FATAL(node->get_logger(), "Failed to create window surface");
        throw std::runtime_error("Failed to create window surface");
    }
}

void GuiEngine::createPhysicalDevice()
{
    uint32_t device_count = 0;
    vkEnumeratePhysicalDevices(getInstance(), &device_count, nullptr);
    if (device_count == 0)
    {
        RCLCPP_FATAL(node->get_logger(), "Failed to find GPUs with Vulkan support");
        throw std::runtime_error("Failed to find GPUs with Vulkan support");
    }

    std::vector<VkPhysicalDevice> devices(device_count);
    vkEnumeratePhysicalDevices(getInstance(), &device_count, devices.data());

    std::vector<VkPhysicalDevice>::iterator it = std::find_if(
        devices.begin(), devices.end(), [this](const VkPhysicalDevice &device) { return isDeviceSuitable(device); });

    if (it == devices.end())
    {
        RCLCPP_FATAL(node->get_logger(), "Failed to find a suitable GPU");
        throw std::runtime_error("Failed to find a suitable GPU");
    }

    physical_device = std::make_unique<VkPhysicalDevice>(*it);
}

void GuiEngine::createLogicalDevice()
{
    QueueFamilyIndices indices = findQueueFamilies(getPhysicalDevice());

    std::vector<VkDeviceQueueCreateInfo> queue_create_infos;
    std::set<uint32_t> unique_queue_families = {indices.graphics_family.value(), indices.present_family.value()};

    float queue_priority = 1.0f;
    for (uint32_t queue_family : unique_queue_families)
    {
        VkDeviceQueueCreateInfo queue_create_info = {};
        queue_create_info.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
        queue_create_info.queueFamilyIndex = queue_family;
        queue_create_info.queueCount = 1;
        queue_create_info.pQueuePriorities = &queue_priority;
        queue_create_infos.push_back(queue_create_info);
    }

    VkPhysicalDeviceFeatures device_features = {};

    VkDeviceCreateInfo create_info = {};
    create_info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;

    create_info.pQueueCreateInfos = queue_create_infos.data();
    create_info.queueCreateInfoCount = static_cast<uint32_t>(queue_create_infos.size());

    create_info.pEnabledFeatures = &device_features;

    create_info.enabledExtensionCount = static_cast<uint32_t>(device_extensions.size());
    create_info.ppEnabledExtensionNames = device_extensions.data();

    create_info.enabledLayerCount = 0;

    std::function<void(VkDevice *)> deleter = [](VkDevice *device) { vkDestroyDevice(*device, nullptr); };
    device = std::unique_ptr<VkDevice, std::function<void(VkDevice *)>>(new VkDevice, deleter);
    if (vkCreateDevice(getPhysicalDevice(), &create_info, nullptr, device.get()) != VK_SUCCESS)
    {
        RCLCPP_FATAL(node->get_logger(), "Failed to create logical device");
        throw std::runtime_error("Failed to create logical device");
    }

    vkGetDeviceQueue(getDevice(), indices.graphics_family.value(), 0, &graphics_queue);
    vkGetDeviceQueue(getDevice(), indices.present_family.value(), 0, &present_queue);
}

void GuiEngine::createSwapChain()
{
    SwapChainSupportDetails swap_chain_support = querySwapChainSupport(getPhysicalDevice());

    VkSurfaceFormatKHR surface_format = chooseSwapSurfaceFormat(swap_chain_support.formats);
    VkPresentModeKHR present_mode = chooseSwapPresentMode(swap_chain_support.present_modes);
    VkExtent2D extent = chooseSwapExtent(swap_chain_support.capabilities);

    uint32_t image_count = swap_chain_support.capabilities.minImageCount + 1;
    if (swap_chain_support.capabilities.maxImageCount > 0 &&
        image_count > swap_chain_support.capabilities.maxImageCount)
    {
        image_count = swap_chain_support.capabilities.maxImageCount;
    }

    VkSwapchainCreateInfoKHR create_info = {};
    create_info.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
    create_info.surface = getSurface();

    create_info.minImageCount = image_count;
    create_info.imageFormat = surface_format.format;
    create_info.imageColorSpace = surface_format.colorSpace;
    create_info.imageExtent = extent;
    create_info.imageArrayLayers = 1;
    create_info.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

    QueueFamilyIndices indices = findQueueFamilies(getPhysicalDevice());
    uint32_t queue_family_indices[] = {indices.graphics_family.value(), indices.present_family.value()};

    if (indices.graphics_family != indices.present_family)
    {
        create_info.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
        create_info.queueFamilyIndexCount = 2;
        create_info.pQueueFamilyIndices = queue_family_indices;
    }
    else
    {
        create_info.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
        create_info.queueFamilyIndexCount = 0;
        create_info.pQueueFamilyIndices = nullptr;
    }

    create_info.preTransform = swap_chain_support.capabilities.currentTransform;
    create_info.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    create_info.presentMode = present_mode;
    create_info.clipped = VK_TRUE;
    create_info.oldSwapchain = VK_NULL_HANDLE;

    std::function<void(VkSwapchainKHR *)> deleter = [this](VkSwapchainKHR *swap_chain)
    { vkDestroySwapchainKHR(getDevice(), *swap_chain, nullptr); };
    swap_chain = std::unique_ptr<VkSwapchainKHR, std::function<void(VkSwapchainKHR *)>>(new VkSwapchainKHR, deleter);
    if (vkCreateSwapchainKHR(getDevice(), &create_info, nullptr, swap_chain.get()) != VK_SUCCESS)
    {
        RCLCPP_FATAL(node->get_logger(), "Failed to create swap chain");
        throw std::runtime_error("Failed to create swap chain");
    }

    vkGetSwapchainImagesKHR(getDevice(), getSwapChain(), &image_count, nullptr);
    swap_chain_images.resize(image_count);
    vkGetSwapchainImagesKHR(getDevice(), getSwapChain(), &image_count, swap_chain_images.data());

    swap_chain_image_format = surface_format.format;
    swap_chain_extent = extent;
}

void GuiEngine::createImageViews()
{
    swap_chain_image_views.resize(swap_chain_images.size());

    VkImageViewCreateInfo create_info = {};
    create_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    create_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
    create_info.format = swap_chain_image_format;
    create_info.components.r = VK_COMPONENT_SWIZZLE_IDENTITY;
    create_info.components.g = VK_COMPONENT_SWIZZLE_IDENTITY;
    create_info.components.b = VK_COMPONENT_SWIZZLE_IDENTITY;
    create_info.components.a = VK_COMPONENT_SWIZZLE_IDENTITY;
    create_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    create_info.subresourceRange.baseMipLevel = 0;
    create_info.subresourceRange.levelCount = 1;
    create_info.subresourceRange.baseArrayLayer = 0;
    create_info.subresourceRange.layerCount = 1;

    std::function<void(VkImageView *)> deleter = [this](VkImageView *image_view)
    { vkDestroyImageView(getDevice(), *image_view, nullptr); };

    for (size_t i = 0; i < swap_chain_images.size(); i++)
    {
        create_info.image = swap_chain_images[i];

        swap_chain_image_views[i] =
            std::unique_ptr<VkImageView, std::function<void(VkImageView *)>>(new VkImageView, deleter);
        if (vkCreateImageView(getDevice(), &create_info, nullptr, swap_chain_image_views[i].get()) != VK_SUCCESS)
        {
            RCLCPP_FATAL(node->get_logger(), "Failed to create image views");
            throw std::runtime_error("Failed to create image views");
        }
    }
}

void GuiEngine::createRenderPass()
{
    VkAttachmentDescription color_attachment = {};
    color_attachment.format = swap_chain_image_format;
    color_attachment.samples = VK_SAMPLE_COUNT_1_BIT;
    color_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    color_attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    color_attachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    color_attachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    color_attachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    color_attachment.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

    VkAttachmentReference color_attachment_ref = {};
    color_attachment_ref.attachment = 0;
    color_attachment_ref.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkSubpassDescription subpass = {};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &color_attachment_ref;

    VkSubpassDependency dependency = {};
    dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
    dependency.dstSubpass = 0;
    dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependency.srcAccessMask = 0;
    dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

    VkRenderPassCreateInfo render_pass_info = {};
    render_pass_info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    render_pass_info.attachmentCount = 1;
    render_pass_info.pAttachments = &color_attachment;
    render_pass_info.subpassCount = 1;
    render_pass_info.pSubpasses = &subpass;
    render_pass_info.dependencyCount = 1;
    render_pass_info.pDependencies = &dependency;

    std::function<void(VkRenderPass *)> deleter = [this](VkRenderPass *render_pass)
    { vkDestroyRenderPass(getDevice(), *render_pass, nullptr); };
    render_pass = std::unique_ptr<VkRenderPass, std::function<void(VkRenderPass *)>>(new VkRenderPass, deleter);
    if (vkCreateRenderPass(getDevice(), &render_pass_info, nullptr, render_pass.get()) != VK_SUCCESS)
    {
        RCLCPP_FATAL(node->get_logger(), "Failed to create render pass");
        throw std::runtime_error("Failed to create render pass");
    }
}

void GuiEngine::createFramebuffers()
{
    swap_chain_framebuffers.resize(swap_chain_image_views.size());

    VkFramebufferCreateInfo framebuffer_info = {};
    framebuffer_info.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
    framebuffer_info.renderPass = getRenderPass();
    framebuffer_info.attachmentCount = 1;
    framebuffer_info.width = swap_chain_extent.width;
    framebuffer_info.height = swap_chain_extent.height;
    framebuffer_info.layers = 1;

    std::function<void(VkFramebuffer *)> deleter = [this](VkFramebuffer *framebuffer)
    { vkDestroyFramebuffer(getDevice(), *framebuffer, nullptr); };

    for (size_t i = 0; i < swap_chain_image_views.size(); i++)
    {
        swap_chain_framebuffers[i] =
            std::unique_ptr<VkFramebuffer, std::function<void(VkFramebuffer *)>>(new VkFramebuffer, deleter);
        framebuffer_info.pAttachments = swap_chain_image_views[i].get();
        if (vkCreateFramebuffer(getDevice(), &framebuffer_info, nullptr, swap_chain_framebuffers[i].get()) !=
            VK_SUCCESS)
        {
            RCLCPP_FATAL(node->get_logger(), "Failed to create framebuffer");
            throw std::runtime_error("Failed to create framebuffer");
        }
    }
}

void GuiEngine::createCommandPool()
{
    QueueFamilyIndices indices = findQueueFamilies(getPhysicalDevice());
    VkCommandPoolCreateInfo pool_info = {};
    pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    pool_info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    pool_info.queueFamilyIndex = indices.graphics_family.value();

    std::function<void(VkCommandPool *)> deleter = [this](VkCommandPool *command_pool)
    { vkDestroyCommandPool(getDevice(), *command_pool, nullptr); };
    command_pool = std::unique_ptr<VkCommandPool, std::function<void(VkCommandPool *)>>(new VkCommandPool, deleter);
    if (vkCreateCommandPool(getDevice(), &pool_info, nullptr, command_pool.get()) != VK_SUCCESS)
    {
        RCLCPP_FATAL(node->get_logger(), "Failed to create command pool");
        throw std::runtime_error("Failed to create command pool");
    }
}

void GuiEngine::createDescriptorPool()
{
    VkDescriptorPoolSize pool_size = {};
    pool_size.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    pool_size.descriptorCount = static_cast<uint32_t>(MAX_FRAMES_IN_FLIGHT);

    VkDescriptorPoolCreateInfo pool_info = {};
    pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    pool_info.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    pool_info.poolSizeCount = 1;
    pool_info.pPoolSizes = &pool_size;
    pool_info.maxSets = static_cast<uint32_t>(MAX_FRAMES_IN_FLIGHT);

    std::function<void(VkDescriptorPool *)> deleter = [this](VkDescriptorPool *pool)
    { vkDestroyDescriptorPool(getDevice(), *pool, nullptr); };
    descriptor_pool =
        std::unique_ptr<VkDescriptorPool, std::function<void(VkDescriptorPool *)>>(new VkDescriptorPool, deleter);
    if (vkCreateDescriptorPool(getDevice(), &pool_info, nullptr, descriptor_pool.get()) != VK_SUCCESS)
    {
        RCLCPP_FATAL(node->get_logger(), "Failed to create descriptor pool");
        throw std::runtime_error("Failed to create descriptor pool");
    }
}

void GuiEngine::createCommandBuffers()
{
    command_buffers.resize(MAX_FRAMES_IN_FLIGHT);

    VkCommandBufferAllocateInfo alloc_info = {};
    alloc_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    alloc_info.commandPool = getCommandPool();
    alloc_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    alloc_info.commandBufferCount = (uint32_t)command_buffers.size();

    if (vkAllocateCommandBuffers(getDevice(), &alloc_info, command_buffers.data()) != VK_SUCCESS)
    {
        RCLCPP_FATAL(node->get_logger(), "Failed to allocate command buffers");
        throw std::runtime_error("Failed to allocate command buffers");
    }
}

void GuiEngine::createSyncObjects()
{
    image_available_semaphores.resize(MAX_FRAMES_IN_FLIGHT);
    render_finished_semaphores.resize(MAX_FRAMES_IN_FLIGHT);
    in_flight_fences.resize(MAX_FRAMES_IN_FLIGHT);

    VkSemaphoreCreateInfo semaphore_info = {};
    semaphore_info.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

    VkFenceCreateInfo fence_info = {};
    fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    fence_info.flags = VK_FENCE_CREATE_SIGNALED_BIT;

    std::function<void(VkSemaphore *)> semaphore_deleter = [this](VkSemaphore *semaphore)
    { vkDestroySemaphore(getDevice(), *semaphore, nullptr); };
    std::function<void(VkFence *)> fence_deleter = [this](VkFence *fence)
    { vkDestroyFence(getDevice(), *fence, nullptr); };
    for (int i = 0; i < MAX_FRAMES_IN_FLIGHT; i++)
    {
        image_available_semaphores[i] =
            std::unique_ptr<VkSemaphore, std::function<void(VkSemaphore *)>>(new VkSemaphore, semaphore_deleter);
        render_finished_semaphores[i] =
            std::unique_ptr<VkSemaphore, std::function<void(VkSemaphore *)>>(new VkSemaphore, semaphore_deleter);
        in_flight_fences[i] = std::unique_ptr<VkFence, std::function<void(VkFence *)>>(new VkFence, fence_deleter);
        if (vkCreateSemaphore(getDevice(), &semaphore_info, nullptr, image_available_semaphores[i].get()) !=
                VK_SUCCESS ||
            vkCreateSemaphore(getDevice(), &semaphore_info, nullptr, render_finished_semaphores[i].get()) !=
                VK_SUCCESS ||
            vkCreateFence(getDevice(), &fence_info, nullptr, in_flight_fences[i].get()) != VK_SUCCESS)
        {
            RCLCPP_FATAL(node->get_logger(), "Failed to create synchronization objects for a frame");
            throw std::runtime_error("Failed to create synchronization objects for a frame");
        }
    }
}

VkSurfaceFormatKHR GuiEngine::chooseSwapSurfaceFormat(const std::vector<VkSurfaceFormatKHR> &available_formats)
{
    auto it = std::find_if(available_formats.begin(), available_formats.end(),
                           [](const VkSurfaceFormatKHR &format) {
                               return format.format == VK_FORMAT_B8G8R8A8_SRGB &&
                                      format.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR;
                           });

    if (it != available_formats.end())
    {
        return *it;
    }
    else
    {
        return available_formats[0];
    }
}

VkPresentModeKHR GuiEngine::chooseSwapPresentMode(const std::vector<VkPresentModeKHR> &available_present_modes)
{
    auto it = std::find_if(available_present_modes.begin(), available_present_modes.end(),
                           [](const VkPresentModeKHR &mode) { return mode == VK_PRESENT_MODE_MAILBOX_KHR; });

    if (it != available_present_modes.end())
    {
        return *it;
    }
    else
    {
        return VK_PRESENT_MODE_FIFO_KHR;
    }
}

VkExtent2D GuiEngine::chooseSwapExtent(const VkSurfaceCapabilitiesKHR &capabilities)
{
    if (capabilities.currentExtent.width != std::numeric_limits<uint32_t>::max())
    {
        return capabilities.currentExtent;
    }
    else
    {
        int width, height;
        glfwGetFramebufferSize(getWindow(), &width, &height);

        VkExtent2D actual_extent = {static_cast<uint32_t>(width), static_cast<uint32_t>(height)};

        actual_extent.width =
            std::clamp(actual_extent.width, capabilities.minImageExtent.width, capabilities.maxImageExtent.width);
        actual_extent.height =
            std::clamp(actual_extent.height, capabilities.minImageExtent.height, capabilities.maxImageExtent.height);

        return actual_extent;
    }
}

QueueFamilyIndices GuiEngine::findQueueFamilies(const VkPhysicalDevice &device)
{
    QueueFamilyIndices indices;

    uint32_t queue_family_count = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(device, &queue_family_count, nullptr);

    std::vector<VkQueueFamilyProperties> queue_families(queue_family_count);
    vkGetPhysicalDeviceQueueFamilyProperties(device, &queue_family_count, queue_families.data());

    int i = 0;
    for (const auto &queue_family : queue_families)
    {
        if (queue_family.queueCount > 0 && queue_family.queueFlags & VK_QUEUE_GRAPHICS_BIT)
        {
            indices.graphics_family = i;
        }

        VkBool32 present_support = false;
        vkGetPhysicalDeviceSurfaceSupportKHR(device, i, getSurface(), &present_support);

        if (queue_family.queueCount > 0 && present_support)
        {
            indices.present_family = i;
        }

        if (indices.isComplete())
        {
            break;
        }

        i++;
    }

    return indices;
}

bool GuiEngine::checkDeviceExtensionSupport(const VkPhysicalDevice &device)
{
    uint32_t extension_count;
    vkEnumerateDeviceExtensionProperties(device, nullptr, &extension_count, nullptr);

    std::vector<VkExtensionProperties> available_extensions(extension_count);
    vkEnumerateDeviceExtensionProperties(device, nullptr, &extension_count, available_extensions.data());

    std::set<std::string> required_extensions(device_extensions.begin(), device_extensions.end());

    for (const auto &extension : available_extensions)
    {
        required_extensions.erase(extension.extensionName);
    }

    return required_extensions.empty();
}

SwapChainSupportDetails GuiEngine::querySwapChainSupport(const VkPhysicalDevice &device)
{
    SwapChainSupportDetails details;

    vkGetPhysicalDeviceSurfaceCapabilitiesKHR(device, getSurface(), &details.capabilities);

    uint32_t format_count;
    vkGetPhysicalDeviceSurfaceFormatsKHR(device, getSurface(), &format_count, nullptr);

    if (format_count != 0)
    {
        details.formats.resize(format_count);
        vkGetPhysicalDeviceSurfaceFormatsKHR(device, getSurface(), &format_count, details.formats.data());
    }

    uint32_t present_mode_count;
    vkGetPhysicalDeviceSurfacePresentModesKHR(device, getSurface(), &present_mode_count, nullptr);

    if (present_mode_count != 0)
    {
        details.present_modes.resize(present_mode_count);
        vkGetPhysicalDeviceSurfacePresentModesKHR(device, getSurface(), &present_mode_count,
                                                  details.present_modes.data());
    }

    return details;
}

bool GuiEngine::isDeviceSuitable(const VkPhysicalDevice &device)
{
    QueueFamilyIndices indices = findQueueFamilies(device);
    bool extensions_supported = checkDeviceExtensionSupport(device);

    bool swap_chain_adequate = false;
    if (extensions_supported)
    {
        SwapChainSupportDetails swap_chain_support = querySwapChainSupport(device);
        swap_chain_adequate = !swap_chain_support.formats.empty() && !swap_chain_support.present_modes.empty();
    }

    return indices.isComplete() && extensions_supported && swap_chain_adequate;
}

void GuiEngine::rebuildSwapChain()
{
    int width = 0, height = 0;
    glfwGetFramebufferSize(getWindow(), &width, &height);
    while (width == 0 || height == 0)
    {
        glfwGetFramebufferSize(getWindow(), &width, &height);
        glfwWaitEvents();
    }

    vkDeviceWaitIdle(getDevice());
    cleanupSwapChain();

    createSwapChain();
    createImageViews();
    createFramebuffers();
}

void GuiEngine::recordRenderPass(const VkCommandBuffer &command_buffer, uint32_t image_index)
{
    VkCommandBufferBeginInfo begin_info{};
    begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;

    if (vkBeginCommandBuffer(command_buffer, &begin_info) != VK_SUCCESS)
    {
        RCLCPP_FATAL(node->get_logger(), "Failed to begin recording command buffer!");
        throw std::runtime_error("Failed to begin recording command buffer!");
    }

    VkRenderPassBeginInfo render_pass_info{};
    render_pass_info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    render_pass_info.renderPass = getRenderPass();
    render_pass_info.framebuffer = getFrameBuffer(image_index);
    render_pass_info.renderArea.offset = {0, 0};
    render_pass_info.renderArea.extent = swap_chain_extent;
    render_pass_info.clearValueCount = 1;
    render_pass_info.pClearValues = &clear_color;

    vkCmdBeginRenderPass(command_buffer, &render_pass_info, VK_SUBPASS_CONTENTS_INLINE);
    ImDrawData *draw_data = ImGui::GetDrawData();
    ImGui_ImplVulkan_RenderDrawData(draw_data, command_buffer);
    vkCmdEndRenderPass(command_buffer);
    if (vkEndCommandBuffer(command_buffer) != VK_SUCCESS)
    {
        RCLCPP_FATAL(node->get_logger(), "Failed to record command buffer!");
        throw std::runtime_error("Failed to record command buffer!");
    }
}

GuiEngine::~GuiEngine() { cleanup(); }

void GuiEngine::draw()
{
    vkWaitForFences(getDevice(), 1, in_flight_fences[current_frame].get(), VK_TRUE, UINT64_MAX);

    uint32_t image_index;
    VkResult result =
        vkAcquireNextImageKHR(getDevice(), getSwapChain(), UINT64_MAX, *image_available_semaphores[current_frame].get(),
                              VK_NULL_HANDLE, &image_index);

    if (result == VK_ERROR_OUT_OF_DATE_KHR)
    {
        rebuildSwapChain();
        return;
    }
    else if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR)
    {
        RCLCPP_FATAL(node->get_logger(), "Failed to acquire swap chain image!");
        throw std::runtime_error("Failed to acquire swap chain image!");
    }

    vkResetFences(getDevice(), 1, in_flight_fences[current_frame].get());

    vkResetCommandBuffer(command_buffers[current_frame], 0);
    recordRenderPass(command_buffers[current_frame], image_index);

    VkSubmitInfo submit_info{};
    submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;

    VkSemaphore wait_semaphores[] = {*image_available_semaphores[current_frame].get()};
    VkPipelineStageFlags wait_stages[] = {VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT};
    submit_info.waitSemaphoreCount = 1;
    submit_info.pWaitSemaphores = wait_semaphores;
    submit_info.pWaitDstStageMask = wait_stages;

    submit_info.commandBufferCount = 1;
    submit_info.pCommandBuffers = &command_buffers[current_frame];

    VkSemaphore signal_semaphores[] = {*render_finished_semaphores[current_frame].get()};
    submit_info.signalSemaphoreCount = 1;
    submit_info.pSignalSemaphores = signal_semaphores;

    if (vkQueueSubmit(graphics_queue, 1, &submit_info, *in_flight_fences[current_frame].get()) != VK_SUCCESS)
    {
        RCLCPP_FATAL(node->get_logger(), "Failed to submit draw command buffer!");
        throw std::runtime_error("Failed to submit draw command buffer!");
    }

    VkPresentInfoKHR present_info{};
    present_info.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;

    present_info.waitSemaphoreCount = 1;
    present_info.pWaitSemaphores = signal_semaphores;

    VkSwapchainKHR swap_chains[] = {getSwapChain()};
    present_info.swapchainCount = 1;
    present_info.pSwapchains = swap_chains;

    present_info.pImageIndices = &image_index;

    result = vkQueuePresentKHR(present_queue, &present_info);

    if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR || framebuffer_resized)
    {
        framebuffer_resized = false;
        rebuildSwapChain();
    }
    else if (result != VK_SUCCESS)
    {
        RCLCPP_FATAL(node->get_logger(), "Failed to present swap chain image!");
        throw std::runtime_error("Failed to present swap chain image!");
    }

    current_frame = (current_frame + 1) % MAX_FRAMES_IN_FLIGHT;
}

void GuiEngine::framebufferResizeCallback(GLFWwindow *window, __attribute__((unused)) int width,
                                          __attribute__((unused)) int height)
{
    auto app = reinterpret_cast<GuiEngine *>(glfwGetWindowUserPointer(window));
    app->framebuffer_resized = true;
}

void GuiEngine::initGlfw()
{
    glfwInit();
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    std::function<void(GLFWwindow *)> deleter = [](GLFWwindow *window)
    {
        glfwDestroyWindow(window);
        glfwTerminate();
    };
    window = std::unique_ptr<GLFWwindow, std::function<void(GLFWwindow *)>>(
        glfwCreateWindow(800, 600, application_name.c_str(), nullptr, nullptr), deleter);
    glfwSetWindowUserPointer(getWindow(), this);
    glfwSetFramebufferSizeCallback(getWindow(), framebufferResizeCallback);
}

void GuiEngine::initVulkan()
{
    createInstance();
    setupDebugMessenger();
    createSurface();
    createPhysicalDevice();
    createLogicalDevice();
    createSwapChain();
    createImageViews();
    createRenderPass();
    createFramebuffers();
    createCommandPool();
    createDescriptorPool();
    createCommandBuffers();
    createSyncObjects();
}

void GuiEngine::init()
{
    initGlfw();
    initVulkan();
    imgui_engine->init(shared_from_this());
    return;
}

void GuiEngine::cleanupSwapChain()
{
    swap_chain_framebuffers.clear();
    swap_chain_image_views.clear();
    swap_chain.reset();
}

void GuiEngine::cleanup()
{
    vkDeviceWaitIdle(getDevice());
    textures.clear();
    imgui_engine.reset();
    cleanupSwapChain();
    render_pass.reset();
    descriptor_pool.reset();
    render_finished_semaphores.clear();
    image_available_semaphores.clear();
    in_flight_fences.clear();
    command_pool.reset();
    if (enable_validation_layers)
    {
        debug_messenger.reset();
    }
    device.reset();
}

GuiEngine::GuiEngine(const std::string &application_name, std::shared_ptr<GuiNode> node)
    : device_extensions({VK_KHR_SWAPCHAIN_EXTENSION_NAME}), application_name(application_name), node(node),
      imgui_engine(std::make_unique<ImGuiEngine>())
{
}

GuiEngine::GuiEngine(const std::string &application_name, std::shared_ptr<GuiNode> node,
                     const std::vector<const char *> &device_extensions)
    : device_extensions(device_extensions), application_name(application_name), node(node),
      imgui_engine(std::make_unique<ImGuiEngine>())
{
}

bool GuiEngine::addTexture(const std::string &name, unsigned char *image_data, int width, int height, int channels)
{
    if (textures.find(name) != textures.end())
    {
        RCLCPP_WARN(node->get_logger(), "Texture %s already exists!", name.c_str());
        return false;
    }
    textures.emplace(name, std::make_shared<TextureLoader>(image_data, width, height, channels, getDevice(),
                                                           getPhysicalDevice(), getCommandPool(), graphics_queue));
    return true;
}

std::shared_ptr<TextureLoader> GuiEngine::getTexture(const std::string &name)
{
    if (textures.find(name) == textures.end())
    {
        RCLCPP_ERROR(node->get_logger(), "Texture with name %s does not exist!", name.c_str());
        throw std::runtime_error("Texture with name " + name + " does not exist!");
    }
    return textures.at(name);
}

void ImGuiEngine::init(std::shared_ptr<GuiEngine> gui_engine)
{
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    initVulkanImpl(gui_engine);
    initFonts(gui_engine);
}

void ImGuiEngine::initVulkanImpl(std::shared_ptr<GuiEngine> gui_engine)
{
    QueueFamilyIndices indices = gui_engine->findQueueFamilies(gui_engine->getPhysicalDevice());
    ImGui_ImplGlfw_InitForVulkan(gui_engine->getWindow(), true);
    ImGui_ImplVulkan_InitInfo init_info{};
    init_info.Instance = gui_engine->getInstance();
    init_info.PhysicalDevice = gui_engine->getPhysicalDevice();
    init_info.Device = gui_engine->getDevice();
    init_info.QueueFamily = indices.graphics_family.value();
    init_info.Queue = gui_engine->getGraphicsQueue();
    init_info.PipelineCache = VK_NULL_HANDLE;
    init_info.DescriptorPool = gui_engine->getDescriptorPool();
    init_info.Subpass = 0;
    init_info.MinImageCount = gui_engine->getSwapChainImages().size();
    init_info.ImageCount = gui_engine->getSwapChainImages().size();
    init_info.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
    init_info.Allocator = nullptr;
    init_info.CheckVkResultFn = nullptr;
    ImGui_ImplVulkan_Init(&init_info, gui_engine->getRenderPass());
}

void ImGuiEngine::initFonts(std::shared_ptr<GuiEngine> gui_engine)
{
    VkCommandBufferBeginInfo begin_info{};
    VkCommandBuffer command_buffer = gui_engine->getCommandBuffer(0);
    vkResetCommandPool(gui_engine->getDevice(), gui_engine->getCommandPool(), 0);
    begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    begin_info.flags |= VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

    vkBeginCommandBuffer(command_buffer, &begin_info);
    ImGui_ImplVulkan_CreateFontsTexture(command_buffer);
    vkEndCommandBuffer(command_buffer);

    VkSubmitInfo submit_info{};
    submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submit_info.commandBufferCount = 1;
    submit_info.pCommandBuffers = &command_buffer;

    vkQueueSubmit(gui_engine->getGraphicsQueue(), 1, &submit_info, VK_NULL_HANDLE);
    vkQueueWaitIdle(gui_engine->getGraphicsQueue());
    ImGui_ImplVulkan_DestroyFontUploadObjects();
}

ImGuiEngine::~ImGuiEngine()
{
    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

TextureLoader::TextureLoader(unsigned char *image_data, int width, int height, int channels, const VkDevice &device,
                             const VkPhysicalDevice &physical_device, const VkCommandPool &command_pool,
                             const VkQueue &graphics_queue)
    : device(device)
{
    size_t image_size = width * height * channels;
    this->width = width;
    this->height = height;
    this->channels = channels;

    createImage(physical_device);
    createImageView();
    createSampler();

    descriptor_set = ImGui_ImplVulkan_AddTexture(sampler, image_view, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

    createUploadBuffer(image_size, physical_device);
    uploadToBuffer(image_size, image_data);

    recordCommandBuffer(command_pool, graphics_queue);
}

uint32_t TextureLoader::findMemoryType(uint32_t type_filter, const VkPhysicalDevice &physical_device,
                                       VkMemoryPropertyFlags properties)
{
    VkPhysicalDeviceMemoryProperties mem_properties;
    vkGetPhysicalDeviceMemoryProperties(physical_device, &mem_properties);

    for (uint32_t i = 0; i < mem_properties.memoryTypeCount; i++)
    {
        if ((type_filter & (1 << i)) && (mem_properties.memoryTypes[i].propertyFlags & properties) == properties)
        {
            return i;
        }
    }

    RCLCPP_FATAL(rclcpp::get_logger("TextureLoader"), "Failed to find suitable memory type!");
    throw std::runtime_error("Failed to find suitable memory type!");
}

void TextureLoader::createImage(const VkPhysicalDevice &physical_device)
{
    VkImageCreateInfo image_info{};
    image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    image_info.imageType = VK_IMAGE_TYPE_2D;
    image_info.format = VK_FORMAT_R8G8B8A8_UNORM;
    image_info.extent.width = width;
    image_info.extent.height = height;
    image_info.extent.depth = 1;
    image_info.mipLevels = 1;
    image_info.arrayLayers = 1;
    image_info.samples = VK_SAMPLE_COUNT_1_BIT;
    image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
    image_info.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    if (vkCreateImage(device, &image_info, nullptr, &image) != VK_SUCCESS)
    {
        RCLCPP_FATAL(rclcpp::get_logger("TextureLoader"), "Failed to create image!");
        throw std::runtime_error("Failed to create image!");
    }

    VkMemoryRequirements mem_requirements;
    vkGetImageMemoryRequirements(device, image, &mem_requirements);

    VkMemoryAllocateInfo alloc_info{};
    alloc_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    alloc_info.allocationSize = mem_requirements.size;
    alloc_info.memoryTypeIndex =
        findMemoryType(mem_requirements.memoryTypeBits, physical_device, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    if (vkAllocateMemory(device, &alloc_info, nullptr, &image_memory) != VK_SUCCESS)
    {
        RCLCPP_FATAL(rclcpp::get_logger("TextureLoader"), "Failed to allocate image memory!");
        throw std::runtime_error("Failed to allocate image memory!");
    }

    vkBindImageMemory(device, image, image_memory, 0);
}

void TextureLoader::createImageView()
{
    VkImageViewCreateInfo view_info{};
    view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    view_info.image = image;
    view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
    view_info.format = VK_FORMAT_R8G8B8A8_UNORM;
    view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    view_info.subresourceRange.baseMipLevel = 0;
    view_info.subresourceRange.levelCount = 1;
    if (vkCreateImageView(device, &view_info, nullptr, &image_view) != VK_SUCCESS)
    {
        RCLCPP_FATAL(rclcpp::get_logger("TextureLoader"), "Failed to create texture image view!");
        throw std::runtime_error("Failed to create texture image view!");
    }
}

void TextureLoader::createSampler()
{
    VkSamplerCreateInfo sampler_info{};
    sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    sampler_info.magFilter = VK_FILTER_LINEAR;
    sampler_info.minFilter = VK_FILTER_LINEAR;
    sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
    sampler_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    sampler_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    sampler_info.minLod = -1000;
    sampler_info.maxLod = 1000;
    sampler_info.maxAnisotropy = 1.0f;
    if (vkCreateSampler(device, &sampler_info, nullptr, &sampler) != VK_SUCCESS)
    {
        RCLCPP_FATAL(rclcpp::get_logger("TextureLoader"), "Failed to create texture sampler!");
        throw std::runtime_error("Failed to create texture sampler!");
    }
}

void TextureLoader::createUploadBuffer(size_t image_size, const VkPhysicalDevice &physical_device)
{
    VkBufferCreateInfo buffer_info{};
    buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    buffer_info.size = image_size;
    buffer_info.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
    buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    if (vkCreateBuffer(device, &buffer_info, nullptr, &upload_buffer) != VK_SUCCESS)
    {
        RCLCPP_FATAL(rclcpp::get_logger("TextureLoader"), "Failed to create buffer!");
        throw std::runtime_error("Failed to create buffer!");
    }

    VkMemoryRequirements mem_requirements;
    vkGetBufferMemoryRequirements(device, upload_buffer, &mem_requirements);

    VkMemoryAllocateInfo alloc_info{};
    alloc_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    alloc_info.allocationSize = mem_requirements.size;
    alloc_info.memoryTypeIndex =
        findMemoryType(mem_requirements.memoryTypeBits, physical_device, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT);
    if (vkAllocateMemory(device, &alloc_info, nullptr, &upload_buffer_memory) != VK_SUCCESS)
    {
        RCLCPP_FATAL(rclcpp::get_logger("TextureLoader"), "Failed to allocate buffer memory!");
        throw std::runtime_error("Failed to allocate buffer memory!");
    }

    if (vkBindBufferMemory(device, upload_buffer, upload_buffer_memory, 0) != VK_SUCCESS)
    {
        RCLCPP_FATAL(rclcpp::get_logger("TextureLoader"), "Failed to bind buffer memory!");
        throw std::runtime_error("Failed to bind buffer memory!");
    }
}

void TextureLoader::uploadToBuffer(size_t image_size, unsigned char *image_data)
{
    void *map = nullptr;
    if (vkMapMemory(device, upload_buffer_memory, 0, VK_WHOLE_SIZE, 0, &map) != VK_SUCCESS)
    {
        RCLCPP_FATAL(rclcpp::get_logger("TextureLoader"), "Failed to map memory!");
        throw std::runtime_error("Failed to map memory!");
    }

    memcpy(map, image_data, image_size);
    VkMappedMemoryRange range[1] = {};
    range[0].sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
    range[0].memory = upload_buffer_memory;
    range[0].size = VK_WHOLE_SIZE;
    if (vkFlushMappedMemoryRanges(device, 1, range) != VK_SUCCESS)
    {
        RCLCPP_FATAL(rclcpp::get_logger("TextureLoader"), "Failed to flush memory!");
        throw std::runtime_error("Failed to flush memory!");
    }
    vkUnmapMemory(device, upload_buffer_memory);
}

void TextureLoader::recordCommandBuffer(const VkCommandPool &command_pool, const VkQueue &graphics_queue)
{
    // Create command buffer
    VkCommandBuffer command_buffer;
    VkCommandBufferAllocateInfo alloc_info{};
    alloc_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    alloc_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    alloc_info.commandPool = command_pool;
    alloc_info.commandBufferCount = 1;

    if (vkAllocateCommandBuffers(device, &alloc_info, &command_buffer) != VK_SUCCESS)
    {
        RCLCPP_FATAL(rclcpp::get_logger("TextureLoader"), "Failed to allocate command buffers!");
        throw std::runtime_error("Failed to allocate command buffers!");
    }

    VkCommandBufferBeginInfo begin_info{};
    begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    begin_info.flags |= VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    if (vkBeginCommandBuffer(command_buffer, &begin_info) != VK_SUCCESS)
    {
        RCLCPP_FATAL(rclcpp::get_logger("TextureLoader"), "Failed to begin recording command buffer!");
        throw std::runtime_error("Failed to begin recording command buffer!");
    }

    // Copy buffer to image
    VkImageMemoryBarrier copy_barrier[1] = {};
    copy_barrier[0].sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    copy_barrier[0].dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    copy_barrier[0].oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    copy_barrier[0].newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    copy_barrier[0].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    copy_barrier[0].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    copy_barrier[0].image = image;
    copy_barrier[0].subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    copy_barrier[0].subresourceRange.levelCount = 1;
    copy_barrier[0].subresourceRange.layerCount = 1;
    vkCmdPipelineBarrier(command_buffer, VK_PIPELINE_STAGE_HOST_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0,
                         nullptr, 1, copy_barrier);

    VkBufferImageCopy region = {};
    region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    region.imageSubresource.layerCount = 1;
    region.imageExtent.width = width;
    region.imageExtent.height = height;
    region.imageExtent.depth = 1;
    vkCmdCopyBufferToImage(command_buffer, upload_buffer, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);

    VkImageMemoryBarrier use_barrier[1] = {};
    use_barrier[0].sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    use_barrier[0].srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    use_barrier[0].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
    use_barrier[0].oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    use_barrier[0].newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    use_barrier[0].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    use_barrier[0].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    use_barrier[0].image = image;
    use_barrier[0].subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    use_barrier[0].subresourceRange.levelCount = 1;
    use_barrier[0].subresourceRange.layerCount = 1;
    vkCmdPipelineBarrier(command_buffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0, 0,
                         nullptr, 0, nullptr, 1, use_barrier);

    // End command buffer
    VkSubmitInfo submit_info{};
    submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submit_info.commandBufferCount = 1;
    submit_info.pCommandBuffers = &command_buffer;
    if (vkEndCommandBuffer(command_buffer) != VK_SUCCESS)
    {
        RCLCPP_FATAL(rclcpp::get_logger("TextureLoader"), "Failed to record command buffer!");
        throw std::runtime_error("Failed to record command buffer!");
    }
    if (vkQueueSubmit(graphics_queue, 1, &submit_info, VK_NULL_HANDLE) != VK_SUCCESS)
    {
        RCLCPP_FATAL(rclcpp::get_logger("TextureLoader"), "Failed to submit draw command buffer!");
        throw std::runtime_error("Failed to submit draw command buffer!");
    }
    if (vkDeviceWaitIdle(device) != VK_SUCCESS)
    {
        RCLCPP_FATAL(rclcpp::get_logger("TextureLoader"), "Failed to wait for device to become idle!");
        throw std::runtime_error("Failed to wait for device to become idle!");
    }
}

TextureLoader::~TextureLoader()
{
    vkFreeMemory(device, upload_buffer_memory, nullptr);
    vkDestroyBuffer(device, upload_buffer, nullptr);
    vkDestroySampler(device, sampler, nullptr);
    vkDestroyImageView(device, image_view, nullptr);
    vkDestroyImage(device, image, nullptr);
    vkFreeMemory(device, image_memory, nullptr);
    ImGui_ImplVulkan_RemoveTexture(descriptor_set);
}

} // namespace gui_node
