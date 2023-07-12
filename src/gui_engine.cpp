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

void checkVulkanResult(VkResult result, const rclcpp::Logger &logger, const std::string &message)
{
    if (result != VK_SUCCESS)
    {
        RCLCPP_FATAL(logger, "%s", message.c_str());
        throw std::runtime_error(message);
    }
}

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

void VkDebugUtilsMessengerEXTDeleter::operator()(VkDebugUtilsMessengerEXT *messenger) const
{
    PFN_vkDestroyDebugUtilsMessengerEXT func = reinterpret_cast<PFN_vkDestroyDebugUtilsMessengerEXT>(
        vkGetInstanceProcAddr(instance, "vkDestroyDebugUtilsMessengerEXT"));
    if (func)
    {
        func(instance, *messenger, nullptr);
    }
}

static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(
    VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
    VkDebugUtilsMessageTypeFlagsEXT messageType,
    const VkDebugUtilsMessengerCallbackDataEXT *pCallbackData,
    __attribute__((unused)) void *pUserData)
{
    if (messageType != VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT)
    {
        if ((messageSeverity >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT) &&
            (messageSeverity < VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT))
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

    instance = VkInstanceSharedPtr(new VkInstance);
    VkResult result = vkCreateInstance(&create_info, nullptr, getInstance().get());
    checkVulkanResult(result, node->get_logger(), "Failed to create Vulkan instance");
}

void GuiEngine::setupDebugMessenger()
{
    if (!enable_validation_layers)
    {
        return;
    }
    VkDebugUtilsMessengerCreateInfoEXT create_info;

    debug_messenger = VkDebugUtilsMessengerEXTUniquePtr(new VkDebugUtilsMessengerEXT, {*getInstance().get()});

    populateDebugMessengerCreateInfo(create_info);

    PFN_vkCreateDebugUtilsMessengerEXT func = reinterpret_cast<PFN_vkCreateDebugUtilsMessengerEXT>(
        vkGetInstanceProcAddr(*getInstance().get(), "vkCreateDebugUtilsMessengerEXT"));

    if (func)
    {
        VkResult result = func(*getInstance().get(), &create_info, nullptr, debug_messenger.get());
        checkVulkanResult(result, node->get_logger(), "Failed to set up debug messenger");
    }
    else
    {
        RCLCPP_FATAL(node->get_logger(), "Failed to set up debug messenger, extension is not present");
        throw std::runtime_error("Failed to set up debug messenger, extension is not present");
    }
}

void GuiEngine::createSurface()
{
    surface = VkSurfaceKHRUniquePtr(new VkSurfaceKHR, {getInstance()});
    VkResult result = glfwCreateWindowSurface(*getInstance().get(), getWindow(), nullptr, surface.get());
    checkVulkanResult(result, node->get_logger(), "Failed to create window surface");
}

void GuiEngine::createPhysicalDevice()
{
    uint32_t device_count = 0;
    vkEnumeratePhysicalDevices(*getInstance().get(), &device_count, nullptr);
    if (device_count == 0)
    {
        RCLCPP_FATAL(node->get_logger(), "Failed to find GPUs with Vulkan support");
        throw std::runtime_error("Failed to find GPUs with Vulkan support");
    }

    std::vector<VkPhysicalDevice> devices(device_count);
    vkEnumeratePhysicalDevices(*getInstance().get(), &device_count, devices.data());

    std::vector<VkPhysicalDevice>::iterator it = std::find_if(
        devices.begin(),
        devices.end(),
        [this](const VkPhysicalDevice &device) { return isDeviceSuitable(device); });

    if (it == devices.end())
    {
        RCLCPP_FATAL(node->get_logger(), "Failed to find a suitable GPU");
        throw std::runtime_error("Failed to find a suitable GPU");
    }

    physical_device = std::make_shared<VkPhysicalDevice>(*it);
}

void GuiEngine::createLogicalDevice()
{
    QueueFamilyIndices indices = findQueueFamilies(*getPhysicalDevice().get());

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
    create_info.ppEnabledExtensionNames = reinterpret_cast<const char *const *>(device_extensions.data());

    create_info.enabledLayerCount = 0;

    device = VkDeviceSharedPtr(new VkDevice);
    VkResult result = vkCreateDevice(*getPhysicalDevice().get(), &create_info, nullptr, getDevice().get());
    checkVulkanResult(result, node->get_logger(), "Failed to create logical device");

    vkGetDeviceQueue(*getDevice().get(), indices.graphics_family.value(), 0, &graphics_queue);
    vkGetDeviceQueue(*getDevice().get(), indices.present_family.value(), 0, &present_queue);
}

void GuiEngine::createSwapChain()
{
    SwapChainSupportDetails swap_chain_support = querySwapChainSupport(*getPhysicalDevice().get());

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
    create_info.surface = *surface.get();

    create_info.minImageCount = image_count;
    create_info.imageFormat = surface_format.format;
    create_info.imageColorSpace = surface_format.colorSpace;
    create_info.imageExtent = extent;
    create_info.imageArrayLayers = 1;
    create_info.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

    QueueFamilyIndices indices = findQueueFamilies(*getPhysicalDevice().get());
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

    swap_chain = VkSwapchainKHRUniquePtr(new VkSwapchainKHR, {getDevice()});
    VkResult result = vkCreateSwapchainKHR(*getDevice().get(), &create_info, nullptr, swap_chain.get());
    checkVulkanResult(result, node->get_logger(), "Failed to create swap chain");

    vkGetSwapchainImagesKHR(*getDevice().get(), *swap_chain.get(), &image_count, nullptr);
    swap_chain_images.resize(image_count);
    vkGetSwapchainImagesKHR(*getDevice().get(), *swap_chain.get(), &image_count, swap_chain_images.data());

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

    for (size_t i = 0; i < swap_chain_images.size(); i++)
    {
        create_info.image = swap_chain_images[i];

        swap_chain_image_views[i] = VkImageViewUniquePtr(new VkImageView, {getDevice()});
        VkResult result = vkCreateImageView(*getDevice().get(), &create_info, nullptr, swap_chain_image_views[i].get());
        checkVulkanResult(result, node->get_logger(), "Failed to create image views");
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

    render_pass = VkRenderPassSharedPtr(new VkRenderPass, VkRenderPassDeleter{getDevice()});
    VkResult result = vkCreateRenderPass(*getDevice().get(), &render_pass_info, nullptr, getRenderPass().get());
    checkVulkanResult(result, node->get_logger(), "Failed to create render pass");
}

void GuiEngine::createFramebuffers()
{
    VkResult result;
    swap_chain_framebuffers.resize(swap_chain_image_views.size());

    VkFramebufferCreateInfo framebuffer_info = {};
    framebuffer_info.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
    framebuffer_info.renderPass = *getRenderPass().get();
    framebuffer_info.attachmentCount = 1;
    framebuffer_info.width = swap_chain_extent.width;
    framebuffer_info.height = swap_chain_extent.height;
    framebuffer_info.layers = 1;

    for (size_t i = 0; i < swap_chain_image_views.size(); i++)
    {
        swap_chain_framebuffers[i] = VkFramebufferUniquePtr(new VkFramebuffer, {getDevice()});
        framebuffer_info.pAttachments = swap_chain_image_views[i].get();
        result = vkCreateFramebuffer(*getDevice().get(), &framebuffer_info, nullptr, swap_chain_framebuffers[i].get());
        checkVulkanResult(result, node->get_logger(), "Failed to create framebuffer");
    }
}

void GuiEngine::createCommandPool()
{
    QueueFamilyIndices indices = findQueueFamilies(*getPhysicalDevice().get());
    VkCommandPoolCreateInfo pool_info = {};
    pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    pool_info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    pool_info.queueFamilyIndex = indices.graphics_family.value();

    command_pool = VkCommandPoolSharedPtr(new VkCommandPool, VkCommandPoolDeleter{getDevice()});
    VkResult result = vkCreateCommandPool(*getDevice().get(), &pool_info, nullptr, getCommandPool().get());
    checkVulkanResult(result, node->get_logger(), "Failed to create command pool");
}

void GuiEngine::createDescriptorPool()
{
    // TODO: This descriptor pool is not optimal.
    // It should be created based on the user's needs (number of textures, etc.)
    VkDescriptorPoolSize pool_sizes[] = {
        {VK_DESCRIPTOR_TYPE_SAMPLER, 1000},
        {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1000},
        {VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1000},
        {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1000},
        {VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER, 1000},
        {VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER, 1000},
        {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1000},
        {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1000},
        {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, 1000},
        {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC, 1000},
        {VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1000}};
    VkDescriptorPoolCreateInfo pool_info = {};
    pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    pool_info.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    pool_info.maxSets = 1000 * IM_ARRAYSIZE(pool_sizes);
    pool_info.poolSizeCount = (uint32_t)IM_ARRAYSIZE(pool_sizes);
    pool_info.pPoolSizes = pool_sizes;

    descriptor_pool = VkDescriptorPoolSharedPtr(new VkDescriptorPool, VkDescriptorPoolDeleter{getDevice()});
    VkResult result = vkCreateDescriptorPool(*getDevice().get(), &pool_info, nullptr, getDescriptorPool().get());
    checkVulkanResult(result, node->get_logger(), "Failed to create descriptor pool");
}

void GuiEngine::createCommandBuffers()
{
    command_buffers.resize(MAX_FRAMES_IN_FLIGHT);

    VkCommandBufferAllocateInfo alloc_info = {};
    alloc_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    alloc_info.commandPool = *getCommandPool().get();
    alloc_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    alloc_info.commandBufferCount = (uint32_t)command_buffers.size();

    VkResult result = vkAllocateCommandBuffers(*getDevice().get(), &alloc_info, command_buffers.data());
    checkVulkanResult(result, node->get_logger(), "Failed to allocate command buffers");
}

void GuiEngine::createSyncObjects()
{
    VkResult result;

    image_available_semaphores.resize(MAX_FRAMES_IN_FLIGHT);
    render_finished_semaphores.resize(MAX_FRAMES_IN_FLIGHT);
    in_flight_fences.resize(MAX_FRAMES_IN_FLIGHT);

    VkSemaphoreCreateInfo semaphore_info = {};
    semaphore_info.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

    VkFenceCreateInfo fence_info = {};
    fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    fence_info.flags = VK_FENCE_CREATE_SIGNALED_BIT;

    for (int i = 0; i < MAX_FRAMES_IN_FLIGHT; i++)
    {
        image_available_semaphores[i] = VkSemaphoreUniquePtr(new VkSemaphore, {getDevice()});
        render_finished_semaphores[i] = VkSemaphoreUniquePtr(new VkSemaphore, {getDevice()});
        in_flight_fences[i] = VkFenceUniquePtr(new VkFence, {getDevice()});
        result = vkCreateSemaphore(*getDevice().get(), &semaphore_info, nullptr, image_available_semaphores[i].get());
        checkVulkanResult(result, node->get_logger(), "Failed to create synchronization objects for a frame");
        result = vkCreateSemaphore(*getDevice().get(), &semaphore_info, nullptr, render_finished_semaphores[i].get());
        checkVulkanResult(result, node->get_logger(), "Failed to create synchronization objects for a frame");
        result = vkCreateFence(*getDevice().get(), &fence_info, nullptr, in_flight_fences[i].get());
        checkVulkanResult(result, node->get_logger(), "Failed to create synchronization objects for a frame");
    }
}

VkSurfaceFormatKHR GuiEngine::chooseSwapSurfaceFormat(const std::vector<VkSurfaceFormatKHR> &available_formats)
{
    if (available_formats.empty())
    {
        RCLCPP_FATAL(node->get_logger(), "No available surface formats");
        throw std::runtime_error("No available surface formats");
    }

    auto it = std::find_if(
        available_formats.begin(),
        available_formats.end(),
        [](const VkSurfaceFormatKHR &format)
        { return format.format == VK_FORMAT_B8G8R8A8_SRGB && format.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR; });

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
    auto it = std::find_if(
        available_present_modes.begin(),
        available_present_modes.end(),
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
        if ((queue_family.queueCount > 0) && (queue_family.queueFlags & VK_QUEUE_GRAPHICS_BIT))
        {
            indices.graphics_family = i;
        }

        VkBool32 present_support = false;
        vkGetPhysicalDeviceSurfaceSupportKHR(device, i, *surface.get(), &present_support);

        if ((queue_family.queueCount > 0) && present_support)
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

    vkGetPhysicalDeviceSurfaceCapabilitiesKHR(device, *surface.get(), &details.capabilities);

    uint32_t format_count;
    vkGetPhysicalDeviceSurfaceFormatsKHR(device, *surface.get(), &format_count, nullptr);

    if (format_count != 0)
    {
        details.formats.resize(format_count);
        vkGetPhysicalDeviceSurfaceFormatsKHR(device, *surface.get(), &format_count, details.formats.data());
    }

    uint32_t present_mode_count;
    vkGetPhysicalDeviceSurfacePresentModesKHR(device, *surface.get(), &present_mode_count, nullptr);

    if (present_mode_count != 0)
    {
        details.present_modes.resize(present_mode_count);
        vkGetPhysicalDeviceSurfacePresentModesKHR(
            device,
            *surface.get(),
            &present_mode_count,
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
    while ((width == 0) || (height == 0))
    {
        glfwGetFramebufferSize(getWindow(), &width, &height);
        glfwWaitEvents();
    }

    vkDeviceWaitIdle(*getDevice().get());
    cleanupSwapChain();

    createSwapChain();
    createImageViews();
    createFramebuffers();
}

void GuiEngine::recordRenderPass(const VkCommandBuffer &command_buffer, uint32_t image_index)
{
    VkResult result;
    VkCommandBufferBeginInfo begin_info{};
    begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;

    result = vkBeginCommandBuffer(command_buffer, &begin_info);
    checkVulkanResult(result, node->get_logger(), "Failed to begin recording command buffer!");

    VkRenderPassBeginInfo render_pass_info{};
    render_pass_info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    render_pass_info.renderPass = *getRenderPass().get();
    render_pass_info.framebuffer = *swap_chain_framebuffers[image_index].get();
    render_pass_info.renderArea.offset = {0, 0};
    render_pass_info.renderArea.extent = swap_chain_extent;
    render_pass_info.clearValueCount = 1;
    render_pass_info.pClearValues = &clear_color;

    vkCmdBeginRenderPass(command_buffer, &render_pass_info, VK_SUBPASS_CONTENTS_INLINE);
    ImDrawData *draw_data = ImGui::GetDrawData();
    ImGui_ImplVulkan_RenderDrawData(draw_data, command_buffer);
    vkCmdEndRenderPass(command_buffer);
    result = vkEndCommandBuffer(command_buffer);
    checkVulkanResult(result, node->get_logger(), "Failed to record command buffer!");
}

GuiEngine::~GuiEngine() { cleanup(); }

void GuiEngine::draw()
{
    vkWaitForFences(*getDevice().get(), 1, in_flight_fences[current_frame].get(), VK_TRUE, UINT64_MAX);

    uint32_t image_index;
    VkResult result = vkAcquireNextImageKHR(
        *getDevice().get(),
        *swap_chain.get(),
        UINT64_MAX,
        *image_available_semaphores[current_frame].get(),
        VK_NULL_HANDLE,
        &image_index);

    if (result == VK_ERROR_OUT_OF_DATE_KHR)
    {
        rebuildSwapChain();
        return;
    }
    else if ((result != VK_SUCCESS) && (result != VK_SUBOPTIMAL_KHR))
    {
        RCLCPP_FATAL(node->get_logger(), "Failed to acquire swap chain image!");
        throw std::runtime_error("Failed to acquire swap chain image!");
    }

    vkResetFences(*getDevice().get(), 1, in_flight_fences[current_frame].get());

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

    result = vkQueueSubmit(graphics_queue, 1, &submit_info, *in_flight_fences[current_frame].get());
    checkVulkanResult(result, node->get_logger(), "Failed to submit draw command buffer!");

    VkPresentInfoKHR present_info{};
    present_info.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;

    present_info.waitSemaphoreCount = 1;
    present_info.pWaitSemaphores = signal_semaphores;

    VkSwapchainKHR swap_chains[] = {*swap_chain.get()};
    present_info.swapchainCount = 1;
    present_info.pSwapchains = swap_chains;

    present_info.pImageIndices = &image_index;

    result = vkQueuePresentKHR(present_queue, &present_info);

    if ((result == VK_ERROR_OUT_OF_DATE_KHR) || (result == VK_SUBOPTIMAL_KHR) || framebuffer_resized)
    {
        framebuffer_resized = false;
        rebuildSwapChain();
    }
    else
    {
        checkVulkanResult(result, node->get_logger(), "Failed to present swap chain image!");
    }
    current_frame = (current_frame + 1) % MAX_FRAMES_IN_FLIGHT;
}

void GuiEngine::framebufferResizeCallback(
    GLFWwindow *window,
    __attribute__((unused)) int width,
    __attribute__((unused)) int height)
{
    auto app = reinterpret_cast<GuiEngine *>(glfwGetWindowUserPointer(window));
    app->framebuffer_resized = true;
}

void GLFWwindowDeleter::operator()(GLFWwindow *window) const
{
    glfwDestroyWindow(window);
    glfwTerminate();
}

void GuiEngine::initGLFW()
{
    glfwInit();
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    window = GLFWwindowUniquePtr(glfwCreateWindow(800, 600, application_name.c_str(), nullptr, nullptr));
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
    initGLFW();
    initVulkan();
    imgui_engine->init(shared_from_this());
    for (const auto &texture : textures)
    {
        texture.second->init();
    }
    initialized = true;
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
    vkDeviceWaitIdle(*getDevice().get());
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

GuiEngine::GuiEngine(const std::string &application_name, std::shared_ptr<rclcpp::Node> node)
    : application_name(application_name), device_extensions({VK_KHR_SWAPCHAIN_EXTENSION_NAME}), node(node),
      imgui_engine(std::make_unique<ImGuiEngine>())
{
}

GuiEngine::GuiEngine(
    const std::string &application_name,
    std::shared_ptr<rclcpp::Node> node,
    const std::vector<std::string> &device_extensions)
    : application_name(application_name), device_extensions(device_extensions), node(node),
      imgui_engine(std::make_unique<ImGuiEngine>())
{
}

bool GuiEngine::addTexture(
    const std::string &name,
    std::vector<unsigned char> image_data,
    int width,
    int height,
    int channels)
{
    if (!initialized)
    {
        RCLCPP_ERROR(node->get_logger(), "Cannot add texture before initialization!");
        throw std::runtime_error("Cannot add texture before initialization!");
        return false;
    }
    if (textures.find(name) != textures.end())
    {
        RCLCPP_WARN(node->get_logger(), "Texture %s already exists!", name.c_str());
        return false;
    }
    textures.emplace(
        name,
        std::make_shared<TextureLoader>(shared_from_this(), node, image_data, width, height, channels));
    textures[name]->init();
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

VkCommandBuffer GuiEngine::getCommandBuffer(int index) const
{
    if ((static_cast<size_t>(index) < 0) || (static_cast<size_t>(index) >= command_buffers.size()))
    {
        RCLCPP_FATAL(node->get_logger(), "Invalid command buffer index: %d", index);
        throw std::runtime_error("Invalid command buffer index");
    }
    return command_buffers[index];
}

std::vector<VkImage> GuiEngine::getSwapChainImages() const
{
    if (swap_chain_images.empty())
    {
        RCLCPP_FATAL(node->get_logger(), "Swap chain images are empty");
        throw std::runtime_error("Swap chain images are empty");
    }
    return swap_chain_images;
}

VkQueue GuiEngine::getGraphicsQueue() const
{
    if (!graphics_queue)
    {
        RCLCPP_FATAL(node->get_logger(), "Graphics queue is not initialized");
        throw std::runtime_error("Graphics queue is not initialized");
    }
    return graphics_queue;
}

std::shared_ptr<VkPhysicalDevice> GuiEngine::getPhysicalDevice() const
{
    if (!physical_device)
    {
        RCLCPP_FATAL(node->get_logger(), "Physical device is not initialized");
        throw std::runtime_error("Physical device is not initialized");
    }
    return physical_device;
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
    QueueFamilyIndices indices = gui_engine->findQueueFamilies(*gui_engine->getPhysicalDevice().get());
    ImGui_ImplGlfw_InitForVulkan(gui_engine->getWindow(), true);
    ImGui_ImplVulkan_InitInfo init_info{};
    init_info.Instance = *gui_engine->getInstance().get();
    init_info.PhysicalDevice = *gui_engine->getPhysicalDevice().get();
    init_info.Device = *gui_engine->getDevice().get();
    init_info.QueueFamily = indices.graphics_family.value();
    init_info.Queue = gui_engine->getGraphicsQueue();
    init_info.PipelineCache = VK_NULL_HANDLE;
    init_info.DescriptorPool = *gui_engine->getDescriptorPool().get();
    init_info.Subpass = 0;
    init_info.MinImageCount = gui_engine->getSwapChainImages().size();
    init_info.ImageCount = gui_engine->getSwapChainImages().size();
    init_info.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
    init_info.Allocator = nullptr;
    init_info.CheckVkResultFn = nullptr;
    ImGui_ImplVulkan_Init(&init_info, *gui_engine->getRenderPass().get());
}

void ImGuiEngine::initFonts(std::shared_ptr<GuiEngine> gui_engine)
{
    VkCommandBufferBeginInfo begin_info{};
    VkCommandBuffer command_buffer = gui_engine->getCommandBuffer(0);
    vkResetCommandPool(*gui_engine->getDevice().get(), *gui_engine->getCommandPool().get(), 0);
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

TextureLoader::TextureLoader(
    std::shared_ptr<GuiEngine> gui_engine,
    std::shared_ptr<rclcpp::Node> node,
    std::vector<unsigned char> image_data,
    int width,
    int height,
    int channels)
    : channels(channels), height(height), width(width), gui_engine(gui_engine), image_data(image_data), node(node)
{
}

void TextureLoader::init()
{
    createImage();
    createImageView();
    createSampler();
    descriptor_set =
        ImGui_ImplVulkan_AddTexture(*sampler.get(), *image_view.get(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    createUploadBuffer();
    uploadToBuffer();
    recordCommandBuffer(gui_engine->getCommandPool(), gui_engine->getGraphicsQueue());
}

uint32_t TextureLoader::findMemoryType(uint32_t type_filter, VkMemoryPropertyFlags properties)
{
    VkPhysicalDeviceMemoryProperties mem_properties;
    vkGetPhysicalDeviceMemoryProperties(*gui_engine->getPhysicalDevice().get(), &mem_properties);

    for (uint32_t i = 0; i < mem_properties.memoryTypeCount; i++)
    {
        if ((type_filter & (1 << i)) && ((mem_properties.memoryTypes[i].propertyFlags & properties) == properties))
        {
            return i;
        }
    }

    RCLCPP_FATAL(node->get_logger(), "Failed to find suitable memory type!");
    throw std::runtime_error("Failed to find suitable memory type!");
}

void TextureLoader::createImage()
{
    VkImageCreateInfo image_info{};
    image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    image_info.imageType = VK_IMAGE_TYPE_2D;
    image_info.format = VK_FORMAT_R8G8B8A8_SRGB;
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
    image = VkImageUniquePtr(new VkImage, {gui_engine->getDevice()});

    VkResult result = vkCreateImage(*gui_engine->getDevice().get(), &image_info, nullptr, image.get());
    checkVulkanResult(result, node->get_logger(), "Failed to create image!");

    VkMemoryRequirements mem_requirements;
    vkGetImageMemoryRequirements(*gui_engine->getDevice().get(), *image.get(), &mem_requirements);

    VkMemoryAllocateInfo alloc_info{};
    alloc_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    alloc_info.allocationSize = mem_requirements.size;
    alloc_info.memoryTypeIndex = findMemoryType(mem_requirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    image_memory = VkDeviceMemoryUniquePtr(new VkDeviceMemory, {gui_engine->getDevice()});
    result = vkAllocateMemory(*gui_engine->getDevice().get(), &alloc_info, nullptr, image_memory.get());
    checkVulkanResult(result, node->get_logger(), "Failed to allocate image memory!");

    vkBindImageMemory(*gui_engine->getDevice().get(), *image.get(), *image_memory.get(), 0);
}

void TextureLoader::createImageView()
{
    VkImageViewCreateInfo view_info{};
    view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    view_info.image = *image.get();
    view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
    view_info.format = VK_FORMAT_R8G8B8A8_SRGB;
    view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    view_info.subresourceRange.baseMipLevel = 0;
    view_info.subresourceRange.levelCount = 1;
    view_info.subresourceRange.layerCount = 1;
    image_view = VkImageViewUniquePtr(new VkImageView, {gui_engine->getDevice()});
    VkResult result = vkCreateImageView(*gui_engine->getDevice().get(), &view_info, nullptr, image_view.get());
    checkVulkanResult(result, node->get_logger(), "Failed to create texture's image view!");
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
    sampler = VkSamplerUniquePtr(new VkSampler, {gui_engine->getDevice()});

    VkResult result = vkCreateSampler(*gui_engine->getDevice().get(), &sampler_info, nullptr, sampler.get());
    checkVulkanResult(result, node->get_logger(), "Failed to create texture's sampler!");
}

void TextureLoader::createUploadBuffer()
{
    size_t image_size = width * height * channels;
    VkBufferCreateInfo buffer_info{};
    buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    buffer_info.size = image_size;
    buffer_info.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
    buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    upload_buffer = VkBufferUniquePtr(new VkBuffer, {gui_engine->getDevice()});
    VkResult result = vkCreateBuffer(*gui_engine->getDevice().get(), &buffer_info, nullptr, upload_buffer.get());
    checkVulkanResult(result, node->get_logger(), "Failed to create upload buffer!");

    VkMemoryRequirements mem_requirements;
    vkGetBufferMemoryRequirements(*gui_engine->getDevice().get(), *upload_buffer.get(), &mem_requirements);

    VkMemoryAllocateInfo alloc_info{};
    alloc_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    alloc_info.allocationSize = mem_requirements.size;
    alloc_info.memoryTypeIndex = findMemoryType(mem_requirements.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT);

    upload_buffer_memory = VkDeviceMemoryUniquePtr(new VkDeviceMemory, VkDeviceMemoryDeleter{gui_engine->getDevice()});
    result = vkAllocateMemory(*gui_engine->getDevice().get(), &alloc_info, nullptr, upload_buffer_memory.get());
    checkVulkanResult(result, node->get_logger(), "Failed to allocate upload buffer memory!");

    result = vkBindBufferMemory(*gui_engine->getDevice().get(), *upload_buffer.get(), *upload_buffer_memory.get(), 0);
    checkVulkanResult(result, node->get_logger(), "Failed to bind upload buffer memory!");
}

void TextureLoader::uploadToBuffer()
{
    size_t image_size = width * height * channels;
    void *map = nullptr;
    VkResult result =
        vkMapMemory(*gui_engine->getDevice().get(), *upload_buffer_memory.get(), 0, VK_WHOLE_SIZE, 0, &map);
    checkVulkanResult(result, node->get_logger(), "Failed to map upload buffer memory!");

    memcpy(map, image_data.data(), image_size);
    VkMappedMemoryRange range = {};
    range.sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
    range.memory = *upload_buffer_memory.get();
    range.size = VK_WHOLE_SIZE;
    result = vkFlushMappedMemoryRanges(*gui_engine->getDevice().get(), 1, &range);
    checkVulkanResult(result, node->get_logger(), "Failed to flush upload buffer memory!");
    vkUnmapMemory(*gui_engine->getDevice().get(), *upload_buffer_memory.get());
}

void TextureLoader::recordCommandBuffer(VkCommandPoolSharedPtr command_pool, const VkQueue &graphics_queue)
{
    // Create command buffer
    VkCommandBuffer command_buffer;
    VkCommandBufferAllocateInfo alloc_info{};
    alloc_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    alloc_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    alloc_info.commandPool = *command_pool.get();
    alloc_info.commandBufferCount = 1;

    VkResult result = vkAllocateCommandBuffers(*gui_engine->getDevice().get(), &alloc_info, &command_buffer);
    checkVulkanResult(result, node->get_logger(), "Failed to allocate command buffer!");

    VkCommandBufferBeginInfo begin_info{};
    begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    begin_info.flags |= VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    result = vkBeginCommandBuffer(command_buffer, &begin_info);
    checkVulkanResult(result, node->get_logger(), "Failed to begin command buffer!");

    // Copy buffer to image
    VkImageMemoryBarrier copy_barrier = {};
    copy_barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    copy_barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    copy_barrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    copy_barrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    copy_barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    copy_barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    copy_barrier.image = *image.get();
    copy_barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    copy_barrier.subresourceRange.levelCount = 1;
    copy_barrier.subresourceRange.layerCount = 1;
    vkCmdPipelineBarrier(
        command_buffer,
        VK_PIPELINE_STAGE_HOST_BIT,
        VK_PIPELINE_STAGE_TRANSFER_BIT,
        0,
        0,
        nullptr,
        0,
        nullptr,
        1,
        &copy_barrier);

    VkBufferImageCopy region = {};
    region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    region.imageSubresource.layerCount = 1;
    region.imageExtent.width = width;
    region.imageExtent.height = height;
    region.imageExtent.depth = 1;
    vkCmdCopyBufferToImage(
        command_buffer,
        *upload_buffer.get(),
        *image.get(),
        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
        1,
        &region);

    VkImageMemoryBarrier use_barrier = {};
    use_barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    use_barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    use_barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
    use_barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    use_barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    use_barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    use_barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    use_barrier.image = *image.get();
    use_barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    use_barrier.subresourceRange.levelCount = 1;
    use_barrier.subresourceRange.layerCount = 1;
    vkCmdPipelineBarrier(
        command_buffer,
        VK_PIPELINE_STAGE_TRANSFER_BIT,
        VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
        0,
        0,
        nullptr,
        0,
        nullptr,
        1,
        &use_barrier);

    // End command buffer
    VkSubmitInfo submit_info{};
    submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submit_info.commandBufferCount = 1;
    submit_info.pCommandBuffers = &command_buffer;
    result = vkEndCommandBuffer(command_buffer);
    checkVulkanResult(result, node->get_logger(), "Failed to end command buffer!");
    result = vkQueueSubmit(graphics_queue, 1, &submit_info, VK_NULL_HANDLE);
    checkVulkanResult(result, node->get_logger(), "Failed to submit command buffer!");
    result = vkDeviceWaitIdle(*gui_engine->getDevice().get());
    checkVulkanResult(result, node->get_logger(), "Failed to wait for device to become idle!");
}

void TextureLoader::updateTexture(std::vector<unsigned char> image_data)
{
    this->image_data = image_data;
    uploadToBuffer();
    recordCommandBuffer(gui_engine->getCommandPool(), gui_engine->getGraphicsQueue());
}

TextureLoader::~TextureLoader() { ImGui_ImplVulkan_RemoveTexture(descriptor_set); }

} // namespace gui_node
