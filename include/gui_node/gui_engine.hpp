/*
 * Copyright (c) 2022-2024 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <GLFW/glfw3.h>
#include <imgui_impl_vulkan.h>
#include <imgui_stdlib.h>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <vulkan/vulkan.hpp>

#define VK_DECLARE_TYPE(obj)                                                    \
    struct Vk##obj##Deleter                                                     \
    {                                                                           \
        void operator()(Vk##obj *obj) const { vkDestroy##obj(*obj, nullptr); }; \
    };                                                                          \
    using Vk##obj##UniquePtr = std::unique_ptr<Vk##obj, Vk##obj##Deleter>;      \
    using Vk##obj##SharedPtr = std::shared_ptr<Vk##obj>;

#define VK_DECLARE_TYPE_WITH_PARENT(obj, parent)                                               \
    struct Vk##obj##Deleter                                                                    \
    {                                                                                          \
        Vk##parent##SharedPtr parent;                                                          \
        void operator()(Vk##obj *obj) const { vkDestroy##obj(*parent.get(), *obj, nullptr); }; \
    };                                                                                         \
    using Vk##obj##UniquePtr = std::unique_ptr<Vk##obj, Vk##obj##Deleter>;                     \
    using Vk##obj##SharedPtr = std::shared_ptr<Vk##obj>;

#define VK_DECLARE_GETTER(obj, var)                                                     \
    Vk##obj##SharedPtr get##obj()                                                       \
    {                                                                                   \
        if (!var)                                                                       \
        {                                                                               \
            RCLCPP_FATAL(node->get_logger(), "The object " #var " wasn't initialized"); \
            throw std::runtime_error("The object " #var " wasn't initialized");         \
        }                                                                               \
        return var;                                                                     \
    }

namespace gui_node
{

VK_DECLARE_TYPE(Instance)                           ///< Define a unique pointer for a VkInstance
VK_DECLARE_TYPE(Device)                             ///< Define a unique pointer for a VkDevice
VK_DECLARE_TYPE_WITH_PARENT(Buffer, Device)         ///< Define a unique pointer for a VkBuffer
VK_DECLARE_TYPE_WITH_PARENT(CommandPool, Device)    ///< Define a unique pointer for a VkCommandPool
VK_DECLARE_TYPE_WITH_PARENT(DescriptorPool, Device) ///< Define a unique pointer for a VkDescriptorPool
VK_DECLARE_TYPE_WITH_PARENT(Fence, Device)          ///< Define a unique pointer for a VkFence
VK_DECLARE_TYPE_WITH_PARENT(Framebuffer, Device)    ///< Define a unique pointer for a VkFramebuffer
VK_DECLARE_TYPE_WITH_PARENT(Image, Device)          ///< Define a unique pointer for a VkImage
VK_DECLARE_TYPE_WITH_PARENT(ImageView, Device)      ///< Define a unique pointer for a VkImageView
VK_DECLARE_TYPE_WITH_PARENT(RenderPass, Device)     ///< Define a unique pointer for a VkRenderPass
VK_DECLARE_TYPE_WITH_PARENT(Sampler, Device)        ///< Define a unique pointer for a VkSampler
VK_DECLARE_TYPE_WITH_PARENT(Semaphore, Device)      ///< Define a unique pointer for a VkSemaphore
VK_DECLARE_TYPE_WITH_PARENT(SurfaceKHR, Instance)   ///< Define a unique pointer for a VkSurfaceKHR
VK_DECLARE_TYPE_WITH_PARENT(SwapchainKHR, Device)   ///< Define a unique pointer for a VkSwapchainKHR

/**
 * Verifies if the given Vulkan result is a success.
 * If not, reports an error and returns false.
 *
 * @param result The result to verify.
 * @param logger The logger to use.
 * @param message The message to display if the result is not a success.
 *
 * @return True if the result is a success, false otherwise.
 */
bool checkVulkanResult(VkResult result, const rclcpp::Logger &logger, const std::string &message);

class GuiEngine; ///< Forward declaration

/**
 * A structure for deleting a Vulkan debug messenger.
 */
struct VkDebugUtilsMessengerEXTDeleter
{
    /**
     * Deletes the messenger.
     *
     * @param debug_utils_messenger Pointer to the messenger.
     */
    void operator()(VkDebugUtilsMessengerEXT *debug_utils_messenger) const;

    VkInstance instance; ///< Vulkan instance the messenger belongs to
};

/// Unique pointer to a Vulkan debug messenger
using VkDebugUtilsMessengerEXTUniquePtr = std::unique_ptr<VkDebugUtilsMessengerEXT, VkDebugUtilsMessengerEXTDeleter>;

/**
 * A structure for deleting a GLFW window.
 */
struct GLFWwindowDeleter
{
    /**
     * Deletes the window and terminates glfw.
     *
     * @param window Pointer to the window.
     */
    void operator()(GLFWwindow *window) const;
};

/// A unique pointer to a GLFW window
using GLFWwindowUniquePtr = std::unique_ptr<GLFWwindow, GLFWwindowDeleter>;

/**
 * A structure for deleting the Vulkan memory.
 */
struct VkDeviceMemoryDeleter
{
    /**
     * Frees the allocated memory.
     *
     * @param memory Pointer to the memory to be freed.
     */
    void operator()(VkDeviceMemory *memory) const { vkFreeMemory(*device.get(), *memory, nullptr); };

    VkDeviceSharedPtr device; ///< Device to which the memory belongs.
};

/// A unique pointer to the Vulkan memory
using VkDeviceMemoryUniquePtr = std::unique_ptr<VkDeviceMemory, VkDeviceMemoryDeleter>;

/**
 * Stores indices of queue families.
 */
struct QueueFamilyIndices
{
    /**
     * Checks if all queue families are present.
     * @return True if all queue families are present, false otherwise.
     */
    bool isComplete() { return graphics_family.has_value() && present_family.has_value(); }

    std::optional<uint32_t> graphics_family; ///< Index of graphics queue family
    std::optional<uint32_t> present_family;  ///< Index of present queue family
};

/**
 * Details about the swap chain
 */
struct SwapChainSupportDetails
{
    VkSurfaceCapabilitiesKHR capabilities;       ///< Surface capabilities
    std::vector<VkSurfaceFormatKHR> formats;     ///< Surface formats swap chain can use
    std::vector<VkPresentModeKHR> present_modes; ///< Presentation modes swap chain can use
};

const VkClearValue clear_color = {{{0.094f, 0.094f, 0.094f, 1.0f}}}; ///< Clear color for the framebuffer

/**
 * The class responsible for initializing ImGui to work with Vulkan.
 */
class ImGuiEngine
{
private:
    /**
     * Initializes Vulkan backend for ImGui.
     *
     * @param gui_engine Reference to the GuiEngine object.
     *
     * @return True if the initialization was successful, false otherwise.
     */
    bool initVulkanImpl(std::shared_ptr<GuiEngine> gui_engine);

    /**
     * Initializes ImGui font.
     *
     * @param gui_engine Pointer to the GUI engine.
     */
    void initFonts(std::shared_ptr<GuiEngine> gui_engine);

    /**
     * Initializes ImGui theme.
     */
    void setColor();

public:
    /**
     * Constructor.
     */
    ImGuiEngine() = default;

    /**
     * Initializes ImGui.
     *
     * @param gui_engine Reference to the GuiEngine object.
     *
     * @return True if the initialization was successful, false otherwise.
     */
    bool init(std::shared_ptr<GuiEngine> gui_engine);

    /**
     * Destroys ImGui context and frees all resources.
     */
    ~ImGuiEngine();
};

/**
 * A class for loading and managing texture objects.
 */
class TextureLoader
{
private:
    /**
     * Finds a memory type that has the requested properties.
     *
     * @param type_filter Bitmask of memory types to consider.
     * @param properties Properties to look for.
     * @return Index of the memory type that has the requested properties.
     *
     * @throw std::runtime_error If no suitable memory type is found.
     */
    uint32_t findMemoryType(uint32_t type_filter, VkMemoryPropertyFlags properties);

    /**
     * Creates an image.
     *
     * @return True if the image was created successfully, false otherwise.
     */
    bool createImage();

    /**
     * Creates image view for the image.
     *
     * @return True if the image view was created successfully, false otherwise.
     */
    bool createImageView();

    /**
     * Creates a sampler for the image.
     *
     * @return True if the sampler was created successfully, false otherwise.
     */
    bool createSampler();

    /**
     * Creates a buffer for uploading data to the GPU.
     *
     * @return True if the buffer was created successfully, false otherwise.
     */
    bool createUploadBuffer();

    /**
     * Uploads the image data to the GPU.
     *
     * return True if the image data was uploaded successfully, false otherwise.
     */
    bool uploadToBuffer();

    /**
     * Records a command buffer for copying the image data to the GPU.
     *
     * @param command_pool Shared pointer to the command pool to use.
     * @param graphics_queue Graphics queue to use.
     *
     * @bool True if the command buffer was recorded successfully, false otherwise.
     */
    bool recordCommandBuffer(VkCommandPoolSharedPtr command_pool, const VkQueue &graphics_queue);

    int channels;                                 ///< Number of channels in the texture
    int height;                                   ///< Height of the texture
    int width;                                    ///< Width of the texture
    std::shared_ptr<GuiEngine> gui_engine;        ///< Pointer to the GUI engine
    std::vector<unsigned char> image_data;        ///< Data of the textures
    VkBufferUniquePtr upload_buffer;              ///< Buffer for uploading data to the GPU
    VkDescriptorSet descriptor_set;               ///< Descriptor set for the textures
    VkDeviceMemoryUniquePtr image_memory;         ///< Memory for the textures
    VkDeviceMemoryUniquePtr upload_buffer_memory; ///< Memory for the upload buffer
    VkImageUniquePtr image;                       ///< Image for the textures
    VkImageViewUniquePtr image_view;              ///< Image view for the textures
    VkSamplerUniquePtr sampler;                   ///< Sampler for the textures
    std::shared_ptr<rclcpp::Node> node;           ///< Pointer to the ROS2 node

public:
    /**
     * Creates a texture loader.
     *
     * @param gui_engine Pointer to the GUI engine.
     * @param node Pointer to the GUI node.
     * @param image_data Vector of the image data.
     * @param width Width of the image.
     * @param height Height of the image.
     * @param channels Number of channels in the image.
     */
    TextureLoader(
        std::shared_ptr<GuiEngine> gui_engine,
        std::shared_ptr<rclcpp::Node> node,
        std::vector<unsigned char> image_data,
        int width,
        int height,
        int channels);

    /**
     * Destroys the texture loader and frees all resources.
     */
    ~TextureLoader();

    /**
     * Loads the texture and creates upload_buffer for it.
     *
     * @return bool True if the texture was loaded successfully.
     */
    bool init();

    /**
     * Returns the descriptor set.
     *
     * @return VkDescriptorSet The descriptor set.
     */
    VkDescriptorSet getDescriptorSet() { return descriptor_set; }

    /**
     * Returns the width of the texture.
     *
     * @return int The width of the texture.
     */
    int getWidth() { return width; }

    /**
     * Returns the height of the texture.
     *
     * @return int The height of the texture.
     */
    int getHeight() { return height; }

    /**
     * Updates the texture with provided image data.
     *
     * @param image_data Vector of the image data.
     * @param width Width of the image.
     * @param height Height of the image.
     * @param channels Number of channels in the image.
     *
     * @return bool True if the texture was updated successfully.
     */
    bool updateTexture(std::vector<unsigned char> image_data, int width, int height, int channels);
};

/**
 * A class for initializing and managing Vulkan objects.
 */
class GuiEngine : public std::enable_shared_from_this<GuiEngine>
{
    /**
     * Checks if the required validation layers are available and supported by the GPU.
     *
     * @return True if the required validation layers are available and supported by the GPU, false otherwise.
     */
    bool checkValidationLayerSupport();

    /**
     * Processes debug messages from the validation layers.
     *
     * @param message_severity Severity of the message.
     * @param message_type Type of the message.
     * @param p_callback_data Pointer to the callback data.
     * @param p_user_data Pointer to user data.
     * @return VK_FALSE.
     */
    static VKAPI_ATTR VkBool32 VKAPI_CALL debugcallback(
        VkDebugUtilsMessageSeverityFlagBitsEXT message_severity,
        VkDebugUtilsMessageTypeFlagsEXT message_type,
        const VkDebugUtilsMessengerCallbackDataEXT *p_callback_data,
        void *p_user_data);

    /**
     * Creates create info for the debug messenger.
     *
     * @param create_info Reference to the creation info to be filled.
     */
    void populateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT &create_info);

    /**
     * Sets up the debug messenger.
     *
     * @return True if the debug messenger was set up successfully, false otherwise.
     */
    bool setupDebugMessenger();

    /**
     * Query the swap chain support details for the given physical device.
     *
     * @param device The physical device to query.
     * @return The swap chain support details for the given physical device.
     */
    SwapChainSupportDetails querySwapChainSupport(const VkPhysicalDevice &device);

    /**
     * Check if the device supports the required extensions.
     *
     * @param device The physical device to check.
     * @return True if the device supports the required extensions, false otherwise.
     */
    bool checkDeviceExtensionSupport(const VkPhysicalDevice &device);

    /**
     * Chooses surface format for swap chain that best matches the requirements.
     *
     * @param availableFormats The available surface formats.
     * @return The surface format for the swap chain that best matches the requirements, else a default format.
     *
     * @throws std::runtime_error If no suitable format is found.
     */
    VkSurfaceFormatKHR chooseSwapSurfaceFormat(const std::vector<VkSurfaceFormatKHR> &available_formats);

    /**
     * Chooses presentation mode for swap chain that best matches the requirements.
     *
     * @param availablePresentModes The available presentation modes.
     * @return The presentation mode for the swap chain that best matches the requirements, else a default fifo mode.
     */
    VkPresentModeKHR chooseSwapPresentMode(const std::vector<VkPresentModeKHR> &available_present_modes);

    /**
     * Chooses swap extent for swap chain that best matches the requirements.
     *
     * @param capabilities The surface capabilities.
     * @return The swap extent for the swap chain that best matches the requirements, else a default extent.
     */
    VkExtent2D chooseSwapExtent(const VkSurfaceCapabilitiesKHR &capabilities);

    /**
     * Checks if the given physical device is suitable.
     *
     * @param device The physical device to check.
     * @return True if the device is suitable, false otherwise.
     */
    bool isDeviceSuitable(const VkPhysicalDevice &device);

    /**
     * Recreates the swap chain, used when the window is resized.
     *
     * @return True if the swap chain was recreated successfully, false otherwise.
     */
    bool rebuildSwapChain();

    /**
     * Writes commands for drawing to the command buffer.
     *
     * @param command_buffer The command buffer to write the commands to.
     * @param image_index The index of the image to draw to.
     *
     * @return True if the commands were written successfully, false otherwise.
     */
    bool recordRenderPass(const VkCommandBuffer &command_buffer, uint32_t image_index);

    /**
     * Creates a Vulkan instance with GLFW extensions.
     *
     * @return True if the instance was created successfully, false otherwise.
     */
    bool createInstance();

    /**
     * Creates a Vulkan surface for the window.
     *
     * @return True if the surface was created successfully, false otherwise.
     */
    bool createSurface();

    /**
     * Creates a logical device and queues.
     *
     * @return True if the logical device and queues were created successfully, false otherwise.
     */
    bool createLogicalDevice();

    /**
     * Chooses the best physical device for the previously initialized Vulkan instance.
     *
     * @return True if a physical device was chosen successfully, false otherwise.
     */
    bool createPhysicalDevice();

    /**
     * Creates the swap chain.
     *
     * @return True if the swap chain was created successfully, false otherwise.
     */
    bool createSwapChain();

    /**
     * Creates the swap chain image views.
     *
     * @return True if the swap chain image views were created successfully, false otherwise.
     */
    bool createImageViews();

    /**
     * Creates the render pass.
     *
     * @return True if the render pass was created successfully, false otherwise.
     */
    bool createRenderPass();

    /**
     * Creates the frame buffers.
     *
     * @return True if the frame buffers were created successfully, false otherwise.
     */
    bool createFramebuffers();

    /**
     * Creates the command pool.
     *
     * @return True if the command pool was created successfully, false otherwise.
     */
    bool createCommandPool();

    /**
     * Creates a descriptor pool for the GUI.
     *
     * @return True if the descriptor pool was created successfully, false otherwise.
     */
    bool createDescriptorPool();

    /**
     * Creates the command buffers.
     *
     * @return True if the command buffers were created successfully, false otherwise.
     */
    bool createCommandBuffers();

    /**
     * Creates the semaphores.
     *
     * @return True if the semaphores were created successfully, false otherwise.
     */
    bool createSyncObjects();

    /**
     * Callback for when the window is resized.
     *
     * @param window The window that was resized.
     * @param width The new width of the window.
     * @param height The new height of the window.
     */
    static void framebufferResizeCallback(GLFWwindow *window, int width, int height);

    /**
     * Initializes GLFW backend.
     *
     * @param maximize Whether to maximize window on startup.
     *
     * @return True if initialization was successful, false otherwise.
     */
    bool initGLFW(bool maximize);

    /**
     * Initializes Vulkan backend.
     *
     * @return True if initialization was successful, false otherwise.
     */
    bool initVulkan();

    /**
     * Cleans up allocated resources and destroys Vulkan, GLFW and ImGui backends.
     */
    void cleanup();

    /**
     * Cleans up allocated swap chain along with its resources.
     */
    void cleanupSwapChain();

    bool framebuffer_resized = false;           ///< Flag for when the framebuffer is resized
    bool initialized = false;                   ///< Flag for when the GUI engine is initialized
    const int MAX_FRAMES_IN_FLIGHT = 2;         ///< Maximum number of frames in flight
    const std::string application_name;         ///< Name of the application
    uint32_t current_frame = 0;                 ///< Current frame
    std::vector<std::string> device_extensions; ///< Device extensions required for the application
    std::shared_ptr<rclcpp::Node> node;         ///< The ROS2 node for logging

    GLFWwindowUniquePtr window; ///< GLFW Window

    VkCommandPoolSharedPtr command_pool;               ///< Command pool
    VkDebugUtilsMessengerEXTUniquePtr debug_messenger; ///< Debug messenger
    VkDescriptorPoolSharedPtr descriptor_pool;         ///< Descriptor pool for ImGui
    VkDeviceSharedPtr device;                          ///< Vulkan logical device
    VkExtent2D swap_chain_extent;                      ///< Swap chain extent
    VkFormat swap_chain_image_format;                  ///< Swap chain image format
    VkInstanceSharedPtr instance;                      ///< Vulkan instance
    VkQueue graphics_queue;                            ///< Graphics queue
    VkQueue present_queue;                             ///< Queue for presenting images to the screen
    VkRenderPassSharedPtr render_pass;                 ///< Render pass
    VkSurfaceKHRUniquePtr surface;                     ///< Surface for rendering
    VkSwapchainKHRUniquePtr swap_chain;                ///< Swapchain for presenting images to the screen

    std::unique_ptr<ImGuiEngine> imgui_engine;                    ///< ImGui engine
    std::shared_ptr<VkPhysicalDevice> physical_device;            ///< Physical device (GPU) that Vulkan will be using
    std::vector<VkCommandBuffer> command_buffers;                 ///< Command buffers
    std::vector<VkFenceUniquePtr> in_flight_fences;               ///< Fences for in flight frames
    std::vector<VkFramebufferUniquePtr> swap_chain_framebuffers;  ///< Framebuffers for the swap chain images
    std::vector<VkImage> swap_chain_images;                       ///< Swap chain images
    std::vector<VkImageViewUniquePtr> swap_chain_image_views;     ///< Swapchain images views
    std::vector<VkSemaphoreUniquePtr> image_available_semaphores; ///< Semaphore for when an image is available
    std::vector<VkSemaphoreUniquePtr> render_finished_semaphores; ///< Semaphore for when rendering is finished

    /// Textures loaded by the application
    std::unordered_map<std::string, std::shared_ptr<TextureLoader>> textures;

#ifdef NDEBUG
    const bool enable_validation_layers = false; ///< Disable validation layers
#else
    const bool enable_validation_layers = true; ///< Enable validation layers
#endif

    const std::vector<const char *> validation_layers = {"VK_LAYER_KHRONOS_validation"}; ///< Validation layers to use

public:
    /**
     * Constructor.
     *
     * @param application_name The name of the application window.
     * @param node The ROS2 node for the GUI.
     */
    GuiEngine(const std::string &application_name, std::shared_ptr<rclcpp::Node> node);

    /**
     * Constructor.
     *
     * @param application_name The name of the application window.
     * @param node The ROS2 node for the GUI.
     * @param device_extensions Vector of required extensions for physical device.
     */
    GuiEngine(
        const std::string &application_name,
        std::shared_ptr<rclcpp::Node> node,
        const std::vector<std::string> &device_extensions);

    /**
     * Destroy the GuiEngine object.
     * Cleanups all the resources and closes GUI.
     */
    ~GuiEngine();

    /**
     * Initialize GUI backend.
     * Initializes GLFW, Vulkan and ImGui.
     *
     * @param maximize Whether to maximize window on startup.
     *
     * @return bool True if the GUI was successfully initialized, false otherwise.
     */
    bool init(bool maximize);

    /**
     * Draws the frame.
     *
     * @return bool True if the frame was successfully drew, false otherwise.
     */
    bool draw();

    /**
     * Adds a new texture object to the GUI.
     *
     * @param name The name of the texture.
     * @param image_data The data of the texture.
     * @param width The width of the texture.
     * @param height The height of the texture.
     * @param channels The number of channels of the texture.
     * @return bool True if the texture was successfully added, false otherwise.
     */
    bool
    addTexture(const std::string &name, std::vector<unsigned char> image_data, int width, int height, int channels);

    /**
     * Get pointer to GLFW window.
     *
     * @return GLFWwindow*.
     */
    GLFWwindow *getWindow() const { return window.get(); }

    /**
     * Gets the texture object with the given name.
     *
     * @param name The name of the texture.
     * @return TextureLoader* The texture object.
     */
    std::shared_ptr<TextureLoader> getTexture(const std::string &name);

    /**
     * Gets the command buffer at the given index.
     *
     * @param index The index of the command buffer.
     * @return VkCommandBuffer The command buffer.
     *
     * @throws std::runtime_error if the index is out of bounds.
     */
    VkCommandBuffer getCommandBuffer(int index) const;

    /**
     * Gets the command pool.
     *
     * @return VkCommandPoolSharedPtr The shared pointer to the command pool.
     *
     * @throws std::runtime_error if the command pool is not initialized.
     */
    VK_DECLARE_GETTER(CommandPool, command_pool)

    /**
     * Gets the render pass.
     *
     * @return VkRenderPassSharedPtr The shared pointer to the render pass.
     *
     * @throws std::runtime_error if the render pass is not initialized.
     */
    VK_DECLARE_GETTER(RenderPass, render_pass)

    /**
     * Gets the swap chain images.
     *
     * @return std::vector<VkImage> The swap chain images.
     *
     * @throws std::runtime_error if the swap chain images are empty.
     */
    std::vector<VkImage> getSwapChainImages() const;

    /**
     * Gets the descriptor pool.
     *
     * @return VkDescriptorPoolSharedPtr Shared pointer to the descriptor pool.
     *
     * @throws std::runtime_error if the descriptor pool is not initialized.
     */
    VK_DECLARE_GETTER(DescriptorPool, descriptor_pool)

    /**
     * Gets the instance object.
     *
     * @return VkInstanceSharedPtr Shared pointer to the instance object.
     *
     * @throws std::runtime_error if the instance is not initialized.
     */
    VK_DECLARE_GETTER(Instance, instance)

    /**
     * Gets the graphics queue.
     *
     * @return VkQueue The graphics queue.
     *
     * @throws std::runtime_error if the graphics queue is not initialized.
     */
    VkQueue getGraphicsQueue() const;

    /**
     * Gets the logical device.
     *
     * @return VkDeviceSharedPtr Shared pointer to the logical device.
     *
     * @throws std::runtime_error if the logical device is not initialized.
     */
    VK_DECLARE_GETTER(Device, device)

    /**
     * Gets the physical device object.
     *
     * @return std::shared_ptr<VkPhysicalDevice> Shared pointer to the physical device object.
     *
     * @throws std::runtime_error if the physical device is not initialized.
     */
    std::shared_ptr<VkPhysicalDevice> getPhysicalDevice() const;

    /**
     * Find the queue families of the physical device.
     *
     * @param physicalDevice The physical device to find the queue families of.
     * @return QueueFamilyIndices The queue families of the physical device.
     */
    QueueFamilyIndices findQueueFamilies(const VkPhysicalDevice &device);
};

} // namespace gui_node
