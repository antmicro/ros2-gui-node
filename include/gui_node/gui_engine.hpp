#ifndef GUI_NODE_GUI_ENGINE_HPP
#define GUI_NODE_GUI_ENGINE_HPP

#pragma once

#include <GLFW/glfw3.h>
#include <imgui_impl_vulkan.h>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <vulkan/vulkan.hpp>

#include "gui_node/gui_node.hpp"

namespace gui_node
{

class GuiEngine; ///< Forward declaration

/**
 * Stores indices of queue families.
 */
struct QueueFamilyIndices
{
    std::optional<uint32_t> graphics_family; ///< Index of graphics queue family.
    std::optional<uint32_t> present_family;  ///< Index of present queue family.

    /**
     * Checks if all queue families are present.
     * @return True if all queue families are present, false otherwise.
     */
    bool isComplete() { return graphics_family.has_value() && present_family.has_value(); }
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

const VkClearValue clear_color = {{{0.45f, 0.55f, 0.60f, 1.0f}}}; ///< Clear color for the framebuffer

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
     */
    void initVulkanImpl(std::shared_ptr<GuiEngine> gui_engine);

    /**
     * Initializes ImGui font.
     *
     * @param gui_engine Pointer to the GUI engine.
     */
    void initFonts(std::shared_ptr<GuiEngine> gui_engine);

public:
    /**
     * Constructor.
     */
    ImGuiEngine() = default;

    /**
     * Initializes ImGui.
     *
     * @param gui_engine Reference to the GuiEngine object.
     */
    void init(std::shared_ptr<GuiEngine> gui_engine);

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
    VkBuffer upload_buffer;              ///< Buffer for uploading data to the GPU
    VkDescriptorSet descriptor_set;      ///< Descriptor set for the textures
    VkDevice device;                     ///< Vulkan device
    VkDeviceMemory image_memory;         ///< Memory for the textures
    VkDeviceMemory upload_buffer_memory; ///< Memory for the upload buffer
    VkImage image;                       ///< Image for the textures
    VkImageView image_view;              ///< Image view for the textures
    VkSampler sampler;                   ///< Sampler for the textures
    int channels{4};                     ///< Number of channels in the textures
    int height;                          ///< Height of the textures
    int width;                           ///< Width of the textures

    /**
     * Finds a memory type that has the requested properties.
     *
     * @param type_filter Bitmask of memory types to consider.
     * @param physical_device Physical device to use.
     * @param properties Properties to look for.
     * @return Index of the memory type that has the requested properties.
     *
     * @throw std::runtime_error If no suitable memory type is found.
     */
    uint32_t findMemoryType(uint32_t type_filter, const VkPhysicalDevice &physical_device,
                            VkMemoryPropertyFlags properties);

    /**
     * Creates an image.
     *
     * @param physical_device Physical device to use.
     *
     * @throw std::runtime_error If the image cannot be created.
     */
    void createImage(const VkPhysicalDevice &physical_device);

    /**
     * Creates image view for the image.
     *
     * @throw std::runtime_error If the image view cannot be created.
     */
    void createImageView();

    /**
     * Creates a sampler for the image.
     *
     * @throw std::runtime_error If the sampler cannot be created.
     */
    void createSampler();

    /**
     * Creates a buffer for uploading data to the GPU.
     *
     * @param image_size Size of the image.
     * @param physical_device Physical device to use.
     *
     * @throw std::runtime_error If the buffer cannot be created.
     */
    void createUploadBuffer(size_t image_size, const VkPhysicalDevice &physical_device);

    /**
     * Uploads the image data to the GPU.
     *
     * @param image_size Size of the image.
     * @param image_data Pointer to the image data.
     *
     * @throw std::runtime_error If the data cannot be uploaded.
     */
    void uploadToBuffer(size_t image_size, unsigned char *image_data);

    /**
     * Records a command buffer for copying the image data to the GPU.
     *
     * @param command_pool Command pool to use.
     * @param graphics_queue Graphics queue to use.
     *
     * @throw std::runtime_error If the command buffer cannot be recorded.
     */
    void recordCommandBuffer(const VkCommandPool &command_pool, const VkQueue &graphics_queue);

public:
    /**
     * Creates a texture loader.
     *
     * @param image_data The image data to be loaded.
     * @param width The width of the image.
     * @param height The height of the image.
     * @param channels The number of channels in the image.
     * @param device Vulkan logical device.
     * @param physical_device Vulkan physical device.
     * @param command_pool Vulkan command pool.
     * @param graphics_queue Graphics queue.
     */
    TextureLoader(unsigned char *image_data, int width, int height, int channels, const VkDevice &device,
                  const VkPhysicalDevice &physical_device, const VkCommandPool &command_pool,
                  const VkQueue &graphics_queue);

    /**
     * Destroys the texture loader and frees all resources.
     */
    ~TextureLoader();

    /**
     * Returns the descriptor set.
     *
     * @return The descriptor set.
     */
    VkDescriptorSet getDescriptorSet() { return descriptor_set; }

    /**
     * Returns the width of the texture.
     *
     * @return The width of the texture.
     */
    int getWidth() { return width; }

    /**
     * Returns the height of the texture.
     *
     * @return The height of the texture.
     */
    int getHeight() { return height; }
};

class GuiEngine : public std::enable_shared_from_this<GuiEngine>
{
    std::unique_ptr<VkPhysicalDevice> physical_device; ///< Physical device (GPU) that Vulkan will be using
    VkQueue graphics_queue;                            ///< Graphics queue
    VkQueue present_queue;                             ///< Queue for presenting images to the screen
    std::vector<VkImage> swap_chain_images;            ///< Swap chain images
    VkFormat swap_chain_image_format;                  ///< Swap chain image format
    VkExtent2D swap_chain_extent;                      ///< Swap chain extent
    std::vector<VkCommandBuffer> command_buffers;      ///< Command buffers
    uint32_t current_frame = 0;                        ///< Current frame
    bool framebuffer_resized = false;                  ///< Flag for when the framebuffer is resized
    std::vector<const char *> device_extensions;       ///< Device extensions required for the application
    const int MAX_FRAMES_IN_FLIGHT = 2;                ///< Maximum number of frames in flight
    const std::string application_name;                ///< Name of the application
    std::shared_ptr<GuiNode> node;                     ///< The ROS2 node for the GUI
    std::unique_ptr<ImGuiEngine> imgui_engine;         ///< ImGui engine

    std::unique_ptr<GLFWwindow, std::function<void(GLFWwindow *)>> window;      ///< GLFW Window
    std::unique_ptr<VkDevice, std::function<void(VkDevice *)>> device;          ///< Vulkan logical device
    std::unique_ptr<VkInstance, std::function<void(VkInstance *)>> instance;    ///< Vulkan instance
    std::unordered_map<std::string, std::shared_ptr<TextureLoader>> textures;   ///< Textures loaded by the application
    std::unique_ptr<VkSurfaceKHR, std::function<void(VkSurfaceKHR *)>> surface; ///< Surface for rendering
    std::unique_ptr<VkRenderPass, std::function<void(VkRenderPass *)>> render_pass;    ///< Render pass
    std::unique_ptr<VkCommandPool, std::function<void(VkCommandPool *)>> command_pool; ///< Command pool

    /// Descriptor pool for ImGui
    std::unique_ptr<VkDescriptorPool, std::function<void(VkDescriptorPool *)>> descriptor_pool;

    /// Swapchain for presenting images to the screen
    std::unique_ptr<VkSwapchainKHR, std::function<void(VkSwapchainKHR *)>> swap_chain;

    /// Swapchain images views
    std::vector<std::unique_ptr<VkImageView, std::function<void(VkImageView *)>>> swap_chain_image_views;

    /// Framebuffers for the swap chain images
    std::vector<std::unique_ptr<VkFramebuffer, std::function<void(VkFramebuffer *)>>> swap_chain_framebuffers;

    /// Semaphore for when an image is available
    std::vector<std::unique_ptr<VkSemaphore, std::function<void(VkSemaphore *)>>> image_available_semaphores;

    /// Semaphore for when rendering is finished
    std::vector<std::unique_ptr<VkSemaphore, std::function<void(VkSemaphore *)>>> render_finished_semaphores;

    /// Fences for in flight frames
    std::vector<std::unique_ptr<VkFence, std::function<void(VkFence *)>>> in_flight_fences;

    /// Debug messenger
    std::unique_ptr<VkDebugUtilsMessengerEXT, std::function<void(VkDebugUtilsMessengerEXT *)>> debug_messenger;

#ifdef NDEBUG
    const bool enable_validation_layers = false; ///< Disable validation layers
#else
    const bool enable_validation_layers = true; ///< Enable validation layers
#endif

    const std::vector<const char *> validation_layers = {"VK_LAYER_KHRONOS_validation"}; ///< Validation layers to use

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
     * @return VK_FALSE
     */
    static VKAPI_ATTR VkBool32 VKAPI_CALL debugcallback(VkDebugUtilsMessageSeverityFlagBitsEXT message_severity,
                                                        VkDebugUtilsMessageTypeFlagsEXT message_type,
                                                        const VkDebugUtilsMessengerCallbackDataEXT *p_callback_data,
                                                        void *p_user_data);

    /**
     * Creates create info for the debug messenger.
     *
     * @param create_info Reference to the create info to be filled.
     */
    void populateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT &create_info);

    /**
     * Sets up the debug messenger.
     */
    void setupDebugMessenger();

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
     */
    void rebuildSwapChain();

    /**
     * Writes commands for drawing to the command buffer.
     *
     * @param command_buffer The command buffer to write the commands to.
     * @param image_index The index of the image to draw to.
     *
     * @throws std::runtime_error If the command buffer fails to begin recording.
     */
    void recordRenderPass(const VkCommandBuffer &command_buffer, uint32_t image_index);

    /**
     * Creates a vulkan instance with GLFW extensions.
     *
     * @throws std::runtime_error if the instance could not be created
     */
    void createInstance();

    /**
     * Creates a vulkan surface for the window
     *
     * @throws std::runtime_error if the surface could not be created
     */
    void createSurface();

    /**
     * Creates a logical device and queues
     *
     * @throws std::runtime_error if failed to create logical device
     */
    void createLogicalDevice();

    /**
     * Chooses the best physical device for the previously initialized vulkan instance.
     *
     * @throws std::runtime_error if no devices are available or no suitable device was found.
     */
    void createPhysicalDevice();

    /**
     * Creates the swap chain.
     *
     * @throws std::runtime_error if the swap chain could not be created
     */
    void createSwapChain();

    /**
     * Creates the swap chain image views.
     *
     * @throws std::runtime_error if the image views could not be created
     */
    void createImageViews();

    /**
     * Creates the render pass.
     *
     * @throws std::runtime_error if the render pass could not be created
     */
    void createRenderPass();

    /**
     * Creates the frame buffers.
     *
     * @throws std::runtime_error if the frame buffers could not be created
     */
    void createFramebuffers();

    /**
     * Creates the command pool.
     *
     * @throws std::runtime_error if the command pool could not be created
     */
    void createCommandPool();

    /**
     * Creates a descriptor pool for the GUI
     *
     * @throws std::runtime_error if the descriptor pool could not be created
     */
    void createDescriptorPool();

    /**
     * Creates the command buffers.
     *
     * @throws std::runtime_error if the command buffers could not be created
     */
    void createCommandBuffers();

    /**
     * Creates the semaphores.
     *
     * @throws std::runtime_error if the semaphores could not be created
     */
    void createSyncObjects();

    /**
     * Callback for when the window is resized.
     *
     * @param window The window that was resized.
     * @param width The new width of the window.
     * @param height The new height of the window.
     */
    static void framebufferResizeCallback(GLFWwindow *window, int width, int height);

    /**
     * Initializes GLFW backend
     */
    void initGlfw();

    /**
     * Initializes Vulkan backend
     */
    void initVulkan();

    /**
     * Cleans up allocated resources and destroys Vulkan, GLFW and ImGui backends.
     */
    void cleanup();

    /**
     * Cleans up allocated swap chain along with its resources.
     */
    void cleanupSwapChain();

public:
    /**
     * Constructor
     *
     * @param application_name The name of the application window.
     * @param node The ROS2 node for the GUI.
     */
    GuiEngine(const std::string &application_name, std::shared_ptr<GuiNode> node);

    /**
     * Constructor.
     *
     * @param application_name The name of the application window.
     * @param node The ROS2 node for the GUI.
     * @param device_extensions Vector of required extensions for physical device.
     */
    GuiEngine(const std::string &application_name, std::shared_ptr<GuiNode> node,
              const std::vector<const char *> &device_extensions);

    /**
     * Destroy the GuiEngine object.
     * Cleanups all the resources and closes GUI.
     */
    ~GuiEngine();

    /**
     * Initialize GUI backend.
     * Initializes GLFW, Vulkan and ImGui.
     */
    void init();

    /**
     * Draws the frame.
     *
     * @throws std::runtime_error if the frame could not be drawn.
     */
    void draw();

    /**
     * Adds a new texture object to the GUI.
     *
     * @param name The name of the texture.
     * @param image_data The image data to be loaded.
     * @param width The width of the image.
     * @param height The height of the image.
     * @param channels The number of channels in the image.
     * @return True if the texture was successfully added, false otherwise.
     */
    bool addTexture(const std::string &name, unsigned char *image_data, int width, int height, int channels);

    /**
     * Get pointer to GLFW window.
     *
     * @return GLFWwindow*
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
     */
    VkCommandBuffer getCommandBuffer(int index) const { return command_buffers[index]; }

    /**
     * Gets the command pool.
     *
     * @return VkCommandPool The command pool.
     */
    VkCommandPool getCommandPool() const { return *command_pool.get(); }

    /**
     * Gets the render pass.
     *
     * @return VkRenderPass The render pass.
     */
    VkRenderPass getRenderPass() const { return *render_pass.get(); }

    /**
     * Gets the swap chain images.
     *
     * @return std::vector<VkImage> The swap chain images.
     */
    std::vector<VkImage> getSwapChainImages() const { return swap_chain_images; }

    /**
     * Gets the descriptor pool.
     *
     * @return VkDescriptorPool The descriptor pool.
     */
    VkDescriptorPool getDescriptorPool() const { return *descriptor_pool.get(); }

    /**
     * Gets the instance object.
     *
     * @return VkInstance The instance object.
     */
    VkInstance getInstance() const { return *instance.get(); }

    /**
     * Gets the surface object.
     *
     * @return VkSurfaceKHR The surface object.
     */
    VkSurfaceKHR getSurface() const { return *surface.get(); }

    /**
     * Gets the graphics queue.
     *
     * @return VkQueue The graphics queue.
     */
    VkQueue getGraphicsQueue() const { return graphics_queue; }

    /**
     * Gets the logical device.
     *
     * @return VkDevice The logical device.
     */
    VkDevice getDevice() const { return *device.get(); }

    /**
     * Gets the swap chain.
     *
     * @return VkSwapchainKHR The swap chain.
     */
    VkSwapchainKHR getSwapChain() const { return *swap_chain.get(); }

    /**
     * Gets the physical device object.
     *
     * @return VkPhysicalDevice The physical device object.
     */
    VkPhysicalDevice getPhysicalDevice() const { return *physical_device.get(); }

    /**
     * Gets the frame buffer at the given index.
     *
     * @param index The index of the frame buffer.
     * @return VkFramebuffer The frame buffer.
     */
    VkFramebuffer getFrameBuffer(int index) const { return *swap_chain_framebuffers[index].get(); }

    /**
     * Find the queue families of the physical device.
     *
     * @param physicalDevice The physical device to find the queue families of.
     * @return The queue families of the physical device.
     */
    QueueFamilyIndices findQueueFamilies(const VkPhysicalDevice &device);
};

} // namespace gui_node
#endif // GUI_NODE_GUI_ENGINE_HPP
