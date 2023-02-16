#ifndef GUI_VULKAN_IMGUI_ENGINE_HPP_
#define GUI_VULKAN_IMGUI_ENGINE_HPP_
#include <GLFW/glfw3.h>
#include <imgui_impl_vulkan.h>
#include <memory>
#include <optional>
#include <vulkan/vulkan.hpp>

namespace gui_node
{

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

class GuiEngine
{
    GLFWwindow *window;                                  ///< GLFW window
    VkDescriptorPool descriptor_pool;                    ///< Descriptor pool for ImGui
    VkDevice device;                                     ///< Vulkan logical device
    VkInstance instance;                                 ///< Vulkan instance
    VkPhysicalDevice physical_device = VK_NULL_HANDLE;   ///< Physical device (GPU) that Vulkan will be using
    VkQueue graphics_queue;                              ///< Graphics queue
    VkQueue present_queue;                               ///< Queue for presenting images to the screen
    VkSurfaceKHR surface;                                ///< The surface to draw on
    VkSwapchainKHR swap_chain;                           ///< Swap chain
    std::vector<VkImage> swap_chain_images;              ///< Swap chain images
    VkFormat swap_chain_image_format;                    ///< Swap chain image format
    VkExtent2D swap_chain_extent;                        ///< Swap chain extent
    std::vector<VkImageView> swap_chain_image_views;     ///< Swap chain image views
    VkRenderPass render_pass;                            ///< Render pass
    std::vector<VkFramebuffer> swap_chain_framebuffers;  ///< Framebuffers
    VkCommandPool command_pool;                          ///< Command pool
    std::vector<VkCommandBuffer> command_buffers;        ///< Command buffers
    std::vector<VkSemaphore> image_available_semaphores; ///< Semaphore for when an image is available
    std::vector<VkSemaphore> render_finished_semaphores; ///< Semaphore for when rendering is finished
    std::vector<VkFence> in_flight_fences;               ///< Fences for in flight frames
    uint32_t current_frame = 0;                          ///< Current frame
    bool framebuffer_resized = false;                    ///< Flag for when the framebuffer is resized
    std::pair<uint32_t, vk::Queue> queue_family;         ///< Queue family index and queue
    std::vector<const char *> device_extensions;         ///< Device extensions required for the application
    const int MAX_FRAMES_IN_FLIGHT = 2;                  ///< Maximum number of frames in flight

    /**
     * Find the queue families of the physical device.
     *
     * @param physicalDevice The physical device to find the queue families of.
     * @return The queue families of the physical device.
     */
    QueueFamilyIndices findQueueFamilies(VkPhysicalDevice device);

    /**
     * Query the swap chain support details for the given physical device.
     *
     * @param device The physical device to query.
     * @return The swap chain support details for the given physical device.
     */
    SwapChainSupportDetails querySwapChainSupport(VkPhysicalDevice device);

    /**
     * Check if the device supports the required extensions.
     *
     * @param device The physical device to check.
     * @return True if the device supports the required extensions, false otherwise.
     */
    bool checkDeviceExtensionSupport(VkPhysicalDevice device);

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
    bool isDeviceSuitable(VkPhysicalDevice device);

    /**
     * Creates and uploads font texture for ImGui.
     */
    void buildImGuiFonts();

    /**
     * Initializes Vulkan backend for ImGui.
     */
    void initImGuiVulkanImpl();

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
    void recordRenderPass(VkCommandBuffer command_buffer, uint32_t image_index);

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
     * Initializes ImGui backend
     */
    void initImGui();

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
     */
    GuiEngine();

    /**
     * Constructor.
     *
     * @param Vector of required extensions for physical device.
     */
    GuiEngine(std::vector<const char *> extensions);

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
     * Get pointer to GLFW window.
     *
     * @return GLFWwindow*
     */
    GLFWwindow *getWindow();
};

} // namespace gui_node
#endif
