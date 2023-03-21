#ifndef GUI_NODE_WIDGET_WIDGET_VIDEO_HPP
#define GUI_NODE_WIDGET_WIDGET_VIDEO_HPP

#pragma once

#include <memory>

#include "gui_node/widget/widget.hpp"

namespace gui_node
{

/**
 * Configuration parameters for the ImGui window.
 */
struct WindowConfigs
{
    float aspect_ratio; ///< The aspect ratio of the window
    ImVec2 offset;      ///< The offset of the window
    ImVec2 window_size; ///< The size of the window
};

/**
 * Base class for video widgets.
 */
class WidgetVideoBase : public Widget
{
protected:
    /**
     * Resize callback for the ImGui window.
     * Sets the size of the video widget with respect to the window aspect ratio.
     *
     * @param configs Pointer to the ImGui callback data.
     */
    static void resizeCallback(ImGuiSizeCallbackData *data);

    /**
     * Calculates the aspect ratio for the ImGui window based on the provided parameters.
     * Calculates the offset for the ImGui window based on the ImGui style.
     *
     * @param width Width of the video widget.
     * @param height Height of the video widget.
     * @return WindowConfigs struct containing the calculated aspect ratio and offset.
     */
    WindowConfigs getWindowConfigs(int width, int height);

    /**
     * Draws the ImGui window with Image from the provided texture.
     *
     * @param texture_loader Pointer to the texture loader.
     */
    void drawImGuiFrame(std::shared_ptr<TextureLoader> texture_loader);

    const std::string window_name;    ///< The name of the ImGui window
    const std::string ros_data_name;  ///< The name of the ROS data, used as a name for the TextureLoader
    bool texture_initialized = false; ///< Whether the texture has been initialized

public:
    /**
     * Constructor.
     *
     * @param gui_node The shared pointer to the GUI node for logging.
     * @param window_name The name for the ImGui window.
     * @param ros_data_name The name of the ROS data associated with this widget. Used as a name for the TextureLoader.
     */
    WidgetVideoBase(std::shared_ptr<GuiNode> gui_node, const std::string &window_name, const std::string &ros_data_name)
        : Widget(gui_node), window_name(window_name), ros_data_name(ros_data_name)
    {
    }
};

/**
 * Widget for displaying a video stream from a sensor_msgs::Image message.
 */
class WidgetVideoMsg : public WidgetVideoBase
{
private:
    /**
     * Converts an image encoding string to amount of channels
     *
     * @param encoding The image encoding string
     * @return The amount of channels
     *
     * @throws std::invalid_argument if the encoding is not supported
     */
    int encoding2channels(const std::string &encoding);

public:
    /**
     * Constructor.
     *
     * @param gui_node The shared pointer to the GUI node for logging.
     * @param window_name The name for the ImGui window.
     * @param ros_data_name The name of the ROS data associated with this widget. Used as a name for the TextureLoader.
     */
    WidgetVideoMsg(std::shared_ptr<GuiNode> gui_node, const std::string window_name, const std::string ros_data_name)
        : WidgetVideoBase(gui_node, window_name, ros_data_name)
    {
    }

    /**
     * Draws the widget.
     */
    void draw() override;
};

/**
 * Widget for displaying a video stream from a cv::Mat.
 */
class WidgetVideoCVMat : public WidgetVideoBase
{
public:
    /**
     * Constructor.
     *
     * @param gui_node The shared pointer to the GUI node for logging.
     * @param window_name The name for the ImGui window.
     * @param ros_data_name The name of the ROS data associated with this widget. Used as a name for the TextureLoader.
     */
    WidgetVideoCVMat(std::shared_ptr<GuiNode> gui_node, const std::string window_name, const std::string ros_data_name)
        : WidgetVideoBase(gui_node, window_name, ros_data_name)
    {
    }

    /**
     * Draws the widget.
     */
    void draw() override;
};

} // namespace gui_node
#endif // GUI_NODE_WIDGET_WIDGET_VIDEO_HPP
