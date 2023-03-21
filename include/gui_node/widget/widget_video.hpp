#ifndef GUI_NODE_WIDGET_WIDGET_VIDEO_HPP
#define GUI_NODE_WIDGET_WIDGET_VIDEO_HPP

#pragma once

#include <memory>

#include "gui_node/widget/widget.hpp"

namespace gui_node
{

/**
 * Base class for video widgets.
 */
class WidgetVideoBase : public Widget
{
protected:
    const std::string window_name;       ///< The name of the ImGui window
    const std::string ros_data_name;     ///< The name of the ROS data, used as a name for the TextureLoader
    bool is_texture_initialized = false; ///< Whether the texture has been initialized

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
