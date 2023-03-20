#ifndef GUI_NODE_WIDGET_WIDGET_VIDEO_HPP
#define GUI_NODE_WIDGET_WIDGET_VIDEO_HPP

#pragma once

#include <memory>

#include "gui_node/widget/widget.hpp"

namespace gui_node
{

class WidgetVideo : public Widget
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
    WidgetVideo(std::shared_ptr<GuiNode> gui_node, const std::string window_name, const std::string ros_data_name);

    /**
     * Draws the widget.
     */
    void draw() override;
};

} // namespace gui_node
#endif // GUI_NODE_WIDGET_WIDGET_VIDEO_HPP
