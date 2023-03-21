#ifndef GUI_NODE_WIDGET_WIDGET_ROSOUT_HPP
#define GUI_NODE_WIDGET_WIDGET_ROSOUT_HPP

#pragma once

#include <boost/circular_buffer.hpp>
#include <imgui.h>
#include <map>
#include <memory>
#include <rcl_interfaces/msg/log.hpp>
#include <string>

#include "gui_node/gui_node.hpp"
#include "gui_node/ros_data/ros_subscriber_data.hpp"
#include "gui_node/widget/widget.hpp"

namespace gui_node
{

/**
 * Widget for displaying ROS log messages.
 */
class WidgetRosout : public Widget
{
private:
    /**
     * Struct to store the data of a single log message.
     */
    struct rosout_message
    {
        uint8_t level;        ///< The log level of the message.
        std::string name;     ///< The name of the node that published the log message.
        std::string msg;      ///< The log message itself.
        std::string file;     ///< The file in which the log message was published.
        std::string function; ///< The function in which the log message was published.
        uint32_t line;        ///< The line in which the log message was published.
    };

    /// Map of ROS log levels to strings
    std::unordered_map<uint8_t, std::string> rosout_level_map = {
        {10, "DEBUG"}, {20, "INFO"}, {30, "WARN"}, {40, "ERROR"}, {50, "FATAL"}};

    /// ImGui table style flags
    ImGuiTableFlags flags =
        ImGuiTableFlags_Reorderable | ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable | ImGuiTableFlags_RowBg;

    const std::string ros_data_name;                 ///< The name of the RosData object.
    const std::string window_name;                   ///< Name of the ImGui window
    boost::circular_buffer<rosout_message> messages; ///< Circular buffer of rosout messages

    /**
     * Checks if a message is the same as the last message in the buffer.
     *
     * @param msg The message to check.
     * @return False if the message is a duplicate, true otherwise.
     */
    bool isUniqueMessage(rcl_interfaces::msg::Log::SharedPtr msg);

public:
    /**
     * Constructor.
     *
     * @param gui_node Pointer to the GUI node the widget is associated with.
     * @param window_name Name of the ImGui window.
     * @param ros_data_name Name of the RosData object that is used to subscribe to the ROS topic.
     * @param max_table_size Maximum number of messages to store in the table.
     */
    WidgetRosout(std::shared_ptr<GuiNode> gui_node, const std::string &window_name, const std::string &ros_data_name,
                 int max_table_size);

    /**
     * Constructor.
     * Default table size is 1000 messages.
     *
     * @param gui_node Pointer to the GUI node the widget is associated with.
     * @param ros_data_name Name of the RosData object that is used to subscribe to the ROS topic.
     * @param window_name Name of the ImGui window.
     */
    WidgetRosout(std::shared_ptr<GuiNode> gui_node, const std::string &ros_data_name, const std::string &window_name);

    ~WidgetRosout() = default;

    /**
     * Draws the widget using ImGui backend.
     */
    void draw() override;

    /**
     * Sets flags for the ImGui table.
     *
     * @param flags The flags to set.
     */
    void setTableFlags(ImGuiTableFlags flags) { this->flags = flags; }
};

} // namespace gui_node
#endif // GUI_NODE_WIDGET_WIDGET_ROSOUT_HPP
