#pragma once

#include <imgui.h>
#include <map>
#include <memory>
#include <rcl_interfaces/msg/log.hpp>
#include <string>

#include "gui_node/gui_node.hpp"
#include "gui_node/ros_data/ros_subscriber_data.hpp"
#include "gui_node/utils/utils.hpp"
#include "gui_node/widget/widget.hpp"

namespace gui_node
{

/**
 * Widget for displaying ROS log messages.
 */
class RosoutWidget : public Widget
{
private:
    /// Map of ROS log levels to strings
    const std::unordered_map<uint8_t, std::string> rosout_level_map = {
        {10, "DEBUG"}, {20, "INFO"}, {30, "WARN"}, {40, "ERROR"}, {50, "FATAL"}};

    /// ImGui table style flags
    ImGuiTableFlags flags =
        ImGuiTableFlags_Reorderable | ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable | ImGuiTableFlags_RowBg;

    FixedDeque<rcl_interfaces::msg::Log::SharedPtr> messages; ///< Circular buffer of rosout messages

public:
    /**
     * Constructor.
     *
     * @param gui_node Pointer to the GUI node the widget is associated with.
     * @param window_name Name of the ImGui window.
     * @param ros_data_name Name of the RosData object that is used to subscribe to the ROS topic.
     * @param max_table_size Maximum number of messages to store in the table. By default, set to 1000.
     */
    RosoutWidget(std::shared_ptr<GuiNode> gui_node, const std::string &window_name, const std::string &ros_data_name,
                 int max_table_size = 1000);

    ~RosoutWidget() = default;

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
