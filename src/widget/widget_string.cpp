#include <std_msgs/msg/string.hpp>

#include "gui_node/ros_data/ros_publisher_data.hpp"
#include "gui_node/ros_data/ros_subscriber_data.hpp"
#include "gui_node/widget/widget_string.hpp"

namespace gui_node
{
using RosStringSubscriberData = RosSubscriberData<std_msgs::msg::String, std::string>;
using RosStringPublisherData = RosPublisherData<std_msgs::msg::String, std::string>;

void BaseStringWidget::draw()
{
    if (gui_node->getRosData(ros_data_name)->hasDataChanged())
    {
        data = getData();
    }
    ImGui::Begin(window_name.c_str());
    ImGui::SetWindowSize(ImVec2(300, 100), ImGuiCond_FirstUseEver);
    ImGui::Text("%s", data.c_str());
    ImGui::End();
}

std::string StringSubscriberWidget::getData()
{
    return gui_node->getRosData(ros_data_name)->as<RosStringSubscriberData>()->getData();
}

std::string StringPublisherWidget::getData()
{
    return gui_node->getRosData(ros_data_name)->as<RosStringPublisherData>()->getData();
}
} // namespace gui_node
