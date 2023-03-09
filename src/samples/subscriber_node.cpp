#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

#include "gui_node/gui_node.hpp"
#include "gui_node/ros_data/ros_subscriber_data.hpp"
#include "gui_node/widget/widget.hpp"

namespace gui_node
{

class TimeSubscriberWidget : public Widget
{
private:
    const std::string window_name;

public:
    TimeSubscriberWidget(std::shared_ptr<GuiNode> gui_node, const std::string window_name)
        : Widget(gui_node), window_name(window_name)
    {
    }

    void draw() override
    {
        auto subscriber =
            this->gui_node->getRosData("time_subscriber")->as<RosSubscriberData<std_msgs::msg::String, std::string>>();
        ImGui::Begin(window_name.c_str());
        ImGui::SetWindowSize(ImVec2(300, 100), ImGuiCond_FirstUseEver);
        ImGui::Text("%s", subscriber->getData().c_str());
        ImGui::End();
    }
};

class TimeSubscriber
{
private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<GuiNode> gui_node_ptr;

public:
    TimeSubscriber(const rclcpp::NodeOptions &options)
    {
        gui_node_ptr = std::make_shared<GuiNode>(options, "gui_node");

        // Adds a subscriber to the node
        auto subscriber_callback = [](const std_msgs::msg::String::SharedPtr msg) -> std::string { return msg->data; };
        auto subscriber = std::make_shared<RosSubscriberData<std_msgs::msg::String, std::string>>(gui_node_ptr, "time",
                                                                                                  subscriber_callback);
        gui_node_ptr->addRosData("time_subscriber", subscriber);

        // Adds a subscriber widget to the GUI
        std::shared_ptr<TimeSubscriberWidget> subscriber_widget =
            std::make_shared<TimeSubscriberWidget>(gui_node_ptr, "[Sub] Date and time");
        gui_node_ptr->addWidget("time_widget", subscriber_widget);
        gui_node_ptr->prepareWidgets("Time subscriber");
    }

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
    {
        return gui_node_ptr->get_node_base_interface();
    }
};

} // namespace gui_node

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gui_node::TimeSubscriber)
