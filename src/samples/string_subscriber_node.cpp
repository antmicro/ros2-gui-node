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

using RosStringSubscriberData = RosSubscriberData<std_msgs::msg::String, std::string>;
using RosStringSubscriberSharedPtr = std::shared_ptr<RosStringSubscriberData>;

/**
 * Widget for displaying the current date and time.
 */
class StringSubscriberWidget : public Widget
{
public:
    StringSubscriberWidget(std::shared_ptr<GuiNode> gui_node, const std::string &window_name,
                           const std::string &ros_data_name)
        : Widget(gui_node, window_name, ros_data_name)
    {
    }

    /**
     * Draw the widget
     */
    void draw() override
    {
        RosStringSubscriberSharedPtr subscriber =
            this->gui_node->getRosData(ros_data_name)->as<RosStringSubscriberData>();
        ImGui::Begin(window_name.c_str());
        ImGui::SetWindowSize(ImVec2(300, 100), ImGuiCond_FirstUseEver);
        ImGui::Text("%s", subscriber->getData().c_str());
        ImGui::End();
    }
};

class StringSubscriber
{
private:
    std::shared_ptr<GuiNode> gui_node_ptr; ///< Pointer to the GUI node

public:
    StringSubscriber(const rclcpp::NodeOptions &options)
    {
        gui_node_ptr = std::make_shared<GuiNode>(options, "gui_node");
        std::string ros_data_name = "time_subscriber";
        std::string window_name = "[Sub] Date and Time";
        std::string topic = "time";

        // Adds a subscriber to the node
        RosStringSubscriberSharedPtr subscriber = std::make_shared<RosStringSubscriberData>(
            gui_node_ptr, topic, [](const std_msgs::msg::String::SharedPtr msg) -> std::string { return msg->data; });
        gui_node_ptr->addRosData(ros_data_name, subscriber);

        // Adds a subscriber widget to the GUI
        std::shared_ptr<StringSubscriberWidget> subscriber_widget =
            std::make_shared<StringSubscriberWidget>(gui_node_ptr, window_name, ros_data_name);
        gui_node_ptr->addWidget(ros_data_name, subscriber_widget);
        gui_node_ptr->prepare(window_name);
    }

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
    {
        return gui_node_ptr->get_node_base_interface();
    }
};

} // namespace gui_node

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gui_node::StringSubscriber)
