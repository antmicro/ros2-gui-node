#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "gui_node/gui_node.hpp"
#include "gui_node/widget/widget_rosout.hpp"

namespace gui_node
{

class RosoutSubscriberNode
{
private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<GuiNode> gui_node_ptr;

public:
    RosoutSubscriberNode(const rclcpp::NodeOptions &options)
    {
        using SubscriberType = RosSubscriberData<rcl_interfaces::msg::Log, rcl_interfaces::msg::Log::SharedPtr>;
        gui_node_ptr = std::make_shared<GuiNode>(options, "gui_node");
        std::string ros_data_name = "rosout_subscriber";

        // Adds a subscriber to the node
        auto subscriber_callback =
            [](const rcl_interfaces::msg::Log::SharedPtr msg) -> rcl_interfaces::msg::Log::SharedPtr { return msg; };
        std::shared_ptr<SubscriberType> subscriber =
            std::make_shared<SubscriberType>(gui_node_ptr, "/rosout", subscriber_callback);
        gui_node_ptr->addRosData(ros_data_name, subscriber);

        // Adds a subscriber widget to the GUI
        std::shared_ptr<WidgetRosout> subscriber_widget =
            std::make_shared<WidgetRosout>(gui_node_ptr, ros_data_name, "[Sub] /rosout log", 10);
        gui_node_ptr->addWidget("rosout_widget", subscriber_widget);
        gui_node_ptr->prepare("[Sub] /rosout");
    }

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
    {
        return gui_node_ptr->get_node_base_interface();
    }
};

} // namespace gui_node

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gui_node::RosoutSubscriberNode)
