#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "gui_node/gui_node.hpp"
#include "gui_node/widget/widget_rosout.hpp"

namespace gui_node
{

using RosRosoutSubscriberData = RosSubscriberData<rcl_interfaces::msg::Log, rcl_interfaces::msg::Log::SharedPtr>;
using RosRosoutSubscriberDataSharedPtr = std::shared_ptr<RosRosoutSubscriberData>;
using MsgRosoutSharedPtr = rcl_interfaces::msg::Log::SharedPtr;

class RosoutSubscriber
{
private:
    std::shared_ptr<GuiNode> gui_node_ptr; ///< Pointer to the GUI node

public:
    RosoutSubscriber(const rclcpp::NodeOptions &options)
    {
        gui_node_ptr = std::make_shared<GuiNode>(options, "gui_node");
        std::string ros_data_name = "rosout_subscriber";
        std::string window_name = "[Sub] /rosout";
        std::string topic = "/rosout";

        // Adds a subscriber to the node
        RosRosoutSubscriberDataSharedPtr subscriber = std::make_shared<RosRosoutSubscriberData>(
            gui_node_ptr, topic, [](const MsgRosoutSharedPtr msg) -> MsgRosoutSharedPtr { return msg; });
        gui_node_ptr->addRosData(ros_data_name, subscriber);

        // Adds a subscriber widget to the GUI
        std::shared_ptr<RosoutWidget> subscriber_widget =
            std::make_shared<RosoutWidget>(gui_node_ptr, window_name, ros_data_name, 10);
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

RCLCPP_COMPONENTS_REGISTER_NODE(gui_node::RosoutSubscriber)
