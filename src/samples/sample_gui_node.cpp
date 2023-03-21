#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include "gui_node/gui_node.hpp"
#include "gui_node/widget/widget_rosout.hpp"
#include "gui_node/widget/widget_string.hpp"
#include "gui_node/widget/widget_video.hpp"

namespace gui_node
{

using MsgImageSharedPtr = sensor_msgs::msg::Image::SharedPtr;
using MsgRosoutSharedPtr = rcl_interfaces::msg::Log::SharedPtr;
using RosImageSubscriberData = RosSubscriberData<sensor_msgs::msg::Image, sensor_msgs::msg::Image::SharedPtr>;
using RosRosoutSubscriberData = RosSubscriberData<rcl_interfaces::msg::Log, rcl_interfaces::msg::Log::SharedPtr>;
using RosStringSubscriberData = RosSubscriberData<std_msgs::msg::String, std::string>;

class SampleGuiComponent
{
private:
    std::shared_ptr<GuiNode> gui_node_ptr; ///< Pointer to the GUI node

public:
    SampleGuiComponent(const rclcpp::NodeOptions &options)
    {
        gui_node_ptr = std::make_shared<GuiNode>(options, "gui_node");

        // Creates a /rosout RosData subscriber
        std::shared_ptr<RosRosoutSubscriberData> subscriber_rosout = std::make_shared<RosRosoutSubscriberData>(
            gui_node_ptr, "/rosout", [](const MsgRosoutSharedPtr msg) -> MsgRosoutSharedPtr { return msg; });
        gui_node_ptr->addRosData("rosout_subscriber", subscriber_rosout);

        // Adds a /rosout subscriber widget to the Node
        std::shared_ptr<RosoutWidget> rosout_widget =
            std::make_shared<RosoutWidget>(gui_node_ptr, "[Sub] /rosout logs", "rosout_subscriber", 10);
        gui_node_ptr->addWidget("rosout_widget", rosout_widget);

        // Create a /video RosData subscriber
        std::shared_ptr<RosImageSubscriberData> subscriber_video = std::make_shared<RosImageSubscriberData>(
            gui_node_ptr, "video", [](const MsgImageSharedPtr msg) -> MsgImageSharedPtr { return msg; });
        gui_node_ptr->addRosData("video_subscriber", subscriber_video);

        // Create a widget to display the video
        std::shared_ptr<MsgVideoWidget> video_widget =
            std::make_shared<MsgVideoWidget>(gui_node_ptr, "[Sub] Video stream", "video_subscriber");
        gui_node_ptr->addWidget("video_widget", video_widget);

        // Create a /dateandtime RosData subscriber
        std::shared_ptr<RosStringSubscriberData> subscriber_dateandtime = std::make_shared<RosStringSubscriberData>(
            gui_node_ptr, "dateandtime",
            [](const std_msgs::msg::String::SharedPtr msg) -> std::string { return msg->data; });
        gui_node_ptr->addRosData("dateandtime_subscriber", subscriber_dateandtime);

        // Create a widget to display the date and time
        std::shared_ptr<StringSubscriberWidget> dateandtime_widget =
            std::make_shared<StringSubscriberWidget>(gui_node_ptr, "[Sub] Date and time", "dateandtime_subscriber");
        gui_node_ptr->addWidget("dateandtime_widget", dateandtime_widget);

        gui_node_ptr->prepare("Sample GUI widgets");
    }

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
    {
        return gui_node_ptr->get_node_base_interface();
    }
};

} // namespace gui_node

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gui_node::SampleGuiComponent)