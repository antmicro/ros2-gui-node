#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "gui_node/gui_node.hpp"
#include "gui_node/widget/widget_counter.hpp"
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
using RosCounterClientData = RosServiceClientData<std_srvs::srv::Trigger, std_srvs::srv::Trigger::Response>;

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
        std::shared_ptr<VideoWidget> video_widget = std::make_shared<VideoWidget>(
            gui_node_ptr, "[Sub] Video stream", "video_subscriber",
            [](std::shared_ptr<GuiNode> gui_node_ptr, sensor_msgs::msg::Image &msg) -> void
            {
                std::shared_ptr<RosImageSubscriberData> subscriber_video =
                    gui_node_ptr->getRosData("video_subscriber")->as<RosImageSubscriberData>();
                msg = *subscriber_video->getData().get();
            });
        gui_node_ptr->addWidget("video_widget", video_widget);

        // Create a /dateandtime RosData subscriber
        std::shared_ptr<RosStringSubscriberData> subscriber_dateandtime = std::make_shared<RosStringSubscriberData>(
            gui_node_ptr, "dateandtime",
            [](const std_msgs::msg::String::SharedPtr msg) -> std::string { return msg->data; });
        gui_node_ptr->addRosData("dateandtime_subscriber", subscriber_dateandtime);

        // Create a widget to display the date and time
        std::shared_ptr<StringWidget> dateandtime_widget = std::make_shared<StringWidget>(
            gui_node_ptr, "[Sub] Date and time", "dateandtime_subscriber",
            [](std::shared_ptr<GuiNode> gui_node_ptr, std::string &data) -> void
            {
                std::shared_ptr<RosStringSubscriberData> subscriber_dateandtime =
                    gui_node_ptr->getRosData("dateandtime_subscriber")->as<RosStringSubscriberData>();
                data = subscriber_dateandtime->getData();
            });
        gui_node_ptr->addWidget("dateandtime_widget", dateandtime_widget);

        // Create a /counter RosData service client
        std::shared_ptr<RosCounterClientData> client_counter = std::make_shared<RosCounterClientData>(
            gui_node_ptr, "/counter",
            [](std_srvs::srv::Trigger::Response::SharedPtr response) -> std_srvs::srv::Trigger::Response::SharedPtr
            { return response; });
        gui_node_ptr->addRosData("counter_service", client_counter);

        // Create a counter widget
        std::shared_ptr<CounterWidget> counter_widget =
            std::make_shared<CounterWidget>(gui_node_ptr, "[Client] Counter", "counter_service");
        gui_node_ptr->addWidget("counter_widget", counter_widget);

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
