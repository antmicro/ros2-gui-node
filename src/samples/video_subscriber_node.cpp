#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "gui_node/gui_node.hpp"
#include "gui_node/ros_data/ros_subscriber_data.hpp"
#include "gui_node/widget/widget_video.hpp"

namespace gui_node
{

using MsgImageSharedPtr = sensor_msgs::msg::Image::SharedPtr;
using RosImageSubscriberData = RosSubscriberData<sensor_msgs::msg::Image, sensor_msgs::msg::Image::SharedPtr>;

class VideoSubscriber
{
private:
    std::shared_ptr<GuiNode> gui_node_ptr; ///< Pointer to the GUI node

public:
    VideoSubscriber(const rclcpp::NodeOptions &options)
    {
        gui_node_ptr = std::make_shared<GuiNode>(options, "gui_node");
        std::string ros_data_name = "video_subscriber";
        std::string window_name = "[Sub] Video";
        std::string topic = "video";

        // Create a ROS subscriber data object
        std::shared_ptr<RosImageSubscriberData> subscriber = std::make_shared<RosImageSubscriberData>(
            gui_node_ptr, topic, [](const MsgImageSharedPtr msg) -> MsgImageSharedPtr { return msg; });
        gui_node_ptr->addRosData(ros_data_name, subscriber);

        // Create a widget to display the video
        std::shared_ptr<MsgVideoWidget> widget =
            std::make_shared<MsgVideoWidget>(gui_node_ptr, window_name, ros_data_name);
        gui_node_ptr->addWidget(ros_data_name, widget);
        gui_node_ptr->prepare(window_name);
    }

    ~VideoSubscriber() {}

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
    {
        return gui_node_ptr->get_node_base_interface();
    }
};

} // namespace gui_node

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gui_node::VideoSubscriber)
