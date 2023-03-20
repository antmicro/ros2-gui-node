#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "gui_node/gui_node.hpp"
#include "gui_node/ros_data/ros_subscriber_data.hpp"
#include "gui_node/widget/widget_video.hpp"

namespace gui_node
{

class VideoSubscriber
{
private:
    std::shared_ptr<GuiNode> gui_node_ptr; ///< Pointer to the GUI node

public:
    VideoSubscriber(const rclcpp::NodeOptions &options)
    {
        gui_node_ptr = std::make_shared<GuiNode>(options, "gui_node");

        // Create a ROS subscriber data object
        auto subscriber =
            std::make_shared<RosSubscriberData<sensor_msgs::msg::Image, sensor_msgs::msg::Image::SharedPtr>>(
                gui_node_ptr, "video", [](const sensor_msgs::msg::Image::SharedPtr msg) { return msg; });
        std::string ros_data_name = "video_subscriber";
        gui_node_ptr->addRosData(ros_data_name, subscriber);

        // Create a widget to display the video
        std::shared_ptr<WidgetVideo> widget =
            std::make_shared<WidgetVideo>(gui_node_ptr, "[Sub] Video subscriber", ros_data_name);
        gui_node_ptr->addWidget(ros_data_name, widget);
        gui_node_ptr->prepare("[Sub] Video subscriber");
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
