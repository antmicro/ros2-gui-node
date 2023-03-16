#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "gui_node/gui_node.hpp"
#include "gui_node/ros_data/ros_subscriber_data.hpp"
#include "gui_node/widget/widget.hpp"

namespace gui_node
{

class VideoSubscriberWidget : public Widget
{
private:
    const std::string window_name;

public:
    VideoSubscriberWidget(std::shared_ptr<GuiNode> gui_node, const std::string window_name)
        : Widget(gui_node), window_name(window_name)
    {
    }

    void draw() override
    {
        std::shared_ptr<RosSubscriberData<sensor_msgs::msg::Image, sensor_msgs::msg::Image::SharedPtr>> subscriber =
            gui_node->getRosData("video_subscriber")
                ->as<RosSubscriberData<sensor_msgs::msg::Image, sensor_msgs::msg::Image::SharedPtr>>();

        sensor_msgs::msg::Image::SharedPtr msg = subscriber->getData();
        if (msg)
        {
            cv::Mat image(msg->height, msg->width, CV_8UC3, msg->data.data());
            cv::imshow(window_name, image);
            cv::waitKey(1);
        }
    }
};

class VideoSubscriber
{
private:
    std::shared_ptr<GuiNode> gui_node_ptr; ///< Pointer to the GUI node

public:
    VideoSubscriber(const rclcpp::NodeOptions &options)
    {
        gui_node_ptr = std::make_shared<GuiNode>(options, "gui_node");
        auto subscriber =
            std::make_shared<RosSubscriberData<sensor_msgs::msg::Image, sensor_msgs::msg::Image::SharedPtr>>(
                gui_node_ptr, "video", [](const sensor_msgs::msg::Image::SharedPtr msg) { return msg; });
        gui_node_ptr->addRosData("video_subscriber", subscriber);
        std::shared_ptr<VideoSubscriberWidget> widget =
            std::make_shared<VideoSubscriberWidget>(gui_node_ptr, "[Sub] Video subscriber");
        gui_node_ptr->addWidget("video_subscriber", widget);
        gui_node_ptr->prepareWidgets("[Sub] Video subscriber");
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
