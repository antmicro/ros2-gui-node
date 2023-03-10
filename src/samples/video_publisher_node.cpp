#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "gui_node/gui_node.hpp"
#include "gui_node/ros_data/ros_publisher_data.hpp"
#include "gui_node/widget/widget.hpp"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>
#include <utility>

namespace gui_node
{

class VideoPublisherWidget : public Widget
{
private:
    const std::string window_name;

public:
    VideoPublisherWidget(std::shared_ptr<GuiNode> gui_node, const std::string window_name)
        : Widget(gui_node), window_name(window_name)
    {
    }

    void draw() override
    {
        auto pulisher =
            gui_node->getRosData("video_publisher")->as<RosPublisherData<sensor_msgs::msg::Image, cv::Mat>>();
        cv::Mat image = pulisher->getData();
        if (!image.empty())
        {
            cv::imshow(window_name, image);
            cv::waitKey(1);
        }
    }
};

class VideoPublisher
{
private:
    std::shared_ptr<GuiNode> gui_node_ptr; ///< Pointer to the GUI node
    cv::VideoCapture video_capture;        ///< Video capture object
    cv::Mat frame;                         ///< Current frame
    std::thread video_thread;              ///< Thread to read the video
    std::atomic<bool> stop_video_thread;   ///< Flag to stop the video thread

    std::string mat_type2encoding(int mat_type)
    {
        switch (mat_type)
        {
        case CV_8UC1:
            return "mono8";
        case CV_8UC3:
            return "bgr8";
        case CV_16SC1:
            return "mono16";
        case CV_8UC4:
            return "rgba8";
        default:
            throw std::runtime_error("Unsupported encoding type");
        }
    }

public:
    VideoPublisher(const rclcpp::NodeOptions &options) : stop_video_thread(false)
    {
        gui_node_ptr = std::make_shared<GuiNode>(options, "gui_node");
        video_capture.open(0);
        if (!video_capture.isOpened())
        {
            RCLCPP_FATAL(gui_node_ptr->get_logger(), "Could not open video capture device");
            return;
        }

        auto publisherCallback = [this](const cv::Mat &frame) -> sensor_msgs::msg::Image
        {
            if (frame.empty())
            {
                RCLCPP_ERROR(gui_node_ptr->get_logger(), "Frame is empty");
                return sensor_msgs::msg::Image();
            }

            sensor_msgs::msg::Image msg = sensor_msgs::msg::Image();
            msg.header.stamp = rclcpp::Clock().now();
            msg.header.frame_id = "camera";
            msg.height = frame.rows;
            msg.width = frame.cols;
            msg.encoding = mat_type2encoding(frame.type());
            msg.is_bigendian = false;
            msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
            msg.data.assign(frame.datastart, frame.dataend);
            return msg;
        };
        auto publisher = std::make_shared<RosPublisherData<sensor_msgs::msg::Image, cv::Mat>>(gui_node_ptr, "video",
                                                                                              publisherCallback);
        gui_node_ptr->addRosData("video_publisher", publisher);
        std::shared_ptr<VideoPublisherWidget> widget =
            std::make_shared<VideoPublisherWidget>(gui_node_ptr, "[Pub] Video publisher");
        gui_node_ptr->addWidget("video_publisher", widget);
        gui_node_ptr->prepareWidgets("[Pub] Video publisher");
        video_thread = std::thread(std::bind(&VideoPublisher::run, this));
    }

    ~VideoPublisher()
    {
        stop_video_thread.store(true);
        video_thread.join();
    }

    void run()
    {
        while (rclcpp::ok() && !stop_video_thread.load())
        {
            video_capture >> frame;
            auto pub = this->gui_node_ptr->getRosData("video_publisher")
                           ->as<RosPublisherData<sensor_msgs::msg::Image, cv::Mat>>();
            pub->publish(frame);
        }
    }

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
    {
        return gui_node_ptr->get_node_base_interface();
    }
};

} // namespace gui_node

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gui_node::VideoPublisher)
