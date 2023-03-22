#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "gui_node/gui_engine.hpp"
#include "gui_node/gui_node.hpp"
#include "gui_node/ros_data/ros_publisher_data.hpp"
#include "gui_node/widget/widget_video.hpp"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>
#include <utility>

namespace gui_node
{

using RosImagePublisherData = RosPublisherData<sensor_msgs::msg::Image, cv::Mat>;
using RosImagePublisherDataSharedPtr = std::shared_ptr<RosImagePublisherData>;

class VideoPublisher
{
private:
    /**
     * Convert a cv::Mat type to a string with the corresponding encoding.
     *
     * @param mat_type The cv::Mat type.
     * @return The encoding string.
     *
     * @throw std::runtime_error if the encoding is not supported.
     */
    std::string mat_type2encoding(int mat_type)
    {
        switch (mat_type)
        {
        case CV_8UC4:
            return "rgba8";
        default:
            throw std::runtime_error("Unsupported encoding type");
        }
    }

    std::shared_ptr<GuiNode> gui_node_ptr; ///< Pointer to the GUI node
    cv::VideoCapture video_capture;        ///< Video capture object
    std::thread video_thread;              ///< Thread to read the video
    std::atomic<bool> stop_video_thread;   ///< Flag to stop the video thread
    std::string ros_data_name;             ///< Name of the ROS data
public:
    VideoPublisher(const rclcpp::NodeOptions &options) : stop_video_thread(false)
    {
        // Initialize the GUI node
        gui_node_ptr = std::make_shared<GuiNode>(options, "gui_node");
        ros_data_name = "video_publisher";
        std::string topic = "video";
        std::string window_name = "[Pub] Video";

        // Create a video capture object
        video_capture.open(0);
        if (!video_capture.isOpened())
        {
            RCLCPP_FATAL(gui_node_ptr->get_logger(), "Could not open video capture device");
            throw std::runtime_error("Could not open video capture device");
        }

        // Create a ROS publisher
        RosImagePublisherDataSharedPtr publisher = std::make_shared<RosImagePublisherData>(
            gui_node_ptr, topic,
            [this](const cv::Mat &frame) -> sensor_msgs::msg::Image
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
            });

        // Add the ROS publisher to the GUI node
        gui_node_ptr->addRosData(ros_data_name, publisher);

        // Create a video publisher widget
        std::shared_ptr<CVMatVideoWidget> widget =
            std::make_shared<CVMatVideoWidget>(gui_node_ptr, window_name, ros_data_name);

        // Add the widget to the GUI node
        gui_node_ptr->addWidget(ros_data_name, widget);
        gui_node_ptr->prepare(window_name);

        // Start the video thread
        video_thread = std::thread(std::bind(&VideoPublisher::run, this));
    }

    ~VideoPublisher()
    {
        stop_video_thread.store(true);
        video_thread.join();
    }

    /**
     * Continuously read the video and publish it
     */
    void run()
    {
        cv::Mat frame;
        while (rclcpp::ok() && !stop_video_thread.load())
        {
            video_capture >> frame;
            // Convert to RGBA
            cv::cvtColor(frame, frame, cv::COLOR_BGR2RGBA);
            RosImagePublisherDataSharedPtr pub =
                this->gui_node_ptr->getRosData(ros_data_name)->as<RosImagePublisherData>();
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
