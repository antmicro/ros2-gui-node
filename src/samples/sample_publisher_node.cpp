#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <thread>

#include "gui_node/gui_node.hpp"
#include "gui_node/ros_data/ros_publisher_data.hpp"
#include "gui_node/ros_data/ros_server_data.hpp"

namespace gui_node
{

using RosStringPublisherData = RosPublisherData<std_msgs::msg::String, std::string>;
using RosImagePublisherData = RosPublisherData<sensor_msgs::msg::Image, cv::Mat>;
using RosCounterServerData = RosServiceServerData<std_srvs::srv::Trigger, std_srvs::srv::Trigger::Response::SharedPtr>;

class SamplePublisherComponent
{
private:
    std::shared_ptr<GuiNode> gui_node_ptr;       ///< Pointer to the GUI node
    rclcpp::TimerBase::SharedPtr timer;          ///< Timer to publish the string
    std::thread video_capture_thread;            ///< Thread to capture and publish video frames
    cv::VideoCapture video_capture;              ///< Video capture object
    std::atomic<bool> stop_video_capture_thread; ///< Flag to stop the video capture thread

public:
    SamplePublisherComponent(const rclcpp::NodeOptions &options) : stop_video_capture_thread(false)
    {
        gui_node_ptr = std::make_shared<GuiNode>(options, "gui_node");

        // Create a string publisher
        std::shared_ptr<RosStringPublisherData> string_publisher_data =
            std::make_shared<RosStringPublisherData>(gui_node_ptr, "dateandtime",
                                                     [](const std::string &date) -> std_msgs::msg::String
                                                     {
                                                         std_msgs::msg::String msg = std_msgs::msg::String();
                                                         msg.data = date;
                                                         return msg;
                                                     });
        gui_node_ptr->addRosData("string_publisher", string_publisher_data);

        // Set the timer to publish the date and time
        timer = gui_node_ptr->create_wall_timer(
            std::chrono::milliseconds(1000),
            [this]
            {
                time_t now = time(0);
                std::string date = ctime(&now);
                // Remove the newline character
                date.pop_back();
                std::shared_ptr<RosStringPublisherData> string_publisher_data =
                    gui_node_ptr->getRosData("string_publisher")->as<RosStringPublisherData>();
                RCLCPP_INFO(gui_node_ptr->get_logger(), "Publishing: %s", date.c_str());
                string_publisher_data->publish(date);
            });

        // Create a video capture object
        video_capture.open(0);
        if (!video_capture.isOpened())
        {
            RCLCPP_ERROR(gui_node_ptr->get_logger(), "Could not open video capture device");
            return;
        }

        // Create a video frames publisher
        std::shared_ptr<RosImagePublisherData> video_publisher_data = std::make_shared<RosImagePublisherData>(
            gui_node_ptr, "video",
            [this](const cv::Mat &frame) -> sensor_msgs::msg::Image
            {
                sensor_msgs::msg::Image msg = sensor_msgs::msg::Image();
                if (frame.empty())
                {
                    RCLCPP_ERROR(gui_node_ptr->get_logger(), "Frame is empty");
                    return msg;
                }
                msg.header.stamp = rclcpp::Clock().now();
                msg.header.frame_id = "video";
                msg.encoding = "bgr8";
                msg.height = frame.rows;
                msg.width = frame.cols;
                msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
                msg.data.assign(frame.datastart, frame.dataend);
                return msg;
            });

        gui_node_ptr->addRosData("video_publisher", video_publisher_data);

        // Start the thread to capture and publish video frames
        video_capture_thread = std::thread(
            [this]
            {
                cv::Mat frame;
                while (rclcpp::ok() && !stop_video_capture_thread.load())
                {
                    video_capture >> frame;
                    std::shared_ptr<RosImagePublisherData> video_publisher_data =
                        gui_node_ptr->getRosData("video_publisher")->as<RosImagePublisherData>();
                    video_publisher_data->publish(frame);
                }
            });

        // Create the /counter RosData service server
        std::shared_ptr<RosCounterServerData> ros_server_data_ptr = std::make_shared<RosCounterServerData>(
            gui_node_ptr, "/counter",
            [](std_srvs::srv::Trigger::Request::SharedPtr request,
               std_srvs::srv::Trigger::Response::SharedPtr response) -> std_srvs::srv::Trigger::Response::SharedPtr
            {
                response->success = true;
                response->message = "triggered";
                return response;
            });
        gui_node_ptr->addRosData("counter_server", ros_server_data_ptr);
    }

    ~SamplePublisherComponent()
    {
        stop_video_capture_thread.store(true);
        video_capture_thread.join();
    }

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
    {
        return gui_node_ptr->get_node_base_interface();
    }
};

} // namespace gui_node

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gui_node::SamplePublisherComponent)
