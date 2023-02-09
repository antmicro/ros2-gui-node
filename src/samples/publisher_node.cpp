#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

#include "gui_node/gui_node.hpp"
#include "gui_node/ros_data/ros_publisher_data.hpp"

namespace gui_node
{
class TimePublisher : public GuiNode
{
private:
    rclcpp::TimerBase::SharedPtr timer_;

public:
    TimePublisher(const rclcpp::NodeOptions &options) : GuiNode(options, "gui_node")
    {
        auto publisherCallback = [](const std::string &date) -> std_msgs::msg::String
        {
            auto msg = std_msgs::msg::String();
            msg.data = date;
            return msg;
        };

        auto publisher = std::make_shared<RosPublisherData<std_msgs::msg::String, std::string>>(
            std::shared_ptr<GuiNode>(this), "time", publisherCallback);
        addRosData("time_publisher", publisher);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            [this]()
            {
                time_t now = time(0);
                char *date = ctime(&now);
                // Remove trailing newline
                date[strlen(date) - 1] = '\0';
                auto pub =
                    this->getRosData("time_publisher")->as<RosPublisherData<std_msgs::msg::String, std::string>>();
                pub.publish(std::string(date));
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing: %s", date);
            });
    }
};

} // namespace gui_node

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gui_node::TimePublisher)
