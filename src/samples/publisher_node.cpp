#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

#include "gui_node/gui_node.hpp"
#include "gui_node/ros_data/ros_publisher_data.hpp"
#include "gui_node/widget/widget.hpp"

namespace gui_node
{

class PublisherWidget : public Widget
{
public:
    PublisherWidget(std::shared_ptr<GuiNode> gui_node) : Widget(gui_node) {}

    void draw() override
    {
        auto publisher =
            gui_node->getRosData("time_publisher")->as<RosPublisherData<std_msgs::msg::String, std::string>>();
        ImGui::Begin("[Pub] Date and time");
        ImGui::SetWindowSize(ImVec2(300, 100), ImGuiCond_FirstUseEver);
        ImGui::Text("%s", publisher->getData().c_str());
        ImGui::End();
    }
};

class TimePublisher
{
private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<GuiNode> gui_node_ptr;

public:
    TimePublisher(const rclcpp::NodeOptions &options)
    {
        gui_node_ptr = std::make_shared<GuiNode>(options, "gui_node");

        auto publisherCallback = [](const std::string &date) -> std_msgs::msg::String
        {
            auto msg = std_msgs::msg::String();
            msg.data = date;
            return msg;
        };

        // Adds time publisher to the node
        auto publisher = std::make_shared<RosPublisherData<std_msgs::msg::String, std::string>>(gui_node_ptr, "time",
                                                                                                publisherCallback);
        gui_node_ptr->addRosData("time_publisher", publisher);

        timer_ = this->gui_node_ptr->create_wall_timer(
            std::chrono::milliseconds(1000),
            [this]()
            {
                time_t now = time(0);
                char *date = ctime(&now);
                // Remove trailing newline
                date[strlen(date) - 1] = '\0';
                auto pub = this->gui_node_ptr->getRosData("time_publisher")
                               ->as<RosPublisherData<std_msgs::msg::String, std::string>>();
                pub->publish(std::string(date));
            });

        // Adds time publisher widget to the node
        std::shared_ptr<PublisherWidget> widget = std::make_shared<PublisherWidget>(gui_node_ptr);
        gui_node_ptr->addWidget("time_widget", widget);
        gui_node_ptr->prepare("Time publisher");
    }

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
    {
        return gui_node_ptr->get_node_base_interface();
    }
};

} // namespace gui_node

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gui_node::TimePublisher)
