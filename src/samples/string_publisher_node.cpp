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

using RosStringPublisherData = RosPublisherData<std_msgs::msg::String, std::string>;
using RosStringPublisherDataSharedPtr = std::shared_ptr<RosStringPublisherData>;

/**
 * Widget for displaying published date and time.
 */
class StringPublisherWidget : public Widget
{
public:
    StringPublisherWidget(std::shared_ptr<GuiNode> gui_node, const std::string &window_name,
                          const std::string &ros_data_name)
        : Widget(gui_node, window_name, ros_data_name)
    {
    }

    /**
     * Draw the widget.
     */
    void draw() override
    {
        RosStringPublisherDataSharedPtr publisher = gui_node->getRosData(ros_data_name)->as<RosStringPublisherData>();
        ImGui::Begin(window_name.c_str());
        ImGui::SetWindowSize(ImVec2(300, 100), ImGuiCond_FirstUseEver);
        ImGui::Text("%s", publisher->getData().c_str());
        ImGui::End();
    }
};

class StringPublisher
{
private:
    rclcpp::TimerBase::SharedPtr timer_;   ///< Timer for triggering the publishing of the date and time
    std::shared_ptr<GuiNode> gui_node_ptr; ///< Pointer to the GUI node

public:
    StringPublisher(const rclcpp::NodeOptions &options)
    {
        gui_node_ptr = std::make_shared<GuiNode>(options, "gui_node");
        std::string ros_data_name = "time_publisher";
        std::string window_name = "[Pub] Date and time";
        std::string topic = "time";

        // Create date and time publisher
        RosStringPublisherDataSharedPtr publisher =
            std::make_shared<RosStringPublisherData>(gui_node_ptr, topic,
                                                     [](const std::string &date) -> std_msgs::msg::String
                                                     {
                                                         std_msgs::msg::String msg = std_msgs::msg::String();
                                                         msg.data = date;
                                                         return msg;
                                                     });
        gui_node_ptr->addRosData(ros_data_name, publisher);

        // Set up timer for publishing the date and time
        timer_ = this->gui_node_ptr->create_wall_timer(
            std::chrono::milliseconds(1000),
            [this, ros_data_name]() -> void
            {
                time_t now = time(0);
                char *date = ctime(&now);
                // Remove trailing newline
                date[strlen(date) - 1] = '\0';
                RosStringPublisherDataSharedPtr pub =
                    this->gui_node_ptr->getRosData(ros_data_name)->as<RosStringPublisherData>();
                pub->publish(std::string(date));
            });

        // Adds widget for displaying the date and time
        std::shared_ptr<StringPublisherWidget> widget =
            std::make_shared<StringPublisherWidget>(gui_node_ptr, window_name, ros_data_name);
        gui_node_ptr->addWidget(ros_data_name, widget);
        gui_node_ptr->prepare(window_name);
    }

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
    {
        return gui_node_ptr->get_node_base_interface();
    }
};

} // namespace gui_node

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gui_node::StringPublisher)
