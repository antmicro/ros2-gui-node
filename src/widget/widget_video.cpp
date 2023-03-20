#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "gui_node/ros_data/ros_subscriber_data.hpp"
#include "gui_node/widget/widget.hpp"
#include "gui_node/widget/widget_video.hpp"

namespace gui_node
{
WidgetVideo::WidgetVideo(std::shared_ptr<GuiNode> gui_node, const std::string window_name,
                         const std::string ros_data_name)
    : Widget(gui_node), window_name(window_name), ros_data_name(ros_data_name)
{
}

int WidgetVideo::encoding2channels(const std::string &encoding)
{
    if (encoding == "rgba8")
    {
        return 4;
    }
    else
    {
        RCLCPP_FATAL(gui_node->get_logger(), "Unsupported encoding: %s", encoding.c_str());
        throw std::invalid_argument("Unsupported encoding: " + encoding);
    }
}

void WidgetVideo::draw()
{
    std::shared_ptr<RosSubscriberData<sensor_msgs::msg::Image, sensor_msgs::msg::Image::SharedPtr>> subscriber =
        gui_node->getRosData(ros_data_name)
            ->as<RosSubscriberData<sensor_msgs::msg::Image, sensor_msgs::msg::Image::SharedPtr>>();

    sensor_msgs::msg::Image::SharedPtr msg = subscriber->getData();
    if (msg)
    {
        std::shared_ptr<GuiEngine> gui_engine = gui_node->getGuiEngine();
        int channels = encoding2channels(msg->encoding);
        if (!is_texture_initialized)
        {
            gui_engine->addTexture(ros_data_name, msg->data, msg->width, msg->height, channels);
            is_texture_initialized = true;
        }
        else
        {
            auto texture_loader = gui_engine->getTexture(ros_data_name);
            texture_loader->updateTexture(msg->data);
            ImGui::Begin(window_name.c_str());
            ImGui::Image((ImTextureID)texture_loader->getDescriptorSet(),
                         ImVec2(texture_loader->getWidth(), texture_loader->getHeight()));
            ImGui::End();
        }
    }
}
} // namespace gui_node
