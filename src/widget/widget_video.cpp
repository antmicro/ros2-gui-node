#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "gui_node/ros_data/ros_publisher_data.hpp"
#include "gui_node/ros_data/ros_subscriber_data.hpp"
#include "gui_node/widget/widget.hpp"
#include "gui_node/widget/widget_video.hpp"

namespace gui_node
{
int WidgetVideoMsg::encoding2channels(const std::string &encoding)
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

void WidgetVideoMsg::draw()
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

void WidgetVideoCVMat::draw()
{
    auto publisher = gui_node->getRosData(ros_data_name)->as<RosPublisherData<sensor_msgs::msg::Image, cv::Mat>>();
    cv::Mat image = publisher->getData();
    if (!image.empty())
    {
        std::shared_ptr<GuiEngine> gui_engine = gui_node->getGuiEngine();
        int channels = image.channels();
        std::vector<unsigned char> buffer(image.data, image.data + image.total() * channels);
        if (!is_texture_initialized)
        {
            gui_engine->addTexture(ros_data_name, buffer, image.cols, image.rows, channels);
            is_texture_initialized = true;
        }
        else
        {
            auto texture_loader = gui_engine->getTexture(ros_data_name);
            texture_loader->updateTexture(buffer);
            ImGui::Begin(window_name.c_str());
            ImGui::Image((ImTextureID)texture_loader->getDescriptorSet(),
                         ImVec2(texture_loader->getWidth(), texture_loader->getHeight()));
            ImGui::End();
        }
    }
}
} // namespace gui_node
