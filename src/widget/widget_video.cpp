#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>

#include "gui_node/ros_data/ros_publisher_data.hpp"
#include "gui_node/ros_data/ros_subscriber_data.hpp"
#include "gui_node/widget/widget.hpp"
#include "gui_node/widget/widget_video.hpp"

namespace gui_node
{
int MsgVideoWidget::encoding2channels(const std::string &encoding)
{
    if (encoding == "rgba8")
    {
        return 4;
    }
    else
    {
        return -1;
    }
}

void MsgVideoWidget::draw()
{
    using RosImageSubscriberData = RosSubscriberData<sensor_msgs::msg::Image, sensor_msgs::msg::Image::SharedPtr>;
    std::shared_ptr<RosImageSubscriberData> subscriber =
        gui_node->getRosData(ros_data_name)->as<RosImageSubscriberData>();

    if (subscriber->hasDataChanged())
    {
        sensor_msgs::msg::Image::SharedPtr msg = subscriber->getData();
        int channels = encoding2channels(msg->encoding);
        if (channels == -1)
        {
            RCLCPP_ERROR(gui_node->get_logger(), "Unsupported encoding: %s", msg->encoding.c_str());
            return;
        }
        updateTexture(msg->data, msg->width, msg->height, channels);
    }
    else if (texture_initialized)
    {
        std::shared_ptr<GuiEngine> gui_engine = gui_node->getGuiEngine();
        std::shared_ptr<TextureLoader> texture_loader = gui_engine->getTexture(ros_data_name);
        drawImGuiFrame(texture_loader);
    }
}

void CVMatVideoWidget::draw()
{
    using RosCVMatPublisherData = RosPublisherData<sensor_msgs::msg::Image, cv::Mat>;
    std::shared_ptr<RosCVMatPublisherData> publisher = gui_node->getRosData(ros_data_name)->as<RosCVMatPublisherData>();
    if (publisher->hasDataChanged())
    {
        cv::Mat image = publisher->getData();
        if (image.empty())
        {
            RCLCPP_WARN(gui_node->get_logger(), "Empty image");
        }
        else
        {
            int channels = image.channels();
            std::vector<unsigned char> buffer(image.data, image.data + image.total() * channels);
            updateTexture(buffer, image.cols, image.rows, channels);
        }
    }
    else if (texture_initialized)
    {
        std::shared_ptr<GuiEngine> gui_engine = gui_node->getGuiEngine();
        std::shared_ptr<TextureLoader> texture_loader = gui_engine->getTexture(ros_data_name);
        drawImGuiFrame(texture_loader);
    }
}

void BaseVideoWidget::updateTexture(const std::vector<unsigned char> &buffer, int width, int height, int channels)
{
    std::shared_ptr<GuiEngine> gui_engine = gui_node->getGuiEngine();
    if (!texture_initialized)
    {
        gui_engine->addTexture(ros_data_name, buffer, width, height, channels);
        last_image_data = buffer;
        texture_initialized = true;
    }
    std::shared_ptr<TextureLoader> texture_loader = gui_engine->getTexture(ros_data_name);
    if (buffer != last_image_data)
    {
        texture_loader->updateTexture(buffer);
        last_image_data = buffer;
    }
    drawImGuiFrame(texture_loader);
}

WindowConfig BaseVideoWidget::getWindowConfig(int width, int height)
{
    ImGuiStyle &style = ImGui::GetStyle();
    // ImGui doesn't provide method to set window content size, so content area is smaller than the image.
    // This is a workaround to create a window that fits the image.
    const int title_bar_size = ImGui::GetFontSize() + style.FramePadding.y * 2;
    // To reduce window flickering the aspect ration is rounded to 1 decimal place
    float aspect_ratio = round((float)width / (float)height * 10) / 10;
    ImVec2 offset = ImVec2(style.WindowPadding.x * 2, style.WindowPadding.y * 2 + title_bar_size);
    ImVec2 window_size = ImVec2(width + offset.x, width / aspect_ratio + offset.y);
    return WindowConfig{aspect_ratio, offset, window_size};
}

void BaseVideoWidget::resizeCallback(ImGuiSizeCallbackData *data)
{
    struct WindowConfig configs = *(struct WindowConfig *)data->UserData;
    float diffx = data->CurrentSize.x - data->DesiredSize.x;
    float diffy = data->CurrentSize.y - data->DesiredSize.y;
    int diff = diffx + diffy;
    if (abs(diff) > 3)
    {
        data->DesiredSize.x = data->CurrentSize.x - diff;
        data->DesiredSize.y = ((data->DesiredSize.x - configs.offset.x) / configs.aspect_ratio) + configs.offset.y;
    }
    else
    {
        data->DesiredSize.x = data->CurrentSize.x;
        data->DesiredSize.y = data->CurrentSize.y;
    }
}

void BaseVideoWidget::drawImGuiFrame(std::shared_ptr<TextureLoader> texture_loader)
{
    WindowConfig window_configs = getWindowConfig(texture_loader->getWidth(), texture_loader->getHeight());
    ImGui::SetNextWindowSizeConstraints(ImVec2(0, 0), ImVec2(FLT_MAX, FLT_MAX), resizeCallback,
                                        (void *)&window_configs);
    ImGui::Begin(window_name.c_str(), NULL, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::SetWindowSize(window_configs.window_size, ImGuiCond_Once);
    ImVec2 view = ImGui::GetWindowSize();
    ImGui::Image((ImTextureID)texture_loader->getDescriptorSet(),
                 ImVec2(view.x - window_configs.offset.x, view.y - window_configs.offset.y));
    ImGui::End();
}
} // namespace gui_node
