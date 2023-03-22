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
    using RosImageSubscriberData = RosSubscriberData<sensor_msgs::msg::Image, sensor_msgs::msg::Image::SharedPtr>;
    std::shared_ptr<RosImageSubscriberData> subscriber =
        gui_node->getRosData(ros_data_name)->as<RosImageSubscriberData>();

    sensor_msgs::msg::Image::SharedPtr msg = subscriber->getData();
    if (msg)
    {
        std::shared_ptr<GuiEngine> gui_engine = gui_node->getGuiEngine();
        if (!texture_initialized)
        {
            gui_engine->addTexture(ros_data_name, msg->data, msg->width, msg->height, encoding2channels(msg->encoding));
            texture_initialized = true;
        }
        std::shared_ptr<TextureLoader> texture_loader = gui_engine->getTexture(ros_data_name);
        texture_loader->updateTexture(msg->data);
        drawImGuiFrame(texture_loader);
    }
}

void WidgetVideoCVMat::draw()
{
    using RosCVMatPublisherData = RosPublisherData<sensor_msgs::msg::Image, cv::Mat>;
    std::shared_ptr<RosCVMatPublisherData> publisher = gui_node->getRosData(ros_data_name)->as<RosCVMatPublisherData>();
    cv::Mat image = publisher->getData();
    if (!image.empty())
    {
        std::shared_ptr<GuiEngine> gui_engine = gui_node->getGuiEngine();
        int channels = image.channels();
        std::vector<unsigned char> buffer(image.data, image.data + image.total() * channels);
        if (!texture_initialized)
        {
            gui_engine->addTexture(ros_data_name, buffer, image.cols, image.rows, channels);
            texture_initialized = true;
        }
        std::shared_ptr<TextureLoader> texture_loader = gui_engine->getTexture(ros_data_name);
        texture_loader->updateTexture(buffer);
        drawImGuiFrame(texture_loader);
    }
}

WindowConfigs WidgetVideoBase::getWindowConfigs(int width, int height)
{
    ImGuiStyle &style = ImGui::GetStyle();
    // ImGui doesn't provide method to set window content size, so content area is smaller than the image.
    // This is a workaround to create a window that fits the image.
    const int title_bar_size = ImGui::GetFontSize() + style.FramePadding.y * 2;
    // To reduce window flickering the aspect ration is rounded to 1 decimal place
    float aspect_ratio = round((float)width / (float)height * 10) / 10;
    ImVec2 offset = ImVec2(style.WindowPadding.x * 2, style.WindowPadding.y * 2 + title_bar_size);
    ImVec2 window_size = ImVec2(width + offset.x, width / aspect_ratio + offset.y);
    return WindowConfigs{aspect_ratio, offset, window_size};
}

void WidgetVideoBase::resizeCallback(ImGuiSizeCallbackData *data)
{
    struct WindowConfigs configs = *(struct WindowConfigs *)data->UserData;
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

void WidgetVideoBase::drawImGuiFrame(std::shared_ptr<TextureLoader> texture_loader)
{
    WindowConfigs window_configs = getWindowConfigs(texture_loader->getWidth(), texture_loader->getHeight());
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