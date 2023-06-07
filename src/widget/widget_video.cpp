#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>

#include "gui_node/widget/widget_video.hpp"

namespace gui_node
{
int VideoWidget::convert2rgba(sensor_msgs::msg::Image &msg)
{
    if (msg.encoding == "rgba8")
    {
        return 4;
    }

    std::vector<std::string> supported_encodings = {"rgb8", "bgr8", "bgra8"};

    if (std::find(supported_encodings.begin(), supported_encodings.end(), msg.encoding) == supported_encodings.end())
    {
        RCLCPP_ERROR(gui_node->get_logger(), "Unsupported encoding: %s", msg.encoding.c_str());
        return -1;
    }

    std::vector<unsigned char> rgba_buffer(msg.width * msg.height * 4);
    unsigned int image_size = msg.width * msg.height;

    if (msg.encoding == "rgb8")
    {
        for (unsigned int i = 0; i < image_size; i++)
        {
            rgba_buffer[i * 4] = msg.data[i * 3];
            rgba_buffer[i * 4 + 1] = msg.data[i * 3 + 1];
            rgba_buffer[i * 4 + 2] = msg.data[i * 3 + 2];
            rgba_buffer[i * 4 + 3] = 255;
        }
    }
    else if (msg.encoding == "bgra8")
    {
        for (unsigned int i = 0; i < image_size; i++)
        {
            rgba_buffer[i * 4] = msg.data[i * 4 + 2];
            rgba_buffer[i * 4 + 1] = msg.data[i * 4 + 1];
            rgba_buffer[i * 4 + 2] = msg.data[i * 4];
            rgba_buffer[i * 4 + 3] = msg.data[i * 4 + 3];
        }
    }
    else if (msg.encoding == "bgr8")
    {
        for (unsigned int i = 0; i < image_size; i++)
        {
            rgba_buffer[i * 4] = msg.data[i * 3 + 2];
            rgba_buffer[i * 4 + 1] = msg.data[i * 3 + 1];
            rgba_buffer[i * 4 + 2] = msg.data[i * 3];
            rgba_buffer[i * 4 + 3] = 255;
        }
    }

    msg.data = rgba_buffer;
    msg.encoding = "rgba8";
    msg.step = msg.width * 4;
    return 4;
}

void VideoWidget::draw()
{
    std::shared_ptr<RosData> ros_data = gui_node->getRosData(ros_data_name);

    if (ros_data->hasDataChanged())
    {
        // Create a shared pointer to the data
        sensor_msgs::msg::Image msg = sensor_msgs::msg::Image();
        frame_converter(gui_node, msg);
        int channels = convert2rgba(msg);
        if (channels == -1)
        {
            RCLCPP_ERROR(gui_node->get_logger(), "Unsupported encoding: %s", msg.encoding.c_str());
            return;
        }
        updateTexture(msg.data, msg.width, msg.height, channels);
    }
    else if (texture_initialized)
    {
        std::shared_ptr<GuiEngine> gui_engine = gui_node->getGuiEngine();
        std::shared_ptr<TextureLoader> texture_loader = gui_engine->getTexture(ros_data_name);
        drawImGuiFrame(texture_loader);
    }
}

void VideoWidget::updateTexture(const std::vector<unsigned char> &buffer, int width, int height, int channels)
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

WindowConfig VideoWidget::getWindowConfig(int width, int height)
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

void VideoWidget::resizeCallback(ImGuiSizeCallbackData *data)
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

void VideoWidget::drawImGuiFrame(std::shared_ptr<TextureLoader> texture_loader)
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
