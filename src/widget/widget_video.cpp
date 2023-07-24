/*
 * Copyright (c) 2022-2023 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>

#include "gui_node/widget/widget_video.hpp"

namespace gui_node
{
int BaseVideoWidget::convert2rgba(sensor_msgs::msg::Image &msg)
{
    if (msg.encoding == "rgba8")
    {
        return 4;
    }

    std::vector<std::string> supported_encodings = {"rgb8", "bgr8", "bgra8", "8UC3", "8UC4"};

    if (std::find(supported_encodings.begin(), supported_encodings.end(), msg.encoding) == supported_encodings.end())
    {
        RCLCPP_ERROR(gui_node->get_logger(), "Unsupported encoding: %s", msg.encoding.c_str());
        return -1;
    }

    cv::Mat buffer;

    if (msg.encoding == "rgb8")
    {
        buffer = cv::Mat(msg.height, msg.width, CV_8UC3, msg.data.data());
        cv::cvtColor(buffer, buffer, cv::COLOR_RGB2RGBA);
    }
    else if (msg.encoding == "bgra8" || msg.encoding == "8UC4")
    {
        buffer = cv::Mat(msg.height, msg.width, CV_8UC4, msg.data.data());
        cv::cvtColor(buffer, buffer, cv::COLOR_BGRA2RGBA);
    }
    else if (msg.encoding == "bgr8" || msg.encoding == "8UC3")
    {
        buffer = cv::Mat(msg.height, msg.width, CV_8UC3, msg.data.data());
        cv::cvtColor(buffer, buffer, cv::COLOR_BGR2RGBA);
    }

    msg.data.assign(buffer.datastart, buffer.dataend);
    msg.encoding = "rgba8";
    msg.step = msg.width * 4;
    return 4;
}

void BaseVideoWidget::draw()
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
    const int title_bar_size = std::round(ImGui::GetFontSize()) + style.FramePadding.y * 2;
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
    if (abs(diff) > 50 && std::abs(diffx) > 0 && std::abs(diffy) > 0)
    {
        data->DesiredSize.x = data->CurrentSize.x - diff;
        data->DesiredSize.y = ((data->DesiredSize.x - configs.offset.x) / configs.aspect_ratio) + configs.offset.y;
    }
    else if (abs(diff) > 0 && (std::abs(diffx) == 0 || std::abs(diffy) == 0))
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
    ImGui::SetNextWindowSizeConstraints(
        ImVec2(0, 0),
        ImVec2(FLT_MAX, FLT_MAX),
        resizeCallback,
        (void *)&window_configs);
    ImGui::Begin(window_name.c_str(), NULL, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    ImVec2 view = ImGui::GetWindowSize();
    scale_factor = view.x / base_width;
    window_configs = getWindowConfig(texture_loader->getWidth(), texture_loader->getHeight());
    ImGui::SetWindowSize(window_configs.window_size, ImGuiCond_Once);
    ImGui::Image(
        (ImTextureID)texture_loader->getDescriptorSet(),
        ImVec2(view.x - window_configs.offset.x, view.y - window_configs.offset.y));
    imgui_callback();
    ImGui::End();
}
} // namespace gui_node
