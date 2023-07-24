/*
 * Copyright (c) 2022-2023 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <memory>
#include <sensor_msgs/msg/image.hpp>
#include <vector>

#include "gui_node/widget/widget.hpp"

namespace gui_node
{

using ConverterFunc = std::function<void(std::shared_ptr<GuiNode>, sensor_msgs::msg::Image &)>;

/**
 * Configuration parameters for the ImGui window.
 */
struct WindowConfig
{
    float aspect_ratio; ///< The aspect ratio of the window
    ImVec2 offset;      ///< The offset of the window
    ImVec2 window_size; ///< The size of the window
};

/**
 * Base class for video widgets.
 */
class BaseVideoWidget : public Widget
{
protected:
    /**
     * Resize callback for the ImGui window.
     * Sets the size of the video widget with respect to the window aspect ratio.
     *
     * @param configs Pointer to the ImGui callback data.
     */
    static void resizeCallback(ImGuiSizeCallbackData *data);

    /**
     * Calculates the aspect ratio for the ImGui window based on the provided parameters.
     * Calculates the offset for the ImGui window based on the ImGui style.
     *
     * @param width Width of the video widget.
     * @param height Height of the video widget.
     * @return WindowConfig structure containing the calculated aspect ratio and offset.
     */
    WindowConfig getWindowConfig(int width, int height);

    /**
     * Updates the TextureLoader with the new image if the image has changed.
     * If the texture was not initialized, this method will initialize it.
     *
     * @param buffer Vector containing the image data.
     * @param width Width of the image.
     * @param height Height of the image.
     * @param channels Number of channels in the image.
     */
    void updateTexture(const std::vector<unsigned char> &buffer, int width, int height, int channels);

    /**
     * Draws the ImGui window with Image from the provided texture.
     *
     * @param texture_loader Pointer to the texture loader.
     */
    void drawImGuiFrame(std::shared_ptr<TextureLoader> texture_loader);

    /*
     * Converts the image data and encoding to RGBA8 format if needed.
     *
     * @param msg The sensor_msgs::msg::Image message to change the encoding of if needed.
     *
     * @return The amount of channels, or -1 if the encoding is not supported.
     */
    int convert2rgba(sensor_msgs::msg::Image &msg);

    bool texture_initialized = false;           ///< Whether the texture has been initialized
    float base_width = 1920;                    ///< The base width for the ImGui window
    float scale_factor = 1.0;                   ///< The scale factor for the ImGui window
    std::vector<unsigned char> last_image_data; ///< The last image data received

    ConverterFunc frame_converter;            ///< Converts received messages to sensor_msgs::msg::Image format
    std::function<void(void)> imgui_callback; ///< Function to call when drawing the ImGui window

public:
    /**
     * Constructor.
     *
     * @param gui_node The shared pointer to the GUI node for logging.
     * @param window_name Name for the ImGui window.
     * @param ros_data_name Name of the ROS data object that widget is associated with.
     * @param frame_converter The frame converter function to convert the image to a sensor_msgs::msg::Image format that
     * can be displayed.
     * @param imgui_callback Function to call when drawing the ImGui window.
     */
    BaseVideoWidget(
        std::shared_ptr<GuiNode> gui_node,
        const std::string &window_name,
        const std::string &ros_data_name,
        ConverterFunc frame_converter,
        std::function<void(void)> imgui_callback)
        : Widget(gui_node, window_name, ros_data_name), frame_converter(frame_converter)
    {
        this->imgui_callback = imgui_callback;
    }

    /**
     * Draws the widget.
     */
    void draw() override;
};

class VideoWidget : public BaseVideoWidget
{
public:
    /**
     * Constructor.
     *
     * @param gui_node The shared pointer to the GUI node for logging.
     * @param window_name Name for the ImGui window.
     * @param ros_data_name Name of the ROS data object that widget is associated with.
     * @param frame_converter The frame converter function to convert the image to a sensor_msgs::msg::Image format that
     * can be displayed.
     */
    VideoWidget(
        std::shared_ptr<GuiNode> gui_node,
        const std::string &window_name,
        const std::string &ros_data_name,
        std::function<void(std::shared_ptr<GuiNode>, sensor_msgs::msg::Image &)> frame_converter)
        : BaseVideoWidget(gui_node, window_name, ros_data_name, frame_converter, [](void) {})
    {
    }
};

} // namespace gui_node
