/*
 * Copyright (c) 2025 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "gui_node/gui_node.hpp"
#include "gui_node/ros_data/ros_client_data.hpp"
#include "gui_node/widget/widget.hpp"
#include "gui_node/widget/widget_video.hpp"

namespace gui_node
{

class DepthWidget : public BaseVideoWidget
{
public:
    enum class eColorMappings
    {
        Rainbow,
        Magma,
    };
    eColorMappings mapping;
    struct DepthData
    {
        std::vector<float> values;
        int rows;
        int cols;
    };
    /**
     * Processes depth values and prepares to display them as an image.
     *
     * @param values The depth estimation prediction
     * @param rows Number of rows of the depth estimation data
     * @param cols Number of columns of the depth estimation data
     * @return sensor_msgs::msg::Image::SharedPtr The image to display.
     */
    sensor_msgs::msg::Image
    prep_image(std::shared_ptr<GuiNode> gui_node_ptr, const std::vector<float> &values, int rows, int cols);
    DepthWidget(
        std::shared_ptr<GuiNode> gui_node,
        const std::string &window_name,
        const std::string &ros_data_name,
        std::function<void(std::shared_ptr<GuiNode>, DepthData &)> frame_converter)
        : BaseVideoWidget(
              gui_node,
              window_name,
              ros_data_name,
              [this, frame_converter](std::shared_ptr<GuiNode> gui_node_ptr, sensor_msgs::msg::Image &imgmsg) -> void
              {
                  DepthData msg;
                  frame_converter(gui_node_ptr, msg);
                  imgmsg = this->prep_image(gui_node_ptr, msg.values, msg.rows, msg.cols);
              },
              [](void) {})
    {
    }
};

} // namespace gui_node
