/*
 * Copyright (c) 2025 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "gui_node/utils/detection.hpp"
#include "gui_node/utils/pose.hpp"
#include "gui_node/widget/widget_video.hpp"

namespace gui_node
{
using PoseExtractor = std::function<void(std::shared_ptr<GuiNode>, sensor_msgs::msg::Image &, std::vector<Pose> &)>;

class PoseWidget : public BaseVideoWidget
{
    /**
     * Extracts poses and image form the message.
     *
     * @param gui_node The shared pointer to the GUI node.
     * @param image The image message.
     */
    void pose_extractor(std::shared_ptr<GuiNode> gui_node, sensor_msgs::msg::Image &image);

    /**
     * Draws poses on the ImGui window.
     */
    void imgui_callback();

    std::vector<Pose> poses; ///< Vector of poses.

    const float base_cornerroundingfactor = 10.0f; ///< Base corner rounding factor.
    const float base_perimeterthickness = 8.0f;    ///< Base perimeter thickness.
    const float base_point_radius = 10.f;          ///< Base point radius.

    PoseExtractor data_extractor; ///< Function to extract poses and image from the message.

public:
    PoseWidget(
        std::shared_ptr<GuiNode> gui_node,
        const std::string &window_name,
        const std::string &ros_data_name,
        PoseExtractor data_extractor)
        : BaseVideoWidget(
              gui_node,
              window_name,
              ros_data_name,
              std::bind(&PoseWidget::pose_extractor, this, std::placeholders::_1, std::placeholders::_2),
              std::bind(&PoseWidget::imgui_callback, this))
    {
        this->data_extractor = data_extractor;
        this->poses = std::vector<Pose>();
    }
};

} // namespace gui_node
