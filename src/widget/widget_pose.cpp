/*
 * Copyright (c) 2025 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "gui_node/widget/widget_pose.hpp"
#include <unordered_map>

namespace gui_node
{
void PoseWidget::pose_extractor(std::shared_ptr<GuiNode> gui_node, sensor_msgs::msg::Image &image)
{
    this->poses.clear();

    this->data_extractor(gui_node, image, this->poses);
}

void PoseWidget::imgui_callback()
{

    std::string label;
    ImColor color;

    ImGuiStyle &style = ImGui::GetStyle();
    const int title_bar_size = ImGui::GetFontSize() + style.FramePadding.y * 2.0f;
    const float cornerroundingfactor = this->base_cornerroundingfactor * this->scale_factor;
    const float perimeterthickness = this->base_perimeterthickness * this->scale_factor;
    const float point_radius = this->base_point_radius * this->scale_factor;

    ImVec2 window_pos = ImGui::GetWindowPos();
    ImVec2 window_size = ImGui::GetWindowSize();
    window_size.x -= style.WindowPadding.x * 2;
    window_size.y -= style.WindowPadding.y * 2 + title_bar_size * 2;
    ImVec2 offset =
        ImVec2(window_pos.x + style.WindowPadding.x, window_pos.y + style.WindowPadding.y + title_bar_size * 2);

    ImDrawList *draw_list = ImGui::GetWindowDrawList();

    ImGui::Begin(this->window_name.c_str());
    ImGui::SetWindowSize(ImVec2(500, 500), ImGuiCond_FirstUseEver);

    // draw pose
    float x1, x2, y1, y2;

    int w, h;

    const std::array<std::pair<int, int>, 19> connect = {
        std::pair{1, 2},
        {1, 3},
        {2, 4},
        {4, 0},
        {3, 0},
        {0, 5},
        {0, 6},
        {5, 6},
        {5, 7},
        {7, 9},
        {6, 8},
        {8, 10},
        {6, 12},
        {5, 11},
        {11, 12},
        {11, 13},
        {13, 15},
        {12, 14},
        {14, 16}};
    const ImColor face_c = ImColor(50, 25, 255);
    const ImColor rarm_c = ImColor(255, 127, 0);
    const ImColor larm_c = ImColor(0, 255, 0);
    const ImColor rleg_c = ImColor(255, 25, 48);
    const ImColor lleg_c = ImColor(43, 255, 232);

    const size_t gradient_step_count = 10;

    const std::array<ImColor, 17> point_colors = {
        face_c, // nose
        face_c, // left eye
        face_c, // right eye
        face_c, // left ear
        face_c, // right ear
        rarm_c, // right arm
        larm_c, // left arm
        rarm_c, // right elbow
        larm_c, // left elbow
        rarm_c, // right hand
        larm_c, // left hand
        rleg_c, // right hip
        lleg_c, // left hip
        rleg_c, // right knee
        lleg_c, // left knee
        rleg_c, // right foot
        lleg_c  // left foot
    };

    for (const auto &pose : this->poses)
    {
        ImGui::SetWindowFontScale(window_size.x / 550);

        const BoundingBox &bbox = pose.bbox;
        const std::vector<Point> &points = pose.points;
        std::unordered_map<int, ImVec2> points_mapped;

        x1 = bbox.xmin * window_size.x + offset.x;
        y1 = bbox.ymin * window_size.y + offset.y;
        x2 = bbox.xmax * window_size.x + offset.x;
        y2 = bbox.ymax * window_size.y + offset.y;

        w = (bbox.xmax - bbox.xmin) * window_size.x;
        h = (bbox.ymax - bbox.ymin) * window_size.y;

        ImU32 color = bbox.object.color;

        draw_list->AddRect(ImVec2(x1, y1), ImVec2(x2, y2), color, cornerroundingfactor, 0, perimeterthickness);

        // draw points

        for (const auto &point : points)
        {
            points_mapped[point.id] = ImVec2(point.x * w + x1, point.y * h + y1);
        }

        int color_idx = 0;
        for (auto [idx1, idx2] : connect)
        {
            auto p1 = points_mapped[idx1];
            auto p2 = points_mapped[idx2];
            float xp1 = p1.x;
            float yp1 = p1.y;
            float xp2 = p2.x;
            float yp2 = p2.y;
            if (xp1 < x1 || xp2 < x1 || yp1 < y1 || yp2 < y1 || xp1 > x2 || xp2 > x2 || yp1 > y2 || yp2 > y2)
            {
                color_idx += 1;
                continue;
            }

            auto p1_color = point_colors[idx1].Value;
            auto p2_color = point_colors[idx2].Value;

            float color_step[3] = {
                (p2_color.x - p1_color.x) / static_cast<float>(gradient_step_count),
                (p2_color.y - p1_color.y) / static_cast<float>(gradient_step_count),
                (p2_color.z - p1_color.z) / static_cast<float>(gradient_step_count),
            };

            ImColor start_color = p1_color;

            float point_gradient[2] = {
                (p2.x - p1.x) / static_cast<float>(gradient_step_count),
                (p2.y - p1.y) / static_cast<float>(gradient_step_count)};

            ImVec2 start_p = p1;
            ImVec2 end_p = ImVec2(start_p.x + point_gradient[0], start_p.y + point_gradient[1]);

            for (size_t step = 0; step < gradient_step_count; ++step)
            {

                draw_list->AddLine(start_p, end_p, start_color, point_radius);

                start_color.Value.x += color_step[0];
                start_color.Value.y += color_step[1];
                start_color.Value.z += color_step[2];

                start_p.x += point_gradient[0];
                start_p.y += point_gradient[1];

                end_p.x += point_gradient[0];
                end_p.y += point_gradient[1];
            }

            draw_list->AddCircleFilled(p1, point_radius, point_colors[idx1]);
            draw_list->AddCircleFilled(p2, point_radius, point_colors[idx2]);
        }

        // draw text

        ImVec2 text_pos = ImVec2(x1 + cornerroundingfactor, y1 - ImGui::GetFontSize() - cornerroundingfactor);
        label = bbox.object.label + " (" + std::to_string(bbox.object.score) + "%)";

        draw_list->AddText(text_pos, bbox.object.color, label.c_str());
        ImGui::SetWindowFontScale(1.0f);
    }

    ImGui::End();
}
} // namespace gui_node
