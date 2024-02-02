/*
 * Copyright (c) 2022-2024 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string>

#include "gui_node/widget/widget_detection.hpp"

namespace gui_node
{
void DetectionWidget::frame_extractor(std::shared_ptr<GuiNode> gui_node, sensor_msgs::msg::Image &image)
{
    bounding_boxes.clear();
    data_extractor(gui_node, image, bounding_boxes, filterclass, threshold);
}

void DetectionWidget::imgui_callback()
{
    float x1, y1, x2, y2;
    std::string label;
    ImColor color;

    ImGuiStyle &style = ImGui::GetStyle();
    const int title_bar_size = ImGui::GetFontSize() + style.FramePadding.y * 2.0f;
    float cornerroundingfactor = base_cornerroundingfactor * scale_factor;
    float perimeterthickness = base_perimeterthickness * scale_factor;

    ImVec2 window_pos = ImGui::GetWindowPos();
    ImVec2 window_size = ImGui::GetWindowSize();
    window_size.x -= style.WindowPadding.x * 2;
    window_size.y -= style.WindowPadding.y * 2 + title_bar_size;
    ImVec2 offset = ImVec2(window_pos.x + style.WindowPadding.x, window_pos.y + style.WindowPadding.y + title_bar_size);

    ImDrawList *draw_list = ImGui::GetWindowDrawList();

    std::string filter_window_name = "Filter '" + window_name + "'";
    ImGui::Begin(filter_window_name.c_str());
    ImGui::SetWindowSize(ImVec2(500, 200), ImGuiCond_FirstUseEver);
    ImGui::InputText(
        "Class name",
        &filterclass,
        ImGuiInputTextFlags_CallbackCharFilter,
        [](ImGuiInputTextCallbackData *d) -> ImGuiInputTextFlags
        {
            ImWchar c = d->EventChar;
            return !(std::isalpha(c) || c == ' ');
        });
    ImGui::SliderFloat("Certainty threshold", &threshold, 0.0f, 100.0f, "%.1f%%");
    ImGui::BeginChild("scrolling");
    if (ImGui::BeginTable("Detections", 2))
    {
        ImGui::TableSetupColumn("Class", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Certainty", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();

        std::transform(filterclass.begin(), filterclass.end(), filterclass.begin(), ::tolower);
        for (const auto &bounding_box : bounding_boxes)
        {
            color = bounding_box.object.color;
            std::string class_name = bounding_box.object.label;
            std::transform(class_name.begin(), class_name.end(), class_name.begin(), ::tolower);
            if (bounding_box.object.score >= threshold &&
                (filterclass.empty() || class_name.find(filterclass) != std::string::npos))
            {
                ImGui::SetWindowFontScale(window_size.x / 550);
                x1 = bounding_box.xmin * window_size.x + offset.x;
                y1 = bounding_box.ymin * window_size.y + offset.y;
                x2 = bounding_box.xmax * window_size.x + offset.x;
                y2 = bounding_box.ymax * window_size.y + offset.y;

                ImVec2 text_pos = ImVec2(x1 + cornerroundingfactor, y1 - ImGui::GetFontSize() - cornerroundingfactor);
                label = bounding_box.object.label + " (" + std::to_string(bounding_box.object.score) + "%)";

                draw_list->AddRect(
                    ImVec2(x1, y1),
                    ImVec2(x2, y2),
                    bounding_box.object.color,
                    cornerroundingfactor,
                    0,
                    perimeterthickness);
                draw_list->AddText(text_pos, bounding_box.object.color, label.c_str());
                ImGui::SetWindowFontScale(1.0f);
            }
            else
            {
                color = hiddenobjectcolor;
            }

            ImGui::TableNextColumn();
            ImGui::TextColored(color, "%s", bounding_box.object.label.c_str());
            ImGui::TableNextColumn();
            ImGui::TextColored(color, "%.1f%%", bounding_box.object.score);
        }

        ImGui::EndTable();
    }
    ImGui::EndChild();
    ImGui::End();
}

} // namespace gui_node
