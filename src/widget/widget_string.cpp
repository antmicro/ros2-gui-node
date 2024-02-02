/*
 * Copyright (c) 2022-2023 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <std_msgs/msg/string.hpp>

#include "gui_node/widget/widget_string.hpp"

namespace gui_node
{
bool StringWidget::draw()
{
    if (gui_node->getRosData(ros_data_name)->hasDataChanged())
    {
        string_converter(gui_node, data);
    }
    ImGui::Begin(window_name.c_str());
    ImGui::SetWindowSize(ImVec2(300, 100), ImGuiCond_FirstUseEver);
    ImGui::Text("%s", data.c_str());
    ImGui::End();
    return true;
}
} // namespace gui_node
