/*
 * Copyright (c) 2022-2024 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <memory>

#include "gui_node/gui_node.hpp"
#include "gui_node/widget/widget_rosout.hpp"

namespace gui_node
{

RosoutWidget::RosoutWidget(
    std::shared_ptr<GuiNode> gui_node_ptr,
    const std::string &window_name,
    const std::string &ros_data_name,
    int max_table_size)
    : Widget(gui_node_ptr, window_name, ros_data_name),
      messages(FixedDeque<rcl_interfaces::msg::Log::SharedPtr>(max_table_size))
{
}

bool RosoutWidget::draw()
{
    using RosoutSubscriber = RosSubscriberData<rcl_interfaces::msg::Log, rcl_interfaces::msg::Log::SharedPtr>;
    std::shared_ptr<RosoutSubscriber> subscriber = this->gui_node->getRosData(ros_data_name)->as<RosoutSubscriber>();
    if (subscriber->hasDataChanged())
    {
        messages.push_front(subscriber->getData());
    }
    ImGui::Begin(window_name.c_str());
    ImGui::SetWindowSize(ImVec2(500, 500), ImGuiCond_FirstUseEver);
    if (ImGui::BeginTable("rosout_table", 6, flags, ImVec2(0.0f, 0.0f)))
    {
        ImGui::TableSetupColumn("Level", ImGuiTableColumnFlags_NoHide);
        ImGui::TableSetupColumn("Name");
        ImGui::TableSetupColumn("Message");
        ImGui::TableSetupColumn("File");
        ImGui::TableSetupColumn("Function");
        ImGui::TableSetupColumn("Line");
        ImGui::TableHeadersRow();
        for (const auto &message : messages)
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%s", rosout_level_map.at(message->level).c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%s", message->name.c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%s", message->msg.c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%s", message->file.c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%s", message->function.c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%d", message->line);
        }
        ImGui::EndTable();
    }
    ImGui::End();
    return true;
}

} // namespace gui_node
