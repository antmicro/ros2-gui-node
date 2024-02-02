/*
 * Copyright (c) 2022-2023 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <std_srvs/srv/trigger.hpp>

#include "gui_node/ros_data/ros_client_data.hpp"
#include "gui_node/widget/widget_counter.hpp"

namespace gui_node
{
bool CounterWidget::draw()
{
    // Get the data
    using RosCounterClientData = RosServiceClientData<std_srvs::srv::Trigger, std_srvs::srv::Trigger::Response>;
    std::shared_ptr<RosCounterClientData> ros_data =
        this->gui_node->getRosData(ros_data_name)->as<RosCounterClientData>();
    if (ros_data->hasDataChanged())
    {
        std_srvs::srv::Trigger::Response response = ros_data->getData();
        if (response.success && response.message == "triggered")
        {
            counter++;
        }
    }

    // Draw the widget
    ImGui::Begin(window_name.c_str());
    ImGui::SetWindowSize(ImVec2(200, 100), ImGuiCond_FirstUseEver);
    ImGui::SetCursorPosX((ImGui::GetWindowWidth() - ImGui::CalcTextSize("Trigger").x) / 2);

    // If service is not available, disable the button
    if (ros_data->isServiceAvailable())
    {
        if (ImGui::Button("Trigger"))
        {
            // Make the request
            std_srvs::srv::Trigger::Request::SharedPtr request = std::make_shared<std_srvs::srv::Trigger::Request>();
            ros_data->sendRequest(request);
        }
    }
    else
    {
        ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(0.0f, 0.6f, 0.6f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor::HSV(0.0f, 0.7f, 0.7f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, (ImVec4)ImColor::HSV(0.0f, 0.8f, 0.8f));
        ImGui::Button("Trigger");
        ImGui::PopStyleColor(3);
    }
    std::string counter_str = "Counter: " + std::to_string(counter);
    ImGui::SetCursorPosX((ImGui::GetWindowWidth() - ImGui::CalcTextSize(counter_str.c_str()).x) / 2);
    ImGui::Text("%s", counter_str.c_str());
    ImGui::End();
    return true;
}
} // namespace gui_node
