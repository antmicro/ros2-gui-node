/*
 * Copyright (c) 2022-2023 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "gui_node/widget/widget_control.hpp"
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rclcpp/rclcpp.hpp>

namespace gui_node
{

using namespace rcl_interfaces::msg;
using namespace rcl_interfaces::srv;

ControlWidget::ControlWidget(
    std::shared_ptr<GuiNode> gui_node,
    const std::string &window_name,
    const std::string &ros_node_name)
    : Widget(gui_node, window_name, ros_node_name + "/parameters")
{
    list_parameters_client_ = gui_node->create_client<ListParameters>(ros_node_name + "/list_parameters");
    get_parameters_client_ = gui_node->create_client<GetParameters>(ros_node_name + "/get_parameters");
    describe_parameters_client_ = gui_node->create_client<DescribeParameters>(ros_node_name + "/describe_parameters");
    set_parameters_client_ = gui_node->create_client<SetParameters>(ros_node_name + "/set_parameters");
    getParameters();
}

void ControlWidget::updateParameters()
{
    std::vector<Parameter> parameters_to_update;
    parameters_mutex.lock();
    for (NodeParameter &parameter : parameters)
    {
        if (!parameter.descriptor)
        {
            continue;
        }
        else if (!parameter.descriptor->read_only && *parameter.value.get() != *parameter.curr_value.get())
        {
            Parameter parameter_to_update;
            parameter_to_update.name = parameter.name;
            parameter_to_update.value = *parameter.value.get();
            parameters_to_update.push_back(parameter_to_update);
            parameter.curr_value.reset(new ParameterValue(*parameter.value.get()));
        }
    }
    if (parameters_to_update.empty() || parameters.empty())
    {
        RCLCPP_DEBUG(gui_node->get_logger(), "No parameters to update");
        parameters_mutex.unlock();
        return;
    }

    auto request = std::make_shared<SetParameters::Request>();
    request->parameters = parameters_to_update;
    set_parameters_client_->async_send_request(
        request,
        [this](rclcpp::Client<SetParameters>::SharedFuture future)
        {
            auto response = future.get();
            for (auto &result : response->results)
            {
                if (!result.successful)
                {
                    parameters_mutex.unlock();
                    RCLCPP_ERROR(gui_node->get_logger(), "%s", result.reason.c_str());
                    getParameters();
                    return;
                }
            }
            RCLCPP_DEBUG(gui_node->get_logger(), "Updated %ld parameters", response->results.size());
        });
    parameters_mutex.unlock();
}

bool ControlWidget::draw()
{
    ImGui::SetNextWindowSizeConstraints(ImVec2(300, 500), ImVec2(FLT_MAX, FLT_MAX));
    ImGui::Begin(window_name.c_str());
    ImGui::SetWindowSize(ImVec2(500, 500), ImGuiCond_FirstUseEver);

    // Setup buttons
    if (ImGui::Button("Refresh"))
    {
        getParameters();
    }
    ImGui::SameLine();
    ImGui::SetCursorPosX(ImGui::GetWindowWidth() - ImGui::CalcTextSize("Update").x - 60);
    if (ImGui::Button("Update"))
    {
        updateParameters();
    }

    // Setup filter by name
    ImGui::PushItemWidth(ImGui::GetWindowWidth() - 60);
    ImGui::InputText(
        "##fparameter_name",
        &filter_name,
        ImGuiInputTextFlags_CallbackCharFilter,
        [](ImGuiInputTextCallbackData *d) -> ImGuiInputTextFlags
        {
            ImWchar c = d->EventChar;
            return !(std::isprint(c) || c == ' ');
        });
    ImGui::PopItemWidth();
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered())
    {
        ImGui::BeginTooltip();
        ImGui::Text("Filter by name");
        ImGui::EndTooltip();
    }

    // Calculate table height
    ImVec2 table_size = ImGui::GetContentRegionAvail();
    if (!curr_param_desc.empty())
    {
        table_size.y -= ImGui::CalcTextSize(curr_param_desc.c_str(), nullptr, false, table_size.x).y;
        table_size.y -= ImGui::GetFrameHeightWithSpacing() * 2 + ImGui::GetStyle().ItemSpacing.y;
        table_size.y = std::max(table_size.y, 100.0f);
    }

    // Draw table
    if (ImGui::BeginTable(ros_data_name.c_str(), 3, table_flags, table_size))
    {
        ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_DefaultSort);
        ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_DefaultSort);
        ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_NoSort);
        ImGui::TableHeadersRow();

        if (first_draw)
        {
            first_draw = false;
            ImGuiTableSortSpecs *sort_specs = ImGui::TableGetSortSpecs();
            sort_specs->SpecsDirty = false;
            sort_specs->Specs = nullptr;
            sort_specs->SpecsCount = 0;
        }

        // Draw table rows
        parameters_mutex.lock();
        sortTable();
        for (NodeParameter &parameter : parameters)
        {
            if (parameter.descriptor && parameter.value &&
                (filter_name.empty() || parameter.name.find(filter_name) != std::string::npos))
            {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                drawName(parameter);
                ImGui::TableSetColumnIndex(1);
                drawType(parameter);
                ImGui::TableSetColumnIndex(2);
                drawValue(parameter);
            }
        }
        ImGui::EndTable();

        // Draw description
        if (!curr_param_desc.empty())
        {
            ImGui::SeparatorText("Description");
            ImGui::TextWrapped("%s", curr_param_desc.c_str());
            ImGui::SetCursorPosX(
                ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x - 100 - ImGui::GetStyle().FramePadding.x * 2);
            if (ImGui::Button("Close", ImVec2(100, ImGui::GetFrameHeight())))
            {
                curr_param_desc.clear();
            }
        }
        parameters_mutex.unlock();
    }
    ImGui::End();
    return true;
}

void ControlWidget::drawName(const NodeParameter &parameter)
{
    if (!parameter.descriptor->description.empty())
    {
        if (ImGui::Selectable(
                parameter.name.c_str(),
                false,
                0,
                ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFrameHeight())))
        {
            curr_param_desc = parameter.descriptor->description;
        }
    }
    else
    {
        ImGui::TextDisabled("%s", parameter.name.c_str());
    }
}

void ControlWidget::drawType(NodeParameter &parameter)
{
    bool is_disabled = !parameter.descriptor->dynamic_typing || parameter.descriptor->read_only;
    if (is_disabled)
    {
        ImGui::BeginDisabled(true);
    }
    ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
    ImGui::PushStyleVar(
        ImGuiStyleVar_FramePadding,
        ImVec2(
            (ImGui::CalcItemWidth() - ImGui::CalcTextSize(parameter.type.c_str()).x) / 2,
            ImGui::GetStyle().FramePadding.y));
    ImGuiTableSortSpecs *sort_specs = ImGui::TableGetSortSpecs();
    if (ImGui::BeginCombo(("##t" + parameter.name).c_str(), parameter.type.c_str(), ImGuiComboFlags_NoArrowButton))
    {
        for (const auto &type : parameter_types)
        {
            if (type.first == ParameterType::PARAMETER_NOT_SET)
            {
                continue;
            }
            bool selected = parameter.descriptor->type == type.first;
            if (ImGui::Selectable(type.second.c_str(), selected))
            {
                parameter.descriptor->type = type.first;
                parameter.value->type = type.first;
                parameter.type = type.second;
                if (sort_specs)
                {
                    sort_specs->SpecsDirty = true;
                }
            }
            if (selected)
            {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }
    ImGui::PopStyleVar();
    ImGui::PopItemWidth();
    if (is_disabled)
    {
        ImGui::EndDisabled();
    }
}

void ControlWidget::drawValue(const NodeParameter &parameter)
{
    if (parameter.descriptor->read_only)
    {
        ImGui::BeginDisabled(true);
    }

    float draw_width = ImGui::GetContentRegionAvail().x;
    std::string from_val = "", to_val = "";
    switch (parameter.value->type)
    {
    case ParameterType::PARAMETER_NOT_SET:
        draw_width -= ImGui::CalcTextSize("Not set").x;
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + draw_width / 2);
        ImGui::Text("Not set");
        break;
    case ParameterType::PARAMETER_BOOL:
        draw_width -= ImGui::GetFrameHeight();
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + draw_width / 2);
        ImGui::Checkbox(("##v" + parameter.name).c_str(), &parameter.value->bool_value);
        break;
    case ParameterType::PARAMETER_INTEGER:
        if (parameter.descriptor->integer_range.size() > 0)
        {
            from_val = std::to_string(parameter.descriptor->integer_range[0].from_value);
            to_val = std::to_string(parameter.descriptor->integer_range[0].to_value);
        }
        drawNumericValue(
            from_val,
            to_val,
            [&parameter]()
            {
                IntegerRange int_range;
                int_range.step = 1;
                if (parameter.descriptor->integer_range.size() > 0)
                {
                    int_range = parameter.descriptor->integer_range[0];
                }
                ImGui::DragScalar(
                    ("##v" + parameter.name).c_str(),
                    ImGuiDataType_S64,
                    &parameter.value->integer_value,
                    int_range.step,
                    &int_range.from_value,
                    &int_range.to_value,
                    "%ld",
                    ImGuiSliderFlags_AlwaysClamp);
                // Clamp value to step if manual input is used
                if (int_range.step != 0)
                {
                    parameter.value->integer_value = parameter.value->integer_value /
                                                     static_cast<int64_t>(int_range.step) *
                                                     static_cast<int64_t>(int_range.step);
                }
            });
        break;
    case ParameterType::PARAMETER_DOUBLE:
        if (parameter.descriptor->floating_point_range.size() > 0)
        {
            from_val = trimZeros(std::to_string(parameter.descriptor->floating_point_range[0].from_value));
            to_val = trimZeros(std::to_string(parameter.descriptor->floating_point_range[0].to_value));
        }
        drawNumericValue(
            from_val,
            to_val,
            [&parameter]()
            {
                FloatingPointRange float_range;
                float_range.step = 1.0;
                if (parameter.descriptor->floating_point_range.size() > 0)
                {
                    float_range = parameter.descriptor->floating_point_range[0];
                }
                ImGui::DragScalar(
                    ("##v" + parameter.name).c_str(),
                    ImGuiDataType_Double,
                    &parameter.value->double_value,
                    float_range.step,
                    &float_range.from_value,
                    &float_range.to_value,
                    "%.3f",
                    ImGuiSliderFlags_AlwaysClamp);
                // Clamp value to step if manual input is used
                if (float_range.step != 0.0f)
                {
                    parameter.value->double_value = parameter.value->double_value / float_range.step * float_range.step;
                }
            });
        break;
    case ParameterType::PARAMETER_STRING:
        draw_width -= ImGui::CalcItemWidth();
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + draw_width / 2);
        ImGui::PushStyleVar(
            ImGuiStyleVar_FramePadding,
            ImVec2(
                (ImGui::CalcItemWidth() - ImGui::CalcTextSize(parameter.value->string_value.c_str()).x) / 2,
                ImGui::GetStyle().FramePadding.y));
        ImGui::InputText(("##v" + parameter.name).c_str(), &parameter.value->string_value);
        ImGui::PopStyleVar();
        break;
    case ParameterType::PARAMETER_INTEGER_ARRAY:
        drawArrayValue<int64_t>(
            parameter,
            parameter.value->integer_array_value,
            0,
            [&parameter](float draw_width, size_t i)
            {
                ImGui::SetCursorPosX(ImGui::GetCursorPosX() + draw_width / 2);
                ImGui::DragScalar(
                    ("##v" + parameter.name + std::to_string(i)).c_str(),
                    ImGuiDataType_S64,
                    &parameter.value->integer_array_value[i],
                    1,
                    NULL,
                    NULL,
                    "%ld",
                    ImGuiSliderFlags_AlwaysClamp);
            });
        break;
    case ParameterType::PARAMETER_DOUBLE_ARRAY:
        drawArrayValue<double>(
            parameter,
            parameter.value->double_array_value,
            0.0f,
            [&parameter](float draw_width, size_t i)
            {
                ImGui::SetCursorPosX(ImGui::GetCursorPosX() + draw_width / 2);
                ImGui::DragScalar(
                    ("##v" + parameter.name + std::to_string(i)).c_str(),
                    ImGuiDataType_Double,
                    &parameter.value->double_array_value[i],
                    1.0f,
                    NULL,
                    NULL,
                    "%.3f",
                    ImGuiSliderFlags_AlwaysClamp);
            });
        break;
    case ParameterType::PARAMETER_STRING_ARRAY:
        drawArrayValue<std::string>(
            parameter,
            parameter.value->string_array_value,
            "",
            [&parameter](float draw_width, size_t i)
            {
                ImGui::PushStyleVar(
                    ImGuiStyleVar_FramePadding,
                    ImVec2(
                        (ImGui::CalcItemWidth() -
                         ImGui::CalcTextSize(parameter.value->string_array_value[i].c_str()).x) /
                            2,
                        ImGui::GetStyle().FramePadding.y));
                ImGui::SetCursorPosX(ImGui::GetCursorPosX() + draw_width / 2);
                ImGui::InputText(
                    ("##v" + parameter.name + std::to_string(i)).c_str(),
                    &parameter.value->string_array_value[i]);
                ImGui::PopStyleVar();
            });
        break;
    case ParameterType::PARAMETER_BOOL_ARRAY:
        drawArrayValue<bool>(
            parameter,
            parameter.value->bool_array_value,
            false,
            [&parameter](float draw_width, size_t i)
            {
                ImGui::SetCursorPosX(ImGui::GetCursorPosX() + draw_width / 2);
                const char *items[] = {"False", "True"};
                int current_item = parameter.value->bool_array_value[i];
                ImGui::PushStyleVar(
                    ImGuiStyleVar_FramePadding,
                    ImVec2(
                        (ImGui::CalcItemWidth() - ImGui::CalcTextSize(items[current_item]).x) / 2,
                        ImGui::GetStyle().FramePadding.y));
                if (ImGui::BeginCombo(
                        ("##v" + parameter.name + std::to_string(i)).c_str(),
                        items[current_item],
                        ImGuiComboFlags_NoArrowButton))
                {
                    for (int n = IM_ARRAYSIZE(items) - 1; n >= 0; n--)
                    {
                        bool is_selected = (current_item == n);
                        if (ImGui::Selectable(items[n], is_selected))
                        {
                            current_item = n;
                            parameter.value->bool_array_value[i] = n;
                        }
                        if (is_selected)
                        {
                            ImGui::SetItemDefaultFocus();
                        }
                    }
                    ImGui::EndCombo();
                }
                ImGui::PopStyleVar();
            });
        break;
    case ParameterType::PARAMETER_BYTE_ARRAY:
        drawArrayValue<uint8_t>(
            parameter,
            parameter.value->byte_array_value,
            0,
            [&parameter](float draw_width, size_t i)
            {
                ImGui::SetCursorPosX(ImGui::GetCursorPosX() + draw_width / 2);
                ImGui::DragScalar(
                    ("##v" + parameter.name + std::to_string(i)).c_str(),
                    ImGuiDataType_U8,
                    &parameter.value->byte_array_value[i],
                    1,
                    NULL,
                    NULL,
                    "%x",
                    ImGuiSliderFlags_AlwaysClamp);
            });
        break;
    }

    if (parameter.descriptor->read_only)
    {
        ImGui::EndDisabled();
    }
}

void ControlWidget::drawNumericValue(
    const std::string &from_val,
    const std::string &to_val,
    std::function<void()> draw_function)
{
    float draw_width = ImGui::GetContentRegionAvail().x - ImGui::CalcItemWidth();
    if (!from_val.empty() && !to_val.empty())
    {
        draw_width -= ImGui::CalcTextSize(from_val.c_str()).x;
        draw_width -= ImGui::CalcTextSize(to_val.c_str()).x;
        draw_width -= ImGui::GetStyle().ItemSpacing.x * 2;
    }
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + draw_width / 2);
    if (!from_val.empty())
    {
        ImGui::Text("%s", from_val.c_str());
        ImGui::SameLine();
    }
    draw_function();
    if (!to_val.empty())
    {
        ImGui::SameLine();
        ImGui::Text("%s", to_val.c_str());
    }
}

template <typename T>
void ControlWidget::drawArrayValue(
    const NodeParameter &parameter,
    std::vector<T> &array,
    T zero_value,
    std::function<void(float, size_t)> draw_function)
{
    float draw_width = ImGui::GetContentRegionAvail().x - ImGui::CalcItemWidth();
    if (array.size() == 0)
    {
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + draw_width / 2);
        std::string text = "Empty";
        ImGui::PushStyleVar(
            ImGuiStyleVar_FramePadding,
            ImVec2(
                (ImGui::CalcItemWidth() - ImGui::CalcTextSize(text.c_str()).x) / 2,
                ImGui::GetStyle().FramePadding.y));
        ImGui::InputText(("##v" + parameter.name).c_str(), &text, ImGuiInputTextFlags_ReadOnly);
        ImGui::PopStyleVar();
        if (ImGui::BeginPopupContextItem(("##v" + parameter.name + std::to_string(0)).c_str()))
        {
            if (ImGui::Selectable("Append"))
            {
                array.push_back(zero_value);
            }
            ImGui::EndPopup();
        }
    }
    else
    {
        for (size_t i = 0; i < array.size(); i++)
        {
            draw_function(draw_width, i);
            if (ImGui::BeginPopupContextItem(("##v" + parameter.name + std::to_string(i)).c_str()))
            {
                if (ImGui::Selectable("Append"))
                {
                    array.insert(array.begin() + i + 1, zero_value);
                }
                else if (ImGui::Selectable("Delete"))
                {
                    array.erase(array.begin() + i);
                }
                else if (ImGui::Selectable("Delete All"))
                {
                    array.clear();
                }
                ImGui::EndPopup();
            }
        }
    }
}

void ControlWidget::sortTable()
{
    if (ImGuiTableSortSpecs *sort_specs = ImGui::TableGetSortSpecs())
    {
        if (sort_specs->SpecsDirty && parameters.size() > 1)
        {
            std::sort(
                parameters.begin(),
                parameters.end(),
                [sort_specs](const NodeParameter &a, const NodeParameter &b)
                { return NodeParameter::compare(sort_specs, a, b); });
        }
        sort_specs->SpecsDirty = false;
    }
}

std::string ControlWidget::trimZeros(const std::string &str)
{
    std::string trimmed = str;
    trimmed.erase(str.find_last_not_of('0') + 1, std::string::npos);
    if (trimmed.back() == '.')
    {
        trimmed += "0";
    }
    return trimmed;
}

void ControlWidget::getParameters()
{
    if (!list_parameters_client_->wait_for_service(std::chrono::milliseconds(100)))
    {
        RCLCPP_WARN(
            gui_node->get_logger(),
            "Service %s is not available.",
            list_parameters_client_->get_service_name());

        // Try again in a second
        if (timer)
        {
            timer->reset();
        }
        else
        {
            timer = gui_node->create_wall_timer(
                std::chrono::milliseconds(1000),
                std::bind(&ControlWidget::getParameters, this));
        }
        return;
    }
    if (timer)
    {
        timer->cancel();
        timer.reset();
    }

    ListParameters::Request::SharedPtr request = std::make_shared<ListParameters::Request>();
    list_parameters_client_->async_send_request(
        request,
        [this](rclcpp::Client<ListParameters>::SharedFuture future)
        {
            if (future.valid())
            {
                ListParameters::Response::SharedPtr response = future.get();
                this->parameters_mutex.lock();
                this->parameters.clear();
                this->first_draw = true;
                NodeParameter node_parameter = NodeParameter();
                for (const std::string &parameter_name : response->result.names)
                {
                    if (parameter_name.find("qos_overrides.") == std::string::npos)
                    {
                        node_parameter.name = parameter_name;
                        parameters.push_back(node_parameter);
                    }
                }
                this->parameters_mutex.unlock();
                this->getParametersValue();
                this->getParametersDescriptors();
            }
        });
}

void ControlWidget::getParametersValue()
{
    GetParameters::Request::SharedPtr request = std::make_shared<GetParameters::Request>();
    parameters_mutex.lock();
    for (const NodeParameter &parameter : parameters)
    {
        request->names.push_back(parameter.name);
    }
    parameters_mutex.unlock();
    get_parameters_client_->async_send_request(
        request,
        [this](rclcpp::Client<GetParameters>::SharedFuture future)
        {
            if (future.valid())
            {
                GetParameters::Response::SharedPtr response = future.get();
                this->parameters_mutex.lock();
                for (size_t i = 0; i < parameters.size(); i++)
                {
                    parameters.at(i).value = std::make_shared<ParameterValue>(response->values.at(i));
                    parameters.at(i).curr_value = std::make_shared<ParameterValue>(response->values.at(i));
                    parameters.at(i).type = parameter_types[parameters.at(i).value->type];
                }
                this->parameters_mutex.unlock();
            }
        });
}

void ControlWidget::getParametersDescriptors()
{
    DescribeParameters::Request::SharedPtr request = std::make_shared<DescribeParameters::Request>();
    parameters_mutex.lock();
    for (const NodeParameter &parameter : parameters)
    {
        request->names.push_back(parameter.name);
    }
    parameters_mutex.unlock();
    describe_parameters_client_->async_send_request(
        request,
        [this](rclcpp::Client<DescribeParameters>::SharedFuture future)
        {
            if (future.valid())
            {
                DescribeParameters::Response::SharedPtr response = future.get();
                this->parameters_mutex.lock();
                for (size_t i = 0; i < parameters.size(); i++)
                {
                    parameters.at(i).descriptor = std::make_shared<ParameterDescriptor>(response->descriptors.at(i));
                }
                this->parameters_mutex.unlock();
            }
        });
}

bool NodeParameter::compare(ImGuiTableSortSpecs *sort_specs, const NodeParameter &a, const NodeParameter &b)
{
    for (int n = 0; n < sort_specs->SpecsCount; n++)
    {
        const ImGuiTableColumnSortSpecs *spec = &sort_specs->Specs[n];
        int delta = 0;
        switch (spec->ColumnIndex)
        {
        case 0:
            delta = strcmp(a.name.c_str(), b.name.c_str());
            break;
        case 1:
            delta = strcmp(a.type.c_str(), b.type.c_str());
            break;
        }
        if (delta > 0)
        {
            return spec->SortDirection != ImGuiSortDirection_Ascending;
        }
        else if (delta < 0)
        {
            return spec->SortDirection != ImGuiSortDirection_Descending;
        }
    }
    return false;
}

} // namespace gui_node
