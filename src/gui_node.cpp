/*
 * Copyright (c) 2022-2024 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <chrono>
#include <imgui_impl_glfw.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "gui_node/gui_node.hpp"

namespace gui_node
{

GuiNode::GuiNode(const rclcpp::NodeOptions &options, const std::string node_name = "gui_node")
    : Node(node_name, options)
{
}

std::shared_ptr<RosData> &GuiNode::getRosData(const std::string &node_name)
{
    if (ros_data_map.find(node_name) == ros_data_map.end())
    {
        RCLCPP_FATAL(this->get_logger(), "RosData %s not found", node_name.c_str());
        throw RosDataException("No such RosData named: " + node_name);
    }
    return ros_data_map.at(node_name);
}

void GuiNode::addRosData(const std::string &node_name, std::shared_ptr<RosData> ros_data)
{
    if (ros_data_map.find(node_name) != ros_data_map.end())
    {
        RCLCPP_FATAL(this->get_logger(), "RosData %s already exists", node_name.c_str());
        throw RosDataException("RosData with given name already exists");
    }
    ros_data_map.emplace(node_name, ros_data);
}

void GuiNode::addWidget(const std::string &widget_name, std::shared_ptr<Widget> widget)
{
    if (widget_map.find(widget_name) != widget_map.end())
    {
        RCLCPP_FATAL(this->get_logger(), "Widget %s already exists", widget_name.c_str());
        throw std::runtime_error("Widget with given name already exists");
    }
    widget_map.emplace(widget_name, widget);
}

bool GuiNode::prepare(const std::string &application_name, bool maximize_window)
{
    gui_engine = std::make_shared<GuiEngine>(application_name, shared_from_this());
    bool status = gui_engine->init(maximize_window);
    if (status)
    {
        if (!this->render())
        {
            RCLCPP_FATAL(get_logger(), "Failed to render");
            return false;
        }
        timer = this->create_wall_timer(
            std::chrono::milliseconds(10),
            [this]() -> void
            {
                if (rclcpp::ok())
                {
                    if (!this->render())
                    {
                        timer->cancel();
                        rclcpp::shutdown();
                    }
                }
                else
                {
                    timer->cancel();
                }
            });
    }
    else
    {
        const std::string error_msg = "Failed to initialize GuiEngine";
        RCLCPP_FATAL(get_logger(), error_msg.c_str());
        timer->cancel();
        rclcpp::shutdown(nullptr, error_msg);
    }
    return status;
}

bool GuiNode::render()
{
    if (!gui_engine)
    {
        RCLCPP_FATAL(get_logger(), "GuiEngine not initialized");
        return false;
    }
    else if (!glfwWindowShouldClose(gui_engine->getWindow()))
    {
        glfwPollEvents();
        ImGui_ImplVulkan_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        ImVec2 pos = ImGui::GetCursorScreenPos();
        const int title_bar_size = ImGui::GetStyle().FramePadding.y * 2 + ImGui::GetFontSize();
        for (auto &widget : widget_map)
        {
            ImGui::SetNextWindowPos(pos, ImGuiCond_FirstUseEver);
            if (!widget.second->draw())
            {
                return false;
            }
            pos.x += title_bar_size;
            pos.y += title_bar_size;
        }
        ImGui::Render();
        return gui_engine->draw();
    }
    return false;
}

std::shared_ptr<GuiEngine> GuiNode::getGuiEngine()
{
    if (!gui_engine)
    {
        RCLCPP_FATAL(get_logger(), "GuiEngine not initialized");
        throw std::runtime_error("GuiEngine not initialized");
    }
    return gui_engine;
}

} // namespace gui_node
