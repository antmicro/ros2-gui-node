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

void GuiNode::prepare(const std::string &application_name)
{
    gui_engine = std::make_shared<GuiEngine>(application_name, shared_from_this());
    gui_engine->init();
    timer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&GuiNode::render, this));
}

void GuiNode::render()
{
    if (!gui_engine)
    {
        RCLCPP_FATAL(get_logger(), "GuiEngine not initialized");
        throw std::runtime_error("GuiEngine not initialized");
    }
    if (!glfwWindowShouldClose(gui_engine->getWindow()))
    {
        glfwPollEvents();
        ImGui_ImplVulkan_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        for (auto &widget : widget_map)
        {
            widget.second->draw();
        }
        ImGui::Render();
        gui_engine->draw();
    }
    else
    {
        rclcpp::shutdown();
    }
}

} // namespace gui_node

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gui_node::GuiNode)
