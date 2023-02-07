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
        throw RosDataException("No such node named: " + node_name);
    }
    return ros_data_map.at(node_name);
}

void GuiNode::addRosData(const std::string &node_name, std::shared_ptr<RosData> ros_data)
{
    if (ros_data_map.find(node_name) != ros_data_map.end())
    {
        throw RosDataException("Node with given name already exists");
    }
    ros_data_map.emplace(node_name, ros_data);
}

} // namespace gui_node

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gui_node::GuiNode)
