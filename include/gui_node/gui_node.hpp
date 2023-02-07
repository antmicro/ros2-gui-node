#ifndef COMPOSITION__GUI_NODE_COMPONENT_HPP_
#define COMPOSITION__GUI_NODE_COMPONENT_HPP_

#pragma once

#include <exception>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>

namespace gui_node
{

/**
 * An exception class thrown by GuiNode when an error with Ros*Data occurs.
 */
class RosDataException : public std::exception
{
private:
    const std::string message; ///< The error message
public:
    /**
     * Constructor.
     *
     * @param message The error message.
     */
    RosDataException(const std::string &message) : message(message) {}

    /**
     * Returns the error message.
     *
     * @return The error message.
     */
    virtual const char *what() const throw() { return message.c_str(); }
};

// Forward declaration
class GuiNode;

/**
 * A class that holds the data for a single ROS2 node element.
 */
class RosData
{
private:
    std::shared_ptr<GuiNode> node; ///< The node that owns this data.
public:
    /**
     * Constructor.
     *
     * @param node The GuiNode object that owns this data.
     */
    RosData(std::shared_ptr<GuiNode> node) : node(node) {}

    /**
     * Casts the object to a Ros*Data object.
     *
     * @tparam T The type of the object to cast to, must be a subclass of RosData.
     * @return The object cast to the specified type.
     */
    template <class T> T &as() { return *dynamic_cast<T *>(this); }

    virtual ~RosData() {}
};

/**
 * Main class for GUI node.
 * Allows to create, manage and vizualize ROS2 topic publishers/subscribers
 * and service servers/clients using user defined data types and callback functions.
 */
class GuiNode : public rclcpp::Node
{
public:
    /**
     * Constructor
     *
     * @param options Node options
     * @param node_name Name of the node
     */
    GuiNode(const rclcpp::NodeOptions &options, const std::string node_name);

    /**
     * Get the RosData node by name
     *
     * @param node_name Name of the node
     * @throws RosDataException if the node with the name doesn't exist
     * @return shared_ptr<RosData> The shared pointer to the RosData node
     */
    std::shared_ptr<RosData> &getRosData(const std::string &node_name);

    /**
     * Adds a RosData node to the nodes map
     *
     * @param node_name Name of the node
     * @param ros_data RosData node
     * @throws RosDataException if the node with the name already exists
     */
    void addRosData(const std::string &node_name, std::shared_ptr<RosData> ros_data);

private:
    std::unordered_map<std::string, std::shared_ptr<RosData>> ros_data_map; ///< Map of RosData nodes
};
} // namespace gui_node

#endif // COMPOSITION__GUI_NODE_COMPONENT_HPP_
