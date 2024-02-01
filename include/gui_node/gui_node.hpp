/*
 * Copyright (c) 2022-2023 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <exception>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>

#include "gui_node/widget/widget.hpp"

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

// Forward declaration
class Widget;

/**
 * A class that holds the data for a single ROS2 node element.
 */
class RosData : public std::enable_shared_from_this<RosData>
{
protected:
    std::shared_ptr<GuiNode> node; ///< The node that owns this data
    bool data_changed = false;     ///< Whether the data has changed since the last update
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
     * @return Cast RosData shared pointer to the specified type.
     */
    template <class T>
    std::shared_ptr<T> as()
    {
        return std::dynamic_pointer_cast<T>(shared_from_this());
    }

    /**
     * Returns whether the data has changed since the last call to the data getter method.
     *
     * @return True if the data has changed, false otherwise.
     */
    bool hasDataChanged() { return data_changed; }

    virtual ~RosData() {}
};

/**
 * Main class for GUI node.
 * Allows to create, manage and visualize ROS2 topic publishers/subscribers
 * and service servers/clients using user defined data types and callback functions.
 */
class GuiNode : public rclcpp::Node
{
private:
    std::unordered_map<std::string, std::shared_ptr<RosData>> ros_data_map; ///< Map of RosData nodes
    std::unordered_map<std::string, std::shared_ptr<Widget>> widget_map;    ///< Map of widgets
    std::shared_ptr<GuiEngine> gui_engine;                                  ///< The GUI engine
    rclcpp::TimerBase::SharedPtr timer;                                     ///< The timer used to update the GUI
public:
    /**
     * Constructor.
     *
     * @param options Node options.
     * @param node_name Name of the node.
     */
    GuiNode(const rclcpp::NodeOptions &options, const std::string node_name);

    /**
     * Get the RosData node by name.
     *
     * @param node_name Name of the node.
     * @return shared_ptr<RosData> The shared pointer to the RosData node.
     *
     * @throws RosDataException if the node with the name doesn't exist.
     */
    std::shared_ptr<RosData> &getRosData(const std::string &node_name);

    /**
     * Adds a RosData node to the nodes map.
     *
     * @param node_name Name of the node.
     * @param ros_data RosData node.
     *
     * @throws RosDataException if the node with the name already exists.
     */
    void addRosData(const std::string &node_name, std::shared_ptr<RosData> ros_data);

    /**
     * Adds a Wdiget to the widgets map.
     *
     * @param widget_name Name of the widget.
     * @param widget Shared pointer to the widget.
     *
     * @throws RosDataException if the widget with the name already exists.
     */
    void addWidget(const std::string &widget_name, std::shared_ptr<Widget> widget);

    /**
     * Prepares the GuiEngine for rendering.
     *
     * @param application_name Name of the application.
     * @param maximize_window Maximizes the window on startup.
     *
     * @return True if the GuiEngine was prepared, false otherwise.
     */
    bool prepare(const std::string &application_name, bool maximize_window = true);

    /**
     * Renders the frame using defined widgets.
     *
     * @return True if the frame was rendered, false otherwise.
     */
    bool render();

    /**
     * Get the GuiEngine.
     *
     * @return shared_ptr<GuiEngine> The shared pointer to the GuiEngine.
     *
     * @throws std::runtime_error if the GuiEngine is not initialized.
     */
    std::shared_ptr<GuiEngine> getGuiEngine();
};
} // namespace gui_node
