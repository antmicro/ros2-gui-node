/*
 * Copyright (c) 2022-2023 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <functional>
#include <memory>
#include <string>

#include "gui_node/gui_engine.hpp"
#include "gui_node/gui_node.hpp"

namespace gui_node
{

class GuiNode; ///< Forward declaration

/**
 * Base class for all GUI widgets.
 */
class Widget
{
protected:
    std::shared_ptr<GuiNode> gui_node; ///< The GUI node that this widget is attached to
    std::string window_name;           ///< The name of the widget's window
    std::string ros_data_name;         ///< The name of the ROS data that this widget is displaying

public:
    /**
     * Constructor.
     *
     * @param gui_node The GUI node that this widget is attached to.
     * @param window_name The name of the widget's window.
     * @param ros_data_name The name of the ROS data that this widget is displaying.
     */
    Widget(std::shared_ptr<GuiNode> gui_node, std::string window_name, std::string ros_data_name)
        : gui_node(gui_node), window_name(window_name), ros_data_name(ros_data_name)
    {
    }

    /**
     * Destructor.
     */
    virtual ~Widget() = 0;

    /**
     * Virtual function that is called to draw the widget using the ImGui API.
     *
     * @return bool True if the widget is still was drew successfully, false otherwise.
     */
    virtual bool draw() = 0;
};

} // namespace gui_node
