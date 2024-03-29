/*
 * Copyright (c) 2022-2024 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <memory>
#include <string>

#include "gui_node/gui_node.hpp"
#include "gui_node/ros_data/ros_client_data.hpp"
#include "gui_node/widget/widget.hpp"

namespace gui_node
{

class CounterWidget : public Widget
{
private:
    int counter = 0; ///< Counter value

public:
    /**
     * Constructor.
     *
     * @param gui_node Pointer to the GuiNode object that owns this widget.
     * @param window_name Name of the widget's window.
     * @param ros_data_name Name of the ROS data object that widget is associated with.
     */
    CounterWidget(std::shared_ptr<GuiNode> gui_node, const std::string &window_name, const std::string &ros_data_name)
        : Widget(gui_node, window_name, ros_data_name)
    {
    }

    /**
     * Draw the widget.
     *
     * @return bool indicating if the widget is still active.
     */
    bool draw() override;
};

} // namespace gui_node
