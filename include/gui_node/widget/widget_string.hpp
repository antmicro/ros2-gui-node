/*
 * Copyright (c) 2022-2023 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <string>

#include "gui_node/gui_node.hpp"
#include "gui_node/widget/widget.hpp"

namespace gui_node
{

/**
 * Class for string widgets.
 */
class StringWidget : public Widget
{
private:
    std::string data = ""; ///< Data to be displayed.

    /// Function to convert data from RosData to std::string.
    std::function<void(std::shared_ptr<GuiNode>, std::string &)> string_converter;

public:
    /*
     * Constructor.
     *
     * @param gui_node Pointer to the gui node.
     * @param window_name Name for the widget window.
     * @param ros_data_name Name for the ROS data.
     * @param string_converter Function to convert data from RosData to std::string.
     */
    StringWidget(
        std::shared_ptr<GuiNode> gui_node,
        const std::string &window_name,
        const std::string &ros_data_name,
        std::function<void(std::shared_ptr<GuiNode>, std::string &)> string_converter)
        : Widget(gui_node, window_name, ros_data_name), string_converter(string_converter)
    {
    }

    /**
     * Draw the widget.
     *
     * @return bool indicating if the widget was drew successfully.
     */
    bool draw() override;
};

} // namespace gui_node
