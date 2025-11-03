/*
 * Copyright (c) 2025 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "gui_node/widget/widget_depth.hpp"
namespace gui_node
{
template <class IterT>
void normalize_values(IterT begin, IterT end)
{
    auto min_value = *std::min_element(begin, end);
    auto max_value = *std::max_element(begin, end);

    for (auto itr = begin; itr != end; itr++)
    {
        *itr = (*itr - min_value) / (max_value - min_value);
    }
}
std::array<uint8_t, 3U>
apply_colormap(float normalized_value, DepthWidget::eColorMappings mapping = DepthWidget::eColorMappings::Magma)
{
    normalized_value = std::clamp(normalized_value, 0.0f, 1.0f);
    float r, g, b;
    switch (mapping)
    {
    case DepthWidget::eColorMappings::Rainbow:
    {
        float ratio = 2 * normalized_value;
        b = std::max(0.f, (1.f - ratio));
        r = std::max(0.f, (ratio - 1.f));
        g = std::max(1.f - b - r, 0.f);
    }
    break;
    case DepthWidget::eColorMappings::Magma:
    {
        float ratio = 2 * normalized_value;
        g = std::max(0.f, (ratio - 1.f));
        r = std::max(ratio - g, g);
        b = 0.f;
    }
    break;
    }
    uint8_t red = static_cast<uint8_t>(r * 255);
    uint8_t green = static_cast<uint8_t>(g * 255);
    uint8_t blue = static_cast<uint8_t>(b * 255);
    return {red, green, blue};
}

sensor_msgs::msg::Image
DepthWidget::prep_image(std::shared_ptr<GuiNode> gui_node_ptr, const std::vector<float> &values, int rows, int cols)
{
    auto image_msg = sensor_msgs::msg::Image();
    std::vector<float> normalized_values = values;
    normalize_values(normalized_values.begin(), normalized_values.end());

    image_msg.header.stamp = gui_node_ptr->get_clock()->now();
    image_msg.header.frame_id = "camera_frame";
    image_msg.height = rows;
    image_msg.width = cols;
    image_msg.encoding = "rgb8";
    image_msg.is_bigendian = false;
    image_msg.step = cols * 3;
    image_msg.data.resize(rows * cols * 3);
    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            float norm_val = normalized_values[i * cols + j];

            auto color = apply_colormap(1 - norm_val);

            int idx = (i * cols + j) * 3;
            image_msg.data[idx] = color[0];     // Red
            image_msg.data[idx + 1] = color[1]; // Green
            image_msg.data[idx + 2] = color[2]; // Blue
        }
    }

    return image_msg;
}
} // namespace gui_node
