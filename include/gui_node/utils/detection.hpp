/*
 * Copyright (c) 2022-2024 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <imgui.h>

namespace gui_node
{

/**
 * Object structure.
 */
struct Object
{
    std::string label; ///< Label of the object
    float score;       ///< Score of the object
    ImColor color;     ///< Color of the object

    /**
     * Constructor.
     *
     * @param label Label of the object.
     * @param score Score of the object.
     * @param color Color of the object.
     */
    Object(std::string label, float score, ImColor color) : label(label), score(score), color(color) {}
};

/**
 * Bounding box structure.
 */
struct BoundingBox
{
    float xmin;    ///< Minimum x coordinate coefficient (0-1)
    float ymin;    ///< Minimum y coordinate coefficient (0-1)
    float xmax;    ///< Maximum x coordinate coefficient (0-1)
    float ymax;    ///< Maximum y coordinate coefficient (0-1)
    Object object; ///< Bounding Box object

    /**
     * Constructor.
     *
     * @param xmin Minimum x coordinate coefficient.
     * @param ymin Minimum y coordinate coefficient.
     * @param xmax Maximum x coordinate coefficient.
     * @param ymax Maximum y coordinate coefficient.
     * @param color Color of the bounding box.
     * @param label Label of the bounding box.
     * @param score Score of the bounding box.
     */
    BoundingBox(float xmin, float ymin, float xmax, float ymax, std::string label, float score, ImColor color)
        : xmin(xmin), ymin(ymin), xmax(xmax), ymax(ymax), object(Object(label, score, color))
    {
    }
};

} // namespace gui_node
