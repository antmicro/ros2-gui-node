/*
 * Copyright (c) 2025 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <imgui.h>
#include <vector>

#include "gui_node/utils/detection.hpp"

namespace gui_node
{
/**
 * Point structure.
 */
struct Point
{
    float x; ///< X coordinate of the point.
    float y; ///< Y coordinate of the point.
    unsigned int id;

    /**
     * Constructor
     *
     * @param x X coordinate of the point.
     * @param y Y coordinate of the point.
     */
    Point(float x, float y, unsigned int id) : x(x), y(y), id(id) {}
};

/**
 * Pose structure.
 */
struct Pose
{
    std::vector<Point> points; ///< Pose points
    BoundingBox bbox;          ///< Pose bounding box

    Pose(const std::vector<Point> &points, const BoundingBox &bbox) : points(points), bbox(bbox) {}
};

} // namespace gui_node
