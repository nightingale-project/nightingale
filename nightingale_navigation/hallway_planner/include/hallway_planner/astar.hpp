#pragma once
/*********************************************************************
 * Software License Agreement (MIT License)
 * Author: Younes Reda
 *********************************************************************/

#include <hallway_planner/geometry_primitives.hpp>
#include <opencv2/opencv.hpp>
#include <optional>

namespace hallway_planner::astar
{

/* An A* grid optimal solver using the euclidean distance as heuristic */
std::optional<geometry_primitives::PixelPath> solve(const cv::Mat &map, const geometry_primitives::Point &start, const geometry_primitives::Point &goal, const uint8_t lethal_cost, const uint32_t tolerance);

/* Compute the cost of a given path */
uint32_t compute_cost(const cv::Mat &map, const geometry_primitives::PixelPath &path);

}

