#pragma once
/*********************************************************************
 * Software License Agreement (MIT License)
 * Author: Younes Reda
 *********************************************************************/

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <hallway_planner/geometry_primitives.hpp>
#include <opencv2/opencv.hpp>

namespace hallway_planner::hallway_locator {

struct HallwayLocatorConfig {
  uint8_t wall_cost = costmap_2d::LETHAL_OBSTACLE;
  uint32_t min_hallway_length = 200;
  struct {
    double threshold1 = 1000;
    double threshold2 = 1400;
    int aperture = 3;
    bool l2grad = false;
  } canny;
  struct {
    double rho_res = 1;
    double theta_res = geometry_primitives::deg_to_rad(1);
    int threshold = 80;
    double min_line_length = 500;
    double max_line_length = 700;
    double rho_tolerance = 20;
    double theta_tolerance = geometry_primitives::deg_to_rad(3);
  } hough;
  static constexpr uint8_t NO_HALLWAY = static_cast<uint8_t>(false);
  static constexpr uint8_t HALLWAY = static_cast<uint8_t>(255);
};

std::optional<cv::Mat> locate_hallways(const cv::Mat &costmap,
                                       const HallwayLocatorConfig &config);

std::optional<cv::Mat> locate_hallways(const costmap_2d::Costmap2DROS *costmap,
                                       const HallwayLocatorConfig &config);

}  // namespace hallway_planner::hallway_locator
