#pragma once
/*********************************************************************
 * Software License Agreement (MIT License)
 * Author: Younes Reda
 *********************************************************************/

#include <opencv2/opencv.hpp>
#include <hallway_planner/geometry_primitives.hpp>

namespace hallway_planner::hallway_locator
{

struct HallwayLocatorConfig
{
    uint8_t wall_cost = 253;
    uint32_t min_hallway_length = 200;
    struct
    {
        double threshold1 = 600;
        double threshold2 = 1000;
        int aperture = 3;
        bool l2grad = false;
    } canny;
    struct
    {
        double rho_res = 1;
        double theta_res = M_PI/180;
        int threshold = 80;
        double min_line_length = 500;
        double max_line_length = 700;
        double rho_tolerance = 20;
        double theta_tolerance = geometry_primitives::deg_to_rad(3);
    } hough;
};

std::optional<cv::Mat> locate_hallways(const cv::Mat &map, const HallwayLocatorConfig &config);

}

