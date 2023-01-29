/*********************************************************************
 * Software License Agreement (MIT License)
 * Author: Younes Reda
 *********************************************************************/

#include <hallway_planner/hallway_planner.hpp>
#include <hallway_planner/astar.hpp>
#include <hallway_planner/hallway_locator.hpp>
#include <opencv2/opencv.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

PLUGINLIB_EXPORT_CLASS(hallway_planner::HallwayPlanner, nav_core::BaseGlobalPlanner)

namespace hallway_planner
{

HallwayPlanner::HallwayPlanner() : name_(), costmap_ros_(nullptr), initialized_(false),
                                   costmap_cv_(), hallway_map_(), tolerance_(DEFAULT_TOLERANCE),
                                   lethal_cost_(DEFAULT_LETHAL_COST),
                                   orientation_filter_window_(DEFAULT_ORIENTATION_FILTER_WINDOW)
{}

void HallwayPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (initialized_) {
        this->warn_already_initialized();
        return;
    }
    if (costmap_ros_ == nullptr) {
        ROS_ERROR("HallwayPlanner::initialize called with null costmap. Initialization rejected.");
        return;
    }
    initialized_ = true;
    name_ = name;

    costmap_ros_ = costmap_ros;
    const auto rows = costmap_ros_->getCostmap()->getSizeInCellsY();
    const auto cols = costmap_ros_->getCostmap()->getSizeInCellsX();
    costmap_cv_.create(rows,cols,CV_8UC1);
    std::memcpy(costmap_cv_.data,costmap_ros_->getCostmap()->getCharMap(),rows*cols);

    ros::NodeHandle private_nh("~/" + name);

    int lethal = DEFAULT_LETHAL_COST;
    private_nh.param<int>("lethal_cost", lethal, DEFAULT_LETHAL_COST);
    lethal_cost_ = static_cast<uint8_t>(lethal);
    private_nh.param<double>("tolerance", tolerance_, DEFAULT_TOLERANCE);
    int window = DEFAULT_ORIENTATION_FILTER_WINDOW;
    private_nh.param<int>("orientation_filter_window", window, DEFAULT_ORIENTATION_FILTER_WINDOW);
    orientation_filter_window_ = window;

    // defaults are initialized
    hallway_locator::HallwayLocatorConfig hallway_config;
    private_nh.param<double>("canny_threshold1", hallway_config.canny.threshold1, hallway_config.canny.threshold1);
    private_nh.param<double>("canny_threshold2", hallway_config.canny.threshold2, hallway_config.canny.threshold2);                                                                                        
    private_nh.param<int>("canny_aperture", hallway_config.canny.aperture, hallway_config.canny.aperture);
    private_nh.param<bool>("canny_l2grad", hallway_config.canny.l2grad, hallway_config.canny.l2grad);
    private_nh.param<double>("hough_rho_res", hallway_config.hough.rho_res, hallway_config.hough.rho_res);
    private_nh.param<double>("hough_theta_res", hallway_config.hough.theta_res, hallway_config.hough.theta_res);
    private_nh.param<int>("hough_threshold", hallway_config.hough.threshold, hallway_config.hough.threshold);
    private_nh.param<double>("hough_min_line_length", hallway_config.hough.min_line_length, hallway_config.hough.min_line_length);
    private_nh.param<double>("hough_max_line_length", hallway_config.hough.max_line_length, hallway_config.hough.max_line_length);
    private_nh.param<double>("hough_rho_tolerance", hallway_config.hough.rho_tolerance, hallway_config.hough.rho_tolerance);
    private_nh.param<double>("hough_theta_tolerance", hallway_config.hough.theta_tolerance, hallway_config.hough.theta_tolerance);

    int wall_cost_tmp;
    private_nh.param<int>("wall_cost", wall_cost_tmp, hallway_config.wall_cost);
    hallway_config.wall_cost = static_cast<uint8_t>(wall_cost_tmp);
    int min_hallway_length_tmp;
    private_nh.param<int>("min_hallway_length", min_hallway_length_tmp, hallway_config.min_hallway_length);
    hallway_config.min_hallway_length = static_cast<uint32_t>(min_hallway_length_tmp);

    auto maybe_hallway_map = hallway_locator::locate_hallways(costmap_cv_,hallway_config);
    if (!maybe_hallway_map) {
        ROS_ERROR("HallwayPlanner::initialize Could not detect hallways. Initialization rejected.");
        return;
    }
    hallway_map_ = std::move(maybe_hallway_map.value());
}

void HallwayPlanner::warn_uninitialized() const
{
    ROS_WARN("HallwayPlanner is uninitialized");
}


void HallwayPlanner::warn_already_initialized() const
{
    ROS_WARN_STREAM("HallwayPlanner named " << name_  << " is alreayd initialized");
}

bool HallwayPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                              const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
    double cost = -1.0;
    return this->makePlan(start,goal,plan,cost);
}

bool HallwayPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                              const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan,
                              double &cost)
{
    std::scoped_lock<std::mutex> lock(mutex_);
    if (!initialized_) {
        this->warn_uninitialized();
        return false;
    }
    if (!costmap_ros_->isCurrent()) {
        ROS_ERROR("HallwayPlanner::makePlan called with not current costmap");
        return false;
    }
    plan.clear();
    const auto maybe_start_pixel_frame = this->map_frame_to_pixel_frame(start.pose);
    const auto maybe_goal_pixel_frame = this->map_frame_to_pixel_frame(goal.pose);
    if (!maybe_start_pixel_frame || !maybe_goal_pixel_frame) {
        ROS_ERROR("HallwayPlanner::makePlan called with invalid start and goal poses");
        return false;
    }
    const auto start_pixel_frame = maybe_start_pixel_frame.value();
    const auto goal_pixel_frame = maybe_goal_pixel_frame.value();
    const uint32_t pixel_tolerance = std::round(tolerance_/costmap_ros_->getCostmap()->getResolution());
    const auto maybe_pixel_path = astar::solve(costmap_cv_,start_pixel_frame,goal_pixel_frame,lethal_cost_,pixel_tolerance);
    if (!maybe_pixel_path)
    {
        ROS_ERROR("HallwayPlanner::makePlan failed to find a plan");
        return false;
    }
    const auto &pixel_path = maybe_pixel_path.value();
    cost = hallway_planner::astar::compute_cost(costmap_cv_, pixel_path);
    geometry_primitives::Path path;
    path.reserve(pixel_path.size());
    // convert the path from pixel to world frame
    std::transform(pixel_path.cbegin(), pixel_path.cend(), std::back_inserter(path), [this](const auto &point){return pixel_frame_to_map_frame(point);});
    this->fill_orientations(path,orientation_filter_window_);
    return false;
}

std::optional<geometry_primitives::Point> HallwayPlanner::map_frame_to_pixel_frame(const geometry_msgs::Pose& pose) const
{
    uint32_t x = 0.0, y = 0.0;
    if (costmap_ros_->getCostmap()->worldToMap(pose.position.x,pose.position.y,x,y))
    {
        return {geometry_primitives::Point(x,y)};
    }
    else
    {
        return std::nullopt;
    }
}

geometry_msgs::Pose HallwayPlanner::pixel_frame_to_map_frame(const geometry_primitives::Point &p) const
{
    geometry_msgs::Pose pose;
    costmap_ros_->getCostmap()->mapToWorld(p.x(), p.y(), pose.position.x, pose.position.y); 
    return pose;
}

void HallwayPlanner::fill_orientations(geometry_primitives::Path &path, const uint32_t window) const
{
    for (size_t i = 0; i < path.size(); ++i)
    {
        const size_t forward = std::min(path.size()-1, i+window);
        const size_t backward = std::max(static_cast<size_t>(0), i-window);
        const auto x0 = path.at(backward).position.x;
        const auto y0 = path.at(backward).position.y;
        const auto x1 = path.at(forward).position.x;
        const auto y1 = path.at(forward).position.y;
        const double yaw = atan2(y1-y0,x1-x0);
        tf2::Quaternion quat;
        quat.setRPY(0,0,yaw);
        quat = quat.normalize();
        path.at(i).orientation = tf2::toMsg(quat);
    }
}

}

