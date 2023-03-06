/*********************************************************************
 * Software License Agreement (MIT License)
 * Author: Younes Reda
 *********************************************************************/
#include <hallway_planner/hallway_planner.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hallway_planner::HallwayPlanner,
                       nav_core::BaseGlobalPlanner)

namespace hallway_planner {
HallwayPlanner::HallwayPlanner()
    : global_planner::GlobalPlanner(),
      hallway_map_(),
      biased_plan_pub_(),
      distance_stop_biasing_(1.5){};

bool HallwayPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                              const geometry_msgs::PoseStamped &goal,
                              std::vector<geometry_msgs::PoseStamped> &plan) {
  if (global_planner::GlobalPlanner::makePlan(start, goal, plan)) {
    if (plan.empty()) {
      ROS_ERROR("HallwayPlanner could not makeplan");
      return false;
    }
    std::vector<geometry_msgs::PoseStamped> biased_plan = plan;
    bias_path(biased_plan);
    nav_msgs::Path gui_path;
    gui_path.header.frame_id = biased_plan.at(0).header.frame_id;
    gui_path.header.stamp = ros::Time::now();
    gui_path.poses = biased_plan;
    biased_plan_pub_.publish(gui_path);
    return true;
  }
  return false;
}

bool HallwayPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                              const geometry_msgs::PoseStamped &goal,
                              std::vector<geometry_msgs::PoseStamped> &plan,
                              double &cost) {
  cost = -1;
  if (hallway_planner::HallwayPlanner::makePlan(start, goal, plan)) {
    cost = plan.size();
    return true;
  }
  return false;
}

void HallwayPlanner::initialize(std::string name,
                                costmap_2d::Costmap2DROS *costmap_ros) {
  global_planner::GlobalPlanner::initialize(name, costmap_ros);
  ros::NodeHandle private_nh("~/" + name);
  biased_plan_pub_ = private_nh.advertise<nav_msgs::Path>("biased_plan", 1);
  const auto config = [&name]() {
    ros::NodeHandle private_nh("~/" + name);
    hallway_locator::HallwayLocatorConfig hallway_config;
    private_nh.param<double>("canny_threshold1",
                             hallway_config.canny.threshold1,
                             hallway_config.canny.threshold1);
    private_nh.param<double>("canny_threshold2",
                             hallway_config.canny.threshold2,
                             hallway_config.canny.threshold2);
    private_nh.param<int>("canny_aperture", hallway_config.canny.aperture,
                          hallway_config.canny.aperture);
    private_nh.param<bool>("canny_l2grad", hallway_config.canny.l2grad,
                           hallway_config.canny.l2grad);
    private_nh.param<double>("hough_rho_res", hallway_config.hough.rho_res,
                             hallway_config.hough.rho_res);
    private_nh.param<double>("hough_theta_res", hallway_config.hough.theta_res,
                             hallway_config.hough.theta_res);
    private_nh.param<int>("hough_threshold", hallway_config.hough.threshold,
                          hallway_config.hough.threshold);
    private_nh.param<double>("hough_min_line_length",
                             hallway_config.hough.min_line_length,
                             hallway_config.hough.min_line_length);
    private_nh.param<double>("hough_max_line_length",
                             hallway_config.hough.max_line_length,
                             hallway_config.hough.max_line_length);
    private_nh.param<double>("hough_rho_tolerance",
                             hallway_config.hough.rho_tolerance,
                             hallway_config.hough.rho_tolerance);
    private_nh.param<double>("hough_theta_tolerance",
                             hallway_config.hough.theta_tolerance,
                             hallway_config.hough.theta_tolerance);
    hallway_config.wall_cost = costmap_2d::LETHAL_OBSTACLE;
    int min_hallway_length_tmp;
    private_nh.param<int>("min_hallway_length", min_hallway_length_tmp,
                          hallway_config.min_hallway_length);
    hallway_config.min_hallway_length =
        static_cast<uint32_t>(min_hallway_length_tmp);
    return hallway_config;
  }();
  const auto maybe_hallway_map =
      hallway_locator::locate_hallways(costmap_ros, config);
  if (!maybe_hallway_map) {
    ROS_ERROR(
        "HallwayPlanner::initialize Could not detect hallways. Ignoring "
        "Hallway Biasing");
    const auto rows = costmap_ros->getCostmap()->getSizeInCellsY();
    const auto cols = costmap_ros->getCostmap()->getSizeInCellsX();
    hallway_map_ =
        cv::Mat(rows, cols, CV_8UC1,
                cv::Scalar(hallway_locator::HallwayLocatorConfig::NO_HALLWAY));
  } else {
    ROS_INFO("HallwayPlanner::initialize successfully found hallways");
    hallway_map_ = maybe_hallway_map.value();
  }
  private_nh.param<double>("distance_stop_biasing", distance_stop_biasing_,
                           distance_stop_biasing_);
}

// shift poses in the plan to the right if theyre part of a hallway
void HallwayPlanner::bias_path(
    std::vector<geometry_msgs::PoseStamped> &plan) const {
  constexpr auto max_alpha = 0.3;
  double alpha = 0.0;
  for (uint32_t i = 0; i < plan.size(); ++i) {
    auto &pose = plan.at(i).pose;
    if (!in_hallway(pose) || (geometry_primitives::euclidean_distance(
                                  pose.position, plan.back().pose.position) <
                              distance_stop_biasing_)) {
      continue;
    }
    alpha += costmap_->getResolution();
    alpha = std::min(max_alpha, alpha);
    // represent each pose as a SE2 matrix
    const auto iso = pose_to_isometry(pose);
    // the translation part of the homogenous transformation
    // is u_hat x z_hat where u_hat is the unit vector
    // of the pose. u_hat x z_hat gives a vector pointing to the right-side
    // of the robot.
    const auto translation = [&iso]() {
      const double yaw = Eigen::Rotation2Dd(iso.rotation()).angle();
      const Eigen::Vector3d u_hat(std::cos(yaw), std::sin(yaw), 0);
      ROS_ASSERT_MSG(is_close(u_hat.norm(), 1),
                     "The u_hat vector should have unit length");
      const auto t = u_hat.cross(Eigen::Vector3d::UnitZ());
      return Eigen::Vector2d(t[0], t[1]);
    }();
    ROS_ASSERT_MSG(is_close(translation.norm(), 1),
                   "The translation vector should have unit length");
    auto T = Eigen::Isometry2d::Identity();
    constexpr auto shifting_high_cost = 150;
    for (double delta = costmap_->getResolution(); delta <= alpha;
         delta += costmap_->getResolution()) {
      T.translation() = translation * delta;
      const auto translated_iso = T * iso;
      const auto temp = isometry_to_pose(translated_iso);
      uint32_t mx = 0, my = 0;
      if (!costmap_->worldToMap(temp.position.x, temp.position.y, mx, my)) {
        ROS_ERROR("HallwayPlanner world to map failed. Should investigate");
        break;
      }
      if (costmap_->getCost(mx, my) < shifting_high_cost) {
        pose = temp;
      } else {
        break;
      }
    }
  }
}

Eigen::Isometry2d HallwayPlanner::pose_to_isometry(
    const geometry_msgs::Pose &pose) const {
  auto iso = Eigen::Isometry2d::Identity();
  iso.translation() << pose.position.x, pose.position.y;
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                       pose.orientation.y, pose.orientation.z);
  // rotate by the yaw
  iso.rotate(Eigen::Rotation2Dd(q.toRotationMatrix().eulerAngles(0, 1, 2)[2]));
  return iso;
}

geometry_msgs::Pose HallwayPlanner::isometry_to_pose(
    const Eigen::Isometry2d &iso) const {
  geometry_msgs::Pose pose;
  pose.position.x = iso.translation().x();
  pose.position.y = iso.translation().y();
  pose.position.z = 0;
  const double yaw = Eigen::Rotation2Dd(iso.rotation()).angle();
  tf2::Quaternion quat;
  quat.setRPY(0, 0, yaw);
  pose.orientation = tf2::toMsg(quat);
  return pose;
}

bool HallwayPlanner::in_hallway(const geometry_msgs::Pose &pose) const {
  uint32_t mx = 0, my = 0;
  if (!costmap_->worldToMap(pose.position.x, pose.position.y, mx, my)) {
    ROS_ERROR(
        "HallwayPlanner failed to transform point from map to pixel frame");
    return false;
  }
  ROS_ASSERT(mx < hallway_map_.cols);
  ROS_ASSERT(my < hallway_map_.rows);
  return hallway_map_.at<uint8_t>(my, mx) ==
         hallway_locator::HallwayLocatorConfig::HALLWAY;
}

}  // namespace hallway_planner
