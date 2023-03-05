#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Pose.h>
#include <global_planner/planner_core.h>
#include <ros/ros.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Geometry>
#include <hallway_planner/hallway_locator.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <vector>

using namespace std;
using namespace hallway_planner;
double distance_stop_biasing_ = 1.5;

bool in_hallway(geometry_msgs::Pose pose) { return true; }

Eigen::Isometry2d pose_to_isometry(const geometry_msgs::Pose &pose) {
  auto iso = Eigen::Isometry2d::Identity();
  iso.translation() << pose.position.x, pose.position.y;
  const tf2::Quaternion quat(pose.orientation.x, pose.orientation.y,
                             pose.orientation.z, pose.orientation.w);
  iso.rotate(Eigen::Rotation2Dd(quat.getAngle()));
  return iso;
}

geometry_msgs::Pose isometry_to_pose(const Eigen::Isometry2d &iso) {
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

constexpr bool is_close(const double a, const double b) {
  constexpr auto tolerance = 1e-6;
  return std::abs(a - b) < tolerance;
}

void bias_path(const geometry_msgs::PoseStamped &start,
               const geometry_msgs::PoseStamped &goal,
               std::vector<geometry_msgs::PoseStamped> &plan) {
  for (uint32_t i = 0; i < plan.size(); ++i) {
    auto &pose = plan.at(i).pose;
    if (!in_hallway(pose) ||
        (geometry_primitives::euclidean_distance(
             pose.position, start.pose.position) < distance_stop_biasing_) ||
        (geometry_primitives::euclidean_distance(
             pose.position, goal.pose.position) < distance_stop_biasing_)) {
      continue;
    }
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
    constexpr auto alpha = 1.5;
    T.translation() = translation * alpha;
    const auto translated_iso = T * iso;
    pose = isometry_to_pose(translated_iso);
  }
}

int main() {
  std::vector<geometry_msgs::PoseStamped> path;
  std::cout << "[";
  for (uint32_t i = 0; i < 1000; i++) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = i / 50.0;
    path.push_back(pose);
    std::cout << "(" << pose.pose.position.x << "," << pose.pose.position.y
              << "),";
  }
  std::cout << "]" << std::endl;
  std::cout << std::endl;
  bias_path(path[0], path[path.size() - 1], path);
  std::cout << "[" << std::endl;
  for (uint32_t i = 0; i < 1000; i++) {
    auto pose = path[i];
    std::cout << "(" << pose.pose.position.x << "," << pose.pose.position.y
              << "),";
  }
  std::cout << "]" << std::endl;
}