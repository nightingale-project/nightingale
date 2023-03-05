#pragma once
/*********************************************************************
 * Software License Agreement (MIT License)
 * Author: Younes Reda
 *********************************************************************/

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

namespace hallway_planner {

class HallwayPlanner : public global_planner::GlobalPlanner {
 public:
  /**
   * @brief  Default con/destructor for the PlannerCore object
   */
  HallwayPlanner();
  ~HallwayPlanner() = default;

  /**
   * @brief Copies
   */
  HallwayPlanner(const HallwayPlanner &) = default;
  HallwayPlanner &operator=(const HallwayPlanner &) = default;

  /**
   * @brief Moves
   */
  HallwayPlanner(HallwayPlanner &&) = default;
  HallwayPlanner &operator=(HallwayPlanner &&) = default;

  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose
   * @param goal The goal pose
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  bool makePlan(const geometry_msgs::PoseStamped &start,
                const geometry_msgs::PoseStamped &goal,
                std::vector<geometry_msgs::PoseStamped> &plan) override;

  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose
   * @param goal The goal pose
   * @param plan The plan... filled by the planner
   * @param cost The plans calculated cost
   * @return True if a valid plan was found, false otherwise
   */
  bool makePlan(const geometry_msgs::PoseStamped &start,
                const geometry_msgs::PoseStamped &goal,
                std::vector<geometry_msgs::PoseStamped> &plan,
                double &cost) override;

  /**
   * @brief  Initialization function for the BaseGlobalPlanner
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for
   * planning
   */
  void initialize(std::string name,
                  costmap_2d::Costmap2DROS *costmap_ros) override;

  // TODO: GlobalPlanner defines additional functions beyond what the base
  // nav_core class defines. Consider overriding them too. Not necessary for
  // move_base.
 private:
  cv::Mat hallway_map_;
  ros::Publisher biased_plan_pub_;
  double distance_stop_biasing_;

 protected:
  // shift poses in the plan to the right if theyre part of a hallway
  void bias_path(std::vector<geometry_msgs::PoseStamped> &plan) const;

  Eigen::Isometry2d pose_to_isometry(const geometry_msgs::Pose &pose) const;

  geometry_msgs::Pose isometry_to_pose(const Eigen::Isometry2d &iso) const;

  bool in_hallway(const geometry_msgs::Pose &pose) const;

  static constexpr bool is_close(const double a, const double b) {
    constexpr auto tolerance = 1e-6;
    return std::abs(a - b) < tolerance;
  }
};

}  // namespace hallway_planner
