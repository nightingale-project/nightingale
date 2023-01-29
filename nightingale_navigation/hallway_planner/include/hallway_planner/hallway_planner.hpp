#pragma once                                     
/*********************************************************************
 * Software License Agreement (MIT License)
 * Author: Younes Reda
 *********************************************************************/


#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Pose.h>
#include <hallway_planner/geometry_primitives.hpp>
#include <vector>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <mutex>

namespace hallway_planner
{

class HallwayPlanner : public nav_core::BaseGlobalPlanner
{
public:

    /**
     * @brief  Default con/destructor for the PlannerCore object
     */
    HallwayPlanner();
    ~HallwayPlanner() = default;


    /**
     * @brief Copies
     */
    HallwayPlanner(const HallwayPlanner &);
    HallwayPlanner &operator=(const HallwayPlanner &);
    

    /**
     * @brief Moves
     */
    HallwayPlanner(HallwayPlanner &&);
    HallwayPlanner &operator=(HallwayPlanner &&);

    /**                                                                                                                                                                                                  
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose 
       * @param goal The goal pose 
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
    bool makePlan(const geometry_msgs::PoseStamped& start, 
                  const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) override;

    /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose 
       * @param goal The goal pose 
       * @param plan The plan... filled by the planner
       * @param cost The plans calculated cost
       * @return True if a valid plan was found, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& start, 
                    const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan,
                    double& cost) override;

     /**
       * @brief  Initialization function for the BaseGlobalPlanner
       * @param  name The name of this planner
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
     void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;

private:
    std::string name_;
    costmap_2d::Costmap2DROS *costmap_ros_;
    bool initialized_;
    cv::Mat costmap_cv_;
    cv::Mat hallway_map_;
    double tolerance_;
    uint8_t lethal_cost_;
    uint32_t orientation_filter_window_;
    mutable std::mutex mutex_;

    /* Convenience functions */
    void warn_uninitialized() const;
    void warn_already_initialized() const;

    /* Frame conversion functions */
    std::optional<geometry_primitives::Point> map_frame_to_pixel_frame(const geometry_msgs::Pose& pose) const;
    geometry_msgs::Pose pixel_frame_to_map_frame(const geometry_primitives::Point &p) const;

    /* Orientation Filler */
    void fill_orientations(geometry_primitives::Path &path, const uint32_t window) const;

    static constexpr uint8_t DEFAULT_LETHAL_COST = 253;
    static constexpr double DEFAULT_TOLERANCE = 0;
    static constexpr uint32_t DEFAULT_ORIENTATION_FILTER_WINDOW = 0;
};

}
