#include <iostream>
#include <hallway_planner/hallway_planner.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "debug_hallway_planner");
    hallway_planner::HallwayPlanner foo;
    foo.initialize("hallway_planner",nullptr);
    ros::spin();
}
