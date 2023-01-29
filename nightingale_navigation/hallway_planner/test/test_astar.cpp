#include <hallway_planner/astar.hpp>
#include <ros/ros.h>

/* Test for an empty map with trivial solution */
void trivial_empty()
{
    using hallway_planner::geometry_primitives::Point;
    constexpr auto width = 100;
    constexpr auto height = width;
    constexpr uint8_t free = 0;
    constexpr uint8_t lethal = 255;
    constexpr uint32_t tol = 0;
    const cv::Mat maze(cv::Size(width,height),CV_8UC1,cv::Scalar(free));
    const Point start(0,0);
    const Point goal(width-1,height-1);
    const auto maybe_soln = hallway_planner::astar::solve(maze,start,goal,lethal,tol);
    // check that soln was found
    assert(maybe_soln);
    const auto soln = maybe_soln.value();
    // assert that the length of the path found is the same as the width
    assert(hallway_planner::astar::compute_cost(maze,soln) == soln.size());
    assert(hallway_planner::astar::compute_cost(maze,soln) == width);
    // assert that all points in the path are in the diagonal
    assert(std::all_of(soln.cbegin(),soln.cend(),[](const auto &p){return p.x()==p.y();}));
    // assert that the start is the start and end is the goal
    assert(soln[0] == start);
    assert(soln[soln.size()-1] == goal);
    // check that all points in the path are adjacent
    const auto adjacent = [&](const Point &a, const Point &b) {
        constexpr auto epsilon = 1e-5;
        return hallway_planner::geometry_primitives::euclidean_distance(a,b) < (std::sqrt(2) + epsilon);
    };
    assert(std::adjacent_find(soln.cbegin(), soln.cend(),std::not_fn(adjacent)) == soln.cend());
}

/* Test for map with no solution */
void trivial_full()
{
    using hallway_planner::geometry_primitives::Point;
    constexpr auto width = 100;
    constexpr auto height = width;
    constexpr uint8_t lethal = 255;
    constexpr uint8_t free = 0;
    constexpr uint32_t tol = 0;
    cv::Mat maze(cv::Size(width,height),CV_8UC1,cv::Scalar(lethal));
    const Point start(height/2,width/2);
    maze.at<uint8_t>(start.y(),start.x()) = free;
    const Point goal(width-1,height-1);
    const auto maybe_soln = hallway_planner::astar::solve(maze,start,goal,lethal,tol);
    // no soln should be found bc entire map is lethal cost
    assert(!maybe_soln);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hallway_planner_astar_test");
    trivial_empty();
    trivial_full();
    ROS_INFO("Hallway Planner A* Tester Done.");

}
