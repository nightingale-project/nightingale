/*********************************************************************
 * Software License Agreement (MIT License)
 * Author: Younes Reda
 *********************************************************************/

#include <hallway_planner/astar.hpp>
#include <unordered_map>
#include <numeric>
#include <ros/ros.h>

namespace hallway_planner::astar
{

uint32_t compute_cost(const cv::Mat &map, const geometry_primitives::PixelPath &path)
{
    return path.size() + std::accumulate(path.cbegin(),path.cend(), static_cast<uint32_t>(0), [&](const auto sum, const auto &point) { return sum + map.at<uint8_t>(point.y(),point.x()); });
}

std::optional<geometry_primitives::PixelPath> solve(const cv::Mat &map, const geometry_primitives::Point &start, const geometry_primitives::Point &goal, const uint8_t lethal_cost, const uint32_t tolerance)
{
    using geometry_primitives::Point;
    if (map.type() != CV_8UC1)
    {
        ROS_ERROR("hallway_planner::astar::solve called with non 1 byte channel map"); 
        return std::nullopt;
    }
    const auto map_at = [&](const Point &p) -> uint8_t { return map.at<uint8_t>(p.y(),p.x()); };
    /* The A* heuristic is f(n)=g(n)+h(n) */
    /* g is the matrix below, f is the euclidean function */
    cv::Mat g_map(map.size(), CV_32F, cv::Scalar(std::numeric_limits<float>::max()));
    const auto g_map_at = [&](const Point &p) -> float& { return g_map.at<float>(p.y(),p.x()); };
    g_map_at(start) = map_at(start);
    const auto f = [&](const Point &left, const Point &right) {
        const auto &h = geometry_primitives::euclidean_distance;
        return (h(left,goal) + g_map_at(left)) > (h(right,goal) + g_map_at(right));
    };
    std::set<Point, decltype(f)> open_set(f);
    open_set.insert(start);
    std::unordered_map<Point,Point> parents;

    enum class Action {
        UP,
        DOWN,
        LEFT,
        RIGHT,
        UP_LEFT,
        UP_RIGHT,
        DOWN_LEFT,
        DOWN_RIGHT
    };
    constexpr std::array<Action,8> actions = {Action::UP, Action::DOWN, Action::LEFT, Action::RIGHT, Action::UP_LEFT, Action::UP_RIGHT, Action::DOWN_LEFT, Action::DOWN_RIGHT};

    const auto map_point_is_valid = [&](const Point &p) { return (p.x() >= 0) && (p.y() >=0 ) && (p.x() < map.cols) && (p.y() < map.rows) && (map_at(p) <  lethal_cost); };
    const auto compute_transition = [&](const Point &p, const Action a) -> Point {
        const size_t x = p.x();
        const size_t y = p.y();
        switch (a) {
            case Action::UP: return Point(x,y+1);
            case Action::DOWN: return Point(x,y-1);
            case Action::LEFT: return Point(x-1,y);
            case Action::RIGHT: return Point(x+1,y);
            case Action::UP_LEFT: return Point(x-1,y+1);
            case Action::UP_RIGHT: return Point(x+1,y+1);
            case Action::DOWN_LEFT: return Point(x-1,y-1);
            case Action::DOWN_RIGHT: return Point(x+1,y-1);
            default: { ROS_FATAL("HallwayPlanner A* Should never get here"); return Point(0,0); }
        }
    };

    while (!open_set.empty())
    {
        // smallest cost is the last one
        const auto cur = open_set.extract(std::prev(open_set.end())).value();
        constexpr auto EPSILON = 1e-5;
        if (euclidean_distance(cur,goal) <= std::min(static_cast<double>(tolerance),EPSILON))
        {
            geometry_primitives::PixelPath path;
            Point tmp = goal;
            while (tmp != start)
            {
                path.push_back(tmp);
                tmp = parents.at(tmp);
            }
            path.push_back(start);
            std::reverse(path.begin(),path.end());
            return path;
        }
        for (const auto action : actions)
        {
            const auto next = compute_transition(cur,action);
            if (!map_point_is_valid(next)) { continue; }
            const auto distance  = euclidean_distance(cur,next);
            // tentative g-score is sum of g-score of cur node, cost of the action (ie. distance), and
            // cost of being in the new node
            const auto tentative_g_score = g_map_at(cur)+distance+map_at(next);
            if (tentative_g_score < g_map_at(next))
            {
                g_map_at(next) = tentative_g_score;
                // insertion into unordered_map requires empty key
                parents.erase(next);
                parents.emplace(next,cur);
                // remove and reinsert bc order changed bc new g value
                open_set.erase(next);
                open_set.insert(next);
            }
        }
    }
    return std::nullopt;
}
}

