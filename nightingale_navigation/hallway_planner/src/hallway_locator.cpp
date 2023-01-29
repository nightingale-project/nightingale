/*********************************************************************
 * Software License Agreement (MIT License)
 * Author: Younes Reda
 *********************************************************************/

#include <hallway_planner/hallway_locator.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <numeric>


namespace hallway_planner::hallway_locator
{

std::optional<cv::Mat> locate_hallways(const cv::Mat &map, const HallwayLocatorConfig &config)
{
    using geometry_primitives::Line;
    using geometry_primitives::Point;
    using geometry_primitives::Hallway;
    if (map.type() != CV_8UC1)
    {
        ROS_ERROR("hallway_planner::hallway_locator::locate_hallways called with non 1 byte channel map"); 
        return std::nullopt;
    }
    // younes todo need to convert costmap to just wall map. check if necessary
    cv::Mat edges(map.size(),map.type());
    cv::Canny(map,edges,config.canny.threshold1,config.canny.threshold2,config.canny.aperture,config.canny.l2grad);
    const std::vector<Line> lines = [&config,&edges]() {
        std::vector<cv::Vec4i> l;
        cv::HoughLinesP(edges,l,config.hough.rho_res,config.hough.theta_res,config.hough.threshold,config.hough.min_line_length,config.hough.max_line_length);
        const auto vec4i_to_line = [](const cv::Vec4i &v) {
            const Point a(v[0],v[1]);
            const Point b(v[2],v[3]);
            return Line(a,b);
        };
        std::vector<Line> ret;
        ret.reserve(l.size());
        std::transform(l.cbegin(),l.cend(),std::back_inserter(ret),vec4i_to_line);
        std::sort(ret.begin(),ret.end(),[](const auto &a, const auto &b){return a.length()>b.length();});
        return ret;
    }();
    if (lines.empty())
    {
        ROS_ERROR("hallway_planner::hallway_locator::locate_hallways could not find hallways"); 
        return std::nullopt;
    }

    // hough returns many lines. we group those that represent the same wall
    // grouping is done in hough space
    auto filtered_lines = [&config,&lines]() {
        const auto similar = [&config](const auto &l1, const auto &l2) {
           return (std::abs(l1.rho()-l2.rho())<config.hough.rho_tolerance) && (std::abs(l1.theta()-l2.theta())<config.hough.theta_tolerance); 
        };
        std::vector<std::vector<Line>> groups;
        for(const auto &l : lines)
        {
            bool group_found = false;
            for (auto &group : groups)
            {
                if (similar(l,group[0]))
                {
                    group.push_back(l);
                    group_found = true;
                    break;
                }
            }
            if (!group_found)
            {
                std::vector new_group = {l};
                groups.push_back(std::move(new_group));
            }
        }
        assert(std::none_of(groups.cbegin(),groups.cend(),[](const auto &v){return v.empty();}));
        assert(std::accumulate(groups.cbegin(),groups.cend(), 0,[](const auto &sum, const auto &v){return sum + v.size();}) == lines.size());
        assert(groups.size()>0 && groups.size()<=lines.size());
        std::deque<Line> filtered_lines;
        std::transform(groups.cbegin(),groups.cend(),std::back_inserter(filtered_lines),[](const auto &v){return v.at(0);});
        return filtered_lines;
    }();
    if (filtered_lines.size()%2)
    {
        ROS_ERROR("hallway_planner::hallway_locator::locate_hallways found odd number of hallways"); 
        return std::nullopt;
    }
    const auto hallways = [&filtered_lines]() {
        // 2 lines are said to be very similar if they are close in hough space
        const auto similarity = [](const auto &a, const auto &b) {
            // epsilon prevents division by 0
            constexpr auto epsilon = 1e-9;
            const Point c(a.rho(),a.theta());
            const Point d(b.rho(),b.theta());
            return 1/(epsilon + geometry_primitives::euclidean_distance(c,d));
        };
        std::vector<Hallway> ret;
        // filtered lines is even and each iteration removes 2 lines
        while (filtered_lines.size() > 1)
        {
            const auto cur = filtered_lines.at(0);
            auto most_similar = std::next(filtered_lines.cbegin());
            double highest_similarity = std::numeric_limits<double>::max();
            for (auto it = filtered_lines.cbegin()+2;it!=filtered_lines.cend();++it)
            {
                if (const auto sim = similarity(cur,*it); sim<highest_similarity) 
                {
                    highest_similarity = sim;
                    most_similar = it;
                }
            }
            filtered_lines.pop_front();
            filtered_lines.erase(most_similar);
            ret.emplace_back(cur,*most_similar);
        }
        return ret;
    }();
    ROS_INFO_STREAM("hallway_planner::hallway_locator::locate_hallways found " << hallways.size() << " hallways."); 
    cv::Mat hallway_map(map.size(),map.type(),false);
    for (const auto &hallway : hallways)
    {
        const auto [p1,p2] = std::get<0>(hallway).points();
        // p3 is the point closest to p2. p4 is the one farther away
        const auto [potentially_p3,potentially_p4] = std::get<1>(hallway).points();
        const auto comp = [&p2](const auto &a, const auto &b) { return geometry_primitives::euclidean_distance(a,p2) < geometry_primitives::euclidean_distance(b,p2); };
        const auto [p3,p4] = std::minmax(potentially_p3,potentially_p4,comp);
        const std::array convex_poly = {static_cast<cv::Point>(p1),static_cast<cv::Point>(p2),static_cast<cv::Point>(p3),static_cast<cv::Point>(p4)};
        cv::fillConvexPoly(hallway_map,convex_poly,true);
    }
    return hallway_map;
}

}

