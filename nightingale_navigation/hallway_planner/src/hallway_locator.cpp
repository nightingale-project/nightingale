/*********************************************************************
 * Software License Agreement (MIT License)
 * Author: Younes Reda
 *********************************************************************/

#include <ros/ros.h>

#include <hallway_planner/hallway_locator.hpp>
#include <numeric>
#include <opencv2/imgproc.hpp>

namespace hallway_planner::hallway_locator {

#ifndef _DEBUG_MAPS
static_assert(false, "_DEBUG_MAPS should always be defined");
#endif
#if _DEBUG_MAPS
constexpr auto DEBUG_MAPS = true;
#else
constexpr auto DEBUG_MAPS = false;
#endif

std::optional<cv::Mat> locate_hallways(const costmap_2d::Costmap2DROS *costmap,
                                       const HallwayLocatorConfig &config) {
  const auto rows = costmap->getCostmap()->getSizeInCellsY();
  const auto cols = costmap->getCostmap()->getSizeInCellsX();
  cv::Mat map(rows, cols, CV_8UC1);
  std::memcpy(map.data, costmap->getCostmap()->getCharMap(), rows * cols);
  return locate_hallways(map, config);
}

std::optional<cv::Mat> locate_hallways(const cv::Mat &costmap,
                                       const HallwayLocatorConfig &config) {
  using geometry_primitives::Hallway;
  using geometry_primitives::Line;
  using geometry_primitives::Point;
  if (costmap.type() != CV_8UC1) {
    ROS_ERROR(
        "hallway_planner::hallway_locator::locate_hallways called with non 1 "
        "byte channel map");
    return std::nullopt;
  }
  if constexpr (DEBUG_MAPS) {
    cv::imwrite("/tmp/costmap_pre_threshold.jpg", costmap);
  }
  costmap.forEach<uchar>([](uint8_t &pixel, const int *position) -> void {
    if (pixel != costmap_2d::LETHAL_OBSTACLE) {
      pixel = costmap_2d::FREE_SPACE;
    }
  });
  if constexpr (DEBUG_MAPS) {
    cv::imwrite("/tmp/costmap_post_threshold.jpg", costmap);
  }
  cv::Canny(costmap, costmap, config.canny.threshold1, config.canny.threshold2,
            config.canny.aperture, config.canny.l2grad);
  const std::vector<Line> lines = [&config, &costmap]() {
    std::vector<cv::Vec4i> l;
    cv::HoughLinesP(costmap, l, config.hough.rho_res, config.hough.theta_res,
                    config.hough.threshold, config.hough.min_line_length,
                    config.hough.max_line_length);
    const auto vec4i_to_line = [](const cv::Vec4i &v) {
      const Point a(v[0], v[1]);
      const Point b(v[2], v[3]);
      return Line(a, b);
    };
    std::vector<Line> ret;
    ret.reserve(l.size());
    std::transform(l.cbegin(), l.cend(), std::back_inserter(ret),
                   vec4i_to_line);
    std::sort(ret.begin(), ret.end(), [](const auto &a, const auto &b) {
      return a.length() > b.length();
    });
    return ret;
  }();
  if (lines.empty()) {
    ROS_ERROR(
        "hallway_planner::hallway_locator::locate_hallways could not find "
        "hallways");
    return std::nullopt;
  }
  ROS_INFO_STREAM("HallwayLocator found " << lines.size() << " lines");

  // hough returns many lines. we group those that represent the same wall
  // grouping is done in hough space
  auto filtered_lines = [&config, &lines]() {
    const auto similar = [&config](const auto &l1, const auto &l2) {
      return (std::abs(l1.rho() - l2.rho()) < config.hough.rho_tolerance) &&
             (std::abs(l1.theta() - l2.theta()) < config.hough.theta_tolerance);
    };
    std::vector<std::vector<Line>> groups;
    for (const auto &l : lines) {
      bool group_found = false;
      for (auto &group : groups) {
        if (similar(l, group[0])) {
          group.push_back(l);
          group_found = true;
          break;
        }
      }
      if (!group_found) {
        std::vector new_group = {l};
        groups.push_back(std::move(new_group));
      }
    }
    assert(std::none_of(groups.cbegin(), groups.cend(),
                        [](const auto &v) { return v.empty(); }));
    assert(std::accumulate(groups.cbegin(), groups.cend(), 0,
                           [](const auto &sum, const auto &v) {
                             return sum + v.size();
                           }) == lines.size());
    assert(groups.size() > 0 && groups.size() <= lines.size());
    std::deque<Line> filtered_lines;
    std::transform(groups.cbegin(), groups.cend(),
                   std::back_inserter(filtered_lines),
                   [](const auto &v) { return v.at(0); });
    return filtered_lines;
  }();
  ROS_INFO_STREAM("HallwayLocator filtered "
                  << lines.size() << " lines down to " << filtered_lines.size()
                  << " lines");
  const auto hallways = [&filtered_lines, &config]() {
    // 2 lines are said to be very similar if they are close in hough space
    const auto dissimilarity = [](const Line &a, const Line &b) {
      const Point c(a.rho(), a.theta());
      const Point d(b.rho(), b.theta());
      return geometry_primitives::euclidean_distance(c, d);
    };
    std::vector<Hallway> ret;
    while (filtered_lines.size() > 1) {
      const auto cur = filtered_lines.at(0);
      const auto most_similar = std::min_element(
          filtered_lines.cbegin() + 1, filtered_lines.cend(),
          [&cur, &dissimilarity](const Line &a, const Line &b) {
            return dissimilarity(a, cur) < dissimilarity(b, cur);
          });
      // sometimes, the detector detects lone lines ie. it detects only half a
      // hallway in these cases, we ignore the line
      constexpr auto experimental_threshold = 100;
      if (dissimilarity(cur, *most_similar) > experimental_threshold) {
        ROS_INFO("HallwayLocator discarding unmatched hallway line");
        filtered_lines.pop_front();
      } else {
        ret.emplace_back(cur, *most_similar);
        filtered_lines.erase(most_similar);
        filtered_lines.pop_front();
      }
    }
    return ret;
  }();
  ROS_INFO_STREAM("hallway_planner::hallway_locator::locate_hallways found "
                  << hallways.size() << " hallways.");
  cv::Mat hallway_map(costmap.size(), costmap.type(),
                      HallwayLocatorConfig::NO_HALLWAY);
  for (const auto &hallway : hallways) {
    const auto [p1, p2] = hallway.first.points();
    // p3 is the point closest to p2. p4 is the one farther away
    const auto [potentially_p3, potentially_p4] = hallway.second.points();
    const auto comp = [&p2](const auto &a, const auto &b) {
      return geometry_primitives::euclidean_distance(a, p2) <
             geometry_primitives::euclidean_distance(b, p2);
    };
    const auto [p3, p4] = std::minmax(potentially_p3, potentially_p4, comp);
    const std::array convex_poly = {
        static_cast<cv::Point>(p1), static_cast<cv::Point>(p2),
        static_cast<cv::Point>(p3), static_cast<cv::Point>(p4)};
    cv::fillConvexPoly(hallway_map, convex_poly, HallwayLocatorConfig::HALLWAY);
  }
  if constexpr (DEBUG_MAPS) {
    cv::imwrite("/tmp/hallways.jpg", hallway_map);
  }
  return hallway_map;
}

}  // namespace hallway_planner::hallway_locator
