#pragma once
/*********************************************************************
 * Software License Agreement (MIT License)
 * Author: Younes Reda
 *********************************************************************/

#include <geometry_msgs/Pose.h>

#include <cmath>
#include <cstdint>
#include <opencv2/opencv.hpp>
#include <utility>
#include <vector>

namespace hallway_planner::geometry_primitives {

struct Point {
 public:
  Point(const uint32_t &X, const uint32_t &Y);
  Point(const Point &p) = default;
  Point &operator=(const Point &p) = default;
  Point(Point &&p) = default;
  Point &operator=(Point &&p) = default;
  bool operator==(const Point &p) const;
  bool operator!=(const Point &p) const;
  int64_t x() const;
  int64_t y() const;

  explicit operator cv::Point() const;

 private:
  uint32_t x_, y_;
};

double euclidean_distance(const Point &a, const Point &b);

double euclidean_distance(const geometry_msgs::Point &a,
                          const geometry_msgs::Point &b);

class Line {
 public:
  /* Constructor & Destrcutor */
  Line(const Point &A, const Point &B);
  ~Line() = default;

  /* Copies */
  Line(const Line &) = default;
  Line &operator=(const Line &) = default;

  /* Moves */
  Line(Line &&) = default;
  Line &operator=(Line &&) = default;

  /* Get a pair of points */
  std::pair<Point, Point> points() const;
  /* Get the slope of the line in the typical y=mx+b repr */
  double slope() const;
  /* Get the y-intercept of the line in the typical y=mx+b repr */
  double y_intercept() const;
  /* Get rho, the shortest distance from the line to the origin. this is from
   * the polar repr */
  double rho() const;
  /* Get theta, the angle of the line's normal vector. this is from the polar
   * repr */
  double theta() const;

  /* Get the length of the line */
  double length() const;

  /* Comparison */
  bool operator==(const Line &other) const;

 private:
  Point a_, b_;
  double slope_, y_intercept_;
  double theta_, rho_;

  /* Static conversions */
  static double calculate_slope(const Point &a, const Point &b);
  static double calculate_y_intercept(const double &slope, const Point &point);
  static double calculate_theta(const Point &a, const Point &b);
  static double calculate_rho(const Point &a, const double theta);
};

using Hallway = std::pair<Line, Line>;

template <typename T>
constexpr double deg_to_rad(const T x) {
  return x * M_PI / 180;
}

}  // namespace hallway_planner::geometry_primitives

// overload of std::hash for Point used for unordered_maps/sets
template <>
struct std::hash<hallway_planner::geometry_primitives::Point> {
  std::size_t operator()(
      const hallway_planner::geometry_primitives::Point &p) const noexcept {
    const auto h1 = std::hash<size_t>{}(p.x());
    const auto h2 = std::hash<size_t>{}(p.y());
    return h1 ^ (h2 << 1);
  }
};
