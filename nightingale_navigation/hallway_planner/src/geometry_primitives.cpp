/*********************************************************************
 * Software License Agreement (MIT License)
 * Author: Younes Reda
 *********************************************************************/

#include <cmath>
#include <hallway_planner/geometry_primitives.hpp>

namespace hallway_planner::geometry_primitives {

Point::Point(const uint32_t &X, const uint32_t &Y) : x_(X), y_(Y) {}

bool Point::operator==(const Point &p) const {
  return (x_ == p.x()) && (y_ == p.y());
}

bool Point::operator!=(const Point &p) const { return !(operator==(p)); }

int64_t Point::x() const { return static_cast<int64_t>(x_); }

int64_t Point::y() const { return static_cast<int64_t>(y_); }

Point::operator cv::Point() const { return cv::Point(x_, y_); }

double euclidean_distance(const Point &a, const Point &b) {
  const auto dy = a.y() - b.y();
  const auto dx = a.x() - b.x();
  return std::sqrt(dy * dy + dx * dx);
}

double euclidean_distance(const geometry_msgs::Point &a,
                          const geometry_msgs::Point &b) {
  const auto dx = a.x - b.x;
  const auto dy = a.y - b.y;
  const auto dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

Line::Line(const Point &A, const Point &B)
    : a_(A),
      b_(B),
      slope_(Line::calculate_slope(a_, b_)),
      y_intercept_(Line::calculate_y_intercept(slope_, a_)),
      theta_(Line::calculate_theta(a_, b_)),
      rho_(Line::calculate_rho(a_, theta_)) {}

double Line::length() const { return euclidean_distance(a_, b_); }

double Line::calculate_slope(const Point &a, const Point &b) {
  // let slope be positive since choosing which point is first is arbitrary
  const double num = a.y() - b.y();
  const double den = a.x() - b.x();
  constexpr auto MIN_DEN = 1e-5;
  return den == 0 ? std::abs(num / MIN_DEN) : std::abs(num / den);
}

double Line::calculate_y_intercept(const double &slope, const Point &point) {
  return point.y() - point.x() * slope;
}

double Line::calculate_theta(const Point &a, const Point &b) {
  // converting line to d=xsin(theta)+ysin(theta) form
  // this is the prefered hough representation of lines
  // we solve the equation
  // 0 = x1cos(theta)+y1sin(theta)-x2cos(theta)-x2sin(theta)
  // 0 = (x1-x2)cos(theta)+(y1-y2)sin(theta)
  // (x2-x1)cos(theta)=(y1-y2)sin(theta)
  // ((x2-x1)/(y1-y2))=tan(theta)
  const double num = b.x() - a.x();
  const double den = a.y() - b.y();
  // div by 0 is well defined for doubles
  return std::atan2(num, den);
}

double Line::calculate_rho(const Point &a, const double theta) {
  // rho=xcos(theta)+ysin(theta)
  return std::abs(a.x() * std::cos(theta) + a.y() * std::sin(theta));
}

std::pair<Point, Point> Line::points() const { return {a_, b_}; }

double Line::slope() const { return slope_; }

double Line::y_intercept() const { return y_intercept_; }

double Line::rho() const { return rho_; }

double Line::theta() const { return theta_; }

bool Line::operator==(const Line &other) const {
  return this->points() == other.points();
}

}  // namespace hallway_planner::geometry_primitives
