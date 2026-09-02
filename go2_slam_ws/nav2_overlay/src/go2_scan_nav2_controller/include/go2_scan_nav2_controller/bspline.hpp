#ifndef GO2_SCAN_NAV2_CONTROLLER__BSPLINE_HPP_
#define GO2_SCAN_NAV2_CONTROLLER__BSPLINE_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace go2_scan_nav2_controller
{

struct Vector3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

inline Vector3 operator+(const Vector3 & left, const Vector3 & right)
{
  return {left.x + right.x, left.y + right.y, left.z + right.z};
}

inline Vector3 operator-(const Vector3 & left, const Vector3 & right)
{
  return {left.x - right.x, left.y - right.y, left.z - right.z};
}

inline Vector3 operator*(double scale, const Vector3 & value)
{
  return {scale * value.x, scale * value.y, scale * value.z};
}

class BSpline
{
public:
  bool set(
    int degree, std::vector<double> knots, std::vector<Vector3> control_points,
    std::string * error = nullptr)
  {
    const auto fail = [error](const std::string & message) {
        if (error) {
          *error = message;
        }
        return false;
      };
    if (degree < 0) {
      return fail("degree is negative");
    }
    if (control_points.empty()) {
      return fail("control point array is empty");
    }
    if (static_cast<std::size_t>(degree) >= control_points.size()) {
      return fail("degree must be smaller than control point count");
    }
    if (knots.size() != control_points.size() + static_cast<std::size_t>(degree) + 1U) {
      return fail("knot/control point dimensions do not match");
    }
    for (std::size_t index = 0; index < knots.size(); ++index) {
      if (!std::isfinite(knots[index])) {
        return fail("knot array contains a non-finite value");
      }
      if (index > 0 && knots[index] < knots[index - 1]) {
        return fail("knot array is not monotonic");
      }
    }
    for (const auto & point : control_points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        return fail("control point array contains a non-finite value");
      }
    }
    const std::size_t end_index = knots.size() - static_cast<std::size_t>(degree) - 1U;
    if (!(knots[end_index] > knots[static_cast<std::size_t>(degree)])) {
      return fail("trajectory duration is not positive");
    }
    degree_ = degree;
    knots_ = std::move(knots);
    control_points_ = std::move(control_points);
    valid_ = true;
    return true;
  }

  bool valid() const {return valid_;}

  double duration() const
  {
    if (!valid_) {
      return 0.0;
    }
    const auto degree = static_cast<std::size_t>(degree_);
    return knots_[knots_.size() - degree - 1U] - knots_[degree];
  }

  Vector3 evaluate(double time_from_start) const
  {
    if (!valid_) {
      return {};
    }
    const int point_count = static_cast<int>(control_points_.size());
    const int last_knot = static_cast<int>(knots_.size()) - 1;
    const int end_span = last_knot - degree_;
    const double lower = knots_[static_cast<std::size_t>(degree_)];
    const double upper = knots_[static_cast<std::size_t>(end_span)];
    const double parameter = std::clamp(time_from_start + lower, lower, upper);

    int span = degree_;
    while (span + 1 < end_span && knots_[static_cast<std::size_t>(span + 1)] < parameter) {
      ++span;
    }
    std::vector<Vector3> values;
    values.reserve(static_cast<std::size_t>(degree_ + 1));
    for (int index = 0; index <= degree_; ++index) {
      const int control_index = std::clamp(span - degree_ + index, 0, point_count - 1);
      values.push_back(control_points_[static_cast<std::size_t>(control_index)]);
    }
    for (int level = 1; level <= degree_; ++level) {
      for (int index = degree_; index >= level; --index) {
        const int left_index = index + span - degree_;
        const int right_index = index + 1 + span - level;
        const double denominator =
          knots_[static_cast<std::size_t>(right_index)] -
          knots_[static_cast<std::size_t>(left_index)];
        const double alpha = std::abs(denominator) < 1e-12 ? 0.0 :
          (parameter - knots_[static_cast<std::size_t>(left_index)]) / denominator;
        values[static_cast<std::size_t>(index)] =
          (1.0 - alpha) * values[static_cast<std::size_t>(index - 1)] +
          alpha * values[static_cast<std::size_t>(index)];
      }
    }
    return values.back();
  }

  BSpline derivative() const
  {
    BSpline result;
    if (!valid_ || degree_ == 0 || control_points_.size() < 2U) {
      return result;
    }
    std::vector<Vector3> derivative_points;
    derivative_points.reserve(control_points_.size() - 1U);
    for (std::size_t index = 0; index + 1U < control_points_.size(); ++index) {
      const double denominator =
        knots_[index + static_cast<std::size_t>(degree_) + 1U] - knots_[index + 1U];
      const double scale = std::abs(denominator) < 1e-12 ? 0.0 : degree_ / denominator;
      derivative_points.push_back(scale * (control_points_[index + 1U] - control_points_[index]));
    }
    std::vector<double> derivative_knots(knots_.begin() + 1, knots_.end() - 1);
    result.set(degree_ - 1, std::move(derivative_knots), std::move(derivative_points));
    return result;
  }

private:
  int degree_{0};
  bool valid_{false};
  std::vector<double> knots_;
  std::vector<Vector3> control_points_;
};

}  // namespace go2_scan_nav2_controller

#endif  // GO2_SCAN_NAV2_CONTROLLER__BSPLINE_HPP_
