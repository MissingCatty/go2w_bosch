#include "bspline_opt/curvature_bspline_optimizer.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "bspline_opt/lbfgs.hpp"

namespace scan_planner
{
namespace
{
double planarYaw(const Eigen::Vector3d &from, const Eigen::Vector3d &to)
{
  const Eigen::Vector2d direction = to.head<2>() - from.head<2>();
  return direction.squaredNorm() > 1e-10
      ? std::atan2(direction.y(), direction.x()) : 0.0;
}
}  // namespace

void CurvatureBsplineOptimizer::setEnvironment(const GridMap::Ptr &env)
{
  grid_map_ = env;
  legacy_optimizer_.setEnvironment(env);
}

void CurvatureBsplineOptimizer::setParam(rclcpp::Node *node)
{
  node_ = node;
  legacy_optimizer_.setParam(node);

  const auto get_double = [node](const std::string &name, double default_value) {
    if (!node->has_parameter(name))
      node->declare_parameter<double>(name, default_value);
    return node->get_parameter(name).as_double();
  };
  const auto get_int = [node](const std::string &name, int default_value) {
    if (!node->has_parameter(name))
      node->declare_parameter<int>(name, default_value);
    return static_cast<int>(node->get_parameter(name).as_int());
  };

  const double min_turn_radius = std::max(
      0.20, get_double("curvature_bspline.min_turn_radius", 0.80));
  max_curvature_ = 1.0 / min_turn_radius;
  curvature_tolerance_ = std::max(
      0.0, get_double("curvature_bspline.curvature_tolerance", 0.15));
  curvature_sample_dt_ = std::clamp(
      get_double("curvature_bspline.sample_dt", 0.05), 0.02, 0.15);
  curvature_min_sample_distance_ = std::max(
      0.005, get_double("curvature_bspline.min_sample_distance", 0.02));
  max_control_point_deviation_ = std::max(
      0.05, get_double("curvature_bspline.max_deviation", 0.35));
  lambda_curvature_ = std::max(
      0.0, get_double("curvature_bspline.lambda_curvature", 30.0));
  lambda_reference_ = std::max(
      0.0, get_double("curvature_bspline.lambda_reference", 2.0));
  lambda_smoothness_ = std::max(
      0.0, get_double("curvature_bspline.lambda_smoothness", 0.20));
  lambda_deviation_barrier_ = std::max(
      0.0, get_double("curvature_bspline.lambda_deviation_barrier", 80.0));
  numerical_gradient_step_ = std::clamp(
      get_double("curvature_bspline.numerical_gradient_step", 1e-4),
      1e-6, 1e-2);
  max_iterations_ = std::clamp(
      get_int("curvature_bspline.max_iterations", 35), 1, 100);
  order_ = std::max(2, get_int("optimization.order", 3));

  RCLCPP_INFO(
      node_->get_logger(),
      "Curvature B-spline ready: minimum rolling radius %.2fm "
      "(curvature %.3f 1/m, tolerance %.0f%%)",
      min_turn_radius, max_curvature_, 100.0 * curvature_tolerance_);
}

void CurvatureBsplineOptimizer::syncLegacyInputs()
{
  legacy_optimizer_.a_star_ = a_star_;
  legacy_optimizer_.ref_pts_ = ref_pts_;
}

std::vector<std::vector<Eigen::Vector3d>>
CurvatureBsplineOptimizer::initControlPoints(
    Eigen::MatrixXd &init_points, bool flag_first_init)
{
  syncLegacyInputs();
  return legacy_optimizer_.initControlPoints(init_points, flag_first_init);
}

bool CurvatureBsplineOptimizer::BsplineOptimizeTrajRebound(
    Eigen::MatrixXd &optimal_points, double ts)
{
  syncLegacyInputs();
  if (!legacy_optimizer_.BsplineOptimizeTrajRebound(optimal_points, ts))
    return false;
  return applyCurvatureConstraint(optimal_points, ts);
}

bool CurvatureBsplineOptimizer::BsplineOptimizeTrajRefine(
    const Eigen::MatrixXd &init_points, double ts,
    Eigen::MatrixXd &optimal_points)
{
  syncLegacyInputs();
  if (!legacy_optimizer_.BsplineOptimizeTrajRefine(
          init_points, ts, optimal_points))
    return false;
  return applyCurvatureConstraint(optimal_points, ts);
}

double CurvatureBsplineOptimizer::discreteCurvature(
    const Eigen::Vector2d &previous, const Eigen::Vector2d &current,
    const Eigen::Vector2d &next) const
{
  const Eigen::Vector2d first = current - previous;
  const Eigen::Vector2d second = next - current;
  const Eigen::Vector2d chord = next - previous;
  const double first_length = first.norm();
  const double second_length = second.norm();
  const double chord_length = chord.norm();
  if (first_length < curvature_min_sample_distance_ ||
      second_length < curvature_min_sample_distance_ ||
      chord_length < curvature_min_sample_distance_)
    return 0.0;

  const double twice_area = std::abs(
      first.x() * second.y() - first.y() * second.x());
  return 2.0 * twice_area /
      (first_length * second_length * chord_length);
}

double CurvatureBsplineOptimizer::measureMaxCurvature(
    const Eigen::MatrixXd &control_points, double ts) const
{
  if (control_points.cols() < order_ + 1 || ts <= 0.0)
    return std::numeric_limits<double>::infinity();

  UniformBspline trajectory(control_points, order_, ts);
  const double duration = trajectory.getTimeSum();
  if (!std::isfinite(duration) || duration <= 0.0)
    return std::numeric_limits<double>::infinity();

  const int segment_count = std::max(
      2, static_cast<int>(std::ceil(duration / curvature_sample_dt_)));
  Eigen::Vector2d previous = trajectory.evaluateDeBoorT(0.0).head<2>();
  Eigen::Vector2d current = trajectory.evaluateDeBoorT(
      duration / segment_count).head<2>();
  double maximum = 0.0;
  for (int index = 2; index <= segment_count; ++index)
  {
    const double time = duration * index / segment_count;
    const Eigen::Vector2d next = trajectory.evaluateDeBoorT(time).head<2>();
    maximum = std::max(maximum, discreteCurvature(previous, current, next));
    previous = current;
    current = next;
  }
  return maximum;
}

Eigen::MatrixXd CurvatureBsplineOptimizer::pointsFromVariables(
    const double *x, int n) const
{
  Eigen::MatrixXd points = seed_points_;
  const int expected = 2 * std::max(0, movable_end_ - movable_begin_);
  if (x == nullptr || n != expected)
    return points;

  for (int column = movable_begin_; column < movable_end_; ++column)
  {
    const int variable = 2 * (column - movable_begin_);
    points(0, column) = x[variable];
    points(1, column) = x[variable + 1];
  }
  return points;
}

double CurvatureBsplineOptimizer::objective(const double *x, int n) const
{
  const Eigen::MatrixXd points = pointsFromVariables(x, n);
  double reference_cost = 0.0;
  double smoothness_cost = 0.0;
  double deviation_cost = 0.0;

  for (int column = movable_begin_; column < movable_end_; ++column)
  {
    const Eigen::Vector2d displacement =
        points.col(column).head<2>() - seed_points_.col(column).head<2>();
    reference_cost += displacement.squaredNorm();
    const double excess = displacement.norm() - max_control_point_deviation_;
    if (excess > 0.0)
      deviation_cost += excess * excess;
  }
  for (int column = 1; column + 1 < points.cols(); ++column)
  {
    const Eigen::Vector2d second_difference =
        points.col(column - 1).head<2>() -
        2.0 * points.col(column).head<2>() +
        points.col(column + 1).head<2>();
    smoothness_cost += second_difference.squaredNorm();
  }

  UniformBspline trajectory(points, order_, active_interval_);
  const double duration = trajectory.getTimeSum();
  const int segment_count = std::max(
      2, static_cast<int>(std::ceil(duration / curvature_sample_dt_)));
  Eigen::Vector2d previous = trajectory.evaluateDeBoorT(0.0).head<2>();
  Eigen::Vector2d current = trajectory.evaluateDeBoorT(
      duration / segment_count).head<2>();
  double curvature_cost = 0.0;
  int curvature_samples = 0;
  for (int index = 2; index <= segment_count; ++index)
  {
    const Eigen::Vector2d next = trajectory.evaluateDeBoorT(
        duration * index / segment_count).head<2>();
    const double excess =
        discreteCurvature(previous, current, next) - max_curvature_;
    if (excess > 0.0)
      curvature_cost += excess * excess;
    ++curvature_samples;
    previous = current;
    current = next;
  }

  const double movable_count = std::max(1, movable_end_ - movable_begin_);
  const double smoothness_count = std::max<Eigen::Index>(1, points.cols() - 2);
  return lambda_reference_ * reference_cost / movable_count +
      lambda_smoothness_ * smoothness_cost / smoothness_count +
      lambda_curvature_ * curvature_cost / std::max(1, curvature_samples) +
      lambda_deviation_barrier_ * deviation_cost / movable_count;
}

double CurvatureBsplineOptimizer::evaluateObjective(
    void *instance, const double *x, double *gradient, int n)
{
  auto *optimizer = static_cast<CurvatureBsplineOptimizer *>(instance);
  const double cost = optimizer->objective(x, n);
  if (gradient == nullptr)
    return cost;

  std::vector<double> perturbed(x, x + n);
  const double epsilon = optimizer->numerical_gradient_step_;
  for (int index = 0; index < n; ++index)
  {
    const double original = perturbed[index];
    perturbed[index] = original + epsilon;
    const double upper = optimizer->objective(perturbed.data(), n);
    perturbed[index] = original - epsilon;
    const double lower = optimizer->objective(perturbed.data(), n);
    perturbed[index] = original;
    gradient[index] = (upper - lower) / (2.0 * epsilon);
  }
  return cost;
}

bool CurvatureBsplineOptimizer::collisionFree(
    const Eigen::MatrixXd &control_points, double ts) const
{
  if (!grid_map_)
    return true;

  UniformBspline trajectory(control_points, order_, ts);
  const double duration = trajectory.getTimeSum();
  constexpr double sample_dt = 0.02;
  for (double time = 0.0; time < duration + 1e-6; time += sample_dt)
  {
    const double current_time = std::min(time, duration);
    const double next_time = std::min(current_time + sample_dt, duration);
    const Eigen::Vector3d position = trajectory.evaluateDeBoorT(current_time);
    const Eigen::Vector3d next = trajectory.evaluateDeBoorT(next_time);
    if (grid_map_->getInflateOccupancy(
            position, planarYaw(position, next)) != 0)
      return false;
  }
  return true;
}

double CurvatureBsplineOptimizer::maxControlPointDeviation(
    const Eigen::MatrixXd &candidate) const
{
  double maximum = 0.0;
  for (int column = movable_begin_; column < movable_end_; ++column)
  {
    maximum = std::max(
        maximum,
        (candidate.col(column).head<2>() -
         seed_points_.col(column).head<2>()).norm());
  }
  return maximum;
}

bool CurvatureBsplineOptimizer::applyCurvatureConstraint(
    Eigen::MatrixXd &control_points, double ts)
{
  if (control_points.rows() < 2 ||
      control_points.cols() < 2 * order_ + 1 || ts <= 0.0)
  {
    RCLCPP_WARN(rclcpp::get_logger("curvature_bspline"),
                "Curvature fitting received an invalid B-spline");
    return false;
  }

  const double allowed_curvature =
      max_curvature_ * (1.0 + curvature_tolerance_);
  const double curvature_before = measureMaxCurvature(control_points, ts);
  if (curvature_before <= allowed_curvature)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("curvature_bspline"),
                 "Legacy seed already satisfies curvature bound: %.3f <= %.3f 1/m",
                 curvature_before, allowed_curvature);
    return true;
  }

  seed_points_ = control_points;
  active_interval_ = ts;
  movable_begin_ = order_;
  movable_end_ = static_cast<int>(control_points.cols()) - order_;
  const int variable_count = 2 * (movable_end_ - movable_begin_);
  if (variable_count <= 0)
    return false;

  std::vector<double> variables(static_cast<size_t>(variable_count));
  for (int column = movable_begin_; column < movable_end_; ++column)
  {
    const int variable = 2 * (column - movable_begin_);
    variables[variable] = control_points(0, column);
    variables[variable + 1] = control_points(1, column);
  }

  lbfgs::lbfgs_parameter_t parameters;
  lbfgs::lbfgs_load_default_parameters(&parameters);
  parameters.mem_size = 8;
  parameters.max_iterations = max_iterations_;
  parameters.max_linesearch = 20;
  parameters.g_epsilon = 1e-3;
  parameters.min_step = 1e-12;

  double final_cost = 0.0;
  const int result = lbfgs::lbfgs_optimize(
      variable_count, variables.data(), &final_cost,
      CurvatureBsplineOptimizer::evaluateObjective, nullptr, nullptr,
      this, &parameters);
  const Eigen::MatrixXd fitted =
      pointsFromVariables(variables.data(), variable_count);

  // A curvature fit can round a corner into a live obstacle. Search along the
  // fitted-to-seed segment and publish only a curve satisfying both hard
  // conditions. The original collision-free seed is never silently used in
  // curvature mode when it exceeds the rolling-radius bound.
  static constexpr std::array<double, 9> kBlendRatios{
      1.0, 0.90, 0.80, 0.70, 0.60, 0.50, 0.40, 0.30, 0.20};
  for (const double ratio : kBlendRatios)
  {
    const Eigen::MatrixXd candidate =
        seed_points_ + ratio * (fitted - seed_points_);
    const double deviation = maxControlPointDeviation(candidate);
    if (deviation > max_control_point_deviation_ + 1e-6)
      continue;
    const double curvature = measureMaxCurvature(candidate, ts);
    if (curvature > allowed_curvature || !collisionFree(candidate, ts))
      continue;

    control_points = candidate;
    RCLCPP_INFO(
        rclcpp::get_logger("curvature_bspline"),
        "Wheel B-spline accepted: curvature %.3f -> %.3f 1/m, "
        "minimum radius %.2fm, blend %.2f, solver=%s",
        curvature_before, curvature, 1.0 / std::max(curvature, 1e-9),
        ratio, lbfgs::lbfgs_strerror(result));
    return true;
  }

  RCLCPP_WARN(
      rclcpp::get_logger("curvature_bspline"),
      "Wheel B-spline rejected: seed curvature %.3f 1/m exceeds %.3f "
      "and no collision-free bounded-curvature fit was found (solver=%s)",
      curvature_before, allowed_curvature, lbfgs::lbfgs_strerror(result));
  return false;
}

}  // namespace scan_planner
