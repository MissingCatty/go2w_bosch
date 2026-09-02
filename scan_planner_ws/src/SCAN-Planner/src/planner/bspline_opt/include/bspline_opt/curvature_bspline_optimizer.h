#ifndef _CURVATURE_BSPLINE_OPTIMIZER_H_
#define _CURVATURE_BSPLINE_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <memory>
#include <vector>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <rclcpp/rclcpp.hpp>

namespace scan_planner
{

// A wheel-friendly alternative to BsplineOptimizer.
//
// The existing SCAN optimizer is intentionally left untouched. This class
// runs it as the collision-aware seed generator, then fits a second cubic
// B-spline whose planar curvature is bounded. Selecting the implementation is
// done by SCANPlannerManager, so the legacy trajectory remains available for
// A/B testing and immediate rollback.
class CurvatureBsplineOptimizer
{
public:
  CurvatureBsplineOptimizer() = default;
  ~CurvatureBsplineOptimizer() = default;

  void setEnvironment(const GridMap::Ptr &env);
  void setParam(rclcpp::Node *node);

  std::vector<std::vector<Eigen::Vector3d>> initControlPoints(
      Eigen::MatrixXd &init_points, bool flag_first_init = true);
  bool BsplineOptimizeTrajRebound(
      Eigen::MatrixXd &optimal_points, double ts);
  bool BsplineOptimizeTrajRefine(
      const Eigen::MatrixXd &init_points, double ts,
      Eigen::MatrixXd &optimal_points);

  // Exposed for deterministic geometry tests and offline comparison tools.
  bool applyCurvatureConstraint(Eigen::MatrixXd &control_points, double ts);
  double measureMaxCurvature(
      const Eigen::MatrixXd &control_points, double ts) const;

  AStar::Ptr a_star_;
  std::vector<Eigen::Vector3d> ref_pts_;

  using Ptr = std::unique_ptr<CurvatureBsplineOptimizer>;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  void syncLegacyInputs();
  Eigen::MatrixXd pointsFromVariables(const double *x, int n) const;
  double objective(const double *x, int n) const;
  static double evaluateObjective(
      void *instance, const double *x, double *gradient, int n);

  bool collisionFree(const Eigen::MatrixXd &control_points, double ts) const;
  double maxControlPointDeviation(const Eigen::MatrixXd &candidate) const;
  double discreteCurvature(
      const Eigen::Vector2d &previous, const Eigen::Vector2d &current,
      const Eigen::Vector2d &next) const;

  BsplineOptimizer legacy_optimizer_;
  GridMap::Ptr grid_map_;
  rclcpp::Node *node_{nullptr};

  Eigen::MatrixXd seed_points_;
  double active_interval_{0.1};
  int movable_begin_{3};
  int movable_end_{3};

  double max_curvature_{1.25};
  double curvature_tolerance_{0.15};
  double curvature_sample_dt_{0.05};
  double curvature_min_sample_distance_{0.02};
  double max_control_point_deviation_{0.35};
  double lambda_curvature_{30.0};
  double lambda_reference_{2.0};
  double lambda_smoothness_{0.20};
  double lambda_deviation_barrier_{80.0};
  double numerical_gradient_step_{1e-4};
  int max_iterations_{35};
  int order_{3};
};

}  // namespace scan_planner

#endif
