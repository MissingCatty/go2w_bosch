#include <gtest/gtest.h>

#include <Eigen/Eigen>

#include "bspline_opt/curvature_bspline_optimizer.h"

namespace scan_planner
{
namespace
{
TEST(CurvatureBsplineOptimizer, StraightSplineHasZeroPlanarCurvature)
{
  Eigen::MatrixXd points(3, 9);
  points.setZero();
  for (int column = 0; column < points.cols(); ++column)
    points(0, column) = 0.15 * column;

  CurvatureBsplineOptimizer optimizer;
  EXPECT_NEAR(optimizer.measureMaxCurvature(points, 0.20), 0.0, 1e-6);
  EXPECT_TRUE(optimizer.applyCurvatureConstraint(points, 0.20));
}

TEST(CurvatureBsplineOptimizer, DetectsSmoothButTightBend)
{
  // The control polygon has no discontinuity, yet its radius is far below
  // the default 0.8 m wheel-friendly radius.
  Eigen::MatrixXd points(3, 10);
  points.setZero();
  points.row(0) << 0.00, 0.10, 0.20, 0.30, 0.38,
                   0.42, 0.42, 0.42, 0.42, 0.42;
  points.row(1) << 0.00, 0.00, 0.00, 0.02, 0.08,
                   0.18, 0.28, 0.38, 0.48, 0.58;

  CurvatureBsplineOptimizer optimizer;
  EXPECT_GT(optimizer.measureMaxCurvature(points, 0.20), 1.25);
}

TEST(CurvatureBsplineOptimizer, RefitsTightBendWithinRollingRadius)
{
  Eigen::MatrixXd points(3, 12);
  points.setZero();
  points.row(0) << 0.00, 0.15, 0.30, 0.45, 0.60, 0.75,
                   0.90, 1.00, 1.05, 1.05, 1.05, 1.05;
  points.row(1) << 0.00, 0.00, 0.00, 0.01, 0.03, 0.08,
                   0.18, 0.35, 0.55, 0.75, 0.95, 1.15;

  CurvatureBsplineOptimizer optimizer;
  ASSERT_GT(optimizer.measureMaxCurvature(points, 0.20), 1.25 * 1.15);
  ASSERT_TRUE(optimizer.applyCurvatureConstraint(points, 0.20));
  EXPECT_LE(optimizer.measureMaxCurvature(points, 0.20), 1.25 * 1.15);
}
}  // namespace
}  // namespace scan_planner
