#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "go2_scan_nav2_controller/bspline.hpp"

using go2_scan_nav2_controller::BSpline;
using go2_scan_nav2_controller::Vector3;

TEST(BSpline, EvaluatesLinearCurveAndDerivative)
{
  BSpline curve;
  std::string error;
  ASSERT_TRUE(curve.set(
      1, {0.0, 0.0, 2.0, 2.0},
      {Vector3{0.0, 0.0, 0.0}, Vector3{2.0, 4.0, 0.0}}, &error)) << error;
  EXPECT_DOUBLE_EQ(curve.duration(), 2.0);
  const auto midpoint = curve.evaluate(1.0);
  EXPECT_NEAR(midpoint.x, 1.0, 1e-9);
  EXPECT_NEAR(midpoint.y, 2.0, 1e-9);

  const auto derivative = curve.derivative();
  ASSERT_TRUE(derivative.valid());
  const auto velocity = derivative.evaluate(0.5);
  EXPECT_NEAR(velocity.x, 1.0, 1e-9);
  EXPECT_NEAR(velocity.y, 2.0, 1e-9);
}

TEST(BSpline, RejectsMalformedMessageData)
{
  BSpline curve;
  EXPECT_FALSE(curve.set(3, {0.0, 1.0}, {Vector3{}}));
  EXPECT_FALSE(curve.set(
      1, {0.0, 0.0, 1.0, 1.0},
      {Vector3{0.0, 0.0, 0.0}, Vector3{NAN, 0.0, 0.0}}));
}
