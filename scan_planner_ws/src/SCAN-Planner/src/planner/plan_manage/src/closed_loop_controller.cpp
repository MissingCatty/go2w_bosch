#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>

#include "bspline_opt/uniform_bspline.h"

namespace scan_planner
{
class ClosedLoopController : public rclcpp::Node
{
public:
  ClosedLoopController() : Node("closed_loop_controller")
  {
    time_forward_ = declare_parameter<double>("time_forward", 0.8);
    heading_error_threshold_ = declare_parameter<double>("heading_error_threshold", 0.8);
    heading_error_release_threshold_ = std::clamp(
        declare_parameter<double>("heading_error_release_threshold", 0.35),
        0.0, heading_error_threshold_);
    yaw_deadband_ = std::max(0.0, declare_parameter<double>("yaw_deadband", 0.06));
    kp_pos_ = declare_parameter<double>("kp_pos", 0.8);
    kp_yaw_ = declare_parameter<double>("kp_yaw", 1.5);
    max_vx_ = declare_parameter<double>("max_vx", 0.75);
    max_vy_ = declare_parameter<double>("max_vy", 0.35);
    max_vyaw_ = std::min(declare_parameter<double>("max_vyaw", 1.0), kMaxVYawLimit);
    curvature_tracking_enabled_ =
        declare_parameter<bool>("curvature_tracking_enabled", false);
    curvature_preview_time_ = std::max(
        0.0, declare_parameter<double>("curvature_preview_time", 0.20));
    curvature_yaw_reserve_ = std::clamp(
        declare_parameter<double>("curvature_yaw_reserve", 0.08),
        0.0, std::max(0.0, max_vyaw_ - 0.05));
    curvature_feedforward_gain_ = std::max(
        0.0, declare_parameter<double>("curvature_feedforward_gain", 1.0));
    curvature_deadband_ = std::max(
        0.0, declare_parameter<double>("curvature_deadband", 0.05));
    finish_dist_ = declare_parameter<double>("finish_dist", 0.15);
    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.5);
    cloud_timeout_ = std::max(0.2, declare_parameter<double>("cloud_timeout", 0.6));
    trajectory_timeout_ = declare_parameter<double>("trajectory_timeout", 0.5);
    trajectory_switch_min_interval_ = std::max(
        0.0, declare_parameter<double>("trajectory_switch_min_interval", 0.60));
    trajectory_switch_max_interval_ = std::max(
        trajectory_switch_min_interval_,
        declare_parameter<double>("trajectory_switch_max_interval", 1.50));
    trajectory_switch_compare_distance_ = std::max(
        0.10, declare_parameter<double>("trajectory_switch_compare_distance", 0.50));
    trajectory_switch_max_deviation_ = std::max(
        0.02, declare_parameter<double>("trajectory_switch_max_deviation", 0.18));
    trajectory_switch_min_remaining_ = std::max(
        0.10, declare_parameter<double>("trajectory_switch_min_remaining", 0.60));
    trajectory_collision_lookahead_ = std::max(
        0.30, declare_parameter<double>("trajectory_collision_lookahead", 1.50));
    goal_stop_distance_ = std::max(
        finish_dist_, declare_parameter<double>("goal_stop_distance", 0.15));
    goal_brake_decel_ = std::max(
        0.05, declare_parameter<double>("goal_brake_decel", 0.45));
    goal_reaction_time_ = std::max(
        0.0, declare_parameter<double>("goal_reaction_time", 0.15));
    obstacle_brake_decel_ = std::max(
        0.05, declare_parameter<double>("obstacle_brake_decel", 0.60));
    obstacle_reaction_time_ = std::max(
        0.0, declare_parameter<double>("obstacle_reaction_time", 0.20));
    obstacle_stop_margin_ = std::max(
        0.0, declare_parameter<double>("obstacle_stop_margin", 0.12));
    obstacle_braking_path_horizon_ = std::max(
        0.10, declare_parameter<double>("obstacle_braking_path_horizon", 0.50));
    obstacle_emergency_stop_enabled_ =
        declare_parameter<bool>("obstacle_emergency_stop_enabled", true);
    obstacle_emergency_clearance_ = std::max(
        0.0, declare_parameter<double>("obstacle_emergency_clearance", 0.12));
    obstacle_rotation_only_clearance_ = std::max(
        obstacle_emergency_clearance_,
        declare_parameter<double>("obstacle_rotation_only_clearance", 0.12));
    obstacle_emergency_speed_threshold_ = std::max(
        0.0, declare_parameter<double>("obstacle_emergency_speed_threshold", 0.15));
    obstacle_emergency_speed_release_ = std::clamp(
        declare_parameter<double>("obstacle_emergency_speed_release", 0.10),
        0.0, obstacle_emergency_speed_threshold_);
    obstacle_body_half_length_ = std::max(
        0.05, declare_parameter<double>("obstacle_body_half_length", 0.28));
    obstacle_body_half_width_ = std::max(
        0.05, declare_parameter<double>("obstacle_body_half_width", 0.15));
    obstacle_corridor_margin_ = std::max(
        0.0, declare_parameter<double>("obstacle_corridor_margin", 0.0));
    obstacle_cloud_max_range_ = std::max(
        0.5, declare_parameter<double>("obstacle_cloud_max_range", 4.0));
    obstacle_body_z_min_ = declare_parameter<double>("obstacle_body_z_min", -0.39);
    obstacle_body_z_max_ = declare_parameter<double>("obstacle_body_z_max", 0.55);
    obstacle_self_filter_radius_ = std::max(
        0.0, declare_parameter<double>("obstacle_self_filter_radius", 0.0));
    obstacle_self_filter_offset_ = std::max(
        0.0, declare_parameter<double>("obstacle_self_filter_offset", 0.12));
    obstacle_self_filter_z_min_ =
        declare_parameter<double>("obstacle_self_filter_z_min", -0.55);
    obstacle_self_filter_z_max_ =
        declare_parameter<double>("obstacle_self_filter_z_max", 0.30);
    base_to_lidar_x_ = declare_parameter<double>("base_to_lidar_x", 0.1701);
    base_to_lidar_y_ = declare_parameter<double>("base_to_lidar_y", 0.0);
    base_to_lidar_z_ = declare_parameter<double>("base_to_lidar_z", 0.0908);
    const double lidar_yaw = declare_parameter<double>(
        "base_to_lidar_yaw", 1.5707963267948966);
    lidar_mount_cos_ = std::cos(lidar_yaw);
    lidar_mount_sin_ = std::sin(lidar_yaw);

    bspline_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(
        "planning/bspline", 10,
        std::bind(&ClosedLoopController::bsplineCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&ClosedLoopController::odomCallback, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud", rclcpp::SensorDataQoS(),
        std::bind(&ClosedLoopController::cloudCallback, this, std::placeholders::_1));
    global_path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "initial_path", 10,
        std::bind(&ClosedLoopController::globalPathCallback, this, std::placeholders::_1));
    navigation_cancel_sub_ = create_subscription<std_msgs::msg::Bool>(
        "navigation_cancel", 10,
        std::bind(&ClosedLoopController::navigationCancelCallback, this, std::placeholders::_1));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 20);
    controller_velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        "planning/controller_velocity_world", 20);
    local_path_pub_ = create_publisher<nav_msgs::msg::Path>("planning/local_path", 10);
    execution_frozen_pub_ = create_publisher<std_msgs::msg::Bool>("planning/go2_execution_frozen", 10);
    emergency_stop_pub_ = create_publisher<std_msgs::msg::Bool>(
        "planning/emergency_stop", 10);
    cmd_timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                   std::bind(&ClosedLoopController::cmdCallback, this));
    last_update_time_ = now();
    last_odom_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_cloud_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    completion_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_trajectory_accept_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    RCLCPP_INFO(
        get_logger(),
        "Closed-loop controller ready: final-goal braking %.2fm/s^2, "
        "live-obstacle braking %.2fm/s^2, hard emergency stop %s, "
        "trajectory switching %.2f-%.2fs",
        goal_brake_decel_, obstacle_brake_decel_,
        obstacle_emergency_stop_enabled_ ? "enabled" : "disabled",
        trajectory_switch_min_interval_, trajectory_switch_max_interval_);
  }

private:
  static constexpr double kMaxVYawLimit = 1.0;

  enum class ObstacleResponse
  {
    kNormal,
    kRotationOnly,
    kEmergencyStop,
  };

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  static Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
  {
    const double norm = value.norm();
    return (norm <= max_norm || norm < 1e-6) ? value : value / norm * max_norm;
  }

  static double brakingSpeedLimit(
      double clearance, double stop_margin, double reaction_time,
      double current_speed, double brake_decel)
  {
    const double usable_distance =
        clearance - stop_margin - reaction_time * std::max(0.0, current_speed);
    return usable_distance <= 0.0
        ? 0.0 : std::sqrt(2.0 * brake_decel * usable_distance);
  }

  void publishEmergencyStop(bool active)
  {
    std_msgs::msg::Bool message;
    message.data = active;
    emergency_stop_pub_->publish(message);
  }

  static void capLinearSpeed(geometry_msgs::msg::Twist &command, double speed_limit)
  {
    const double speed = std::hypot(command.linear.x, command.linear.y);
    if (speed <= speed_limit || speed < 1e-6)
      return;
    const double scale = std::max(0.0, speed_limit) / speed;
    command.linear.x *= scale;
    command.linear.y *= scale;
  }

  double liveTrajectoryObstacleClearance(
      double path_horizon, double footprint_margin = 0.0) const
  {
    if (!receive_traj_ || traj_.size() < 2 || !have_odom_ ||
        obstacle_points_body_.empty() || path_horizon <= 0.0)
      return std::numeric_limits<double>::infinity();

    // Anchor the remaining accepted B-spline at the measured chassis pose.
    // This checks the actual curved route instead of an infinite straight
    // corridor in the instantaneous command direction. Points outside the
    // swept body footprint are deliberately ignored, even when nearby.
    const Eigen::Vector2d reference_start =
        traj_[0].evaluateDeBoorT(exec_time_).head<2>();
    const double cos_odom = std::cos(odom_yaw_);
    const double sin_odom = std::sin(odom_yaw_);
    const double nearby_limit =
        path_horizon + std::hypot(
            obstacle_body_half_length_ + footprint_margin,
            obstacle_body_half_width_ + footprint_margin) + 0.10;
    const double nearby_limit_squared = nearby_limit * nearby_limit;
    std::vector<const Eigen::Vector2d *> nearby_obstacles;
    nearby_obstacles.reserve(obstacle_points_body_.size());
    for (const auto &point : obstacle_points_body_)
    {
      if (point.squaredNorm() <= nearby_limit_squared)
        nearby_obstacles.push_back(&point);
    }
    if (nearby_obstacles.empty())
      return std::numeric_limits<double>::infinity();

    Eigen::Vector2d previous_center = Eigen::Vector2d::Zero();
    double travelled = 0.0;
    bool first_sample = true;
    constexpr double kSampleTime = 0.05;
    for (double time = exec_time_;
         time <= traj_duration_ + 1e-6 && travelled <= path_horizon;
         time = std::min(traj_duration_, time + kSampleTime))
    {
      const Eigen::Vector3d reference_position = traj_[0].evaluateDeBoorT(time);
      const Eigen::Vector2d world_offset = reference_position.head<2>() - reference_start;
      const Eigen::Vector2d center_body(
          cos_odom * world_offset.x() + sin_odom * world_offset.y(),
          -sin_odom * world_offset.x() + cos_odom * world_offset.y());
      const double segment_length = (center_body - previous_center).norm();
      if (!first_sample && segment_length < 0.02 && time < traj_duration_ - 1e-6)
        continue;
      travelled += segment_length;
      if (travelled > path_horizon + 1e-6)
        break;

      const Eigen::Vector3d velocity = traj_[1].evaluateDeBoorT(time);
      const double path_yaw = velocity.head<2>().norm() > 1e-4
          ? std::atan2(velocity.y(), velocity.x()) : odom_yaw_;
      const double relative_yaw = normalizeAngle(path_yaw - odom_yaw_);
      const double c = std::cos(relative_yaw);
      const double s = std::sin(relative_yaw);
      for (const Eigen::Vector2d *obstacle : nearby_obstacles)
      {
        const Eigen::Vector2d delta = *obstacle - center_body;
        const double longitudinal = c * delta.x() + s * delta.y();
        const double lateral = -s * delta.x() + c * delta.y();
        if (std::abs(longitudinal) <= obstacle_body_half_length_ + footprint_margin &&
            std::abs(lateral) <= obstacle_body_half_width_ + footprint_margin)
          return travelled;
      }

      previous_center = center_body;
      first_sample = false;
      if (time >= traj_duration_ - 1e-6)
        break;
    }
    return std::numeric_limits<double>::infinity();
  }

  double liveRotationClearance() const
  {
    if (obstacle_points_body_.empty())
      return std::numeric_limits<double>::infinity();
    const double swept_radius = std::hypot(
        obstacle_body_half_length_, obstacle_body_half_width_);
    double clearance = std::numeric_limits<double>::infinity();
    for (const auto &point : obstacle_points_body_)
      clearance = std::min(clearance, point.norm() - swept_radius);
    return clearance;
  }

  bool closeObstacleNeedsEmergencyStop(double clearance)
  {
    if (!obstacle_emergency_stop_enabled_)
    {
      speed_emergency_active_ = false;
      return false;
    }

    if (clearance >= obstacle_emergency_clearance_)
    {
      speed_emergency_active_ = false;
      return false;
    }

    // A close obstacle is only a hard emergency while the measured chassis
    // speed is too high for controlled low-speed alignment. Hysteresis keeps
    // the output from toggling as the filtered LIO velocity crosses the limit.
    if (speed_emergency_active_)
      speed_emergency_active_ =
          odom_planar_speed_ > obstacle_emergency_speed_release_;
    else
      speed_emergency_active_ =
          odom_planar_speed_ > obstacle_emergency_speed_threshold_;
    return speed_emergency_active_;
  }

  ObstacleResponse applySpeedEnvelopes(geometry_msgs::msg::Twist &command)
  {
    const double requested_speed = std::hypot(command.linear.x, command.linear.y);
    if (requested_speed < 1e-3)
      return ObstacleResponse::kNormal;
    const double current_speed = std::max(odom_planar_speed_, requested_speed);

    if (have_final_goal_)
    {
      const double goal_distance = (final_goal_ - odom_pos_.head<2>()).norm();
      const double goal_limit = brakingSpeedLimit(
          goal_distance, goal_stop_distance_, goal_reaction_time_,
          current_speed, goal_brake_decel_);
      if (goal_limit < requested_speed)
      {
        capLinearSpeed(command, goal_limit);
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "Final-goal braking: distance %.2fm, linear limit %.2fm/s",
            goal_distance, goal_limit);
      }
    }

    const double speed_after_goal = std::hypot(command.linear.x, command.linear.y);
    if (speed_after_goal < 1e-3)
      return ObstacleResponse::kNormal;
    const double obstacle_clearance = liveTrajectoryObstacleClearance(
        obstacle_braking_path_horizon_);
    if (!std::isfinite(obstacle_clearance))
      return ObstacleResponse::kNormal;

    // Only an obstacle intersecting the accepted local B-spline inside the
    // configured short horizon participates in braking. A hard stop suppresses
    // every axis. In the outer band, translation is blocked but yaw remains
    // available to align with the collision-free local trajectory.
    if (closeObstacleNeedsEmergencyStop(obstacle_clearance))
    {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 500,
          "Live-obstacle emergency stop: path collision in %.2fm, speed %.2fm/s",
          obstacle_clearance, odom_planar_speed_);
      return ObstacleResponse::kEmergencyStop;
    }
    if (obstacle_clearance <= obstacle_rotation_only_clearance_)
    {
      capLinearSpeed(command, 0.0);
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 500,
          "Live obstacle intersects path in %.2fm: translation blocked, yaw allowed",
          obstacle_clearance);
      return ObstacleResponse::kRotationOnly;
    }

    const double obstacle_limit = brakingSpeedLimit(
        obstacle_clearance, obstacle_stop_margin_, obstacle_reaction_time_,
        std::max(odom_planar_speed_, speed_after_goal), obstacle_brake_decel_);
    if (obstacle_limit < speed_after_goal)
    {
      capLinearSpeed(command, obstacle_limit);
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 500,
          "Live-obstacle braking: path collision in %.2fm, linear limit %.2fm/s",
          obstacle_clearance, obstacle_limit);
    }

    return ObstacleResponse::kNormal;
  }

  double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des) const
  {
    const double t_look = std::min(traj_duration_, t_cur + time_forward_);
    Eigen::Vector3d direction = traj_[0].evaluateDeBoorT(t_look) - pos_des;
    if (direction.head<2>().squaredNorm() < 1e-4)
      direction = traj_[1].evaluateDeBoorT(t_cur);
    return direction.head<2>().squaredNorm() < 1e-4
        ? odom_yaw_ : std::atan2(direction.y(), direction.x());
  }

  double signedTrajectoryCurvature(double trajectory_time) const
  {
    if (!curvature_tracking_enabled_ || traj_.size() < 3)
      return 0.0;
    const double sample_time = std::clamp(
        trajectory_time + curvature_preview_time_, 0.0, traj_duration_);
    const Eigen::Vector3d velocity = traj_[1].evaluateDeBoorT(sample_time);
    const Eigen::Vector3d acceleration = traj_[2].evaluateDeBoorT(sample_time);
    const double planar_speed_squared = velocity.head<2>().squaredNorm();
    // Curvature is undefined at a stationary endpoint. The preview normally
    // avoids it; returning zero here prevents numerical yaw spikes.
    if (planar_speed_squared < 2.5e-3)
      return 0.0;
    const double cross =
        velocity.x() * acceleration.y() - velocity.y() * acceleration.x();
    const double curvature = cross /
        (planar_speed_squared * std::sqrt(planar_speed_squared));
    return std::isfinite(curvature) ? curvature : 0.0;
  }

  void publishControllerVelocityWorld(const geometry_msgs::msg::Twist &body_command)
  {
    geometry_msgs::msg::TwistStamped feedback;
    feedback.header.stamp = now();
    feedback.header.frame_id = "odom";
    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);
    feedback.twist.linear.x =
        c * body_command.linear.x - s * body_command.linear.y;
    feedback.twist.linear.y =
        s * body_command.linear.x + c * body_command.linear.y;
    feedback.twist.linear.z = 0.0;
    controller_velocity_pub_->publish(feedback);
  }

  void publishCommand(const geometry_msgs::msg::Twist &command)
  {
    cmd_vel_pub_->publish(command);
    publishControllerVelocityWorld(command);
  }

  void publishStop(double yaw_rate = 0.0)
  {
    publishEmergencyStop(false);
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = std::clamp(yaw_rate, -max_vyaw_, max_vyaw_);
    publishCommand(cmd);
  }

  void publishExecutionFrozen(bool frozen)
  {
    std_msgs::msg::Bool msg;
    msg.data = frozen;
    execution_frozen_pub_->publish(msg);
  }

  double trajectoryArcLength(
      const UniformBspline &trajectory, double start_time, double end_time,
      double stop_after = std::numeric_limits<double>::infinity()) const
  {
    start_time = std::clamp(start_time, 0.0, end_time);
    Eigen::Vector2d previous = trajectory.evaluateDeBoorT(start_time).head<2>();
    double length = 0.0;
    constexpr double kSampleTime = 0.05;
    for (double time = std::min(end_time, start_time + kSampleTime);
         time <= end_time + 1e-6;
         time = std::min(end_time, time + kSampleTime))
    {
      const Eigen::Vector2d point = trajectory.evaluateDeBoorT(time).head<2>();
      length += (point - previous).norm();
      if (length >= stop_after || time >= end_time - 1e-6)
        break;
      previous = point;
    }
    return length;
  }

  bool trajectoryPointAtDistance(
      const UniformBspline &trajectory, double start_time, double end_time,
      double target_distance, Eigen::Vector2d &result) const
  {
    start_time = std::clamp(start_time, 0.0, end_time);
    Eigen::Vector2d previous = trajectory.evaluateDeBoorT(start_time).head<2>();
    if (target_distance <= 1e-6)
    {
      result = previous;
      return true;
    }

    double accumulated = 0.0;
    constexpr double kSampleTime = 0.05;
    for (double time = std::min(end_time, start_time + kSampleTime);
         time <= end_time + 1e-6;
         time = std::min(end_time, time + kSampleTime))
    {
      const Eigen::Vector2d point = trajectory.evaluateDeBoorT(time).head<2>();
      const double segment_length = (point - previous).norm();
      if (accumulated + segment_length >= target_distance && segment_length > 1e-6)
      {
        const double ratio =
            (target_distance - accumulated) / segment_length;
        result = previous + std::clamp(ratio, 0.0, 1.0) * (point - previous);
        return true;
      }
      accumulated += segment_length;
      if (time >= end_time - 1e-6)
        break;
      previous = point;
    }
    return false;
  }

  double trajectoryNearFieldDeviation(
      const UniformBspline &candidate, double candidate_duration) const
  {
    const double active_length = trajectoryArcLength(
        traj_[0], exec_time_, traj_duration_, trajectory_switch_compare_distance_);
    const double candidate_length = trajectoryArcLength(
        candidate, 0.0, candidate_duration, trajectory_switch_compare_distance_);
    const double compare_distance = std::min(
        {trajectory_switch_compare_distance_, active_length, candidate_length});
    if (compare_distance < 0.05)
      return 0.0;

    double max_deviation = 0.0;
    for (double fraction : {0.25, 0.50, 0.75, 1.00})
    {
      Eigen::Vector2d active_point;
      Eigen::Vector2d candidate_point;
      const double distance = fraction * compare_distance;
      if (!trajectoryPointAtDistance(
              traj_[0], exec_time_, traj_duration_, distance, active_point) ||
          !trajectoryPointAtDistance(
              candidate, 0.0, candidate_duration, distance, candidate_point))
        continue;
      max_deviation = std::max(max_deviation, (active_point - candidate_point).norm());
    }
    return max_deviation;
  }

  bool activeTrajectoryBlockedByLiveCloud() const
  {
    return std::isfinite(liveTrajectoryObstacleClearance(
        trajectory_collision_lookahead_, obstacle_corridor_margin_));
  }

  bool shouldAcceptTrajectory(
      const UniformBspline &candidate, double candidate_duration,
      std::string &reason, double &deviation)
  {
    deviation = 0.0;
    if (!receive_traj_ || traj_.size() < 2)
    {
      reason = "no active trajectory";
      return true;
    }
    if (force_next_trajectory_)
    {
      reason = "new global reference";
      return true;
    }

    const double remaining = trajectoryArcLength(
        traj_[0], exec_time_, traj_duration_, trajectory_switch_min_remaining_);
    if (remaining < trajectory_switch_min_remaining_)
    {
      reason = "active trajectory nearly exhausted";
      return true;
    }
    if (activeTrajectoryBlockedByLiveCloud())
    {
      reason = "active trajectory blocked by live cloud";
      return true;
    }

    const double age = last_trajectory_accept_time_.nanoseconds() > 0
        ? (now() - last_trajectory_accept_time_).seconds()
        : trajectory_switch_max_interval_;
    if (age < trajectory_switch_min_interval_)
    {
      reason = "minimum hold interval";
      return false;
    }

    deviation = trajectoryNearFieldDeviation(candidate, candidate_duration);
    if (deviation <= trajectory_switch_max_deviation_)
    {
      reason = "continuous refresh";
      return true;
    }
    if (age >= trajectory_switch_max_interval_)
    {
      reason = "maximum refresh interval";
      return true;
    }

    reason = "near-field path jump";
    return false;
  }

  void bsplineCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg)
  {
    if (navigation_cancelled_)
    {
      RCLCPP_WARN(get_logger(), "Ignoring trajectory while navigation is cancelled");
      return;
    }
    if (msg->pos_pts.empty() || msg->knots.empty() || msg->order <= 0)
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid B-spline");
      return;
    }
    Eigen::MatrixXd points(3, msg->pos_pts.size());
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
      points.col(i) << msg->pos_pts[i].x, msg->pos_pts[i].y, msg->pos_pts[i].z;
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];
    UniformBspline position(points, msg->order, 0.1);
    position.setKnot(knots);
    const double candidate_duration = position.getTimeSum();
    std::string switch_reason;
    double near_field_deviation = 0.0;
    if (!shouldAcceptTrajectory(
            position, candidate_duration, switch_reason, near_field_deviation))
    {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Keeping trajectory %lld; deferred candidate %lld (%s, deviation %.3fm)",
          static_cast<long long>(traj_id_), static_cast<long long>(msg->traj_id),
          switch_reason.c_str(), near_field_deviation);
      return;
    }

    traj_ = {position, position.getDerivative()};
    traj_.push_back(traj_[1].getDerivative());
    traj_duration_ = candidate_duration;
    traj_id_ = msg->traj_id;
    exec_time_ = 0.0;
    last_update_time_ = now();
    completion_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    heading_only_ = false;
    receive_traj_ = true;
    force_next_trajectory_ = false;
    last_trajectory_accept_time_ = now();
    publishLocalPath();
    RCLCPP_INFO(
        get_logger(), "Accepted trajectory %lld, duration %.3fs (%s, deviation %.3fm)",
        static_cast<long long>(traj_id_), traj_duration_, switch_reason.c_str(),
        near_field_deviation);
  }

  void publishLocalPath()
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = "odom";
    const double sample_dt = 0.10;
    const int count = std::max(2, static_cast<int>(std::ceil(traj_duration_ / sample_dt)) + 1);
    path.poses.reserve(count);
    for (int i = 0; i < count; ++i)
    {
      const double t = traj_duration_ * i / (count - 1);
      const Eigen::Vector3d point = traj_[0].evaluateDeBoorT(t);
      const Eigen::Vector3d velocity = traj_[1].evaluateDeBoorT(t);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = point.x();
      pose.pose.position.y = point.y();
      pose.pose.position.z = point.z();
      const double yaw = velocity.head<2>().squaredNorm() > 1e-6
          ? std::atan2(velocity.y(), velocity.x()) : odom_yaw_;
      pose.pose.orientation.z = std::sin(0.5 * yaw);
      pose.pose.orientation.w = std::cos(0.5 * yaw);
      path.poses.push_back(pose);
    }
    local_path_pub_->publish(path);
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    odom_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    odom_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    odom_planar_speed_ = std::hypot(
        msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    have_odom_ = true;
    last_odom_time_ = now();
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    if (!msg || msg->data.empty() || msg->width * msg->height == 0)
      return;
    std::vector<Eigen::Vector2d> points;
    points.reserve(std::min<std::size_t>(msg->width * msg->height, 12000));
    const double max_range_squared = obstacle_cloud_max_range_ * obstacle_cloud_max_range_;
    try
    {
      sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*msg, "z");
      for (; x != x.end(); ++x, ++y, ++z)
      {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z))
          continue;
        const double body_x =
            lidar_mount_cos_ * *x - lidar_mount_sin_ * *y + base_to_lidar_x_;
        const double body_y =
            lidar_mount_sin_ * *x + lidar_mount_cos_ * *y + base_to_lidar_y_;
        const double body_z = *z + base_to_lidar_z_;
        const double front_self_distance_squared =
            (body_x - obstacle_self_filter_offset_) *
                (body_x - obstacle_self_filter_offset_) +
            body_y * body_y;
        const double rear_self_distance_squared =
            (body_x + obstacle_self_filter_offset_) *
                (body_x + obstacle_self_filter_offset_) +
            body_y * body_y;
        const bool inside_self_filter =
            obstacle_self_filter_radius_ > 0.0 &&
            std::min(front_self_distance_squared, rear_self_distance_squared) <=
                obstacle_self_filter_radius_ * obstacle_self_filter_radius_ &&
            body_z >= obstacle_self_filter_z_min_ &&
            body_z <= obstacle_self_filter_z_max_;
        if (inside_self_filter)
          continue;
        if (body_z < obstacle_body_z_min_ || body_z > obstacle_body_z_max_ ||
            body_x * body_x + body_y * body_y > max_range_squared)
          continue;
        points.emplace_back(body_x, body_y);
      }
    }
    catch (const std::runtime_error &error)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Live cloud has no usable xyz fields: %s", error.what());
      return;
    }
    obstacle_points_body_.swap(points);
    have_cloud_ = true;
    last_cloud_time_ = now();
  }

  void globalPathCallback(const nav_msgs::msg::Path::ConstSharedPtr msg)
  {
    if (!msg || msg->poses.empty())
    {
      have_final_goal_ = false;
      return;
    }
    const auto &position = msg->poses.back().pose.position;
    if (!std::isfinite(position.x) || !std::isfinite(position.y))
      return;
    const Eigen::Vector2d updated_goal(position.x, position.y);
    if (!have_final_goal_ || (updated_goal - final_goal_).norm() > 0.05)
      force_next_trajectory_ = true;
    final_goal_ = updated_goal;
    have_final_goal_ = true;
    RCLCPP_INFO(get_logger(), "Final navigation goal updated: (%.2f, %.2f)",
                final_goal_.x(), final_goal_.y());
  }

  void navigationCancelCallback(const std_msgs::msg::Bool::ConstSharedPtr msg)
  {
    if (!msg)
      return;
    navigation_cancelled_ = msg->data;
    if (!navigation_cancelled_)
    {
      receive_traj_ = false;
      RCLCPP_INFO(get_logger(), "Navigation re-armed; waiting for a fresh trajectory");
      return;
    }

    receive_traj_ = false;
    exec_time_ = 0.0;
    completion_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_trajectory_accept_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    heading_only_ = false;
    force_next_trajectory_ = false;
    have_final_goal_ = false;
    publishEmergencyStop(false);
    publishExecutionFrozen(true);
    publishStop();
    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = now();
    empty_path.header.frame_id = "odom";
    local_path_pub_->publish(empty_path);
    RCLCPP_WARN(get_logger(), "Navigation cancelled; zero velocity published");
  }

  void cmdCallback()
  {
    if (navigation_cancelled_)
    {
      publishExecutionFrozen(true);
      publishStop();
      return;
    }
    if (!receive_traj_ || !have_odom_)
    {
      // No executable trajectory is available yet. Keep the chassis stopped
      // until SCAN publishes a fresh one.
      publishExecutionFrozen(true);
      publishStop();
      return;
    }
    const auto current_time = now();
    if ((current_time - last_odom_time_).seconds() > odom_timeout_)
    {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Odometry stale for more than %.2fs; commanding stop", odom_timeout_);
      publishExecutionFrozen(true);
      publishStop();
      return;
    }
    const double cloud_age = have_cloud_
        ? (current_time - last_cloud_time_).seconds()
        : std::numeric_limits<double>::infinity();
    if (!have_cloud_ || cloud_age < 0.0 || cloud_age > cloud_timeout_)
    {
      if (!cloud_stale_stop_)
        RCLCPP_ERROR(get_logger(),
                     "Live cloud stale (age %.3fs, limit %.3fs); forcing zero velocity",
                     cloud_age, cloud_timeout_);
      cloud_stale_stop_ = true;
      publishExecutionFrozen(true);
      publishStop();
      return;
    }
    if (cloud_stale_stop_)
      RCLCPP_INFO(get_logger(), "Live cloud recovered; SCAN must provide a confirmed fresh route");
    cloud_stale_stop_ = false;
    if (completion_time_.nanoseconds() > 0 &&
        (current_time - completion_time_).seconds() > trajectory_timeout_)
    {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Completed trajectory was not refreshed; commanding stop until a fresh plan arrives");
      // Do not freeze SCAN's clock here: it must observe the completed local
      // trajectory and either finish at the actual goal or produce a new one.
      publishExecutionFrozen(false);
      publishStop();
      return;
    }
    double dt = (current_time - last_update_time_).seconds();
    if (dt < 0.0 || dt > 0.2) dt = 0.0;
    const double t_eval = std::min(exec_time_, traj_duration_);
    Eigen::Vector3d pos_des = traj_[0].evaluateDeBoorT(t_eval);
    const double yaw_error = normalizeAngle(estimateDesiredYaw(t_eval, pos_des) - odom_yaw_);
    const double yaw_abs = std::abs(yaw_error);
    const double yaw_control_error = yaw_abs <= yaw_deadband_
        ? 0.0 : std::copysign(yaw_abs - yaw_deadband_, yaw_error);
    const double yaw_feedback_command = std::clamp(
        kp_yaw_ * yaw_control_error, -max_vyaw_, max_vyaw_);

    const bool previous_heading_only = heading_only_;
    if (heading_only_)
      heading_only_ = yaw_abs > heading_error_release_threshold_;
    else
      heading_only_ = yaw_abs > heading_error_threshold_;
    if (heading_only_ != previous_heading_only)
      RCLCPP_INFO(get_logger(), "Heading-only alignment %s at %.1f deg error",
                  heading_only_ ? "entered" : "released", yaw_abs * 180.0 / M_PI);
    if (heading_only_)
    {
      publishExecutionFrozen(true);
      const double rotation_clearance = liveRotationClearance();
      if (closeObstacleNeedsEmergencyStop(rotation_clearance))
      {
        publishEmergencyStop(true);
        publishCommand(geometry_msgs::msg::Twist());
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 500,
            "Rotation emergency stop: swept-body clearance %.2fm, speed %.2fm/s",
            rotation_clearance, odom_planar_speed_);
      }
      else
      {
        publishStop(yaw_feedback_command);
      }
      last_update_time_ = current_time;
      return;
    }

    // Keep executing the active trajectory even when odometry temporarily
    // deviates from its time-indexed reference. Live collision checks and
    // stream timeouts remain responsible for stopping the chassis.
    publishExecutionFrozen(false);
    exec_time_ = std::min(traj_duration_, exec_time_ + dt);
    last_update_time_ = current_time;
    pos_des = traj_[0].evaluateDeBoorT(exec_time_);
    const Eigen::Vector3d vel_des = traj_[1].evaluateDeBoorT(exec_time_);
    Eigen::Vector2d pos_error(pos_des.x() - odom_pos_.x(), pos_des.y() - odom_pos_.y());
    Eigen::Vector2d reference_tangent(vel_des.x(), vel_des.y());
    if (reference_tangent.norm() > 1e-3)
    {
      reference_tangent.normalize();
      const double along_track_error = pos_error.dot(reference_tangent);
      // A small odometry jump may put the reference just behind the robot.
      // Retain lateral correction but never chase that old point backwards;
      // no amount of reference overshoot discards the active trajectory.
      if (along_track_error < 0.0)
        pos_error -= along_track_error * reference_tangent;
    }
    const Eigen::Vector2d vel_world = clampNorm(
        Eigen::Vector2d(vel_des.x(), vel_des.y()) + kp_pos_ * pos_error,
        std::max(max_vx_, max_vy_));
    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);
    geometry_msgs::msg::Twist command;
    const double raw_body_vx = c * vel_world.x() + s * vel_world.y();
    // Normal path tracking never drives backwards. A route requiring a large
    // heading change enters heading-only mode first; holonomic detours may use
    // lateral velocity. Deliberate recovery commands use a separate channel.
    command.linear.x = std::clamp(raw_body_vx, 0.0, max_vx_);
    command.linear.y = std::clamp(-s * vel_world.x() + c * vel_world.y(), -max_vy_, max_vy_);
    const double path_curvature = signedTrajectoryCurvature(exec_time_);
    if (curvature_tracking_enabled_ &&
        std::abs(path_curvature) > curvature_deadband_)
    {
      // Reserve part of the yaw budget for correcting pose error. The rest
      // limits rolling speed so v*kappa remains executable by the chassis.
      const double feedforward_yaw_budget = std::max(
          0.05, max_vyaw_ - curvature_yaw_reserve_);
      const double curvature_speed_limit =
          feedforward_yaw_budget / std::abs(path_curvature);
      const double requested_speed =
          std::hypot(command.linear.x, command.linear.y);
      capLinearSpeed(command, curvature_speed_limit);
      if (curvature_speed_limit + 1e-3 < requested_speed)
      {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "Curvature speed limit: kappa %.3f 1/m, %.2f -> %.2fm/s",
            path_curvature, requested_speed, curvature_speed_limit);
      }
    }
    const auto update_rolling_yaw = [&]() {
      const double rolling_speed =
          std::hypot(command.linear.x, command.linear.y);
      const double yaw_feedforward =
          curvature_tracking_enabled_ &&
                  std::abs(path_curvature) > curvature_deadband_
              ? curvature_feedforward_gain_ * path_curvature * rolling_speed
              : 0.0;
      command.angular.z = std::clamp(
          yaw_feedback_command + yaw_feedforward, -max_vyaw_, max_vyaw_);
    };
    update_rolling_yaw();
    if (raw_body_vx < -0.02)
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Suppressed %.3fm/s reverse path-tracking command", raw_body_vx);
    if (exec_time_ >= traj_duration_ && completion_time_.nanoseconds() == 0)
      completion_time_ = current_time;
    if (exec_time_ >= traj_duration_ && pos_error.norm() < finish_dist_)
      command = geometry_msgs::msg::Twist();
    const ObstacleResponse obstacle_response = applySpeedEnvelopes(command);
    const bool emergency_stop =
        obstacle_response == ObstacleResponse::kEmergencyStop;
    publishEmergencyStop(emergency_stop);
    if (emergency_stop)
    {
      command = geometry_msgs::msg::Twist();
      publishExecutionFrozen(true);
    }
    else if (obstacle_response == ObstacleResponse::kRotationOnly)
    {
      // Do not consume the B-spline while physical translation is held.
      exec_time_ = t_eval;
      completion_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      publishExecutionFrozen(true);
    }
    // Goal/obstacle braking may have reduced translation after the first
    // calculation. Recompute v*kappa so the robot does not oversteer at the
    // new lower speed. A completed path remains a full zero command.
    if (!(exec_time_ >= traj_duration_ && pos_error.norm() < finish_dist_) &&
        !emergency_stop)
      update_rolling_yaw();
    publishCommand(command);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr controller_velocity_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_frozen_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_pub_;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr navigation_cancel_sub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  bool receive_traj_{false};
  bool have_odom_{false};
  bool have_cloud_{false};
  bool navigation_cancelled_{false};
  bool cloud_stale_stop_{false};
  bool have_final_goal_{false};
  bool speed_emergency_active_{false};
  bool obstacle_emergency_stop_enabled_{true};
  bool curvature_tracking_enabled_{false};
  bool force_next_trajectory_{false};
  std::vector<UniformBspline> traj_;
  double traj_duration_{0.0};
  std::int64_t traj_id_{0};
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector2d final_goal_{Eigen::Vector2d::Zero()};
  std::vector<Eigen::Vector2d> obstacle_points_body_;
  double odom_yaw_{0.0};
  double odom_planar_speed_{0.0};
  double exec_time_{0.0};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_cloud_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time completion_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_trajectory_accept_time_{0, 0, RCL_ROS_TIME};
  bool heading_only_{false};
  double time_forward_, heading_error_threshold_, heading_error_release_threshold_;
  double yaw_deadband_, kp_pos_, kp_yaw_;
  double curvature_preview_time_, curvature_yaw_reserve_;
  double curvature_feedforward_gain_, curvature_deadband_;
  double max_vx_, max_vy_, max_vyaw_, finish_dist_, odom_timeout_, cloud_timeout_,
      trajectory_timeout_;
  double trajectory_switch_min_interval_, trajectory_switch_max_interval_;
  double trajectory_switch_compare_distance_, trajectory_switch_max_deviation_;
  double trajectory_switch_min_remaining_, trajectory_collision_lookahead_;
  double goal_stop_distance_, goal_brake_decel_, goal_reaction_time_;
  double obstacle_brake_decel_, obstacle_reaction_time_, obstacle_stop_margin_;
  double obstacle_braking_path_horizon_;
  double obstacle_emergency_clearance_, obstacle_rotation_only_clearance_;
  double obstacle_emergency_speed_threshold_, obstacle_emergency_speed_release_;
  double obstacle_body_half_length_;
  double obstacle_body_half_width_, obstacle_corridor_margin_, obstacle_cloud_max_range_;
  double obstacle_body_z_min_, obstacle_body_z_max_;
  double obstacle_self_filter_radius_, obstacle_self_filter_offset_;
  double obstacle_self_filter_z_min_, obstacle_self_filter_z_max_;
  double base_to_lidar_x_, base_to_lidar_y_, base_to_lidar_z_;
  double lidar_mount_cos_{1.0}, lidar_mount_sin_{0.0};
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::ClosedLoopController>());
  rclcpp::shutdown();
  return 0;
}
