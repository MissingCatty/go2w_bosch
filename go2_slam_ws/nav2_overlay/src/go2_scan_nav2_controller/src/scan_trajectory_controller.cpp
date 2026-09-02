#include "go2_scan_nav2_controller/scan_trajectory_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace go2_scan_nav2_controller
{

namespace
{

double planarNorm(double x, double y)
{
  return std::hypot(x, y);
}

void clampPlanarNorm(double & x, double & y, double limit)
{
  const double norm = planarNorm(x, y);
  if (norm > limit && norm > 1e-9) {
    const double scale = std::max(0.0, limit) / norm;
    x *= scale;
    y *= scale;
  }
}

}  // namespace

void ScanTrajectoryController::configure(
  const LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::PlannerException("SCAN controller parent lifecycle node expired");
  }
  node_ = parent;
  plugin_name_ = std::move(name);
  logger_ = node->get_logger();
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  local_frame_ = costmap_ros_->getGlobalFrameID();
  base_frame_ = costmap_ros_->getBaseFrameID();

  const auto parameter = [this](const std::string & key) {
      return plugin_name_ + "." + key;
    };
  const auto get_double = [&node, &parameter](const std::string & key, double fallback) {
      const std::string full_name = parameter(key);
      nav2_util::declare_parameter_if_not_declared(
        node, full_name, rclcpp::ParameterValue(fallback));
      return node->get_parameter(full_name).as_double();
    };
  const auto get_string = [&node, &parameter](
    const std::string & key, const std::string & fallback)
    {
      const std::string full_name = parameter(key);
      nav2_util::declare_parameter_if_not_declared(
        node, full_name, rclcpp::ParameterValue(fallback));
      return node->get_parameter(full_name).as_string();
    };
  transform_tolerance_ = get_double("transform_tolerance", transform_tolerance_);
  initial_plan_timeout_ = get_double("initial_plan_timeout", initial_plan_timeout_);
  local_waiting_timeout_ = get_double("local_waiting_timeout", local_waiting_timeout_);
  trajectory_refresh_timeout_ = get_double(
    "trajectory_refresh_timeout", trajectory_refresh_timeout_);
  compute_watchdog_timeout_ = get_double("compute_watchdog_timeout", compute_watchdog_timeout_);
  preview_time_ = get_double("preview_time", preview_time_);
  heading_error_enter_ = get_double("heading_error_enter", heading_error_enter_);
  heading_error_exit_ = get_double("heading_error_exit", heading_error_exit_);
  yaw_deadband_ = get_double("yaw_deadband", yaw_deadband_);
  position_gain_ = get_double("position_gain", position_gain_);
  yaw_gain_ = get_double("yaw_gain", yaw_gain_);
  max_vx_ = get_double("max_vx", max_vx_);
  max_vy_ = get_double("max_vy", max_vy_);
  max_wz_ = get_double("max_wz", max_wz_);
  curvature_yaw_reserve_ = get_double("curvature_yaw_reserve", curvature_yaw_reserve_);
  curvature_deadband_ = get_double("curvature_deadband", curvature_deadband_);
  finish_distance_ = get_double("finish_distance", finish_distance_);
  goal_stop_distance_ = get_double("goal_stop_distance", goal_stop_distance_);
  goal_brake_decel_ = get_double("goal_brake_decel", goal_brake_decel_);
  goal_reaction_time_ = get_double("goal_reaction_time", goal_reaction_time_);
  tracking_replan_error_ = get_double("tracking_replan_error", tracking_replan_error_);
  replan_request_interval_ = get_double("replan_request_interval", replan_request_interval_);
  local_path_sample_dt_ = get_double("local_path_sample_dt", local_path_sample_dt_);
  speed_limit_ = max_vx_;

  const std::string global_path_topic = get_string(
    "global_path_topic", "/scan_planner/global_path");
  const std::string trajectory_topic = get_string(
    "trajectory_topic", "/scan_planner/planning/bspline");
  const std::string local_path_topic = get_string(
    "local_path_topic", "/scan_planner/local_path");
  const std::string waiting_topic = get_string(
    "local_waiting_topic", "/scan_planner/planning/local_waiting");
  const std::string frozen_topic = get_string(
    "execution_frozen_topic", "/scan_planner/planning/execution_frozen");
  const std::string velocity_topic = get_string(
    "controller_velocity_topic", "/scan_planner/planning/controller_velocity_world");
  const std::string replan_topic = get_string(
    "replan_request_topic", "/scan_planner/planning/replan_request");

  global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(global_path_topic, 1);
  local_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(local_path_topic, 10);
  frozen_pub_ = node->create_publisher<std_msgs::msg::Bool>(frozen_topic, 10);
  replan_pub_ = node->create_publisher<std_msgs::msg::Bool>(replan_topic, 10);
  velocity_pub_ = node->create_publisher<geometry_msgs::msg::TwistStamped>(velocity_topic, 20);
  trajectory_sub_ = node->create_subscription<scan_planner_msgs::msg::Bspline>(
    trajectory_topic, 10,
    std::bind(&ScanTrajectoryController::onTrajectory, this, std::placeholders::_1));
  waiting_sub_ = node->create_subscription<std_msgs::msg::Bool>(
    waiting_topic, 10,
    std::bind(&ScanTrajectoryController::onLocalWaiting, this, std::placeholders::_1));
  watchdog_timer_ = node->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&ScanTrajectoryController::watchdog, this));

  const auto clock_type = node->get_clock()->get_clock_type();
  generation_started_ = rclcpp::Time(0, 0, clock_type);
  last_compute_ = rclcpp::Time(0, 0, clock_type);
  last_control_update_ = rclcpp::Time(0, 0, clock_type);
  trajectory_completed_at_ = rclcpp::Time(0, 0, clock_type);
  waiting_started_ = rclcpp::Time(0, 0, clock_type);
  last_replan_request_ = rclcpp::Time(0, 0, clock_type);

  RCLCPP_INFO(
    logger_,
    "Configured SCAN local-trajectory controller: frame=%s, limits=%.2f/%.2f m/s %.2f rad/s",
    local_frame_.c_str(), max_vx_, max_vy_, max_wz_);
}

void ScanTrajectoryController::cleanup()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
  plan_active_ = false;
  have_trajectory_ = false;
  watchdog_timer_.reset();
  waiting_sub_.reset();
  trajectory_sub_.reset();
  velocity_pub_.reset();
  replan_pub_.reset();
  frozen_pub_.reset();
  local_path_pub_.reset();
  global_path_pub_.reset();
  tf_.reset();
  costmap_ros_.reset();
}

void ScanTrajectoryController::activate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  global_path_pub_->on_activate();
  local_path_pub_->on_activate();
  frozen_pub_->on_activate();
  replan_pub_->on_activate();
  velocity_pub_->on_activate();
  active_ = true;
  RCLCPP_INFO(logger_, "Activated SCAN local-trajectory controller");
}

void ScanTrajectoryController::deactivate()
{
  publishFrozen(true);
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
  plan_active_ = false;
  have_trajectory_ = false;
  velocity_pub_->on_deactivate();
  replan_pub_->on_deactivate();
  frozen_pub_->on_deactivate();
  local_path_pub_->on_deactivate();
  global_path_pub_->on_deactivate();
  RCLCPP_INFO(logger_, "Deactivated SCAN local-trajectory controller");
}

nav_msgs::msg::Path ScanTrajectoryController::transformPlan(const nav_msgs::msg::Path & path) const
{
  if (path.poses.empty()) {
    throw nav2_core::PlannerException("Nav2 supplied an empty global path to SCAN");
  }
  nav_msgs::msg::Path transformed;
  auto node = node_.lock();
  if (!node) {
    throw nav2_core::PlannerException("Controller node expired while transforming global path");
  }
  transformed.header.stamp = node->now();
  transformed.header.frame_id = local_frame_;
  transformed.poses.reserve(path.poses.size());
  for (auto pose : path.poses) {
    if (pose.header.frame_id.empty()) {
      pose.header.frame_id = path.header.frame_id;
    }
    geometry_msgs::msg::PoseStamped output;
    if (pose.header.frame_id == local_frame_) {
      output = pose;
    } else if (!nav2_util::transformPoseInTargetFrame(
        pose, output, *tf_, local_frame_, transform_tolerance_))
    {
      throw nav2_core::PlannerException(
              "Cannot transform Nav2 global path from " + pose.header.frame_id + " to " +
              local_frame_);
    }
    output.header = transformed.header;
    transformed.poses.push_back(output);
  }
  return transformed;
}

geometry_msgs::msg::PoseStamped ScanTrajectoryController::transformPose(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  if (pose.header.frame_id.empty() || pose.header.frame_id == local_frame_) {
    auto output = pose;
    output.header.frame_id = local_frame_;
    return output;
  }
  geometry_msgs::msg::PoseStamped output;
  if (!nav2_util::transformPoseInTargetFrame(
      pose, output, *tf_, local_frame_, transform_tolerance_))
  {
    throw nav2_core::PlannerException(
            "Cannot transform controller pose from " + pose.header.frame_id + " to " +
            local_frame_);
  }
  return output;
}

void ScanTrajectoryController::setPlan(const nav_msgs::msg::Path & path)
{
  auto node = node_.lock();
  if (!node) {
    throw nav2_core::PlannerException("Controller node expired while accepting a global path");
  }
  nav_msgs::msg::Path transformed = transformPlan(path);
  const auto now = node->now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_plan_ = transformed;
    const auto & goal = transformed.poses.back().pose.position;
    final_goal_ = {goal.x, goal.y, goal.z};
    generation_started_ = now;
    last_compute_ = now;
    last_control_update_ = now;
    trajectory_completed_at_ = rclcpp::Time(0, 0, now.get_clock_type());
    waiting_started_ = rclcpp::Time(0, 0, now.get_clock_type());
    trajectory_id_ = -1;
    trajectory_duration_ = 0.0;
    execution_time_ = 0.0;
    plan_active_ = true;
    have_trajectory_ = false;
    local_waiting_ = false;
    heading_only_ = false;
    watchdog_cancelled_ = false;
  }
  global_path_pub_->publish(transformed);
  publishFrozen(true);
  RCLCPP_INFO(
    logger_, "Forwarded Nav2 path generation to SCAN: %zu poses, goal (%.2f, %.2f)",
    transformed.poses.size(), final_goal_.x, final_goal_.y);
}

void ScanTrajectoryController::onTrajectory(
  const scan_planner_msgs::msg::Bspline::ConstSharedPtr message)
{
  auto node = node_.lock();
  if (!node || !message) {
    return;
  }
  std::vector<Vector3> points;
  points.reserve(message->pos_pts.size());
  for (const auto & point : message->pos_pts) {
    points.push_back({point.x, point.y, point.z});
  }
  BSpline position;
  std::string error;
  if (!position.set(message->order, message->knots, std::move(points), &error)) {
    RCLCPP_WARN(logger_, "Rejected malformed SCAN B-spline: %s", error.c_str());
    return;
  }
  BSpline velocity = position.derivative();
  BSpline acceleration = velocity.derivative();
  if (!velocity.valid()) {
    RCLCPP_WARN(logger_, "Rejected SCAN B-spline without a usable derivative");
    return;
  }
  const auto now = node->now();
  const rclcpp::Time trajectory_stamp(message->start_time, now.get_clock_type());
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_ || !plan_active_ || watchdog_cancelled_) {
      return;
    }
    if (trajectory_stamp.nanoseconds() > 0 &&
      (generation_started_ - trajectory_stamp).seconds() > 0.10)
    {
      RCLCPP_WARN(
        logger_, "Rejected stale SCAN trajectory %ld from a previous Nav2 path generation",
        static_cast<long>(message->traj_id));
      return;
    }
    if (have_trajectory_ && message->traj_id == trajectory_id_) {
      return;
    }
    position_ = position;
    velocity_trajectory_ = velocity;
    acceleration_trajectory_ = acceleration;
    trajectory_id_ = message->traj_id;
    trajectory_duration_ = position.duration();
    execution_time_ = 0.0;
    last_control_update_ = now;
    trajectory_completed_at_ = rclcpp::Time(0, 0, now.get_clock_type());
    waiting_started_ = rclcpp::Time(0, 0, now.get_clock_type());
    have_trajectory_ = true;
    local_waiting_ = false;
    heading_only_ = false;
  }
  publishLocalPath(position, position.duration(), now);
  RCLCPP_INFO(
    logger_, "Accepted fresh SCAN local trajectory %ld (%.3f s)",
    static_cast<long>(message->traj_id), position.duration());
}

void ScanTrajectoryController::onLocalWaiting(
  const std_msgs::msg::Bool::ConstSharedPtr message)
{
  auto node = node_.lock();
  if (!node || !message) {
    return;
  }
  const auto now = node->now();
  std::lock_guard<std::mutex> lock(mutex_);
  if (!plan_active_) {
    return;
  }
  if (message->data && !local_waiting_) {
    waiting_started_ = now;
  } else if (!message->data) {
    waiting_started_ = rclcpp::Time(0, 0, now.get_clock_type());
  }
  local_waiting_ = message->data;
}

void ScanTrajectoryController::watchdog()
{
  auto node = node_.lock();
  if (!node) {
    return;
  }
  const auto now = node->now();
  bool expired = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_ && plan_active_ && last_compute_.nanoseconds() > 0 &&
      (now - last_compute_).seconds() > compute_watchdog_timeout_)
    {
      plan_active_ = false;
      have_trajectory_ = false;
      watchdog_cancelled_ = true;
      expired = true;
    }
  }
  if (expired) {
    publishFrozen(true);
    RCLCPP_WARN(
      logger_, "Nav2 stopped requesting commands; invalidated the SCAN trajectory locally");
  }
}

void ScanTrajectoryController::publishFrozen(bool frozen)
{
  if (!frozen_pub_ || !frozen_pub_->is_activated()) {
    return;
  }
  std_msgs::msg::Bool message;
  message.data = frozen;
  frozen_pub_->publish(message);
}

void ScanTrajectoryController::publishWorldVelocity(
  const geometry_msgs::msg::Twist & command, double yaw, const rclcpp::Time & stamp)
{
  if (!velocity_pub_ || !velocity_pub_->is_activated()) {
    return;
  }
  geometry_msgs::msg::TwistStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = local_frame_;
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  message.twist.linear.x = cosine * command.linear.x - sine * command.linear.y;
  message.twist.linear.y = sine * command.linear.x + cosine * command.linear.y;
  velocity_pub_->publish(message);
}

void ScanTrajectoryController::publishLocalPath(
  const BSpline & position, double duration, const rclcpp::Time & stamp)
{
  if (!local_path_pub_ || !local_path_pub_->is_activated()) {
    return;
  }
  const BSpline velocity = position.derivative();
  nav_msgs::msg::Path path;
  path.header.stamp = stamp;
  path.header.frame_id = local_frame_;
  const int samples = std::max(
    2, static_cast<int>(std::ceil(duration / std::max(0.02, local_path_sample_dt_))) + 1);
  path.poses.reserve(static_cast<std::size_t>(samples));
  double previous_yaw = 0.0;
  for (int index = 0; index < samples; ++index) {
    const double time = duration * index / static_cast<double>(samples - 1);
    const auto point = position.evaluate(time);
    const auto tangent = velocity.evaluate(time);
    if (planarNorm(tangent.x, tangent.y) > 1e-4) {
      previous_yaw = std::atan2(tangent.y, tangent.x);
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = point.z;
    pose.pose.orientation.z = std::sin(0.5 * previous_yaw);
    pose.pose.orientation.w = std::cos(0.5 * previous_yaw);
    path.poses.push_back(pose);
  }
  local_path_pub_->publish(path);
}

geometry_msgs::msg::TwistStamped ScanTrajectoryController::zeroCommand(
  const rclcpp::Time & stamp, bool frozen)
{
  publishFrozen(frozen);
  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = stamp;
  command.header.frame_id = base_frame_;
  return command;
}

double ScanTrajectoryController::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

geometry_msgs::msg::TwistStamped ScanTrajectoryController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & measured_velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  auto node = node_.lock();
  if (!node) {
    throw nav2_core::PlannerException("Controller node expired while computing velocity");
  }
  const auto now = node->now();
  const auto robot_pose = transformPose(pose);
  const double robot_x = robot_pose.pose.position.x;
  const double robot_y = robot_pose.pose.position.y;
  const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

  std::lock_guard<std::mutex> lock(mutex_);
  last_compute_ = now;
  watchdog_cancelled_ = false;
  if (!active_ || !plan_active_) {
    throw nav2_core::PlannerException("No active Nav2 path generation for SCAN");
  }
  double dt = (now - last_control_update_).seconds();
  last_control_update_ = now;
  if (dt < 0.0 || dt > 0.20) {
    dt = 0.0;
  }

  if (local_waiting_) {
    const double waiting_age = waiting_started_.nanoseconds() > 0 ?
      (now - waiting_started_).seconds() : 0.0;
    if (waiting_age > local_waiting_timeout_) {
      throw nav2_core::PlannerException("SCAN could not generate a local collision-free trajectory");
    }
    return zeroCommand(now, true);
  }
  if (!have_trajectory_) {
    if ((now - generation_started_).seconds() > initial_plan_timeout_) {
      throw nav2_core::PlannerException("Timed out waiting for the first SCAN local trajectory");
    }
    return zeroCommand(now, true);
  }

  const double current_time = std::min(execution_time_, trajectory_duration_);
  const auto current_reference = position_.evaluate(current_time);
  const auto preview_velocity = velocity_trajectory_.evaluate(
    std::min(trajectory_duration_, current_time + preview_time_));
  const double desired_yaw = planarNorm(preview_velocity.x, preview_velocity.y) > 1e-4 ?
    std::atan2(preview_velocity.y, preview_velocity.x) : robot_yaw;
  const double yaw_error = normalizeAngle(desired_yaw - robot_yaw);
  const double absolute_yaw_error = std::abs(yaw_error);
  if (heading_only_) {
    heading_only_ = absolute_yaw_error > heading_error_exit_;
  } else {
    heading_only_ = absolute_yaw_error > heading_error_enter_;
  }
  const double controlled_yaw_error = absolute_yaw_error <= yaw_deadband_ ? 0.0 :
    std::copysign(absolute_yaw_error - yaw_deadband_, yaw_error);
  const double yaw_feedback = std::clamp(yaw_gain_ * controlled_yaw_error, -max_wz_, max_wz_);
  if (heading_only_) {
    geometry_msgs::msg::TwistStamped command = zeroCommand(now, true);
    command.twist.angular.z = yaw_feedback;
    publishWorldVelocity(command.twist, robot_yaw, now);
    return command;
  }

  execution_time_ = std::min(trajectory_duration_, execution_time_ + dt);
  const auto desired_position = position_.evaluate(execution_time_);
  const auto desired_velocity = velocity_trajectory_.evaluate(execution_time_);
  const auto desired_acceleration = acceleration_trajectory_.valid() ?
    acceleration_trajectory_.evaluate(execution_time_) : Vector3{};
  double error_x = desired_position.x - robot_x;
  double error_y = desired_position.y - robot_y;
  const double tangent_norm = planarNorm(desired_velocity.x, desired_velocity.y);
  if (tangent_norm > 1e-4) {
    const double tangent_x = desired_velocity.x / tangent_norm;
    const double tangent_y = desired_velocity.y / tangent_norm;
    const double along_track_error = error_x * tangent_x + error_y * tangent_y;
    if (along_track_error < 0.0) {
      error_x -= along_track_error * tangent_x;
      error_y -= along_track_error * tangent_y;
    }
  }
  const double tracking_error = planarNorm(error_x, error_y);
  if (tracking_error > tracking_replan_error_ &&
    (last_replan_request_.nanoseconds() == 0 ||
    (now - last_replan_request_).seconds() > replan_request_interval_))
  {
    std_msgs::msg::Bool request;
    request.data = true;
    replan_pub_->publish(request);
    last_replan_request_ = now;
    RCLCPP_WARN(
      logger_, "SCAN trajectory tracking error %.3f m; requested a fresh local trajectory",
      tracking_error);
  }

  double world_vx = desired_velocity.x + position_gain_ * error_x;
  double world_vy = desired_velocity.y + position_gain_ * error_y;
  const double configured_limit = std::min(max_vx_, std::max(0.0, speed_limit_));
  clampPlanarNorm(world_vx, world_vy, configured_limit);
  const double cosine = std::cos(robot_yaw);
  const double sine = std::sin(robot_yaw);
  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = now;
  command.header.frame_id = base_frame_;
  command.twist.linear.x = std::clamp(cosine * world_vx + sine * world_vy, 0.0, max_vx_);
  command.twist.linear.y = std::clamp(-sine * world_vx + cosine * world_vy, -max_vy_, max_vy_);

  double curvature = 0.0;
  if (tangent_norm > 1e-3) {
    curvature =
      (desired_velocity.x * desired_acceleration.y -
      desired_velocity.y * desired_acceleration.x) /
      std::max(1e-9, tangent_norm * tangent_norm * tangent_norm);
  }
  if (std::abs(curvature) > curvature_deadband_) {
    const double yaw_budget = std::max(0.05, max_wz_ - curvature_yaw_reserve_);
    clampPlanarNorm(
      command.twist.linear.x, command.twist.linear.y,
      yaw_budget / std::abs(curvature));
  }
  const double command_speed = planarNorm(command.twist.linear.x, command.twist.linear.y);
  command.twist.angular.z = std::clamp(
    yaw_feedback + curvature * command_speed, -max_wz_, max_wz_);

  const double goal_distance = planarNorm(final_goal_.x - robot_x, final_goal_.y - robot_y);
  const double current_speed = std::max(
    planarNorm(measured_velocity.linear.x, measured_velocity.linear.y), command_speed);
  const double usable_goal_distance =
    goal_distance - goal_stop_distance_ - goal_reaction_time_ * current_speed;
  const double goal_speed_limit = usable_goal_distance <= 0.0 ? 0.0 :
    std::sqrt(2.0 * std::max(0.05, goal_brake_decel_) * usable_goal_distance);
  clampPlanarNorm(command.twist.linear.x, command.twist.linear.y, goal_speed_limit);
  const double speed_after_braking = planarNorm(command.twist.linear.x, command.twist.linear.y);
  command.twist.angular.z = std::clamp(
    yaw_feedback + curvature * speed_after_braking, -max_wz_, max_wz_);

  if (execution_time_ >= trajectory_duration_ - 1e-6) {
    if (trajectory_completed_at_.nanoseconds() == 0) {
      trajectory_completed_at_ = now;
    }
    if (tracking_error < finish_distance_) {
      command.twist = geometry_msgs::msg::Twist();
    }
    if ((now - trajectory_completed_at_).seconds() > trajectory_refresh_timeout_) {
      throw nav2_core::PlannerException("SCAN local trajectory ended without a fresh continuation");
    }
  } else {
    trajectory_completed_at_ = rclcpp::Time(0, 0, now.get_clock_type());
  }

  publishFrozen(false);
  publishWorldVelocity(command.twist, robot_yaw, now);
  return command;
}

void ScanTrajectoryController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!std::isfinite(speed_limit) || speed_limit < 0.0) {
    speed_limit_ = max_vx_;
  } else if (percentage) {
    speed_limit_ = max_vx_ * std::clamp(speed_limit, 0.0, 100.0) / 100.0;
  } else {
    speed_limit_ = std::min(max_vx_, speed_limit);
  }
}

}  // namespace go2_scan_nav2_controller

PLUGINLIB_EXPORT_CLASS(
  go2_scan_nav2_controller::ScanTrajectoryController,
  nav2_core::Controller)
