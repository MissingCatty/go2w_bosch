#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
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
    finish_dist_ = declare_parameter<double>("finish_dist", 0.15);
    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.5);
    cloud_timeout_ = std::max(0.2, declare_parameter<double>("cloud_timeout", 0.6));
    trajectory_timeout_ = declare_parameter<double>("trajectory_timeout", 0.5);
    max_tracking_error_ = std::max(
        0.05, declare_parameter<double>("max_tracking_error", 0.25));
    tracking_error_release_ = std::clamp(
        declare_parameter<double>("tracking_error_release", 0.12),
        0.0, max_tracking_error_);
    tracking_stall_timeout_ = std::max(
        0.5, declare_parameter<double>("tracking_stall_timeout", 2.5));

    bspline_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(
        "planning/bspline", 10,
        std::bind(&ClosedLoopController::bsplineCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&ClosedLoopController::odomCallback, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud", rclcpp::SensorDataQoS(),
        std::bind(&ClosedLoopController::cloudCallback, this, std::placeholders::_1));
    navigation_cancel_sub_ = create_subscription<std_msgs::msg::Bool>(
        "navigation_cancel", 10,
        std::bind(&ClosedLoopController::navigationCancelCallback, this, std::placeholders::_1));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 20);
    local_path_pub_ = create_publisher<nav_msgs::msg::Path>("planning/local_path", 10);
    execution_frozen_pub_ = create_publisher<std_msgs::msg::Bool>("planning/go2_execution_frozen", 10);
    replan_request_pub_ = create_publisher<std_msgs::msg::Bool>("planning/replan_request", 10);
    cmd_timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                   std::bind(&ClosedLoopController::cmdCallback, this));
    last_update_time_ = now();
    last_odom_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_cloud_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    completion_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    tracking_pause_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    RCLCPP_INFO(get_logger(), "Closed-loop controller ready");
  }

private:
  static constexpr double kMaxVYawLimit = 1.0;

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

  double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des) const
  {
    const double t_look = std::min(traj_duration_, t_cur + time_forward_);
    Eigen::Vector3d direction = traj_[0].evaluateDeBoorT(t_look) - pos_des;
    if (direction.head<2>().squaredNorm() < 1e-4)
      direction = traj_[1].evaluateDeBoorT(t_cur);
    return direction.head<2>().squaredNorm() < 1e-4
        ? odom_yaw_ : std::atan2(direction.y(), direction.x());
  }

  void publishStop(double yaw_rate = 0.0)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = std::clamp(yaw_rate, -max_vyaw_, max_vyaw_);
    cmd_vel_pub_->publish(cmd);
  }

  void publishExecutionFrozen(bool frozen)
  {
    std_msgs::msg::Bool msg;
    msg.data = frozen;
    execution_frozen_pub_->publish(msg);
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
    traj_ = {position, position.getDerivative()};
    traj_.push_back(traj_[1].getDerivative());
    traj_duration_ = traj_[0].getTimeSum();
    traj_id_ = msg->traj_id;
    exec_time_ = 0.0;
    last_update_time_ = now();
    completion_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    tracking_pause_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    heading_only_ = false;
    tracking_frozen_ = false;
    replan_requested_ = false;
    receive_traj_ = true;
    publishLocalPath();
    RCLCPP_INFO(get_logger(), "Received trajectory %lld, duration %.3fs",
                static_cast<long long>(traj_id_), traj_duration_);
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
    have_odom_ = true;
    last_odom_time_ = now();
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    if (!msg || msg->data.empty() || msg->width * msg->height == 0)
      return;
    have_cloud_ = true;
    last_cloud_time_ = now();
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
    tracking_pause_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    heading_only_ = false;
    tracking_frozen_ = false;
    replan_requested_ = false;
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
      publishExecutionFrozen(false);
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
    const double yaw_command = std::clamp(
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
      publishStop(yaw_command);
      last_update_time_ = current_time;
      return;
    }

    // SCAN plans at a higher nominal speed than the host safety gate permits.
    // Advance trajectory time only while the physical robot is close enough
    // to the current reference; otherwise the reference runs away and expires
    // before the limited chassis can catch it.
    const double tracking_error = (pos_des - odom_pos_).head<2>().norm();
    const bool previous_tracking_frozen = tracking_frozen_;
    if (tracking_frozen_)
      tracking_frozen_ = tracking_error > tracking_error_release_;
    else
      tracking_frozen_ = tracking_error > max_tracking_error_;
    if (tracking_frozen_ != previous_tracking_frozen)
    {
      RCLCPP_INFO(get_logger(), "Trajectory tracking clock %s at %.3f m error",
                  tracking_frozen_ ? "paused" : "resumed", tracking_error);
      tracking_pause_time_ = tracking_frozen_
          ? current_time : rclcpp::Time(0, 0, get_clock()->get_clock_type());
      if (!tracking_frozen_)
        replan_requested_ = false;
    }

    if (tracking_frozen_ && !replan_requested_ && tracking_pause_time_.nanoseconds() > 0 &&
        (current_time - tracking_pause_time_).seconds() > tracking_stall_timeout_)
    {
      std_msgs::msg::Bool request;
      request.data = true;
      replan_request_pub_->publish(request);
      replan_requested_ = true;
      RCLCPP_WARN(get_logger(),
                  "Tracking remained %.3fm behind for %.1fs; requesting a fresh local trajectory",
                  tracking_error, tracking_stall_timeout_);
    }

    publishExecutionFrozen(tracking_frozen_);
    if (!tracking_frozen_)
      exec_time_ = std::min(traj_duration_, exec_time_ + dt);
    last_update_time_ = current_time;
    pos_des = traj_[0].evaluateDeBoorT(exec_time_);
    const Eigen::Vector3d vel_des = traj_[1].evaluateDeBoorT(exec_time_);
    const Eigen::Vector2d pos_error(pos_des.x() - odom_pos_.x(), pos_des.y() - odom_pos_.y());
    const Eigen::Vector2d vel_world = clampNorm(
        Eigen::Vector2d(vel_des.x(), vel_des.y()) + kp_pos_ * pos_error,
        std::max(max_vx_, max_vy_));
    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);
    geometry_msgs::msg::Twist command;
    command.linear.x = std::clamp(c * vel_world.x() + s * vel_world.y(), -max_vx_, max_vx_);
    command.linear.y = std::clamp(-s * vel_world.x() + c * vel_world.y(), -max_vy_, max_vy_);
    command.angular.z = yaw_command;
    if (exec_time_ >= traj_duration_ && completion_time_.nanoseconds() == 0)
      completion_time_ = current_time;
    if (exec_time_ >= traj_duration_ && pos_error.norm() < finish_dist_)
      command = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(command);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_frozen_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr replan_request_pub_;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr navigation_cancel_sub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  bool receive_traj_{false};
  bool have_odom_{false};
  bool have_cloud_{false};
  bool navigation_cancelled_{false};
  bool cloud_stale_stop_{false};
  std::vector<UniformBspline> traj_;
  double traj_duration_{0.0};
  std::int64_t traj_id_{0};
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  double odom_yaw_{0.0};
  double exec_time_{0.0};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_cloud_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time completion_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time tracking_pause_time_{0, 0, RCL_ROS_TIME};
  bool heading_only_{false};
  bool tracking_frozen_{false};
  bool replan_requested_{false};
  double time_forward_, heading_error_threshold_, heading_error_release_threshold_;
  double yaw_deadband_, kp_pos_, kp_yaw_;
  double max_tracking_error_, tracking_error_release_;
  double tracking_stall_timeout_;
  double max_vx_, max_vy_, max_vyaw_, finish_dist_, odom_timeout_, cloud_timeout_,
      trajectory_timeout_;
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::ClosedLoopController>());
  rclcpp::shutdown();
  return 0;
}
