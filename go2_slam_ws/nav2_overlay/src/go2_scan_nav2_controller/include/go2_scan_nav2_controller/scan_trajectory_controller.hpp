#ifndef GO2_SCAN_NAV2_CONTROLLER__SCAN_TRAJECTORY_CONTROLLER_HPP_
#define GO2_SCAN_NAV2_CONTROLLER__SCAN_TRAJECTORY_CONTROLLER_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "scan_planner_msgs/msg/bspline.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"

#include "go2_scan_nav2_controller/bspline.hpp"

namespace go2_scan_nav2_controller
{

class ScanTrajectoryController : public nav2_core::Controller
{
public:
  ScanTrajectoryController() = default;
  ~ScanTrajectoryController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  using LifecycleNode = rclcpp_lifecycle::LifecycleNode;

  template<typename MessageT>
  using LifecyclePublisher = rclcpp_lifecycle::LifecyclePublisher<MessageT>;

  void onTrajectory(const scan_planner_msgs::msg::Bspline::ConstSharedPtr message);
  void onLocalWaiting(const std_msgs::msg::Bool::ConstSharedPtr message);
  void watchdog();
  void publishFrozen(bool frozen);
  void publishWorldVelocity(
    const geometry_msgs::msg::Twist & command, double yaw, const rclcpp::Time & stamp);
  void publishLocalPath(const BSpline & position, double duration, const rclcpp::Time & stamp);
  nav_msgs::msg::Path transformPlan(const nav_msgs::msg::Path & path) const;
  geometry_msgs::msg::PoseStamped transformPose(
    const geometry_msgs::msg::PoseStamped & pose) const;
  geometry_msgs::msg::TwistStamped zeroCommand(const rclcpp::Time & stamp, bool frozen);
  static double normalizeAngle(double angle);

  LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("ScanTrajectoryController")};
  std::string plugin_name_;
  std::string local_frame_{"odom"};
  std::string base_frame_{"base_link"};

  typename LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
  typename LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  typename LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr frozen_pub_;
  typename LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr replan_pub_;
  typename LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr waiting_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  mutable std::mutex mutex_;
  bool active_{false};
  bool plan_active_{false};
  bool have_trajectory_{false};
  bool local_waiting_{false};
  bool heading_only_{false};
  bool watchdog_cancelled_{false};
  BSpline position_;
  BSpline velocity_trajectory_;
  BSpline acceleration_trajectory_;
  nav_msgs::msg::Path local_plan_;
  Vector3 final_goal_;
  std::int64_t trajectory_id_{-1};
  double trajectory_duration_{0.0};
  double execution_time_{0.0};
  double speed_limit_{0.0};
  rclcpp::Time generation_started_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_compute_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_control_update_{0, 0, RCL_ROS_TIME};
  rclcpp::Time trajectory_completed_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time waiting_started_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_replan_request_{0, 0, RCL_ROS_TIME};

  double transform_tolerance_{0.30};
  double initial_plan_timeout_{2.0};
  double local_waiting_timeout_{3.0};
  double trajectory_refresh_timeout_{0.8};
  double compute_watchdog_timeout_{0.6};
  double preview_time_{0.20};
  double heading_error_enter_{0.80};
  double heading_error_exit_{0.35};
  double yaw_deadband_{0.08};
  double position_gain_{0.70};
  double yaw_gain_{0.80};
  double max_vx_{0.50};
  double max_vy_{0.10};
  double max_wz_{0.45};
  double curvature_yaw_reserve_{0.08};
  double curvature_deadband_{0.05};
  double finish_distance_{0.15};
  double goal_stop_distance_{0.15};
  double goal_brake_decel_{0.45};
  double goal_reaction_time_{0.15};
  double tracking_replan_error_{0.50};
  double replan_request_interval_{0.75};
  double local_path_sample_dt_{0.10};
};

}  // namespace go2_scan_nav2_controller

#endif  // GO2_SCAN_NAV2_CONTROLLER__SCAN_TRAJECTORY_CONTROLLER_HPP_
