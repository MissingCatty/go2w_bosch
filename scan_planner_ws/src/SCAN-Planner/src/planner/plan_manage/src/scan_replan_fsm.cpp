
#include <plan_manage/scan_replan_fsm.h>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
  template <typename T>
  T load_parameter(rclcpp::Node *node, const std::string &name, const T &default_value)
  {
    if (!node->has_parameter(name)) node->declare_parameter<T>(name, default_value);
    return node->get_parameter(name).get_value<T>();
  }
} // namespace

namespace scan_planner
{

  void SCANReplanFSM::init(rclcpp::Node *node)
  {
    node_ = node;
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    trigger_ = false;
    have_target_ = false;
    have_odom_ = false;
    have_new_target_ = false;
    rviz_height_ready_ = false;
    go2_execution_frozen_ = false;
    flag_escape_emergency_ = true;
    need_hover_stop_ = false;
    replan_fail_count_ = 0;
    last_freeze_update_time_ = node_->now();
    next_replan_time_ = node_->now();
    next_periodic_replan_time_ = node_->now();
    last_controller_velocity_time_ = rclcpp::Time(
        0, 0, node_->get_clock()->get_clock_type());

    /*  fsm param  */
    navi_mode_ = load_parameter<int>(node_, "fsm.navi_mode", -1);
    no_replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_no_replan", -1.0);
    planning_horizon_ = load_parameter<double>(node_, "fsm.planning_horizon", -1.0);
    emergency_time_ = load_parameter<double>(node_, "fsm.emergency_time", 1.0);
    replan_retry_interval_ = std::max(
        0.2, load_parameter<double>(node_, "fsm.replan_retry_interval", 1.0));
    periodic_replan_interval_ = std::max(
        0.05, load_parameter<double>(node_, "fsm.periodic_replan_interval", 0.20));
    periodic_replan_enabled_ =
        load_parameter<bool>(node_, "fsm.periodic_replan_enabled", true);
    collision_replan_horizon_ = std::max(
        0.10, load_parameter<double>(node_, "fsm.collision_replan_horizon", 0.50));
    controller_velocity_timeout_ = std::max(
        periodic_replan_interval_,
        load_parameter<double>(node_, "fsm.controller_velocity_timeout", 0.35));
    goal_stop_speed_ = std::clamp(
        load_parameter<double>(node_, "fsm.goal_stop_speed", 0.08), 0.02, 0.30);
    // Keep the trajectory replanning threshold precise, but allow a stopped
    // ground robot to finish when the final sub-metre spline cannot satisfy
    // acceleration limits.  This is intentionally independent from
    // thresh_no_replan so it cannot shorten normal path tracking.
    goal_arrival_tolerance_ = std::clamp(
        load_parameter<double>(node_, "fsm.goal_arrival_tolerance", 0.30),
        no_replan_thresh_, 0.50);
    live_cloud_timeout_ = std::max(
        0.2, load_parameter<double>(node_, "fsm.live_cloud_timeout", 0.60));
    reference_entry_tolerance_ = std::clamp(
        load_parameter<double>(node_, "fsm.reference_entry_tolerance", 0.18),
        0.08, 0.30);
    resume_clear_observations_ = std::clamp(
        load_parameter<int>(node_, "fsm.resume_clear_observations", 3), 1, 20);
    enable_fail_safe_ = load_parameter<bool>(node_, "fsm.fail_safe", true);
    max_replan_fail_count_ = load_parameter<int>(node_, "fsm.max_replan_fail_count", 1000);
    self_inflation_z_up_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_up", 0.0);
    self_inflation_z_down_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_down", 0.0);
    self_double_cylinder_radius_ = load_parameter<double>(node_, "grid_map.double_cylinder_radius", 0.0);
    self_double_cylinder_offset_ = load_parameter<double>(node_, "grid_map.double_cylinder_offset", 0.0);
    body_height_ = load_parameter<double>(node_, "grid_map.body_height", 0.4);
    self_inflation_frame_id_ = load_parameter<std::string>(node_, "grid_map.frame_id", "world");

    if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
    {
      const auto flat_waypoints = load_parameter<std::vector<double>>(node_, "fsm.waypoints", {});
      if (flat_waypoints.empty() || flat_waypoints.size() % 3 != 0)
        throw std::runtime_error("navi_mode=2 requires non-empty fsm.waypoints with x,y,z triples");
      waypoint_num_ = static_cast<int>(flat_waypoints.size() / 3);
      preset_waypoints_.resize(waypoint_num_);
      for (int i = 0; i < waypoint_num_; i++)
      {
        preset_waypoints_[i] = Eigen::Vector3d(flat_waypoints[3 * i], flat_waypoints[3 * i + 1],
                                               flat_waypoints[3 * i + 2]);
      }
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(node_));
    planner_manager_.reset(new SCANPlannerManager);
    planner_manager_->initPlanModules(node_, visualization_);

    /* callback */
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&SCANReplanFSM::execFSMCallback, this));
    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                             std::bind(&SCANReplanFSM::checkCollisionCallback, this));
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&SCANReplanFSM::odometryCallback, this, std::placeholders::_1));
    go2_execution_frozen_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "planning/go2_execution_frozen", 10,
        std::bind(&SCANReplanFSM::go2ExecutionFrozenCallback, this, std::placeholders::_1));
    replan_request_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "planning/replan_request", 10,
        std::bind(&SCANReplanFSM::replanRequestCallback, this, std::placeholders::_1));
    navigation_cancel_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "navigation_cancel", 10,
        std::bind(&SCANReplanFSM::navigationCancelCallback, this, std::placeholders::_1));
    controller_velocity_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
        "planning/controller_velocity_world", 20,
        std::bind(&SCANReplanFSM::controllerVelocityCallback, this, std::placeholders::_1));

    bspline_pub_ = node_->create_publisher<scan_planner_msgs::msg::Bspline>("planning/bspline", 10);
    data_disp_pub_ = node_->create_publisher<scan_planner_msgs::msg::DataDisp>("planning/data_display", 100);
    local_horizon_pub_ =
        node_->create_publisher<nav_msgs::msg::Path>("planning/local_horizon", 10);
    self_inflation_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "self_inflation", rclcpp::QoS(1).reliable().transient_local());
    navigation_completed_pub_ =
        node_->create_publisher<std_msgs::msg::Bool>("navigation_completed", 10);
    local_waiting_pub_ =
        node_->create_publisher<std_msgs::msg::Bool>("planning/local_waiting", 10);
    publishLocalWaiting(false);
    RCLCPP_INFO(
        node_->get_logger(),
        "Local replanning policy: %s, live-path collision horizon %.2fm",
        periodic_replan_enabled_ ? "periodic" : "event-triggered",
        collision_replan_horizon_);

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET)
      goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "move_base_simple/goal", 1,
          std::bind(&SCANReplanFSM::rvizGoalCallback, this, std::placeholders::_1));
    else if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
      path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
          "initial_path", 1, std::bind(&SCANReplanFSM::pathCallback, this, std::placeholders::_1));
    else if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
      RCLCPP_INFO(node_->get_logger(), "Preset waypoint mode will start after the first odometry message");
    else
      throw std::runtime_error("fsm.navi_mode must be 1, 2, or 3");
  }

  void SCANReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps = preset_waypoints_;

    for (size_t i = 0; i < wps.size(); i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    }

    active_waypoints_ = wps;
    current_wp_ = 0;
    trigger_ = true;
    init_pt_ = odom_pos_;

    if (planNextWaypoint())
    {
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory to first preset waypoint");
    }
  }

  void SCANReplanFSM::rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
  {
    if (!msg)
      return;

    if (!rviz_height_ready_)
    {
      RCLCPP_WARN(node_->get_logger(), "Ignore RViz goal before receiving initial body pose");
      return;
    }

    auto path = std::make_shared<nav_msgs::msg::Path>();
    path->header = msg->header;
    path->poses.push_back(*msg);
    waypointCallback(path);
  }

  void SCANReplanFSM::waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Empty waypoint message; ignoring");
      return;
    }

    if (msg->poses[0].pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = odom_pos_;

    bool success = false;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, rviz_goal_height_;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
      success = adjustGlobalTargetIfOccupied();

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory");
    }
  }

  bool SCANReplanFSM::planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints)
  {
    if (waypoints.empty())
    {
      RCLCPP_WARN(node_->get_logger(), "No waypoint supplied for global trajectory");
      return false;
    }

    end_pt_ = waypoints.back();

    for (size_t i = 0; i < waypoints.size(); i++)
    {
      visualization_->displayGoalPoint(waypoints[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    }

    bool success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_,
        odom_vel_,
        Eigen::Vector3d::Zero(),
        waypoints,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from waypoints");
      return false;
    }

    // A live obstacle at the final Web goal may be temporary (for example a
    // person standing there).  Keep the original reference-path goal and let
    // the local FSM wait/retry instead of silently shortening the mission.
    if (navi_mode_ != NAVI_MODE::REFERENCE_PATH &&
        !adjustGlobalTargetIfOccupied())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, static_cast<int>(waypoints.size()) - 1);

    return true;
  }

  bool SCANReplanFSM::planNextWaypoint()
  {
    if (current_wp_ < 0 || current_wp_ >= (int)active_waypoints_.size())
    {
      RCLCPP_WARN(node_->get_logger(), "[navi_mode=%d] No active waypoint to plan", navi_mode_);
      return false;
    }

    end_pt_ = active_waypoints_[current_wp_];
    setStartStateFromOdomOrCurrentTraj();

    bool success = planner_manager_->planGlobalTraj(
        start_pt_,
        start_vel_,
        start_acc_,
        end_pt_,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(), "[navi_mode=%d] Unable to generate trajectory to waypoint %d",
                   navi_mode_, current_wp_ + 1);
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, current_wp_);
    RCLCPP_INFO(node_->get_logger(), "[navi_mode=%d] Planning to waypoint %d/%zu: [%.2f, %.2f, %.2f]",
                navi_mode_, current_wp_ + 1, active_waypoints_.size(), end_pt_(0), end_pt_(1), end_pt_(2));

    return true;
  }

  bool SCANReplanFSM::isWaypointSequenceMode() const
  {
    return navi_mode_ == NAVI_MODE::PRESET_TARGET;
  }

  bool SCANReplanFSM::adjustGlobalTargetIfOccupied()
  {
    auto map = planner_manager_->grid_map_;
    auto &global_data = planner_manager_->global_data_;
    const double duration = global_data.global_duration_;
    if (!map || duration < 1e-3)
      return true;

    constexpr double sample_dt = 0.05;
    const int sample_num = std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));
    const Eigen::Vector3d final_pt = global_data.global_traj_.evaluate(duration);
    const Eigen::Vector3d final_prev = global_data.global_traj_.evaluate(duration * (sample_num - 1) / sample_num);
    const int final_occ = map->getInflateOccupancy(final_pt, estimateYawFromSegment(final_prev, final_pt));
    if (final_occ <= 0)
      return true;

    for (int i = sample_num; i >= 0; --i)
    {
      const double t = duration * i / sample_num;
      const double prev_t = duration * std::max(0, i - 1) / sample_num;
      const Eigen::Vector3d pt = global_data.global_traj_.evaluate(t);
      const Eigen::Vector3d prev_pt = global_data.global_traj_.evaluate(prev_t);

      if (map->getInflateOccupancy(pt, estimateYawFromSegment(prev_pt, pt)) == 0)
      {
        const Eigen::Vector3d raw_end = end_pt_;
        end_pt_ = pt;
        global_data.global_duration_ = t;
        global_data.last_progress_time_ = std::min(global_data.last_progress_time_, t);
        RCLCPP_WARN(node_->get_logger(),
                    "Target [%.2f, %.2f, %.2f] is occupied; using [%.2f, %.2f, %.2f]",
                    raw_end(0), raw_end(1), raw_end(2), end_pt_(0), end_pt_(1), end_pt_(2));
        return true;
      }
    }

    RCLCPP_ERROR(node_->get_logger(),
                 "Target is occupied and no collision-free point was found on the global trajectory");
    return false;
  }

  void SCANReplanFSM::pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Received empty initial_path; ignoring");
      return;
    }

    trigger_ = true;

    std::vector<Eigen::Vector3d> waypoints;
    waypoints.reserve(msg->poses.size());

    for (const auto& pose_stamped : msg->poses)
    {
      Eigen::Vector3d wp;
      wp(0) = pose_stamped.pose.position.x;
      wp(1) = pose_stamped.pose.position.y;
      wp(2) = pose_stamped.pose.position.z + body_height_; // Adjust for body height
      waypoints.push_back(wp);
    }
    // Keep the Web reference for every future local replan.  The previous
    // implementation rebuilt a straight global trajectory to the final goal
    // after the first local failure, which could cut through a wall even
    // though the Web A* path correctly went around it.
    active_waypoints_ = waypoints;
    current_wp_ = 0;
    // A normal Web route starts within 15 cm of odometry. A farther first
    // point is an explicit temporary static-map entry: retain it across 5 Hz
    // replans until the live local planner has actually reached it.
    const double entry_distance =
        (active_waypoints_.front() - odom_pos_).head<2>().norm();
    staging_reference_entry_ =
        active_waypoints_.size() > 1 && have_odom_ &&
        entry_distance > std::max(0.25, reference_entry_tolerance_ + 0.05);
    if (staging_reference_entry_)
      RCLCPP_WARN(node_->get_logger(),
                  "Staging temporary reference entry [%.2f, %.2f], %.2fm from odometry",
                  active_waypoints_.front().x(), active_waypoints_.front().y(),
                  entry_distance);
    bool success = planGlobalTrajByWaypoints(active_waypoints_);

    if (success)
    {
      resetResumeConfirmation();
      publishLocalWaiting(false);
      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET || exec_state_ == WAIT_REPLAN ||
          exec_state_ == EMERGENCY_STOP)
      {
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }
      else if (exec_state_ == EXEC_TRAJ)
      {
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      RCLCPP_INFO(node_->get_logger(), "Reference path accepted");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from reference path");
    }
  }

  void SCANReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET && !rviz_height_ready_)
    {
      rviz_goal_height_ = odom_pos_(2);
      rviz_height_ready_ = true;
      RCLCPP_INFO(node_->get_logger(), "Set RViz goal height from initial body_pose z: %.3f", rviz_goal_height_);
    }

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
    publishSelfInflationMarker();
    if (navi_mode_ == NAVI_MODE::PRESET_TARGET && !preset_started_)
    {
      preset_started_ = true;
      planGlobalTrajbyGivenWps();
    }
  }

  void SCANReplanFSM::controllerVelocityCallback(
      const geometry_msgs::msg::TwistStamped::ConstSharedPtr &msg)
  {
    if (!msg || !std::isfinite(msg->twist.linear.x) ||
        !std::isfinite(msg->twist.linear.y) ||
        !std::isfinite(msg->twist.linear.z))
      return;

    controller_velocity_world_ << msg->twist.linear.x,
        msg->twist.linear.y, msg->twist.linear.z;
    last_controller_velocity_time_ = node_->now();
    have_controller_velocity_ = true;
  }

  bool SCANReplanFSM::useFreshControllerVelocity(Eigen::Vector3d &velocity)
  {
    if (!have_controller_velocity_)
      return false;
    const double age = (node_->now() - last_controller_velocity_time_).seconds();
    if (age < 0.0 || age > controller_velocity_timeout_)
      return false;
    velocity = controller_velocity_world_;
    if (velocity.head<2>().norm() > 0.01)
      RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "Seeding replacement trajectory from controller velocity %.3f %.3f m/s",
          velocity.x(), velocity.y());
    return true;
  }

  void SCANReplanFSM::navigationCancelCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
  {
    if (!msg || !msg->data)
      return;

    trigger_ = false;
    have_target_ = false;
    have_new_target_ = false;
    active_waypoints_.clear();
    local_reference_points_.clear();
    current_wp_ = 0;
    staging_reference_entry_ = false;
    replan_fail_count_ = 0;
    resetResumeConfirmation();
    force_replan_from_odom_ = false;
    periodic_replan_attempt_ = false;
    need_hover_stop_ = false;
    publishLocalWaiting(false);
    flag_escape_emergency_ = true;
    changeFSMExecState(have_odom_ ? WAIT_TARGET : INIT, "CANCEL");
    RCLCPP_WARN(node_->get_logger(), "Navigation cancelled; waiting for a new target");
  }

  void SCANReplanFSM::go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
  {
    go2_execution_frozen_ = msg->data;
  }

  void SCANReplanFSM::replanRequestCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
  {
    if (!msg || !msg->data || !have_target_ ||
        (exec_state_ != EXEC_TRAJ && exec_state_ != REPLAN_TRAJ))
      return;
    force_replan_from_odom_ = true;
    // A tracking-invalidated trajectory is not a best-effort 5 Hz refresh.
    // Convert an already queued periodic attempt into a mandatory replacement
    // so failure cannot return to the stale trajectory.
    periodic_replan_attempt_ = false;
    if (exec_state_ == EXEC_TRAJ)
      changeFSMExecState(REPLAN_TRAJ, "TRACKING");
    RCLCPP_WARN(node_->get_logger(),
                "Controller tracking lag requested replanning from current odometry");
  }

  void SCANReplanFSM::updateLocalTrajTimeFreeze()
  {
    const rclcpp::Time now = node_->now();
    double dt = (now - last_freeze_update_time_).seconds();
    last_freeze_update_time_ = now;

    if (dt <= 0.0 || dt > 0.2)
      return;

    LocalTrajData *info = &planner_manager_->local_data_;
    if (go2_execution_frozen_ && info->start_time_.seconds() > 1e-5)
      info->start_time_ += rclcpp::Duration::from_seconds(dt);
  }

  void SCANReplanFSM::publishLocalWaiting(bool waiting)
  {
    if (!local_waiting_pub_)
      return;
    std_msgs::msg::Bool message;
    message.data = waiting;
    local_waiting_pub_->publish(message);
  }

  void SCANReplanFSM::completeNavigation(double actual_goal_distance)
  {
    active_waypoints_.clear();
    local_reference_points_.clear();
    current_wp_ = 0;
    staging_reference_entry_ = false;
    have_target_ = false;
    have_new_target_ = false;
    trigger_ = false;
    replan_fail_count_ = 0;
    resetResumeConfirmation();
    force_replan_from_odom_ = false;
    periodic_replan_attempt_ = false;
    need_hover_stop_ = false;
    flag_escape_emergency_ = true;

    std_msgs::msg::Bool completed;
    completed.data = true;
    navigation_completed_pub_->publish(completed);
    publishLocalWaiting(false);
    RCLCPP_INFO(node_->get_logger(),
                "Navigation goal reached (actual distance %.3fm)",
                actual_goal_distance);
    changeFSMExecState(WAIT_TARGET, "GOAL_REACHED");
  }

  bool SCANReplanFSM::completeNavigationIfArrivedAndStopped()
  {
    if (!have_odom_ || !have_target_ || !trigger_)
      return false;
    // Preset mode treats each intermediate point as a mission step.  Its
    // existing EXEC_TRAJ logic advances those waypoints before completion.
    if (isWaypointSequenceMode() &&
        current_wp_ + 1 < static_cast<int>(active_waypoints_.size()))
      return false;

    const double goal_distance =
        (end_pt_.head<2>() - odom_pos_.head<2>()).norm();
    const double planar_speed = odom_vel_.head<2>().norm();
    if (goal_distance >= goal_arrival_tolerance_ ||
        planar_speed >= goal_stop_speed_)
      return false;

    // This check also runs in WAIT_REPLAN/EMERGENCY_STOP, where the previous
    // local trajectory may already have expired. Publish an explicit hold
    // before reporting completion so actuator output reaches zero immediately.
    callEmergencyStop(odom_pos_);
    RCLCPP_INFO(node_->get_logger(),
                "Goal reached outside normal trajectory completion: distance %.3fm, speed %.3fm/s",
                goal_distance, planar_speed);
    completeNavigation(goal_distance);
    return true;
  }

  void SCANReplanFSM::resetResumeConfirmation()
  {
    resume_clear_count_ = 0;
    last_resume_observation_sequence_ = 0;
  }

  bool SCANReplanFSM::holdIfLiveObservationStale()
  {
    if (!have_target_ || !trigger_ ||
        planner_manager_->grid_map_->hasFreshObservation(live_cloud_timeout_))
      return false;

    resetResumeConfirmation();
    if (exec_state_ != WAIT_REPLAN)
    {
      callEmergencyStop(odom_pos_);
      force_replan_from_odom_ = true;
      next_replan_time_ = node_->now() +
          rclcpp::Duration::from_seconds(replan_retry_interval_);
      publishLocalWaiting(true);
      changeFSMExecState(WAIT_REPLAN, "LIDAR_TIMEOUT");
    }
    RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "No fused live cloud for %.2fs; holding zero velocity and refusing to plan",
        live_cloud_timeout_);
    return true;
  }

  double SCANReplanFSM::getOdomYaw() const
  {
    Eigen::Vector3d heading = odom_orient_.toRotationMatrix().col(0);
    if (heading.head<2>().squaredNorm() < 1e-8)
      return 0.0;
    return std::atan2(heading(1), heading(0));
  }

  double SCANReplanFSM::estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const
  {
    Eigen::Vector2d diff(to(0) - from(0), to(1) - from(1));
    if (diff.squaredNorm() < 1e-8)
      return getOdomYaw();
    return std::atan2(diff(1), diff(0));
  }

  void SCANReplanFSM::publishSelfInflationMarker()
  {
    const double radius = std::max(0.0, self_double_cylinder_radius_);
    const double z_up = std::max(0.0, self_inflation_z_up_);
    const double z_down = std::max(0.0, self_inflation_z_down_);
    const double height = std::max(1e-3, z_up + z_down);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = self_inflation_frame_id_.empty() ? "world" : self_inflation_frame_id_;
    marker.header.stamp = node_->now();
    marker.ns = "self_inflation";
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = height;
    marker.color.r = 0.1;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
    marker.color.a = 0.4;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    Eigen::Vector3d center = odom_pos_;
    center(2) += 0.5 * (z_up - z_down);

    Eigen::Vector3d heading(std::cos(getOdomYaw()), std::sin(getOdomYaw()), 0.0);
    Eigen::Vector3d front = center + self_double_cylinder_offset_ * heading;
    Eigen::Vector3d rear = center - self_double_cylinder_offset_ * heading;

    marker.id = 0;
    marker.pose.position.x = front(0);
    marker.pose.position.y = front(1);
    marker.pose.position.z = front(2);
    self_inflation_pub_->publish(marker);

    marker.id = 1;
    marker.pose.position.x = rear(0);
    marker.pose.position.y = rear(1);
    marker.pose.position.z = rear(2);
    self_inflation_pub_->publish(marker);
  }

  void SCANReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continuously_called_times_++;
    else
      continuously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "WAIT_REPLAN", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> SCANReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continuously_called_times_, exec_state_);
  }

  void SCANReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "WAIT_REPLAN", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void SCANReplanFSM::execFSMCallback()
  {
    updateLocalTrajTimeFreeze();
    completeNavigationIfArrivedAndStopped();
    if (holdIfLiveObservationStale())
      return;

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      setStartStateFromOdomOrCurrentTraj();

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init);
      if (success)
      {

        replan_fail_count_ = 0;
        publishLocalWaiting(false);
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      const bool periodic_attempt = periodic_replan_attempt_;
      if (planFromCurrentTraj())
      {
        periodic_replan_attempt_ = false;
        replan_fail_count_ = 0;
        publishLocalWaiting(false);
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else if (periodic_attempt)
      {
        // A 5 Hz baseline refresh must not discard the trajectory currently
        // being executed merely because one candidate failed optimization.
        // The 20 Hz collision checker remains responsible for stopping an
        // unsafe old trajectory.
        periodic_replan_attempt_ = false;
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 2000,
            "Periodic local refresh failed; keeping the current collision-checked trajectory");
        changeFSMExecState(EXEC_TRAJ, "PERIODIC");
      }
      else
      {
        replan_fail_count_++;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = node_->now();
      double t_cur = (time_now - info->start_time_).seconds();
      t_cur = min(info->duration_, t_cur);

      if (isWaypointSequenceMode() &&
          current_wp_ + 1 < (int)active_waypoints_.size() &&
          (end_pt_ - odom_pos_).norm() < 0.5)
      {
        current_wp_++;
        if (planNextWaypoint())
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        return;
      }

      if (t_cur > info->duration_ - 1e-2)
      {
        if (isWaypointSequenceMode() && current_wp_ + 1 < (int)active_waypoints_.size())
        {
          current_wp_++;
          if (planNextWaypoint())
          {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            return;
          }
          replan_fail_count_++;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }

        const double actual_goal_distance =
            (end_pt_.head<2>() - odom_pos_.head<2>()).norm();
        if (actual_goal_distance > no_replan_thresh_)
        {
          RCLCPP_WARN(node_->get_logger(),
                      "Local trajectory ended %.2fm from actual goal; replanning",
                      actual_goal_distance);
          changeFSMExecState(REPLAN_TRAJ, "FSM");
          return;
        }

        completeNavigation(actual_goal_distance);
        return;
      }
      else if ((end_pt_.head<2>() - odom_pos_.head<2>()).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        return;
      }
      else if (!periodic_replan_enabled_)
      {
        // Keep executing the accepted local B-spline while its next collision
        // horizon is clear. The 20 Hz safety callback below owns live-obstacle
        // replanning; tracking recovery and trajectory-end extension remain
        // independent event triggers.
        return;
      }
      else if (time_now < next_periodic_replan_time_)
      {
        return;
      }
      else
      {
        // Baseline policy: replace distance-triggered replanning with a fixed
        // 5 Hz refresh.  When tracking is frozen, seed from measured odometry
        // so a stationary robot does not wait for virtual path progress.
        periodic_replan_attempt_ = true;
        force_replan_from_odom_ = go2_execution_frozen_;
        next_periodic_replan_time_ = time_now +
            rclcpp::Duration::from_seconds(periodic_replan_interval_);
        changeFSMExecState(REPLAN_TRAJ, "PERIODIC");
      }
      break;
    }

    case WAIT_REPLAN:
    {
      // Keep the global target and the reference path alive while the live
      // lidar grid has no local solution.  The emergency trajectory published
      // on entry holds position; retry at a bounded rate so a moving obstacle
      // can clear without spinning the optimizer at 100 Hz.
      if (!have_target_ || !trigger_)
      {
        publishLocalWaiting(false);
        changeFSMExecState(WAIT_TARGET, "WAIT_REPLAN");
        break;
      }
      if (odom_vel_.norm() >= 0.1 || node_->now() < next_replan_time_)
        break;

      const uint64_t observation_sequence =
          planner_manager_->grid_map_->observationSequence();
      if (observation_sequence == 0 ||
          observation_sequence == last_resume_observation_sequence_)
        break;
      last_resume_observation_sequence_ = observation_sequence;

      force_replan_from_odom_ = true;
      if (planFromCurrentTraj(false))
      {
        ++resume_clear_count_;
        if (resume_clear_count_ >= resume_clear_observations_)
        {
          publishCurrentLocalTrajectory();
          replan_fail_count_ = 0;
          publishLocalWaiting(false);
          RCLCPP_INFO(node_->get_logger(),
                      "Local route stayed clear for %d fresh observations; resuming navigation",
                      resume_clear_count_);
          resetResumeConfirmation();
          changeFSMExecState(EXEC_TRAJ, "WAIT_REPLAN");
        }
        else
        {
          next_replan_time_ = node_->now() +
              rclcpp::Duration::from_seconds(replan_retry_interval_);
          RCLCPP_INFO(node_->get_logger(),
                      "Local route clear confirmation %d/%d; keeping zero velocity",
                      resume_clear_count_, resume_clear_observations_);
        }
      }
      else
      {
        resume_clear_count_ = 0;
        next_replan_time_ = node_->now() +
            rclcpp::Duration::from_seconds(replan_retry_interval_);
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 5000,
            "No executable local route; holding position and retrying every %.1fs",
            replan_retry_interval_);
      }
      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && !need_hover_stop_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        else if (enable_fail_safe_ && need_hover_stop_ && odom_vel_.norm() < 0.1)
        {
          RCLCPP_INFO(node_->get_logger(),
                      "Robot stopped; preserving the target and waiting for a local route");
          need_hover_stop_ = false;
          next_replan_time_ = node_->now() +
              rclcpp::Duration::from_seconds(replan_retry_interval_);
          resetResumeConfirmation();
          publishLocalWaiting(true);
          changeFSMExecState(WAIT_REPLAN, "EMERGENCY_EXIT");
        }
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    finishProcess();

    data_disp_.header.stamp = node_->now();
    data_disp_pub_->publish(data_disp_);
  }

  void SCANReplanFSM::finishProcess()
  {
    if (replan_fail_count_ >= max_replan_fail_count_)
    {
      RCLCPP_WARN(node_->get_logger(),
                  "Replan failed %d times; stop and keep retrying the active target",
                  replan_fail_count_);
      replan_fail_count_ = 0;
      need_hover_stop_ = true;
      flag_escape_emergency_ = true;
      changeFSMExecState(EMERGENCY_STOP, "finishProcess");
    }
  }

  bool SCANReplanFSM::planFromCurrentTraj(bool publish_trajectory)
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    rclcpp::Time time_now = node_->now();
    double t_cur = (time_now - info->start_time_).seconds();
    t_cur = std::min(std::max(t_cur, 0.0), info->duration_);

    // The upstream mode-3 recovery first tries to preserve the trajectory
    // that is already being executed, then falls back to a freshly generated
    // seed and finally a randomized seed.  Reusing an old trajectory is only
    // safe while it still has a meaningful, closely tracked tail.
    const bool forced_odom_start = force_replan_from_odom_ || go2_execution_frozen_;
    const Eigen::Vector3d previous_position =
        info->position_traj_.evaluateDeBoorT(t_cur);
    const Eigen::Vector3d previous_end =
        info->position_traj_.evaluateDeBoorT(info->duration_);
    const bool can_reuse_previous_trajectory =
        !forced_odom_start && info->start_time_.seconds() > 1e-5 &&
        info->duration_ - t_cur > 0.30 &&
        (previous_position - odom_pos_).head<2>().norm() < 0.15 &&
        (previous_end - previous_position).head<2>().norm() > 0.20;

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    start_pt_ = odom_pos_;
    if (force_replan_from_odom_ || go2_execution_frozen_)
    {
      start_vel_ = odom_vel_;
      start_acc_.setZero();
      force_replan_from_odom_ = false;
    }
    else
    {
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);
      // The controller replaces this spline at 5 Hz.  Seed the replacement
      // with the velocity it is actually commanding, otherwise every new
      // spline starts from zero and only its first 200 ms are ever executed.
      // Frozen/error replans intentionally take the odometry branch above.
      if (useFreshControllerVelocity(start_vel_))
        start_acc_.setZero();
    }

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }

    bool global_success = false;
    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH && !active_waypoints_.empty())
    {
      if (staging_reference_entry_)
      {
        const double entry_distance =
            (active_waypoints_.front() - odom_pos_).head<2>().norm();
        if (entry_distance <= reference_entry_tolerance_)
        {
          staging_reference_entry_ = false;
          current_wp_ = std::min(
              1, static_cast<int>(active_waypoints_.size()) - 1);
          RCLCPP_INFO(node_->get_logger(),
                      "Temporary reference entry reached at %.2fm; continuing global route",
                      entry_distance);
        }
        else
        {
          // Do not let projection onto the following segment skip a temporary
          // entry that the robot has not reached. The current-to-entry leg is
          // regenerated from odometry and checked only against live lidar.
          current_wp_ = 0;
        }
      }

      // Find the closest point on the remaining polyline and retain every
      // future A* corner.  Replanning directly from odom to end_pt_ used to
      // replace the collision-checked Web route with a straight line.
      if (!staging_reference_entry_ && active_waypoints_.size() > 1)
      {
        const int first_segment = std::max(0, current_wp_ - 1);
        double best_distance = std::numeric_limits<double>::infinity();
        int best_next = std::min(
            static_cast<int>(active_waypoints_.size()) - 1,
            std::max(1, current_wp_));
        for (int i = first_segment;
             i + 1 < static_cast<int>(active_waypoints_.size()); ++i)
        {
          const Eigen::Vector2d a = active_waypoints_[i].head<2>();
          const Eigen::Vector2d b = active_waypoints_[i + 1].head<2>();
          const Eigen::Vector2d ab = b - a;
          const double denom = ab.squaredNorm();
          const double ratio = denom > 1e-9 ? std::clamp(
              (odom_pos_.head<2>() - a).dot(ab) / denom, 0.0, 1.0) : 0.0;
          const double distance = (odom_pos_.head<2>() - (a + ratio * ab)).squaredNorm();
          if (distance < best_distance)
          {
            best_distance = distance;
            best_next = i + 1;
          }
        }
        current_wp_ = std::max(current_wp_, best_next);
      }

      const int begin = std::min(
          current_wp_, static_cast<int>(active_waypoints_.size()) - 1);
      std::vector<Eigen::Vector3d> remaining(
          active_waypoints_.begin() + begin, active_waypoints_.end());
      end_pt_ = active_waypoints_.back();
      global_success = planner_manager_->planGlobalTrajWaypoints(
          start_pt_, start_vel_, start_acc_, remaining,
          Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    }
    else
    {
      global_success = planner_manager_->planGlobalTraj(
          start_pt_, start_vel_, start_acc_, end_pt_,
          Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    }

    if (!global_success)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "[navi_mode=%d] Unable to refresh global trajectory from odom to current target", navi_mode_);
      return false;
    }

    if (navi_mode_ != NAVI_MODE::REFERENCE_PATH &&
        !adjustGlobalTargetIfOccupied())
      return false;

    bool success = false;
    if (can_reuse_previous_trajectory)
    {
      // Match the upstream first-stage seed: continue from the currently
      // executed B-spline.  The 15 cm tracking gate above prevents a stale
      // virtual trajectory from pulling the physical robot backwards.
      const Eigen::Vector3d odom_start_pt = start_pt_;
      start_pt_ = previous_position;
      success = callReboundReplan(false, false, publish_trajectory);
      if (!success)
        start_pt_ = odom_start_pt;
    }

    // Our second stage uses the complete Web global-reference segment rather
    // than the upstream straight polynomial.  This retains all A* corners and
    // still lets SCAN's rebound optimizer move away from them around live
    // obstacles.
    if (!success)
      success = callReboundReplan(true, false, publish_trajectory);

    // Last resort follows upstream: perturb the polynomial seed so A* can
    // escape a poor local minimum.  Full-horizon live collision validation in
    // SCANPlannerManager remains mandatory before this result is published.
    if (!success)
      success = callReboundReplan(true, true, publish_trajectory);

    if (!success)
      return false;

    return true;
  }

  void SCANReplanFSM::setStartStateFromOdomOrCurrentTraj()
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    LocalTrajData *info = &planner_manager_->local_data_;
    if (info->start_time_.seconds() < 1e-5 || info->duration_ <= 1e-5)
      return;

    const double raw_t_cur = (node_->now() - info->start_time_).seconds();
    if (raw_t_cur < -1e-3 || raw_t_cur > info->duration_ + 0.2)
      return;

    const double t_cur = std::min(std::max(raw_t_cur, 0.0), info->duration_);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);
    if (!go2_execution_frozen_ && useFreshControllerVelocity(start_vel_))
      start_acc_.setZero();

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }
  }

  void SCANReplanFSM::checkCollisionCallback()
  {
    updateLocalTrajTimeFreeze();

    if (holdIfLiveObservationStale())
      return;

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || exec_state_ == WAIT_REPLAN ||
        exec_state_ == EMERGENCY_STOP || info->start_time_.seconds() < 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = std::clamp(
        (node_->now() - info->start_time_).seconds(), 0.0, info->duration_);
    Eigen::Vector3d previous = info->position_traj_.evaluateDeBoorT(t_cur);
    double path_distance = 0.0;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t);
      path_distance += (pos - previous).head<2>().norm();
      if (path_distance > collision_replan_horizon_)
        break;
      previous = pos;
      Eigen::Vector3d pos_next = info->position_traj_.evaluateDeBoorT(std::min(t + time_step, info->duration_));
      if (map->getInflateOccupancy(pos, estimateYawFromSegment(pos, pos_next)))
      {
        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            RCLCPP_WARN(node_->get_logger(), "Obstacle discovered; emergency stop in %.3fs", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool SCANReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj,
                                        bool publish_trajectory)
  {

    getLocalTarget();

    // Seed selection mirrors upstream mode 3 while retaining our global-path
    // topology fix: no-poly means reuse the previous local B-spline; normal
    // poly uses the sampled Web route; randomized poly deliberately starts
    // without that reference so it is a genuinely different recovery attempt.
    static const std::vector<Eigen::Vector3d> empty_reference;
    const auto &seed_reference =
        (flag_use_poly_init && !flag_randomPolyTraj)
            ? local_reference_points_
            : empty_reference;

    bool plan_success =
        planner_manager_->reboundReplan(
            start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_,
            (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj,
            seed_reference);
    have_new_target_ = false;

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success && publish_trajectory)
      publishCurrentLocalTrajectory();

    return plan_success;
  }

  void SCANReplanFSM::publishCurrentLocalTrajectory()
  {
    auto info = &planner_manager_->local_data_;
    scan_planner_msgs::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
      bspline.knots.push_back(knots(i));

    bspline_pub_->publish(bspline);
    visualization_->displayOptimalTraj(info->position_traj_, 0);
    if (!periodic_replan_attempt_)
      next_periodic_replan_time_ = node_->now() +
          rclcpp::Duration::from_seconds(periodic_replan_interval_);
  }

  bool SCANReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    scan_planner_msgs::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_->publish(bspline);

    return true;
  }

  void SCANReplanFSM::getLocalTarget()
  {
    const double max_vel = planner_manager_->pp_.max_vel_;
    const double max_acc = planner_manager_->pp_.max_acc_;
    auto &global_data = planner_manager_->global_data_;
    // global_duration_ is extended whenever a local trajectory needs extra
    // execution time. Sampling GlobalTrajData::getPosition() here therefore
    // feeds the previous (possibly obstacle-detoured) local B-spline back into
    // the next replan and lets lateral error accumulate. Keep target selection
    // anchored to the original collision-checked Web reference instead.
    const double reference_duration =
        std::max(0.0, global_data.global_duration_ - global_data.time_increase_);
    auto referencePosition = [&](double reference_t) {
      return global_data.global_traj_.evaluate(
          std::clamp(reference_t, 0.0, reference_duration));
    };
    auto referenceVelocity = [&](double reference_t) {
      return global_data.global_traj_.evaluateVel(
          std::clamp(reference_t, 0.0, reference_duration));
    };

    const double t_step = std::max(
        0.01, max_vel > 1e-6
                  ? planning_horizon_ / 20.0 / max_vel
                  : 0.01);

    // Upstream mode 3 projects the current start onto the reference first.
    // This prevents a curved route from consuming the horizon according to a
    // straight-line chord and accidentally selecting a target after the next
    // wall/corner.
    double projection_t = 0.0;
    double projection_distance = std::numeric_limits<double>::infinity();
    for (double sample_t = 0.0; sample_t < reference_duration;
         sample_t += t_step)
    {
      const double distance = (referencePosition(sample_t) - start_pt_).norm();
      if (distance < projection_distance)
      {
        projection_distance = distance;
        projection_t = sample_t;
      }
    }
    const double end_distance =
        (referencePosition(reference_duration) - start_pt_).norm();
    if (end_distance < projection_distance)
    {
      projection_distance = end_distance;
      projection_t = reference_duration;
    }

    // Walk forward by accumulated route length, not Euclidean distance from
    // the robot.  Interpolate the final sample so the configured horizon is an
    // actual path-length bound instead of an overshooting approximation.
    double target_t = reference_duration;
    double accumulated_distance = 0.0;
    bool target_found = false;
    double previous_t = projection_t;
    Eigen::Vector3d previous_point = referencePosition(projection_t);
    local_target_pt_ = end_pt_;
    for (double sample_t = std::min(projection_t + t_step, reference_duration);
         sample_t <= reference_duration + 1e-6 && projection_t < reference_duration;
         sample_t += t_step)
    {
      const double clamped_t = std::min(sample_t, reference_duration);
      const Eigen::Vector3d point = referencePosition(clamped_t);
      const double segment_length = (point - previous_point).norm();
      if (accumulated_distance + segment_length >= planning_horizon_ &&
          segment_length > 1e-9)
      {
        const double ratio = std::clamp(
            (planning_horizon_ - accumulated_distance) / segment_length,
            0.0, 1.0);
        local_target_pt_ = previous_point + ratio * (point - previous_point);
        target_t = previous_t + ratio * (clamped_t - previous_t);
        target_found = true;
        break;
      }

      accumulated_distance += segment_length;
      previous_t = clamped_t;
      previous_point = point;
      if (clamped_t >= reference_duration)
        break;
    }

    if (!target_found)
    {
      local_target_pt_ = end_pt_;
      target_t = reference_duration;
    }

    auto targetOccupancy = [&](const Eigen::Vector3d &point, double point_t) {
      const double before_t = std::max(projection_t, point_t - t_step);
      const double after_t = std::min(reference_duration, point_t + t_step);
      const double yaw = estimateYawFromSegment(
          referencePosition(before_t), referencePosition(after_t));
      return planner_manager_->grid_map_->getInflateOccupancy(point, yaw);
    };

    // First construct the nominal global reference for this horizon. The
    // global map is allowed to tell us where to make progress, but live lidar
    // decides how to reach that progress point.
    std::vector<Eigen::Vector3d> global_reference_points;
    const double seed_start_t = std::clamp(
        std::min(projection_t, target_t), 0.0, reference_duration);
    const double seed_end_t = std::clamp(target_t, seed_start_t, reference_duration);
    const double seed_dt = std::max(
        0.02, planner_manager_->pp_.ctrl_pt_dist /
                  std::max(0.05, planner_manager_->pp_.max_vel_));
    for (double seed_t = seed_start_t; seed_t < seed_end_t; seed_t += seed_dt)
      global_reference_points.push_back(referencePosition(seed_t));
    global_reference_points.push_back(local_target_pt_);

    // Checking only the endpoint is insufficient: a chair may block the
    // global centreline before an otherwise free endpoint. Detect any live
    // collision along the complete nominal local reference.
    bool global_reference_blocked =
        targetOccupancy(local_target_pt_, target_t) != 0;
    const double collision_sample_step = std::max(
        0.04, planner_manager_->grid_map_->getResolution() * 0.5);
    Eigen::Vector3d collision_from = start_pt_;
    for (const auto &collision_to : global_reference_points)
    {
      if (global_reference_blocked)
        break;
      const Eigen::Vector3d segment = collision_to - collision_from;
      const double segment_length = segment.head<2>().norm();
      const int sample_count = std::max(
          1, static_cast<int>(std::ceil(segment_length / collision_sample_step)));
      const double segment_yaw = estimateYawFromSegment(collision_from, collision_to);
      for (int sample = 1; sample <= sample_count; ++sample)
      {
        const Eigen::Vector3d point =
            collision_from + segment * (static_cast<double>(sample) / sample_count);
        if (planner_manager_->grid_map_->getInflateOccupancy(point, segment_yaw) != 0)
        {
          global_reference_blocked = true;
          break;
        }
      }
      collision_from = collision_to;
    }

    bool using_live_detour = false;
    if (global_reference_blocked)
    {
      // Search target candidates in the 2-D live free space around the
      // nominal progress point. Candidate ordering keeps the current side of
      // the route when possible, which prevents a 5 Hz replan from alternating
      // left/right around the same obstacle.
      Eigen::Vector3d tangent = referenceVelocity(target_t);
      tangent.z() = 0.0;
      if (tangent.head<2>().norm() < 1e-4)
      {
        const double before_t = std::max(projection_t, target_t - t_step);
        const double after_t = std::min(reference_duration, target_t + t_step);
        tangent = referencePosition(after_t) - referencePosition(before_t);
        tangent.z() = 0.0;
      }
      if (tangent.head<2>().norm() < 1e-4)
        tangent = local_target_pt_ - start_pt_;
      tangent.z() = 0.0;
      if (tangent.head<2>().norm() < 1e-4)
        tangent = Eigen::Vector3d::UnitX();
      tangent.normalize();
      const Eigen::Vector3d normal(-tangent.y(), tangent.x(), 0.0);
      const double route_yaw = std::atan2(tangent.y(), tangent.x());
      const Eigen::Vector3d nominal_target = local_target_pt_;
      const double current_lateral = std::clamp(
          (start_pt_ - nominal_target).dot(normal), -0.85, 0.85);
      const double preferred_sign = current_lateral < -0.05 ? -1.0 : 1.0;

      std::vector<double> lateral_offsets;
      auto appendOffset = [&](double offset) {
        for (const double existing : lateral_offsets)
          if (std::abs(existing - offset) < 0.04)
            return;
        lateral_offsets.push_back(offset);
      };
      appendOffset(current_lateral);
      appendOffset(0.0);
      for (const double magnitude : {0.25, 0.45, 0.65, 0.85})
      {
        appendOffset(preferred_sign * magnitude);
        appendOffset(-preferred_sign * magnitude);
      }

      const auto search_started = std::chrono::steady_clock::now();
      std::vector<Eigen::Vector3d> live_path;
      double selected_lateral = 0.0;
      int attempted_searches = 0;
      for (const double lateral : lateral_offsets)
      {
        Eigen::Vector3d candidate = nominal_target + lateral * normal;
        candidate.z() = nominal_target.z();
        if ((candidate - start_pt_).head<2>().norm() < 0.25 ||
            planner_manager_->grid_map_->getInflateOccupancy(candidate, route_yaw) != 0)
          continue;

        ++attempted_searches;
        std::vector<Eigen::Vector3d> candidate_path;
        if (planner_manager_->searchLivePath(start_pt_, candidate, candidate_path))
        {
          live_path = std::move(candidate_path);
          selected_lateral = lateral;
          break;
        }

        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - search_started).count();
        if (attempted_searches >= 6 || elapsed > 0.45)
          break;
      }

      if (!live_path.empty())
      {
        local_target_pt_ = live_path.back();
        local_reference_points_ = std::move(live_path);
        using_live_detour = true;
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "Global local reference blocked; selected live reachable target "
            "[%.2f, %.2f], lateral %.2fm",
            local_target_pt_.x(), local_target_pt_.y(), selected_lateral);
      }
      else
      {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "Global local reference blocked and no live reachable 2-D target "
            "was found; holding for the next lidar update");
      }
    }

    global_data.last_progress_time_ = target_t;

    // Keep the target beyond a live obstacle. The rebound A* and B-spline
    // optimizer can then leave the global centreline, pass the obstacle and
    // rejoin it. Collision validity comes only from the live lidar GridMap;
    // the saved map contributes the global reference path and nothing else.
    // If no route exists, fail-safe handling stops and retries the same target
    // as fresh scans arrive.
    if ((end_pt_ - local_target_pt_).norm() <
            (max_vel * max_vel) / (2 * max_acc))
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else if (using_live_detour)
    {
      // Do not force the terminal derivative back along the blocked global
      // centreline. The next 5 Hz plan will continue from the reached live
      // path and naturally rejoin the global route after the obstacle.
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = referenceVelocity(target_t);
      if (local_target_vel_.norm() > max_vel)
        local_target_vel_ = local_target_vel_.normalized() * max_vel;
      // cout << "AA" << endl;
    }

    // Retain the geometry between the closest progress point and the local
    // horizon target.  Sampling at the control-point spacing preserves every
    // global corner without coupling local obstacle decisions to static-map
    // occupancy: SCAN still checks/optimizes this seed against the live grid.
    if (!using_live_detour)
      local_reference_points_ = std::move(global_reference_points);

    // Publish the exact pair used by this planning attempt.  A Path keeps the
    // message available to both Humble SCAN and the host Foxy Web bridge
    // without introducing another custom interface.  It is visualization
    // only and has no feedback into planning or control.
    nav_msgs::msg::Path horizon;
    horizon.header.stamp = node_->now();
    horizon.header.frame_id =
        self_inflation_frame_id_.empty() ? "odom" : self_inflation_frame_id_;
    for (const Eigen::Vector3d &point : {start_pt_, local_target_pt_})
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = horizon.header;
      pose.pose.position.x = point.x();
      pose.pose.position.y = point.y();
      pose.pose.position.z = point.z();
      pose.pose.orientation.w = 1.0;
      horizon.poses.push_back(pose);
    }
    local_horizon_pub_->publish(horizon);
  }

} // namespace scan_planner
