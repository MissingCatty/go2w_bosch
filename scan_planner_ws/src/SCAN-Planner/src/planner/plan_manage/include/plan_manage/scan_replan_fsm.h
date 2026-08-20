#ifndef _SCAN_REPLAN_FSM_H_
#define _SCAN_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <cstdint>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <scan_planner_msgs/msg/data_disp.hpp>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

using std::vector;

namespace scan_planner
{

  class SCANReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      WAIT_REPLAN,
      EMERGENCY_STOP
    };
    enum NAVI_MODE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFERENCE_PATH = 3,
    };

    /* planning utils */
    SCANPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    scan_planner_msgs::msg::DataDisp data_disp_;

    /* parameters */
    int navi_mode_; // 1 manual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    std::vector<Eigen::Vector3d> preset_waypoints_;
    int waypoint_num_;
    double planning_horizon_;
    double emergency_time_;
    double replan_retry_interval_;
    double goal_stop_speed_;
    double goal_arrival_tolerance_;
    double live_cloud_timeout_;
    int resume_clear_observations_;
    double rviz_goal_height_;
    double self_inflation_z_up_, self_inflation_z_down_;
    double self_double_cylinder_radius_, self_double_cylinder_offset_;
    double body_height_;
    std::string self_inflation_frame_id_;

    /* planning data */
    bool trigger_, have_target_, have_odom_, have_new_target_;
    bool preset_started_{false};
    bool rviz_height_ready_;
    bool go2_execution_frozen_;
    bool force_replan_from_odom_{false};
    bool enable_fail_safe_, need_hover_stop_;
    FSM_EXEC_STATE exec_state_;
    int continuously_called_times_{0};
    int replan_fail_count_{0};
    int max_replan_fail_count_{1000};
    int resume_clear_count_{0};
    uint64_t last_resume_observation_sequence_{0};
    rclcpp::Time last_freeze_update_time_;
    rclcpp::Time next_replan_time_;

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    std::vector<Eigen::Vector3d> active_waypoints_;
    // The complete portion of the collision-checked global polyline between
    // the robot and the current local target.  A local target point alone is
    // insufficient at corners because a straight chord can cut through walls.
    std::vector<Eigen::Vector3d> local_reference_points_;
    int current_wp_;

    bool flag_escape_emergency_;

    /* ROS utils */
    rclcpp::Node *node_{nullptr};
    rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr go2_execution_frozen_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr replan_request_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr navigation_cancel_sub_;
    rclcpp::Publisher<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_pub_;
    rclcpp::Publisher<scan_planner_msgs::msg::DataDisp>::SharedPtr data_disp_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr self_inflation_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr navigation_completed_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr local_waiting_pub_;

    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj,
                           bool publish_trajectory = true); // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromCurrentTraj(bool publish_trajectory = true);
    void publishCurrentLocalTrajectory();
    bool holdIfLiveObservationStale();
    void resetResumeConfirmation();
    void setStartStateFromOdomOrCurrentTraj();
    void replanRequestCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg);

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void planGlobalTrajbyGivenWps();
    bool planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints);
    bool planNextWaypoint();
    bool isWaypointSequenceMode() const;
    bool adjustGlobalTargetIfOccupied();
    void getLocalTarget();
    void finishProcess();
    void publishSelfInflationMarker();
    double getOdomYaw() const;
    double estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const;
    void updateLocalTrajTimeFreeze();
    void publishLocalWaiting(bool waiting);
    bool completeNavigationIfArrivedAndStopped();
    void completeNavigation(double actual_goal_distance);

    /* ROS functions */
    void execFSMCallback();
    void checkCollisionCallback();
    void rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);
    void waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);
    void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);
    void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
    void go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg);
    void navigationCancelCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg);

    bool checkCollision();

  public:
    SCANReplanFSM(/* args */)
    {
    }
    ~SCANReplanFSM()
    {
    }

    void init(rclcpp::Node *node);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace scan_planner

#endif
