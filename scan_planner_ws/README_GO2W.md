# SCAN-Planner × GO2-W

This workspace contains the ROS 2 community port of SCAN-Planner plus a
GO2-W/LIO-SAM adapter.  The controller always publishes the isolated topic
`/scan_planner/cmd_vel_test`.  A separate host-side safety gate may forward it
to the Unitree Sport API only after explicit arming in the Web UI.

## Data path

```text
/lio_sam/mapping/odometry (lidar pose)
  -> lio_pose_adapter
     -> /scan_planner/sensor_pose
     -> /scan_planner/body_pose (base_link pose)

/lio_sam/deskew/cloud_deskewed + poses
  -> SCAN local 3D occupancy grid

saved map_*.npz
  -> prepare_navigation_map.py (level / evidence filter / height slice / inflate)
     -> Web A* global reference path

/lio_sam/deskew/cloud_deskewed + poses
  -> SCAN live 3D occupancy + immutable inflated 2D wall boundary

SCAN local 3D occupancy grid
  -> collision-aware B-spline
  -> closed-loop controller + tracking-progress watchdog
  -> /scan_planner/cmd_vel_test (isolated)
  -> go2_chassis_safety_gate (locked by default)
  -> /api/sport/request (only while explicitly armed and healthy)

Web “取消导航” -> /scan_planner/cancel
  -> FSM waits for a new target + controller publishes zero velocity
  -> physical chassis gate locks and sends a stop burst
```

The adapter applies the calibrated `base_link -> rslidar` transform
`(0.1701, 0, 0.0908, yaw=+90deg)`.  `world_z_offset=0.53m` aligns the LIO
startup origin with the floor measured in the saved map; it is configurable in
`src/go2_scan_planner_bridge/config/go2w.yaml`.

The saved-map `map -> odom` transform is loaded by `go2_static_navigation_map`
after the Web automatic registration validates it for the current boot and map.
SCAN does not inject the saved 3D obstacle cloud into its clearable local
occupancy memory.  The current lidar owns temporary-obstacle avoidance, while
the separately published footprint-inflated 2D grid remains an immutable hard
boundary for local A* and final trajectory checks.  This lets a local path move
up to `0.75m` away from the global centreline to pass a chair without reopening
the earlier wall-crossing failure mode.

The controller does not advance its reference clock blindly.  A large heading
error first enters turn-only mode with hysteresis, while moderate error is
corrected during translation.  If the desired point gets more than `0.25m`
ahead, trajectory time pauses until the robot catches up.  A lag lasting over
`2.5s` asks the FSM to replan from current odometry; reaching the nominal
trajectory duration away from the real target also replans instead of falsely
reporting completion.

## Commands

```bash
cd ~/go2_slam_ws && ./start.sh
~/go2_slam_ws/tools/prepare_navigation_map.py \
  ~/go2_slam_ws/maps/map_20260811_155640_273.npz \
  --clear-start-x -0.05 --clear-start-y 0.21 --clear-start-radius 0.80
cd ~/scan_planner_ws && ./start_go2w_dry.sh

# Plan one metre in the robot's current forward direction. This does not move it.
./send_forward_test_goal.sh 1.0

./stop_go2w.sh
```

Build from source with `./build_go2w.sh` or start with
`./start_go2w_dry.sh --build`.

Do not connect `/scan_planner/cmd_vel_test` to another `/cmd_vel` or Sport API
bridge.  The host safety gate is the only intended actuator path and includes
explicit arming, Web heartbeat, command/odometry/LowState timeouts, tilt checks,
velocity/acceleration limits and stop-on-cancel.  The SCAN launch itself remains
dry-run and rejects `dry_run:=false`.
