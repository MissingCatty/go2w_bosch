// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <chrono>
#include <limits>
#include <thread>

namespace scan_planner
{
  namespace
  {
    void applyLinearZReference(std::vector<Eigen::Vector3d> &points, const double start_z, const double target_z)
    {
      if (points.empty())
        return;

      if (points.size() == 1)
      {
        points.front()(2) = start_z;
        return;
      }

      std::vector<double> accumulated_xy_length(points.size(), 0.0);
      for (size_t i = 1; i < points.size(); ++i)
      {
        accumulated_xy_length[i] = accumulated_xy_length[i - 1] +
                                   (points[i].head<2>() - points[i - 1].head<2>()).norm();
      }

      const double total_xy_length = accumulated_xy_length.back();
      for (size_t i = 0; i < points.size(); ++i)
      {
        const double ratio = total_xy_length > 1e-6
                                 ? accumulated_xy_length[i] / total_xy_length
                                 : static_cast<double>(i) / static_cast<double>(points.size() - 1);
        points[i](2) = start_z + ratio * (target_z - start_z);
      }

      points.front()(2) = start_z;
      points.back()(2) = target_z;
    }

    double estimatePlanarYaw(const Eigen::Vector3d &from, const Eigen::Vector3d &to)
    {
      const Eigen::Vector2d direction = to.head<2>() - from.head<2>();
      return direction.squaredNorm() > 1e-8 ? std::atan2(direction(1), direction(0)) : 0.0;
    }
  } // namespace

  // SECTION interfaces for setup and query

  SCANPlannerManager::SCANPlannerManager() {}

  SCANPlannerManager::~SCANPlannerManager() { std::cout << "des manager" << std::endl; }

  void SCANPlannerManager::initPlanModules(rclcpp::Node *node, PlanningVisualization::Ptr vis)
  {
    node_ = node;
    /* read algorithm parameters */
    const auto get_double = [node](const std::string &name, double default_value) {
      if (!node->has_parameter(name)) node->declare_parameter<double>(name, default_value);
      return node->get_parameter(name).as_double();
    };
    pp_.max_vel_ = get_double("manager.max_vel", -1.0);
    pp_.max_acc_ = get_double("manager.max_acc", -1.0);
    pp_.max_jerk_ = get_double("manager.max_jerk", -1.0);
    pp_.vel_tolerance_ = get_double("optimization.vel_tolerance", 1.0);
    pp_.acc_tolerance_ = get_double("optimization.acc_tolerance", 1.0);
    pp_.feasibility_tolerance_ = get_double("manager.feasibility_tolerance", 0.0);
    pp_.ctrl_pt_dist = get_double("manager.control_points_distance", -1.0);
    pp_.planning_horizon_ = get_double("manager.planning_horizon", 5.0);
    pp_.reference_corridor_tolerance_ =
        get_double("manager.reference_corridor_tolerance", 0.08);

    local_data_.traj_id_ = 0;
    grid_map_.reset(new GridMap);
    grid_map_->initMap(node_);

    bspline_optimizer_rebound_.reset(new BsplineOptimizer);
    bspline_optimizer_rebound_->setParam(node_);
    bspline_optimizer_rebound_->setEnvironment(grid_map_);
    bspline_optimizer_rebound_->a_star_.reset(new AStar);
    bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

    visualization_ = vis;
  }

  // !SECTION

  // SECTION rebond replanning

  bool SCANPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit,
                                        bool flag_randomPolyTraj,
                                        const std::vector<Eigen::Vector3d> &reference_points)
  {

    static int count = 0;
    std::cout << endl
              << "[rebo replan]: -------------------------------------" << count++ << std::endl;
    cout.precision(3);
    cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
         << endl;

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
      continuous_failures_count_++;
      return false;
    }

    auto t_start = std::chrono::steady_clock::now();
    double t_init = 0.0, t_opt = 0.0, t_refine = 0.0;

    /*** STEP 1: INIT ***/
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;
    do
    {
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      if (!reference_points.empty())
      {
        // Seed the local B-spline from the complete global reference segment,
        // not from a straight chord to its horizon endpoint.  The latter can
        // cut across a wall even when Web A* correctly routed around it.
        std::vector<Eigen::Vector3d> polyline;
        polyline.reserve(reference_points.size() + 2);
        polyline.push_back(start_pt);
        for (const auto &point : reference_points)
        {
          if ((point - polyline.back()).norm() > 1e-4)
            polyline.push_back(point);
        }
        if ((local_target_pt - polyline.back()).norm() > 1e-4)
          polyline.push_back(local_target_pt);
        else
          polyline.back() = local_target_pt;

        // LIO position corrections can look like a large lateral velocity even
        // while the locked robot is stationary.  Feeding that vector into the
        // endpoint derivative makes the B-spline initially swing away from an
        // otherwise valid global route.  Preserve only forward motion along
        // the current reference tangent.
        Eigen::Vector3d reference_tangent = Eigen::Vector3d::Zero();
        for (size_t i = 1; i < polyline.size(); ++i)
        {
          reference_tangent = polyline[i] - start_pt;
          if (reference_tangent.head<2>().norm() > 0.05)
            break;
        }
        reference_tangent(2) = 0.0;
        if (reference_tangent.norm() > 1e-6)
        {
          reference_tangent.normalize();
          const Eigen::Vector3d raw_start_vel = start_vel;
          const double forward_speed = std::clamp(
              start_vel.dot(reference_tangent), 0.0, pp_.max_vel_);
          start_vel = reference_tangent * forward_speed;
          start_acc.setZero();
          if ((raw_start_vel - start_vel).head<2>().norm() > 0.05)
            RCLCPP_WARN(node_->get_logger(),
                        "Rejected %.3fm/s lateral LIO start velocity; following global reference tangent",
                        (raw_start_vel - start_vel).head<2>().norm());
        }

        std::vector<double> lengths(polyline.size(), 0.0);
        for (size_t i = 1; i < polyline.size(); ++i)
          lengths[i] = lengths[i - 1] + (polyline[i] - polyline[i - 1]).norm();

        const double total_length = lengths.back();
        if (total_length < 0.2)
        {
          continuous_failures_count_++;
          return false;
        }
        const int segment_count = std::max(
            6, static_cast<int>(std::ceil(total_length / pp_.ctrl_pt_dist)));
        const double sample_spacing = total_length / segment_count;
        size_t segment = 0;
        point_set.reserve(segment_count + 1);
        for (int i = 0; i <= segment_count; ++i)
        {
          const double distance = total_length * i / segment_count;
          while (segment + 1 < lengths.size() && lengths[segment + 1] < distance)
            ++segment;
          if (segment + 1 >= polyline.size())
          {
            point_set.push_back(polyline.back());
            continue;
          }
          const double span = lengths[segment + 1] - lengths[segment];
          const double ratio = span > 1e-9 ? (distance - lengths[segment]) / span : 0.0;
          point_set.push_back(polyline[segment] * (1.0 - ratio) + polyline[segment + 1] * ratio);
        }
        point_set.front() = start_pt;
        point_set.back() = local_target_pt;
        ts = std::max(0.05, sample_spacing / pp_.max_vel_ * 1.2);
        start_end_derivatives.push_back(start_vel);
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(start_acc);
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());
        flag_first_call = false;
        flag_force_polynomial = false;
      }
      else if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
      {
        flag_first_call = false;
        flag_force_polynomial = false;

        PolynomialTraj gl_traj;

        double dist = (start_pt - local_target_pt).norm();
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

        if (!flag_randomPolyTraj)
        {
          gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
        else
        {
          Eigen::Vector3d horizon_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizon_dir)).normalized();
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizon_dir * 0.8 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989) +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989);
          Eigen::MatrixXd pos(3, 3);
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        double t;
        bool flag_too_far;
        ts *= 1.5; // ts will be divided by 1.5 in the next
        do
        {
          ts /= 1.5;
          point_set.clear();
          flag_too_far = false;
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);
          for (t = 0; t < time; t += ts)
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        t -= ts;
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
      }
      else // Initial path generated from previous trajectory.
      {

        double t;
        double t_cur = (node_->now() - local_data_.start_time_).seconds();

        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
          if (t > t_cur)
          {
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;

        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
        if (poly_time > ts)
        {
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              RCLCPP_ERROR(node_->get_logger(), "pseudo_arc_length is empty; aborting replan");
              continuous_failures_count_++;
              return false;
            }
          }
        }

        double sample_length = 0;
        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        size_t id = 0;
        do
        {
          cps_dist /= 1.5;
          point_set.clear();
          sample_length = 0;
          id = 0;
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
          {
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          point_set.push_back(local_target_pt);
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help

        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        if (point_set.size() > pp_.planning_horizon_ / pp_.ctrl_pt_dist * 3) // The initial path is abnormally too long!
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);

    applyLinearZReference(point_set, start_pt(2), local_target_pt(2));

    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    // Keep a sampled copy of the chosen seed.  The rebound optimizer may move
    // away from it to avoid live obstacles, but smoothness alone must not turn
    // a global-path corner back into a wall-cutting chord.
    UniformBspline reference_traj(ctrl_pts, 3, ts);
    const double reference_dt = reference_traj.getTimeSum() /
                                std::max<Eigen::Index>(1, ctrl_pts.cols() - 3);
    bspline_optimizer_rebound_->ref_pts_.clear();
    for (double reference_t = 0.0;
         reference_t < reference_traj.getTimeSum() + 1e-4;
         reference_t += reference_dt)
      bspline_optimizer_rebound_->ref_pts_.push_back(
          reference_traj.evaluateDeBoorT(
              std::min(reference_t, reference_traj.getTimeSum())));

    vector<vector<Eigen::Vector3d>> a_star_paths;
    a_star_paths = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);

    t_init = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();

    static int vis_id = 0;
    visualization_->displayInitPathList(point_set, 0.2, 0);
    visualization_->displayAStarList(a_star_paths, vis_id);

    t_start = std::chrono::steady_clock::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
    cout << "first_optimize_step_success=" << flag_step_1_success << endl;
    if (!flag_step_1_success)
    {
      // visualization_->displayOptimalList( ctrl_pts, vis_id );
      continuous_failures_count_++;
      return false;
    }
    //visualization_->displayOptimalList( ctrl_pts, vis_id );

    t_opt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
    t_start = std::chrono::steady_clock::now();

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    double ratio;
    bool flag_step_2_success = true;
    if (!pos.checkFeasibility(ratio, false))
    {
      cout << "Need to reallocate time." << endl;

      Eigen::MatrixXd optimal_control_points;
      flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
      if (flag_step_2_success)
        pos = UniformBspline(optimal_control_points, 3, ts);
    }

    if (!flag_step_2_success || !checkDynamicFeasibility(pos) ||
        !checkTrajectoryCollision(pos) ||
        !checkReferenceCorridor(pos, reference_points))
    {
      printf("\033[34mThis refined trajectory is unsafe or dynamically infeasible. Skip publishing it.\n\033[0m");
      continuous_failures_count_++;
      return false;
    }

    t_refine = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();

    // save planned results
    updateTrajInfo(pos, node_->now());

    cout << "total time:\033[42m" << (t_init + t_opt + t_refine)
         << "\033[0m,optimize:" << (t_init + t_opt) << ",refine:" << t_refine << endl;

    // success. YoY
    continuous_failures_count_ = 0;
    return true;
  }

  bool SCANPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), node_->now());

    return true;
  }

  bool SCANPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    if (waypoints.empty())
      return false;

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    for (size_t i = 0; i < points.size() - 1; i++)
    {
      total_len += (points[i + 1] - points[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // for ( int i=0; i<inter_points.size(); i++ )
    // {
    //   cout << inter_points[i].transpose() << endl;
    // }

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    // The caller supplied a collision-checked global polyline.  A minimum-snap
    // interpolation through all waypoints may overshoot far outside that
    // corridor at corners (and did so on the GO2 office map).  Preserve the
    // reference geometry exactly here; the downstream local B-spline still
    // provides the smooth, dynamically feasible trajectory used by control.
    PolynomialTraj gl_traj;
    if (pos.cols() < 2)
      return false;
    for (int i = 0; i < pos.cols() - 1; ++i)
    {
      const Eigen::Vector3d delta = pos.col(i + 1) - pos.col(i);
      const double duration = time(i);
      if (!std::isfinite(duration) || duration <= 1e-4)
        continue;
      std::vector<double> cx{delta(0) / duration, pos(0, i)};
      std::vector<double> cy{delta(1) / duration, pos(1, i)};
      std::vector<double> cz{delta(2) / duration, pos(2, i)};
      gl_traj.addSegment(cx, cy, cz, duration);
    }
    if (gl_traj.getTimes().empty())
      return false;

    auto time_now = node_->now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool SCANPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = node_->now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool SCANPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_rebound_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  void SCANPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }

  bool SCANPlannerManager::checkDynamicFeasibility(UniformBspline position_traj)
  {
    UniformBspline vel_traj = position_traj.getDerivative();
    UniformBspline acc_traj = vel_traj.getDerivative();
    const double duration = position_traj.getTimeSum();
    const double sample_dt = std::max(0.01, std::min(0.05, duration / 50.0));
    const double vel_limit = pp_.max_vel_ + pp_.vel_tolerance_;
    const double acc_limit = pp_.max_acc_ + pp_.acc_tolerance_;

    for (double t = 0.0; t < duration + 1e-6; t += sample_dt)
    {
      const double tc = std::min(t, duration);
      Eigen::Vector3d vel = vel_traj.evaluateDeBoorT(tc);
      if (vel.norm() > vel_limit)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "Dynamic feasibility failed: velocity at t=%.3f is %.3f > %.3f",
                    tc, vel.norm(), vel_limit);
        return false;
      }

      Eigen::Vector3d acc = acc_traj.evaluateDeBoorT(tc);
      if (acc.norm() > acc_limit)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "Dynamic feasibility failed: acceleration at t=%.3f is %.3f > %.3f",
                    tc, acc.norm(), acc_limit);
        return false;
      }
    }

    return true;
  }

  bool SCANPlannerManager::checkTrajectoryCollision(UniformBspline position_traj)
  {
    const double duration = position_traj.getTimeSum();
    constexpr double sample_dt = 0.02;
    for (double t = 0.0; t < duration + 1e-6; t += sample_dt)
    {
      const double tc = std::min(t, duration);
      const double tn = std::min(tc + sample_dt, duration);
      const Eigen::Vector3d position = position_traj.evaluateDeBoorT(tc);
      const Eigen::Vector3d next = position_traj.evaluateDeBoorT(tn);
      if (grid_map_->getInflateOccupancy(position, estimatePlanarYaw(position, next)) != 0)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "Final local trajectory collision check failed at t=%.3f, point=(%.2f, %.2f, %.2f)",
                    tc, position(0), position(1), position(2));
        return false;
      }
    }
    return true;
  }

  bool SCANPlannerManager::checkReferenceCorridor(
      UniformBspline position_traj,
      const std::vector<Eigen::Vector3d> &reference_points)
  {
    if (reference_points.empty())
      return true;

    std::vector<Eigen::Vector2d> polyline;
    polyline.reserve(reference_points.size() + 1);
    polyline.push_back(position_traj.evaluateDeBoorT(0.0).head<2>());
    for (const auto &point : reference_points)
    {
      const Eigen::Vector2d xy = point.head<2>();
      if ((xy - polyline.back()).norm() > 1e-4)
        polyline.push_back(xy);
    }
    if (polyline.size() < 2)
      return true;

    const auto point_segment_distance = [](
        const Eigen::Vector2d &point, const Eigen::Vector2d &from,
        const Eigen::Vector2d &to) {
      const Eigen::Vector2d segment = to - from;
      const double length_sq = segment.squaredNorm();
      const double ratio = length_sq > 1e-12
                               ? std::clamp((point - from).dot(segment) / length_sq,
                                            0.0, 1.0)
                               : 0.0;
      return (point - (from + ratio * segment)).norm();
    };

    const double duration = position_traj.getTimeSum();
    constexpr double sample_dt = 0.02;
    for (double t = 0.0; t < duration + 1e-6; t += sample_dt)
    {
      const double tc = std::min(t, duration);
      const Eigen::Vector2d position =
          position_traj.evaluateDeBoorT(tc).head<2>();
      double min_distance = std::numeric_limits<double>::infinity();
      for (size_t i = 1; i < polyline.size(); ++i)
        min_distance = std::min(
            min_distance,
            point_segment_distance(position, polyline[i - 1], polyline[i]));
      if (min_distance > pp_.reference_corridor_tolerance_)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "Final local trajectory left global corridor at t=%.3f: "
                    "deviation=%.3f > %.3f",
                    tc, min_distance, pp_.reference_corridor_tolerance_);
        return false;
      }
    }
    return true;
  }

  void SCANPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;
    // double length = bspline.getLength(0.1);
    // int seg_num = ceil(length / pp_.ctrl_pt_dist);

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

} // namespace scan_planner
