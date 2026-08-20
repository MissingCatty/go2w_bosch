#include "bspline_opt/bspline_optimizer.h"
#include "bspline_opt/gradient_descent_optimizer.h"
#include <algorithm>
#include <chrono>
#include <cmath>
// using namespace std;

namespace scan_planner
{

  void BsplineOptimizer::setParam(rclcpp::Node *node)
  {
    const auto get_double = [node](const std::string &name, double default_value) {
      if (!node->has_parameter(name)) node->declare_parameter<double>(name, default_value);
      return node->get_parameter(name).as_double();
    };
    lambda1_ = get_double("optimization.lambda_smooth", -1.0);
    lambda2_ = get_double("optimization.lambda_collision", -1.0);
    lambda3_ = get_double("optimization.lambda_feasibility", -1.0);
    lambda4_ = get_double("optimization.lambda_fitness", -1.0);
    dist0_ = get_double("optimization.dist0", -1.0);
    max_vel_ = get_double("optimization.max_vel", -1.0);
    max_acc_ = get_double("optimization.max_acc", -1.0);
    if (!node->has_parameter("optimization.order")) node->declare_parameter<int>("optimization.order", 3);
    order_ = static_cast<int>(node->get_parameter("optimization.order").as_int());
  }

  void BsplineOptimizer::setEnvironment(const GridMap::Ptr &env)
  {
    this->grid_map_ = env;
  }

  double BsplineOptimizer::estimateSegmentYaw(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const
  {
    Eigen::Vector2d diff(to(0) - from(0), to(1) - from(1));
    if (diff.squaredNorm() < 1e-8)
      return 0.0;
    return std::atan2(diff(1), diff(0));
  }

  double BsplineOptimizer::estimateControlPointYaw(const Eigen::MatrixXd &q, int id) const
  {
    if (q.cols() <= 1)
      return 0.0;

    // A clamped B-spline repeats its first/last control point.  Looking only
    // one column to either side therefore produces a zero-length segment at
    // the terminal control points and estimateSegmentYaw() falls back to 0.
    // With the double-cylinder footprint that rotates the robot body to an
    // unrelated heading for collision checks and can reject an otherwise
    // clear path.  Find the nearest geometrically distinct points instead.
    const int last = static_cast<int>(q.cols()) - 1;
    const int current = std::max(0, std::min(id, last));
    int prev_id = current - 1;
    while (prev_id >= 0 &&
           (q.col(current).head<2>() - q.col(prev_id).head<2>()).squaredNorm() < 1e-8)
      --prev_id;

    int next_id = current + 1;
    while (next_id <= last &&
           (q.col(next_id).head<2>() - q.col(current).head<2>()).squaredNorm() < 1e-8)
      ++next_id;

    if (prev_id >= 0 && next_id <= last)
      return estimateSegmentYaw(q.col(prev_id), q.col(next_id));
    if (prev_id >= 0)
      return estimateSegmentYaw(q.col(prev_id), q.col(current));
    if (next_id <= last)
      return estimateSegmentYaw(q.col(current), q.col(next_id));
    return 0.0;
  }

  void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd &points)
  {
    cps_.points = points;
  }

  void BsplineOptimizer::setBsplineInterval(const double &ts) { bspline_interval_ = ts; }

  /* This function is very similar to check_collision_and_rebound(). 
   * It was written separately, just because I did it once and it has been running stably since March 2020.
   * But I will merge then someday.*/
  std::vector<std::vector<Eigen::Vector3d>> BsplineOptimizer::initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init /*= true*/)
  {
    control_points_collision_valid_ = true;

    if (flag_first_init)
    {
      cps_.clearance = dist0_;
      cps_.resize(init_points.cols());
      cps_.points = init_points;
      astar_seed_reference_points_ = init_points;
      astar_seed_generated_ = false;
    }

    /*** Segment the initial trajectory according to obstacles ***/
    constexpr int ENOUGH_INTERVAL = 2;
    double step_size = grid_map_->getResolution() / ((init_points.col(0) - init_points.rightCols(1)).norm() / (init_points.cols() - 1)) / 2;
    int in_id = -1, out_id = -1;
    vector<std::pair<int, int>> segment_ids;
    int same_occ_state_times = ENOUGH_INTERVAL + 1;
    bool occ, last_occ = false;
    bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;
    // Check the complete local horizon.  The legacy implementation ignored
    // its final third, so an obstacle near the local target survived rebound
    // optimization and was rejected only by the final safety check.
    int i_end = (int)init_points.cols() - order_;
    for (int i = order_; i <= i_end; ++i)
    {
      for (double a = 1.0; a >= 0.0; a -= step_size)
      {
        Eigen::Vector3d sample_pt = a * init_points.col(i - 1) + (1 - a) * init_points.col(i);
        double sample_yaw = estimateSegmentYaw(init_points.col(i - 1), init_points.col(i));
        occ = grid_map_->getInflateOccupancy(sample_pt, sample_yaw);
        // cout << setprecision(5);
        // cout << (a * init_points.col(i-1) + (1-a) * init_points.col(i)).transpose() << " occ1=" << occ << endl;

        if (occ && !last_occ)
        {
          if (same_occ_state_times > ENOUGH_INTERVAL || i == order_)
          {
            in_id = i - 1;
            flag_got_start = true;
          }
          same_occ_state_times = 0;
          flag_got_end_maybe = false; // terminate in advance
        }
        else if (!occ && last_occ)
        {
          out_id = i;
          flag_got_end_maybe = true;
          same_occ_state_times = 0;
        }
        else
        {
          ++same_occ_state_times;
        }

        if (flag_got_end_maybe && (same_occ_state_times > ENOUGH_INTERVAL || (i == (int)init_points.cols() - order_)))
        {
          flag_got_end_maybe = false;
          flag_got_end = true;
        }

        last_occ = occ;

        if (flag_got_start && flag_got_end)
        {
          flag_got_start = false;
          flag_got_end = false;
          if (in_id >= 0 && out_id >= in_id && out_id < init_points.cols())
            segment_ids.push_back(std::pair<int, int>(in_id, out_id));
          else
            RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                        "Skip invalid collision segment: in_id=%d, out_id=%d", in_id, out_id);
        }
      }
    }

    /*** a star search ***/
    vector<vector<Eigen::Vector3d>> a_star_paths;
    for (size_t i = 0; i < segment_ids.size(); ++i)
    {
      //cout << "in=" << in.transpose() << " out=" << out.transpose() << endl;
      Eigen::Vector3d in(init_points.col(segment_ids[i].first)), out(init_points.col(segment_ids[i].second));
      ASTAR_RET ret = a_star_->AstarSearch(grid_map_->getResolution(), in, out);
      if (ret == ASTAR_RET::SUCCESS)
      {
        vector<Eigen::Vector3d> path = a_star_->getPath();
        if (path.size() < 2)
        {
          RCLCPP_WARN(rclcpp::get_logger("bspline_opt"), "A-star path has fewer than 2 points");
          control_points_collision_valid_ = false;
          return a_star_paths;
        }
        a_star_paths.push_back(path);
      }
      else
      {
        // A collision segment is delimited by the first occupied sample and
        // the first free sample after it.  Around a chair or a wall corner,
        // those two points can sit on opposite sides of a locally closed
        // obstacle pocket even though a perfectly valid path exists from the
        // beginning to the end of the complete local horizon.  Do not reject
        // the trajectory solely because that artificial subproblem failed:
        // retry once across the full local segment and use that path to seed
        // every movable control point.  The same live/static occupancy and
        // the final dense collision check still apply.
        const int full_in_id = std::max(0, order_ - 1);
        const int full_out_id = std::min(
            static_cast<int>(init_points.cols()) - 1,
            static_cast<int>(init_points.cols()) - order_);
        RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                    "A-star failed for collision segment %zu/%zu; "
                    "retrying complete local horizon (control points %d..%d)",
                    i + 1, segment_ids.size(), full_in_id, full_out_id);

        const ASTAR_RET full_ret = a_star_->AstarSearch(
            grid_map_->getResolution(), init_points.col(full_in_id),
            init_points.col(full_out_id));
        if (full_ret == ASTAR_RET::SUCCESS)
        {
          vector<Eigen::Vector3d> full_path = a_star_->getPath();
          if (full_path.size() >= 2)
          {
            segment_ids.clear();
            segment_ids.emplace_back(full_in_id, full_out_id);
            a_star_paths.clear();
            a_star_paths.push_back(std::move(full_path));
            RCLCPP_INFO(rclcpp::get_logger("bspline_opt"),
                        "Complete local-horizon A-star fallback succeeded");
            break;
          }
        }

        RCLCPP_ERROR(rclcpp::get_logger("bspline_opt"),
                     "Collision-segment and complete-horizon A-star both failed; "
                     "rejecting trajectory");
        control_points_collision_valid_ = false;
        return a_star_paths;
      }
    }

    /*** calculate bounds ***/
    int id_low_bound, id_up_bound;
    vector<std::pair<int, int>> bounds(segment_ids.size());
    for (size_t i = 0; i < segment_ids.size(); i++)
    {

      if (i == 0) // first segment
      {
        id_low_bound = order_;
        if (segment_ids.size() > 1)
        {
          id_up_bound = (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0f) / 2); // id_up_bound : -1.0f fix()
        }
        else
        {
          id_up_bound = init_points.cols() - order_ - 1;
        }
      }
      else if (i == segment_ids.size() - 1) // last segment, i != 0 here
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = init_points.cols() - order_ - 1;
      }
      else
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0f) / 2);  // id_up_bound : -1.0f fix()
      }

      bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound);
    }

    // cout << "+++++++++" << endl;
    // for ( int j=0; j<bounds.size(); ++j )
    // {
    //   cout << bounds[j].first << "  " << bounds[j].second << endl;
    // }

    /*** Adjust segment length ***/
    vector<std::pair<int, int>> final_segment_ids(segment_ids.size());
    constexpr double MINIMUM_PERCENT = 0.0; // Each segment is guaranteed to have sufficient points to generate sufficient thrust
    int minimum_points = round(init_points.cols() * MINIMUM_PERCENT), num_points;
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      /*** Adjust segment length ***/
      num_points = segment_ids[i].second - segment_ids[i].first + 1;
      //cout << "i = " << i << " first = " << segment_ids[i].first << " second = " << segment_ids[i].second << endl;
      if (num_points < minimum_points)
      {
        double add_points_each_side = (int)(((minimum_points - num_points) + 1.0f) / 2);

        final_segment_ids[i].first = segment_ids[i].first - add_points_each_side >= bounds[i].first ? segment_ids[i].first - add_points_each_side : bounds[i].first;

        final_segment_ids[i].second = segment_ids[i].second + add_points_each_side <= bounds[i].second ? segment_ids[i].second + add_points_each_side : bounds[i].second;
      }
      else
      {
        final_segment_ids[i].first = segment_ids[i].first;
        final_segment_ids[i].second = segment_ids[i].second;
      }

      //cout << "final:" << "i = " << i << " first = " << final_segment_ids[i].first << " second = " << final_segment_ids[i].second << endl;
    }

    /*** Assign data to each segment ***/
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      if (i >= a_star_paths.size() || a_star_paths[i].size() < 2)
      {
        RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                    "Skip invalid A-star path while assigning control point directions");
        continue;
      }

      // step 1
      for (int j = final_segment_ids[i].first; j <= final_segment_ids[i].second; ++j)
        cps_.flag_temp[j] = false;

      // step 2
      int got_intersection_id = -1;
      for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
      {
        Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
        int Astar_id = a_star_paths[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        double val = (a_star_paths[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), last_val = val;
        while (Astar_id >= 0 && Astar_id < (int)a_star_paths[i].size())
        {
          last_Astar_id = Astar_id;

          if (val >= 0)
            --Astar_id;
          else
            ++Astar_id;

          if (Astar_id < 0 || Astar_id >= (int)a_star_paths[i].size())
            break;

          val = (a_star_paths[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

          if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
          {
            const double denom = ctrl_pts_law.dot(a_star_paths[i][Astar_id] - a_star_paths[i][last_Astar_id]);
            if (std::abs(denom) < 1e-8)
              break;

            intersection_point =
                a_star_paths[i][Astar_id] +
                ((a_star_paths[i][Astar_id] - a_star_paths[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(cps_.points.col(j) - a_star_paths[i][Astar_id]) / denom) // = t
                );

            //cout << "i=" << i << " j=" << j << " Astar_id=" << Astar_id << " last_Astar_id=" << last_Astar_id << " intersection_point = " << intersection_point.transpose() << endl;

            got_intersection_id = j;
            break;
          }
        }

        if (got_intersection_id >= 0)
        {
          cps_.flag_temp[j] = true;
          double length = (intersection_point - cps_.points.col(j)).norm();
          if (length > 1e-5)
          {
            for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
            {
              Eigen::Vector3d sample_pt = (a / length) * intersection_point + (1 - a / length) * cps_.points.col(j);
              double sample_yaw = estimateControlPointYaw(cps_.points, j);
              occ = grid_map_->getInflateOccupancy(sample_pt, sample_yaw);

              if (occ || a < grid_map_->getResolution())
              {
                if (occ)
                  a += grid_map_->getResolution();
                cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
                break;
              }
            }
          }
        }
      }

      /* Corner case: the segment length is too short. Here the control points may outside the A* path, leading to opposite gradient direction. So I have to take special care of it */
      if (segment_ids[i].second - segment_ids[i].first == 1)
      {
        Eigen::Vector3d ctrl_pts_law(cps_.points.col(segment_ids[i].second) - cps_.points.col(segment_ids[i].first)), intersection_point;
        Eigen::Vector3d middle_point = (cps_.points.col(segment_ids[i].second) + cps_.points.col(segment_ids[i].first)) / 2;
        int Astar_id = a_star_paths[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        double val = (a_star_paths[i][Astar_id] - middle_point).dot(ctrl_pts_law), last_val = val;
        while (Astar_id >= 0 && Astar_id < (int)a_star_paths[i].size())
        {
          last_Astar_id = Astar_id;

          if (val >= 0)
            --Astar_id;
          else
            ++Astar_id;

          if (Astar_id < 0 || Astar_id >= (int)a_star_paths[i].size())
            break;

          val = (a_star_paths[i][Astar_id] - middle_point).dot(ctrl_pts_law);

          if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
          {
            const double denom = ctrl_pts_law.dot(a_star_paths[i][Astar_id] - a_star_paths[i][last_Astar_id]);
            if (std::abs(denom) < 1e-8)
              break;

            intersection_point =
                a_star_paths[i][Astar_id] +
                ((a_star_paths[i][Astar_id] - a_star_paths[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(middle_point - a_star_paths[i][Astar_id]) / denom) // = t
                );

            if ((intersection_point - middle_point).norm() > 0.01) // 1cm.
            {
              cps_.flag_temp[segment_ids[i].first] = true;
              cps_.base_point[segment_ids[i].first].push_back(cps_.points.col(segment_ids[i].first));
              cps_.direction[segment_ids[i].first].push_back((intersection_point - middle_point).normalized());

              got_intersection_id = segment_ids[i].first;
            }
            break;
          }
        }
      }

      //step 3
      if (got_intersection_id >= 0)
      {
        for (int j = got_intersection_id + 1; j <= final_segment_ids[i].second; ++j)
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
            cps_.direction[j].push_back(cps_.direction[j - 1].back());
          }

        for (int j = got_intersection_id - 1; j >= final_segment_ids[i].first; --j)
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
            cps_.direction[j].push_back(cps_.direction[j + 1].back());
          }
      }
      else
      {
        // Just ignore, it does not matter ^_^.
        // ROS_ERROR("Failed to generate direction! segment_id=%d", i);
      }
    }

    if (flag_got_start && !flag_got_end)
    {
      RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                  "Local target/control-point tail is occupied; reject this "
                  "seed instead of treating the unchecked tail as free");
      control_points_collision_valid_ = false;
      return {};
    }

    // The original rebound implementation only used the A-star path to build
    // obstacle gradients.  L-BFGS still started from the colliding reference
    // curve, so a perfectly valid detour could be pulled back into the
    // obstacle by the smoothness and reference costs on every retry.  Seed the
    // movable control points with the actual A-star geometry on the first
    // optimization pass.  The optimizer still smooths this seed and the
    // resulting B-spline is collision-checked again before it can be
    // published.
    if (flag_first_init)
    {
      const Eigen::MatrixXd original_points = cps_.points;

      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        if (i >= a_star_paths.size() || a_star_paths[i].size() < 2)
          continue;

        // Bounds partition the movable control-point range when several
        // obstacle segments exist.  Using that whole partition gives a long
        // enough transition into and out of the detour instead of creating a
        // sharp kink at the two colliding control points.
        const int seed_first = std::max(order_, bounds[i].first);
        const int seed_last = std::min(
            static_cast<int>(cps_.points.cols()) - order_ - 1,
            bounds[i].second);
        if (seed_last <= seed_first)
          continue;

        std::vector<Eigen::Vector3d> seed_polyline;
        seed_polyline.reserve(
            static_cast<size_t>(seed_last - seed_first + 1) +
            a_star_paths[i].size());

        const auto append_distinct = [&seed_polyline](const Eigen::Vector3d &point) {
          if (seed_polyline.empty() ||
              (seed_polyline.back() - point).squaredNorm() > 1e-10)
            seed_polyline.push_back(point);
        };

        // Preserve the original reference before and after the obstructed
        // interval, replacing only that interval with the collision-free
        // A-star path.
        for (int j = seed_first; j <= segment_ids[i].first; ++j)
          append_distinct(original_points.col(j));
        for (const auto &point : a_star_paths[i])
          append_distinct(point);
        for (int j = segment_ids[i].second; j <= seed_last; ++j)
          append_distinct(original_points.col(j));

        if (seed_polyline.size() < 2)
          continue;

        std::vector<double> arc_length(seed_polyline.size(), 0.0);
        for (size_t j = 1; j < seed_polyline.size(); ++j)
          arc_length[j] = arc_length[j - 1] +
                          (seed_polyline[j] - seed_polyline[j - 1]).norm();

        const double total_length = arc_length.back();
        if (total_length < 1e-6)
          continue;

        double max_xy_shift = 0.0;
        size_t path_index = 0;
        for (int j = seed_first; j <= seed_last; ++j)
        {
          const double ratio = static_cast<double>(j - seed_first) /
                               static_cast<double>(seed_last - seed_first);
          const double wanted_length = ratio * total_length;
          while (path_index + 1 < arc_length.size() &&
                 arc_length[path_index + 1] < wanted_length)
            ++path_index;

          Eigen::Vector3d seeded_point = seed_polyline[path_index];
          if (path_index + 1 < seed_polyline.size())
          {
            const double segment_length =
                arc_length[path_index + 1] - arc_length[path_index];
            if (segment_length > 1e-9)
            {
              const double segment_ratio =
                  (wanted_length - arc_length[path_index]) / segment_length;
              seeded_point =
                  (1.0 - segment_ratio) * seed_polyline[path_index] +
                  segment_ratio * seed_polyline[path_index + 1];
            }
          }

          // Local obstacle avoidance is planar here.  Retain the height
          // profile established by the global route; this also keeps the
          // change compatible with future sloped/stair trajectories.
          seeded_point(2) = original_points(2, j);
          max_xy_shift = std::max(
              max_xy_shift,
              (seeded_point.head<2>() - original_points.col(j).head<2>()).norm());
          cps_.points.col(j) = seeded_point;
          init_points.col(j) = seeded_point;
        }

        RCLCPP_INFO(rclcpp::get_logger("bspline_opt"),
                    "Seeded control points %d..%d from A-star detour "
                    "(path=%.2fm, max_xy_shift=%.2fm)",
                    seed_first, seed_last, total_length, max_xy_shift);
        astar_seed_generated_ = true;
      }
    }

    return a_star_paths;
  }

  int BsplineOptimizer::earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
    // cout << "k=" << k << endl;
    // cout << "opt->flag_continue_to_optimize_=" << opt->flag_continue_to_optimize_ << endl;
    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  double BsplineOptimizer::costFunctionRebound(void *func_data, const double *x, double *grad, const int n)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);

    double cost;
    opt->combineCostRebound(x, grad, cost, n);

    opt->iter_num_ += 1;
    return cost;
  }

  double BsplineOptimizer::costFunctionRefine(void *func_data, const double *x, double *grad, const int n)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);

    double cost;
    opt->combineCostRefine(x, grad, cost, n);

    opt->iter_num_ += 1;
    return cost;
  }

  void BsplineOptimizer::calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost,
                                                 Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost)
  {
    cost = 0.0;
    int end_idx = q.cols() - order_;
    double demarcation = cps_.clearance;
    double a = 3 * demarcation, b = -3 * pow(demarcation, 2), c = pow(demarcation, 3);

    force_stop_type_ = DONT_STOP;
    // One mid-optimization rebound attempt is enough.  Re-running the nested
    // A-star on every subsequent L-BFGS evaluation both wastes time and can
    // repeatedly reject the same corner, while the dense post-optimization
    // collision check remains the final authority.
    if (iter_num == 4 &&
        smoothness_cost / (cps_.size - 2 * order_) < 0.1)
    {
      check_collision_and_rebound();
    }

    /*** calculate distance cost and gradient ***/
    for (auto i = order_; i < end_idx; ++i)
    {
      for (size_t j = 0; j < cps_.direction[i].size(); ++j)
      {
        double dist = (cps_.points.col(i) - cps_.base_point[i][j]).dot(cps_.direction[i][j]);
        double dist_err = cps_.clearance - dist;
        Eigen::Vector3d dist_grad = cps_.direction[i][j];

        if (dist_err < 0)
        {
          /* do nothing */
        }
        else if (dist_err < demarcation)
        {
          cost += pow(dist_err, 3);
          gradient.col(i) += -3.0 * dist_err * dist_err * dist_grad;
        }
        else
        {
          cost += a * dist_err * dist_err + b * dist_err + c;
          gradient.col(i) += -(2.0 * a * dist_err + b) * dist_grad;
        }
      }
    }
  }

  void BsplineOptimizer::calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
  {

    cost = 0.0;

    int end_idx = q.cols() - order_;

    // def: f = |x*v|^2/a^2 + |x×v|^2/b^2
    double a2 = 25, b2 = 1;
    for (auto i = order_ - 1; i < end_idx + 1; ++i)
    {
      Eigen::Vector3d x = (q.col(i - 1) + 4 * q.col(i) + q.col(i + 1)) / 6.0 - ref_pts_[i - 1];
      Eigen::Vector3d v = (ref_pts_[i] - ref_pts_[i - 2]).normalized();

      double xdotv = x.dot(v);
      Eigen::Vector3d xcrossv = x.cross(v);

      double f = pow((xdotv), 2) / a2 + pow(xcrossv.norm(), 2) / b2;
      cost += f;

      Eigen::Matrix3d m;
      m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
      Eigen::Vector3d df_dx = 2 * xdotv / a2 * v + 2 / b2 * m * xcrossv;

      gradient.col(i - 1) += df_dx / 6;
      gradient.col(i) += 4 * df_dx / 6;
      gradient.col(i + 1) += df_dx / 6;
    }
  }

  void BsplineOptimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                            Eigen::MatrixXd &gradient, bool falg_use_jerk /* = true*/)
  {

    cost = 0.0;

    if (falg_use_jerk)
    {
      Eigen::Vector3d jerk, temp_j;

      for (int i = 0; i < q.cols() - 3; i++)
      {
        /* evaluate jerk */
        jerk = q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i);
        cost += jerk.squaredNorm();
        temp_j = 2.0 * jerk;
        /* jerk gradient */
        gradient.col(i + 0) += -temp_j;
        gradient.col(i + 1) += 3.0 * temp_j;
        gradient.col(i + 2) += -3.0 * temp_j;
        gradient.col(i + 3) += temp_j;
      }
    }
    else
    {
      Eigen::Vector3d acc, temp_acc;

      for (int i = 0; i < q.cols() - 2; i++)
      {
        /* evaluate acc */
        acc = q.col(i + 2) - 2 * q.col(i + 1) + q.col(i);
        cost += acc.squaredNorm();
        temp_acc = 2.0 * acc;
        /* acc gradient */
        gradient.col(i + 0) += temp_acc;
        gradient.col(i + 1) += -2.0 * temp_acc;
        gradient.col(i + 2) += temp_acc;
      }
    }
  }

  void BsplineOptimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                             Eigen::MatrixXd &gradient)
  {

    //#define SECOND_DERIVATIVE_CONTINOUS

#ifdef SECOND_DERIVATIVE_CONTINOUS

    cost = 0.0;
    double demarcation = 1.0; // 1m/s, 1m/s/s
    double ar = 3 * demarcation, br = -3 * pow(demarcation, 2), cr = pow(demarcation, 3);
    double al = ar, bl = -br, cl = cr;

    /* abbreviation */
    double ts, ts_inv2, ts_inv3;
    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts;
    ts_inv3 = 1 / ts / ts / ts;

    /* velocity feasibility */
    for (int i = 0; i < q.cols() - 1; i++)
    {
      Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;

      for (int j = 0; j < 3; j++)
      {
        if (vi(j) > max_vel_ + demarcation)
        {
          double diff = vi(j) - max_vel_;
          cost += (ar * diff * diff + br * diff + cr) * ts_inv3; // multiply ts_inv3 to make vel and acc has similar magnitude

          double grad = (2.0 * ar * diff + br) / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) > max_vel_)
        {
          double diff = vi(j) - max_vel_;
          cost += pow(diff, 3) * ts_inv3;
          ;

          double grad = 3 * diff * diff / ts * ts_inv3;
          ;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) < -(max_vel_ + demarcation))
        {
          double diff = vi(j) + max_vel_;
          cost += (al * diff * diff + bl * diff + cl) * ts_inv3;

          double grad = (2.0 * al * diff + bl) / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) < -max_vel_)
        {
          double diff = vi(j) + max_vel_;
          cost += -pow(diff, 3) * ts_inv3;

          double grad = -3 * diff * diff / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else
        {
          /* nothing happened */
        }
      }
    }

    /* acceleration feasibility */
    for (int i = 0; i < q.cols() - 2; i++)
    {
      Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      for (int j = 0; j < 3; j++)
      {
        if (ai(j) > max_acc_ + demarcation)
        {
          double diff = ai(j) - max_acc_;
          cost += ar * diff * diff + br * diff + cr;

          double grad = (2.0 * ar * diff + br) * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) > max_acc_)
        {
          double diff = ai(j) - max_acc_;
          cost += pow(diff, 3);

          double grad = 3 * diff * diff * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) < -(max_acc_ + demarcation))
        {
          double diff = ai(j) + max_acc_;
          cost += al * diff * diff + bl * diff + cl;

          double grad = (2.0 * al * diff + bl) * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) < -max_acc_)
        {
          double diff = ai(j) + max_acc_;
          cost += -pow(diff, 3);

          double grad = -3 * diff * diff * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else
        {
          /* nothing happened */
        }
      }
    }

#else

    cost = 0.0;
    /* abbreviation */
    double ts, /*vm2, am2, */ ts_inv2;
    // vm2 = max_vel_ * max_vel_;
    // am2 = max_acc_ * max_acc_;

    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts;

    /* velocity feasibility */
    for (int i = 0; i < q.cols() - 1; i++)
    {
      Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;

      //cout << "temp_v * vi=" ;
      for (int j = 0; j < 3; j++)
      {
        if (vi(j) > max_vel_)
        {
          // cout << "fuck VEL" << endl;
          // cout << vi(j) << endl;
          cost += pow(vi(j) - max_vel_, 2) * ts_inv2; // multiply ts_inv3 to make vel and acc has similar magnitude

          gradient(j, i + 0) += -2 * (vi(j) - max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) - max_vel_) / ts * ts_inv2;
        }
        else if (vi(j) < -max_vel_)
        {
          cost += pow(vi(j) + max_vel_, 2) * ts_inv2;

          gradient(j, i + 0) += -2 * (vi(j) + max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) + max_vel_) / ts * ts_inv2;
        }
        else
        {
          /* code */
        }
      }
    }

    /* acceleration feasibility */
    for (int i = 0; i < q.cols() - 2; i++)
    {
      Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      //cout << "temp_a * ai=" ;
      for (int j = 0; j < 3; j++)
      {
        if (ai(j) > max_acc_)
        {
          // cout << "fuck ACC" << endl;
          // cout << ai(j) << endl;
          cost += pow(ai(j) - max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) - max_acc_) * ts_inv2;
        }
        else if (ai(j) < -max_acc_)
        {
          cost += pow(ai(j) + max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) + max_acc_) * ts_inv2;
        }
        else
        {
          /* code */
        }
      }
      //cout << endl;
    }

#endif
  }

  bool BsplineOptimizer::check_collision_and_rebound(void)
  {

    int end_idx = cps_.size - order_;

    /*** Check and segment the initial trajectory according to obstacles ***/
    int in_id, out_id;
    vector<std::pair<int, int>> segment_ids;
    bool flag_new_obs_valid = false;
    int i_end = end_idx;
    for (int i = order_ - 1; i <= i_end; ++i)
    {

      bool occ = grid_map_->getInflateOccupancy(cps_.points.col(i), estimateControlPointYaw(cps_.points, i));

      /*** check if the new collision will be valid ***/
      if (occ)
      {
        for (size_t k = 0; k < cps_.direction[i].size(); ++k)
        {
          cout.precision(2);
          if ((cps_.points.col(i) - cps_.base_point[i][k]).dot(cps_.direction[i][k]) < 1 * grid_map_->getResolution()) // current point is outside all the collision_points.
          {
            occ = false; // Not really takes effect, just for better hunman understanding.
            break;
          }
        }
      }

      if (occ)
      {
        flag_new_obs_valid = true;

        int j;
        for (j = i - 1; j >= 0; --j)
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j), estimateControlPointYaw(cps_.points, j));
          if (!occ)
          {
            in_id = j;
            break;
          }
        }
        if (j < 0) // fail to get the obs free point
        {
          RCLCPP_ERROR(rclcpp::get_logger("bspline_opt"), "The robot is inside an obstacle");
          in_id = 0;
        }

        for (j = i + 1; j < cps_.size; ++j)
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j), estimateControlPointYaw(cps_.points, j));

          if (!occ)
          {
            out_id = j;
            break;
          }
        }
        if (j >= cps_.size) // fail to get the obs free point
        {
          RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                      "Nested rebound found an occupied trajectory tail; "
                      "continue the outer optimizer and let the dense final "
                      "collision check decide");
          return false;
        }

        i = j + 1;

        segment_ids.push_back(std::pair<int, int>(in_id, out_id));
      }
    }

    if (flag_new_obs_valid)
    {
      vector<vector<Eigen::Vector3d>> a_star_paths;
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        /*** a star search ***/
        Eigen::Vector3d in(cps_.points.col(segment_ids[i].first)), out(cps_.points.col(segment_ids[i].second));
        ASTAR_RET ret = a_star_->AstarSearch(grid_map_->getResolution(), in, out);
        if (ret == ASTAR_RET::SUCCESS)
        {
          vector<Eigen::Vector3d> path = a_star_->getPath();
          if (path.size() < 2)
          {
            RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                        "Nested A-star path has fewer than 2 points; keeping "
                        "the existing outer-optimizer obstacle constraints");
            return false;
          }
          a_star_paths.push_back(path);
        }
        else if (ret == ASTAR_RET::SEARCH_ERR && i + 1 < segment_ids.size())
        {
          segment_ids[i].second = segment_ids[i + 1].second;
          segment_ids.erase(segment_ids.begin() + i + 1);
          --i;
          RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                      "A-star failed on a collision segment; merge it with the next segment");
        }
        else
        {
          // initControlPoints() has already produced the primary A-star
          // detour and its rebound directions.  This second A-star is only an
          // optional update after L-BFGS has started moving the curve.  A
          // failure here must not discard the primary solution; keep
          // optimizing, then accept only if the dense full-trajectory safety
          // check passes.
          RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                      "Nested A-star could not update a collision segment; "
                      "continuing with the primary detour constraints");
          return false;
        }
      }

      /*** Assign parameters to each segment ***/
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        if (i >= a_star_paths.size() || a_star_paths[i].size() < 2)
        {
          RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                      "Skip invalid A-star path while assigning rebound directions");
          continue;
        }

        // step 1
        for (int j = segment_ids[i].first; j <= segment_ids[i].second; ++j)
          cps_.flag_temp[j] = false;

        // step 2
        int got_intersection_id = -1;
        for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
        {
          Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
          int Astar_id = a_star_paths[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
          double val = (a_star_paths[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), last_val = val;
          while (Astar_id >= 0 && Astar_id < (int)a_star_paths[i].size())
          {
            last_Astar_id = Astar_id;

            if (val >= 0)
              --Astar_id;
            else
              ++Astar_id;

            if (Astar_id < 0 || Astar_id >= (int)a_star_paths[i].size())
              break;

            val = (a_star_paths[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

            // cout << val << endl;

            if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
            {
              const double denom = ctrl_pts_law.dot(a_star_paths[i][Astar_id] - a_star_paths[i][last_Astar_id]);
              if (std::abs(denom) < 1e-8)
                break;

              intersection_point =
                  a_star_paths[i][Astar_id] +
                  ((a_star_paths[i][Astar_id] - a_star_paths[i][last_Astar_id]) *
                   (ctrl_pts_law.dot(cps_.points.col(j) - a_star_paths[i][Astar_id]) / denom) // = t
                  );

              got_intersection_id = j;
              break;
            }
          }

          if (got_intersection_id >= 0)
          {
            cps_.flag_temp[j] = true;
            double length = (intersection_point - cps_.points.col(j)).norm();
            if (length > 1e-5)
            {
              for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
              {
                Eigen::Vector3d sample_pt = (a / length) * intersection_point + (1 - a / length) * cps_.points.col(j);
                double sample_yaw = estimateControlPointYaw(cps_.points, j);
                bool occ = grid_map_->getInflateOccupancy(sample_pt, sample_yaw);

                if (occ || a < grid_map_->getResolution())
                {
                  if (occ)
                    a += grid_map_->getResolution();
                  cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                  cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
                  break;
                }
              }
            }
            else
            {
              got_intersection_id = -1;
            }
          }
        }

        //step 3
        if (got_intersection_id >= 0)
        {
          for (int j = got_intersection_id + 1; j <= segment_ids[i].second; ++j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
              cps_.direction[j].push_back(cps_.direction[j - 1].back());
            }

          for (int j = got_intersection_id - 1; j >= segment_ids[i].first; --j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
              cps_.direction[j].push_back(cps_.direction[j + 1].back());
            }
        }
        else
          RCLCPP_WARN(rclcpp::get_logger("bspline_opt"), "Failed to generate rebound direction");
      }

      force_stop_type_ = STOP_FOR_REBOUND;
      return true;
    }

    return false;
  }

  bool BsplineOptimizer::BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts)
  {
    if (!control_points_collision_valid_)
    {
      RCLCPP_ERROR(rclcpp::get_logger("bspline_opt"),
                   "Initial collision recovery failed; reject trajectory");
      return false;
    }
    setBsplineInterval(ts);

    bool flag_success = rebound_optimize();

    optimal_points = cps_.points;

    return flag_success;
  }

  bool BsplineOptimizer::BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points)
  {

    setControlPoints(init_points);
    setBsplineInterval(ts);

    bool flag_success = refine_optimize();

    optimal_points = cps_.points;

    return flag_success;
  }

  bool BsplineOptimizer::rebound_optimize()
  {
    iter_num_ = 0;
    int start_id = order_;
    int end_id = this->cps_.size - order_;
    variable_num_ = 3 * (end_id - start_id);
    if (start_id < 0 || end_id <= start_id ||
        end_id > this->cps_.points.cols() || variable_num_ <= 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("bspline_opt"),
                   "Invalid rebound optimization range: start=%d end=%d cols=%ld variables=%d",
                   start_id, end_id,
                   static_cast<long>(this->cps_.points.cols()), variable_num_);
      return false;
    }
    double final_cost;

    auto t0 = std::chrono::steady_clock::now();
    auto t1 = t0;
    auto t2 = t0;
    int restart_nums = 0, rebound_times = 0;
    ;
    bool flag_force_return, flag_occ, success;
    new_lambda2_ = lambda2_;
    // Large live obstacles can need more than the upstream three collision
    // weight levels, especially when the saved reference passes through the
    // middle of the detour.  Each candidate still has to pass the dense full
    // trajectory collision check below.
    constexpr int MAX_RESART_NUMS_SET = 5;

    // initControlPoints() may already have replaced the colliding reference
    // with a complete A-star detour.  Keep that verified geometry as a
    // fallback: the legacy nested-rebound pass can occasionally fail while
    // trying to improve an already safe seed, and must not throw the safe
    // solution away with it.
    Eigen::MatrixXd safe_seed_points = cps_.points;
    const auto trajectory_is_collision_free = [this](const Eigen::MatrixXd &points) {
      UniformBspline trajectory(points, 3, bspline_interval_);
      const double duration = trajectory.getTimeSum();
      if (!std::isfinite(duration) || duration <= 0.0)
        return false;

      constexpr double sample_dt = 0.02;
      for (double t = 0.0; t < duration + 1e-6; t += sample_dt)
      {
        const double current_t = std::min(t, duration);
        const double next_t = std::min(current_t + sample_dt, duration);
        const Eigen::Vector3d position = trajectory.evaluateDeBoorT(current_t);
        const Eigen::Vector3d next = trajectory.evaluateDeBoorT(next_t);
        if (grid_map_->getInflateOccupancy(
                position, estimateSegmentYaw(position, next)) != 0)
          return false;
      }
      return true;
    };
    bool safe_seed_available =
        trajectory_is_collision_free(safe_seed_points);

    // The sampled A-star polygon is collision-free, but a cubic uniform
    // B-spline only approximates that polygon and rounds its corners.  If the
    // rounded curve cuts the obstacle, progressively expand the detour away
    // from the original (colliding) reference.  This is not an unchecked
    // geometric shortcut: every candidate is sampled over its complete time
    // span against the same oriented robot footprint, and only a completely
    // free curve may become the optimizer seed/fallback.
    if (!safe_seed_available && astar_seed_generated_ &&
        astar_seed_reference_points_.rows() == cps_.points.rows() &&
        astar_seed_reference_points_.cols() == cps_.points.cols())
    {
      const Eigen::MatrixXd direct_astar_seed = cps_.points;
      constexpr double detour_scales[] = {
          1.15, 1.30, 1.50, 1.75, 2.00, 2.25, 2.50};

      for (const double scale : detour_scales)
      {
        Eigen::MatrixXd candidate = direct_astar_seed;
        for (int col = start_id; col < end_id; ++col)
        {
          candidate.col(col).head<2>() =
              astar_seed_reference_points_.col(col).head<2>() +
              scale * (direct_astar_seed.col(col).head<2>() -
                       astar_seed_reference_points_.col(col).head<2>());
          // Obstacle avoidance is planar.  Never alter the global route's
          // height profile; this remains valid for future ramps/stairs.
          candidate(2, col) = direct_astar_seed(2, col);
        }

        if (!trajectory_is_collision_free(candidate))
          continue;

        safe_seed_points = candidate;
        cps_.points = candidate;
        safe_seed_available = true;
        RCLCPP_INFO(rclcpp::get_logger("bspline_opt"),
                    "Expanded A-star detour by %.2fx; the interpolated "
                    "B-spline now passes the dense collision check",
                    scale);
        break;
      }
    }

    if (safe_seed_available)
      RCLCPP_INFO(rclcpp::get_logger("bspline_opt"),
                  "A-star-seeded B-spline passed dense collision check; "
                  "keeping it as optimizer fallback");
    else
      RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                  "A-star control-point seed still cuts an occupied cell "
                  "after B-spline interpolation; fallback disabled");

    do
    {
      /* ---------- prepare ---------- */
      min_cost_ = std::numeric_limits<double>::max();
      iter_num_ = 0;
      flag_force_return = false;
      flag_occ = false;
      success = false;

      // A variable-length stack array is a GCC extension, not ISO C++, and
      // was the only optimizer storage whose behaviour changed under the
      // -O3 build that produced the observed SIGSEGV. Keep it owned by a
      // standard container and validate the range above before copying.
      std::vector<double> q(static_cast<size_t>(variable_num_));
      memcpy(q.data(), cps_.points.data() + 3 * start_id,
             static_cast<size_t>(variable_num_) * sizeof(q[0]));

      lbfgs::lbfgs_parameter_t lbfgs_params;
      lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
      lbfgs_params.mem_size = 16;
      lbfgs_params.max_iterations = 200;
      lbfgs_params.g_epsilon = 0.01;

      /* ---------- optimize ---------- */
      t1 = std::chrono::steady_clock::now();
      int result = lbfgs::lbfgs_optimize(variable_num_, q.data(), &final_cost, BsplineOptimizer::costFunctionRebound, NULL, BsplineOptimizer::earlyExit, this, &lbfgs_params);
      t2 = std::chrono::steady_clock::now();
      double time_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
      double total_time_ms = std::chrono::duration<double, std::milli>(t2 - t0).count();

      /* ---------- success temporary, check collision again ---------- */
      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP)
      {
        //ROS_WARN("Solver error in planning!, return = %s", lbfgs::lbfgs_strerror(result));
        flag_force_return = false;

        UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
        double tm, tmp;
        traj.getTimeSpan(tm, tmp);
        // Match the final manager's dense safety check.  Chord-length based
        // sampling became too sparse after an A-star detour lengthened the
        // curve and could jump over a single occupied 5 cm voxel.
        constexpr double t_step = 0.02;
        for (double t = tm; t < tmp + 1e-6; t += t_step)
        {
          Eigen::Vector3d pos = traj.evaluateDeBoorT(t);
          Eigen::Vector3d pos_next = traj.evaluateDeBoorT(std::min(t + t_step, tmp));
          flag_occ = grid_map_->getInflateOccupancy(pos, estimateSegmentYaw(pos, pos_next));
          if (flag_occ)
          {
            //cout << "hit_obs, t=" << t << " P=" << traj.evaluateDeBoorT(t).transpose() << endl;

            if (t <= bspline_interval_) // First 3 control points in obstacles!
            {
              cout << cps_.points.col(1).transpose() << "\n"
                   << cps_.points.col(2).transpose() << "\n"
                   << cps_.points.col(3).transpose() << "\n"
                   << cps_.points.col(4).transpose() << endl;
              RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                          "First three control points are in obstacles; t=%f", t);
              return false;
            }

            break;
          }
        }

        if (!flag_occ)
        {
          printf("\033[32miter(+1)=%d,time(ms)=%5.3f,total_t(ms)=%5.3f,cost=%5.3f\n\033[0m", iter_num_, time_ms, total_time_ms, final_cost);
          success = true;
        }
        else // restart
        {
          restart_nums++;
          initControlPoints(cps_.points, false);
          if (!control_points_collision_valid_)
          {
            // The primary initControlPoints() call already found one or more
            // complete A-star detours and populated usable obstacle
            // directions.  A later refresh can fail because the partially
            // optimized curve makes a poor temporary collision segment.  Do
            // not discard the primary constraints: retain them, increase the
            // collision weight and continue.  Publication is still blocked
            // unless the dense check becomes completely free.
            control_points_collision_valid_ = true;
            RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                        "Collision-constraint refresh failed; retaining the "
                        "primary A-star constraints for the next optimizer "
                        "weight level");
          }
          new_lambda2_ *= 2;

          printf("\033[32miter(+1)=%d,time(ms)=%5.3f,keep optimizing\n\033[0m", iter_num_, time_ms);
        }
      }
      else if (result == lbfgs::LBFGSERR_CANCELED)
      {
        flag_force_return = true;
        rebound_times++;
        cout << "iter=" << iter_num_ << ",time(ms)=" << time_ms << ",rebound." << endl;
      }
      else
      {
        RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                    "Solver error. Return=%d, %s; skip this plan", result, lbfgs::lbfgs_strerror(result));
        // while (ros::ok());
      }

    } while ((flag_occ && restart_nums < MAX_RESART_NUMS_SET) ||
             (flag_force_return && force_stop_type_ == STOP_FOR_REBOUND && rebound_times <= 20));

    if (!success && safe_seed_available)
    {
      cps_.points = safe_seed_points;
      RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                  "Nested rebound optimizer failed; using the previously "
                  "collision-checked A-star seed");
      return true;
    }

    return success;
  }

  bool BsplineOptimizer::refine_optimize()
  {
    iter_num_ = 0;
    int start_id = order_;
    int end_id = this->cps_.points.cols() - order_;
    variable_num_ = 3 * (end_id - start_id);
    if (start_id < 0 || end_id <= start_id ||
        end_id > this->cps_.points.cols() || variable_num_ <= 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("bspline_opt"),
                   "Invalid refinement range: start=%d end=%d cols=%ld variables=%d",
                   start_id, end_id,
                   static_cast<long>(this->cps_.points.cols()), variable_num_);
      return false;
    }

    std::vector<double> q(static_cast<size_t>(variable_num_));
    double final_cost;

    memcpy(q.data(), cps_.points.data() + 3 * start_id,
           static_cast<size_t>(variable_num_) * sizeof(q[0]));

    double origin_lambda4 = lambda4_;
    bool flag_safe = true;
    int iter_count = 0;
    do
    {
      lbfgs::lbfgs_parameter_t lbfgs_params;
      lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
      lbfgs_params.mem_size = 16;
      lbfgs_params.max_iterations = 200;
      lbfgs_params.g_epsilon = 0.001;

      int result = lbfgs::lbfgs_optimize(variable_num_, q.data(), &final_cost, BsplineOptimizer::costFunctionRefine, NULL, NULL, this, &lbfgs_params);
      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP)
      {
        //pass
      }
      else
      {
        RCLCPP_ERROR(rclcpp::get_logger("bspline_opt"),
                     "Solver error while refining: return=%d, %s", result, lbfgs::lbfgs_strerror(result));
      }

      UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
      double tm, tmp;
      traj.getTimeSpan(tm, tmp);
      constexpr double t_step = 0.02;
      for (double t = tm; t < tmp + 1e-6; t += t_step)
      {
        Eigen::Vector3d pos = traj.evaluateDeBoorT(t);
        Eigen::Vector3d pos_next = traj.evaluateDeBoorT(std::min(t + t_step, tmp));
        if (grid_map_->getInflateOccupancy(pos, estimateSegmentYaw(pos, pos_next)))
        {
          // cout << "Refined traj hit_obs, t=" << t << " P=" << traj.evaluateDeBoorT(t).transpose() << endl;

          Eigen::MatrixXd ref_pts(ref_pts_.size(), 3);
          for (size_t i = 0; i < ref_pts_.size(); i++)
          {
            ref_pts.row(i) = ref_pts_[i].transpose();
          }

          flag_safe = false;
          break;
        }
      }

      if (!flag_safe)
        lambda4_ *= 2;

      iter_count++;
    } while (!flag_safe && iter_count <= 0);

    lambda4_ = origin_lambda4;

    //cout << "iter_num_=" << iter_num_ << endl;

    return flag_safe;
  }

  void BsplineOptimizer::combineCostRebound(const double *x, double *grad, double &f_combine, const int n)
  {

    memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

    /* ---------- evaluate cost and gradient ---------- */
    double f_smoothness, f_distance, f_feasibility, f_fitness = 0.0;

    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_distance = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_fitness = Eigen::MatrixXd::Zero(3, cps_.size);

    calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
    calcDistanceCostRebound(cps_.points, f_distance, g_distance, iter_num_, f_smoothness);
    calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);
    // calcFitnessCost() evaluates i=[order-1, cols-order] and its largest
    // reference index is therefore cols-order.  For the cubic spline used by
    // SCAN that requires exactly cols-2 samples.  Requiring cols-1 here kept
    // the reference term permanently disabled: the locally optimized curve
    // then smoothed an A* corner into the straight chord through a wall.
    const size_t required_reference_points = static_cast<size_t>(
        std::max(0, cps_.size - order_ + 1));
    if (ref_pts_.size() >= required_reference_points)
      calcFitnessCost(cps_.points, f_fitness, g_fitness);

    f_combine = lambda1_ * f_smoothness + new_lambda2_ * f_distance +
                lambda3_ * f_feasibility + lambda4_ * f_fitness;
    //printf("origin %f %f %f %f\n", f_smoothness, f_distance, f_feasibility, f_combine);

    Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + new_lambda2_ * g_distance +
                              lambda3_ * g_feasibility + lambda4_ * g_fitness;
    grad_3D.row(2).setZero();
    memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
  }

  void BsplineOptimizer::combineCostRefine(const double *x, double *grad, double &f_combine, const int n)
  {

    memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

    /* ---------- evaluate cost and gradient ---------- */
    double f_smoothness, f_fitness, f_feasibility;

    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_fitness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.points.cols());

    //time_satrt = ros::Time::now();

    calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
    calcFitnessCost(cps_.points, f_fitness, g_fitness);
    calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);

    /* ---------- convert to solver format...---------- */
    f_combine = lambda1_ * f_smoothness + lambda4_ * f_fitness + lambda3_ * f_feasibility;
    // printf("origin %f %f %f %f\n", f_smoothness, f_fitness, f_feasibility, f_combine);

    Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + lambda4_ * g_fitness +
                              lambda3_ * g_feasibility;
    grad_3D.row(2).setZero();
    memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
  }

} // namespace scan_planner
