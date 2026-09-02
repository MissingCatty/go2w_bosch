#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace
{
using Point = std::array<float, 3>;
using Cloud = sensor_msgs::msg::PointCloud2;
using SteadyClock = std::chrono::steady_clock;

struct NearSample
{
  std::vector<Point> points;
  std::string frame;
  std::int64_t stamp_ns{0};
  SteadyClock::time_point received{};
  std::size_t candidates{0};
  std::size_t self_filtered{0};
  std::size_t height_filtered{0};
};

std::int64_t stampNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

bool finitePoint(float x, float y, float z)
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

std::uint64_t voxelKey(float x, float y, float z, double resolution)
{
  // Near-field coordinates are bounded to about +/-1.1 m. Sixteen bits per
  // signed voxel coordinate leave ample margin while producing a fast scalar
  // key for first-point-per-voxel filtering.
  const auto encode = [resolution](float value) {
    const auto cell = static_cast<std::int32_t>(std::floor(value / resolution));
    return static_cast<std::uint64_t>(
      static_cast<std::uint16_t>(cell + 32768));
  };
  return encode(x) | (encode(y) << 16U) | (encode(z) << 32U);
}
}  // namespace

class NearFieldCloudFuser : public rclcpp::Node
{
public:
  NearFieldCloudFuser()
  : Node("go2_near_field_cloud_fuser")
  {
    far_topic_ = declare_parameter<std::string>(
      "far_topic", "/lio_sam/deskew/cloud_deskewed");
    raw_topic_ = declare_parameter<std::string>(
      "raw_topic", "/unitree/slam_lidar/points");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/scan_planner/local_cloud");
    near_min_ = declare_parameter<double>("near_min_range", 0.25);
    near_max_ = declare_parameter<double>("near_max_range", 1.05);
    near_voxel_ = declare_parameter<double>("near_voxel_size", 0.04);
    max_raw_age_ = declare_parameter<double>("max_raw_age", 0.25);
    max_raw_stamp_delta_ = declare_parameter<double>(
      "max_raw_stamp_delta", 0.03);

    base_to_lidar_x_ = declare_parameter<double>("base_to_lidar_x", 0.1701);
    base_to_lidar_y_ = declare_parameter<double>("base_to_lidar_y", 0.0);
    base_to_lidar_z_ = declare_parameter<double>("base_to_lidar_z", 0.0908);
    const double yaw = declare_parameter<double>(
      "base_to_lidar_yaw", 1.5707963267948966);
    mount_cos_ = std::cos(yaw);
    mount_sin_ = std::sin(yaw);

    body_radius_ = declare_parameter<double>("self_filter_radius", 0.24);
    body_offset_ = declare_parameter<double>("self_filter_offset", 0.12);
    self_z_min_ = declare_parameter<double>("self_filter_z_min", -0.55);
    self_z_max_ = declare_parameter<double>("self_filter_z_max", 0.30);
    obstacle_z_min_ = declare_parameter<double>("obstacle_body_z_min", -0.39);
    obstacle_z_max_ = declare_parameter<double>("obstacle_body_z_max", 0.55);

    if (!(near_min_ >= 0.0 && near_min_ < near_max_) || near_voxel_ <= 0.0) {
      throw std::invalid_argument("invalid near-field range or voxel size");
    }
    if (body_radius_ <= 0.0 || body_offset_ < 0.0) {
      throw std::invalid_argument("invalid self-filter footprint");
    }

    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    publisher_ = create_publisher<Cloud>(output_topic_, qos);
    raw_subscription_ = create_subscription<Cloud>(
      raw_topic_, qos,
      std::bind(&NearFieldCloudFuser::onRaw, this, std::placeholders::_1));
    far_subscription_ = create_subscription<Cloud>(
      far_topic_, qos,
      std::bind(&NearFieldCloudFuser::onFar, this, std::placeholders::_1));
    last_log_ = SteadyClock::now() - std::chrono::seconds(5);

    RCLCPP_INFO(
      get_logger(),
      "Near-field fusion ready: %.2f..%.2f m raw XT16 + LIO deskew -> %s",
      near_min_, near_max_, output_topic_.c_str());
  }

private:
  void onRaw(const Cloud::ConstSharedPtr message)
  {
    std::vector<Point> filtered;
    std::unordered_set<std::uint64_t> occupied_voxels;
    filtered.reserve(1024);
    occupied_voxels.reserve(2048);
    std::size_t candidates = 0;
    std::size_t self_filtered = 0;
    std::size_t height_filtered = 0;
    const double near_min_squared = near_min_ * near_min_;
    const double near_max_squared = near_max_ * near_max_;
    const double body_radius_squared = body_radius_ * body_radius_;

    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*message, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!finitePoint(*x, *y, *z)) {
          continue;
        }
        const double range_squared =
          static_cast<double>(*x) * *x + static_cast<double>(*y) * *y +
          static_cast<double>(*z) * *z;
        if (range_squared < near_min_squared || range_squared > near_max_squared) {
          continue;
        }
        ++candidates;

        // Lidar frame -> body frame, used only for filtering. Retained points
        // stay in the lidar frame consumed by SCAN's sensor_pose transform.
        const double body_x = mount_cos_ * *x - mount_sin_ * *y + base_to_lidar_x_;
        const double body_y = mount_sin_ * *x + mount_cos_ * *y + base_to_lidar_y_;
        const double body_z = *z + base_to_lidar_z_;
        const double front_distance_squared =
          (body_x - body_offset_) * (body_x - body_offset_) + body_y * body_y;
        const double rear_distance_squared =
          (body_x + body_offset_) * (body_x + body_offset_) + body_y * body_y;
        const bool inside_body =
          std::min(front_distance_squared, rear_distance_squared) <= body_radius_squared &&
          body_z >= self_z_min_ && body_z <= self_z_max_;
        if (inside_body) {
          ++self_filtered;
          continue;
        }
        if (body_z < obstacle_z_min_ || body_z > obstacle_z_max_) {
          ++height_filtered;
          continue;
        }

        const auto key = voxelKey(*x, *y, *z, near_voxel_);
        if (occupied_voxels.insert(key).second) {
          filtered.push_back(Point{*x, *y, *z});
        }
      }
    } catch (const std::runtime_error & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Raw cloud has no usable xyz fields: %s", error.what());
      filtered.clear();
    }

    NearSample sample;
    sample.points = std::move(filtered);
    sample.frame = message->header.frame_id;
    sample.stamp_ns = stampNanoseconds(message->header.stamp);
    sample.received = SteadyClock::now();
    sample.candidates = candidates;
    sample.self_filtered = self_filtered;
    sample.height_filtered = height_filtered;
    near_samples_.push_back(std::move(sample));
    // The matching deskewed cloud currently arrives about 0.2 s after the
    // raw scan. Keep enough timestamped scans to select the same acquisition
    // instead of accidentally fusing the next scan just because it arrived
    // most recently.
    while (near_samples_.size() > 16U) {
      near_samples_.pop_front();
    }
  }

  void onFar(const Cloud::ConstSharedPtr message)
  {
    const auto now = SteadyClock::now();
    const auto far_stamp_ns = stampNanoseconds(message->header.stamp);
    const NearSample * matching_near = nullptr;
    double matching_stamp_delta = std::numeric_limits<double>::infinity();
    for (const auto & sample : near_samples_) {
      if (sample.frame != message->header.frame_id) {
        continue;
      }
      const double delta = std::abs(
        static_cast<double>(far_stamp_ns - sample.stamp_ns) * 1e-9);
      if (delta < matching_stamp_delta) {
        matching_stamp_delta = delta;
        matching_near = &sample;
      }
    }
    const double matching_receive_age = matching_near ?
      std::chrono::duration<double>(now - matching_near->received).count() :
      std::numeric_limits<double>::infinity();
    const bool near_fresh = matching_near &&
      matching_receive_age <= max_raw_age_ &&
      matching_stamp_delta <= max_raw_stamp_delta_;
    const std::size_t near_count = near_fresh ? matching_near->points.size() : 0U;
    std::vector<Point> points;
    points.reserve(static_cast<std::size_t>(message->width) * message->height + near_count);
    std::size_t far_input_count = 0;
    std::size_t far_ground_filtered = 0;

    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*message, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!finitePoint(*x, *y, *z)) {
          continue;
        }
        ++far_input_count;
        // Both inputs use the same lidar frame. Apply the lower body-height
        // cutoff to the deskewed cloud too, otherwise the 1 m handover leaves
        // concentric ground rings in the local obstacle layer.
        const double body_z = *z + base_to_lidar_z_;
        if (body_z < obstacle_z_min_) {
          ++far_ground_filtered;
          continue;
        }
        points.push_back(Point{*x, *y, *z});
      }
    } catch (const std::runtime_error & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Deskewed cloud has no usable xyz fields: %s", error.what());
      return;
    }
    const std::size_t far_count = points.size();
    if (near_fresh) {
      points.insert(
        points.end(), matching_near->points.begin(), matching_near->points.end());
    }

    Cloud output;
    output.header = message->header;
    output.height = 1;
    output.is_bigendian = false;
    output.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> output_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> output_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> output_z(output, "z");
    for (const auto & point : points) {
      *output_x = point[0];
      *output_y = point[1];
      *output_z = point[2];
      ++output_x;
      ++output_y;
      ++output_z;
    }
    publisher_->publish(output);

    if (std::chrono::duration<double>(now - last_log_).count() >= 5.0) {
      last_log_ = now;
      RCLCPP_INFO(
        get_logger(),
        "local_cloud: far=%zu/%zu ground=%zu near=%zu/%zu self=%zu height=%zu "
        "fresh=%s stamp_delta=%.3fs fused=%zu",
        far_count, far_input_count, far_ground_filtered, near_count,
        matching_near ? matching_near->candidates : 0U,
        matching_near ? matching_near->self_filtered : 0U,
        matching_near ? matching_near->height_filtered : 0U,
        near_fresh ? "true" : "false", matching_stamp_delta, points.size());
    }
  }

  std::string far_topic_;
  std::string raw_topic_;
  std::string output_topic_;
  double near_min_{0.25};
  double near_max_{1.05};
  double near_voxel_{0.04};
  double max_raw_age_{0.25};
  double max_raw_stamp_delta_{0.03};
  double base_to_lidar_x_{0.1701};
  double base_to_lidar_y_{0.0};
  double base_to_lidar_z_{0.0908};
  double mount_cos_{0.0};
  double mount_sin_{1.0};
  double body_radius_{0.24};
  double body_offset_{0.12};
  double self_z_min_{-0.55};
  double self_z_max_{0.30};
  double obstacle_z_min_{-0.39};
  double obstacle_z_max_{0.55};

  std::deque<NearSample> near_samples_;
  SteadyClock::time_point last_log_{};

  rclcpp::Publisher<Cloud>::SharedPtr publisher_;
  rclcpp::Subscription<Cloud>::SharedPtr raw_subscription_;
  rclcpp::Subscription<Cloud>::SharedPtr far_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NearFieldCloudFuser>());
  rclcpp::shutdown();
  return 0;
}
