#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

class LidarPreviewBridge : public rclcpp::Node
{
public:
  LidarPreviewBridge()
  : Node("lidar_preview_bridge")
  {
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/unitree/slam_lidar/points", rclcpp::SensorDataQoS(),
      std::bind(&LidarPreviewBridge::on_cloud, this, std::placeholders::_1));
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/go2_slam/lidar_preview", rclcpp::SensorDataQoS());
    RCLCPP_INFO(get_logger(), "雷达预览桥启动: 原始点云 -> /go2_slam/lidar_preview");
  }

private:
  static constexpr std::size_t kMaxPoints = 1400;

  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (msg->is_bigendian || msg->point_step == 0 || msg->data.empty()) {
      return;
    }
    int x_offset = -1;
    int y_offset = -1;
    int z_offset = -1;
    for (const auto & field : msg->fields) {
      if (field.datatype != sensor_msgs::msg::PointField::FLOAT32) {
        continue;
      }
      if (field.name == "x") x_offset = static_cast<int>(field.offset);
      if (field.name == "y") y_offset = static_cast<int>(field.offset);
      if (field.name == "z") z_offset = static_cast<int>(field.offset);
    }
    if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
      return;
    }

    const std::size_t count = static_cast<std::size_t>(msg->width) * msg->height;
    const std::size_t stride = std::max<std::size_t>(1, count / kMaxPoints);
    std::vector<float> xyz;
    xyz.reserve(kMaxPoints * 3);
    for (std::size_t i = 0; i < count && xyz.size() < kMaxPoints * 3; i += stride) {
      const std::size_t base = i * msg->point_step;
      if (base + msg->point_step > msg->data.size()) {
        break;
      }
      float x;
      float y;
      float z;
      std::memcpy(&x, msg->data.data() + base + x_offset, sizeof(float));
      std::memcpy(&y, msg->data.data() + base + y_offset, sizeof(float));
      std::memcpy(&z, msg->data.data() + base + z_offset, sizeof(float));
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        z < -1.0F || z > 3.0F || std::hypot(x, y) >= 30.0F)
      {
        continue;
      }
      xyz.push_back(x);
      xyz.push_back(y);
      xyz.push_back(z);
    }
    if (xyz.size() < 90) {
      return;
    }

    sensor_msgs::msg::PointCloud2 out;
    out.header = msg->header;
    out.height = 1;
    out.width = static_cast<uint32_t>(xyz.size() / 3);
    out.is_bigendian = false;
    out.is_dense = true;
    out.point_step = 12;
    out.row_step = out.point_step * out.width;
    out.fields.resize(3);
    const char * names[3] = {"x", "y", "z"};
    for (std::size_t i = 0; i < 3; ++i) {
      out.fields[i].name = names[i];
      out.fields[i].offset = static_cast<uint32_t>(i * sizeof(float));
      out.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
      out.fields[i].count = 1;
    }
    out.data.resize(xyz.size() * sizeof(float));
    std::memcpy(out.data.data(), xyz.data(), out.data.size());
    pub_->publish(out);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarPreviewBridge>());
  rclcpp::shutdown();
  return 0;
}
