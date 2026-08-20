#include <array>
#include <cmath>
#include <cstddef>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "unitree_go/msg/low_state.hpp"

class ImuAttitudeBridge : public rclcpp::Node
{
public:
  ImuAttitudeBridge()
  : Node("imu_attitude_bridge")
  {
    auto input_qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable().durability_volatile();
    sub_ = create_subscription<unitree_go::msg::LowState>(
      "/lowstate", input_qos,
      std::bind(&ImuAttitudeBridge::on_lowstate, this, std::placeholders::_1));
    report_timer_ = create_wall_timer(
      std::chrono::seconds(5), std::bind(&ImuAttitudeBridge::report, this));
    RCLCPP_INFO(get_logger(), "IMU 姿态桥启动: 正在用 2 秒静止样本标定内置 IMU");
  }

private:
  static constexpr std::size_t kCalibrationSamples = 1000;
  static constexpr double kGravity = 9.46036;

  void on_lowstate(const unitree_go::msg::LowState::SharedPtr msg)
  {
    // 源为 500 Hz；标定后隔帧输出 250 Hz，匹配 10 Hz 机械雷达并降低 DDS 负载。
    if (pub_) {
      ++input_seq_;
      if (input_seq_ % 2 != 0) {
        return;
      }
    }

    const auto & imu = msg->imu_state;
    const auto & q = imu.quaternion;  // Unitree: [w, x, y, z]
    const double q_norm = std::sqrt(
      static_cast<double>(q[0]) * q[0] + static_cast<double>(q[1]) * q[1] +
      static_cast<double>(q[2]) * q[2] + static_cast<double>(q[3]) * q[3]);
    if (!std::isfinite(q_norm) || q_norm < 0.5) {
      return;
    }
    const double qw = q[0] / q_norm;
    const double qx = q[1] / q_norm;
    const double qy = q[2] / q_norm;
    const double qz = q[3] / q_norm;

    if (!pub_) {
      std::array<double, 3> gyro{{imu.gyroscope[0], imu.gyroscope[1], imu.gyroscope[2]}};
      std::array<double, 3> acc{{imu.accelerometer[0], imu.accelerometer[1],
        imu.accelerometer[2]}};
      const double gyro_norm = std::sqrt(
        gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);
      const double acc_norm = std::sqrt(
        acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
      if (gyro_norm > 0.1 || acc_norm <= 8.0 || acc_norm >= 11.0) {
        return;
      }

      // q 是 body->world，旋转矩阵第三行是 world-z 在 body 坐标中的方向。
      const std::array<double, 3> expected{{
        kGravity * 2.0 * (qx * qz - qw * qy),
        kGravity * 2.0 * (qy * qz + qw * qx),
        kGravity * (1.0 - 2.0 * (qx * qx + qy * qy))}};
      for (std::size_t i = 0; i < 3; ++i) {
        acc_residual_sum_[i] += acc[i] - expected[i];
        gyro_sum_[i] += gyro[i];
      }
      ++calibration_count_;
      if (calibration_count_ < kCalibrationSamples) {
        return;
      }
      for (std::size_t i = 0; i < 3; ++i) {
        acc_bias_[i] = acc_residual_sum_[i] / calibration_count_;
        gyro_bias_[i] = gyro_sum_[i] / calibration_count_;
      }
      pub_ = create_publisher<sensor_msgs::msg::Imu>("/dog_imu_lio", rclcpp::SensorDataQoS());
      RCLCPP_INFO(
        get_logger(),
        "内置 IMU 标定完成: acc_bias=[%.4f, %.4f, %.4f], gyro_bias=[%.5f, %.5f, %.5f]",
        acc_bias_[0], acc_bias_[1], acc_bias_[2],
        gyro_bias_[0], gyro_bias_[1], gyro_bias_[2]);
      return;
    }

    sensor_msgs::msg::Imu out;
    out.header.stamp = now();
    out.header.frame_id = "base_link";
    out.orientation.x = qx;
    out.orientation.y = qy;
    out.orientation.z = qz;
    out.orientation.w = qw;
    out.angular_velocity.x = imu.gyroscope[0] - gyro_bias_[0];
    out.angular_velocity.y = imu.gyroscope[1] - gyro_bias_[1];
    out.angular_velocity.z = imu.gyroscope[2] - gyro_bias_[2];
    out.linear_acceleration.x = imu.accelerometer[0] - acc_bias_[0];
    out.linear_acceleration.y = imu.accelerometer[1] - acc_bias_[1];
    out.linear_acceleration.z = imu.accelerometer[2] - acc_bias_[2];
    out.orientation_covariance = {{0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.02}};
    out.angular_velocity_covariance =
      {{0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001}};
    out.linear_acceleration_covariance =
      {{0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01}};
    pub_->publish(out);
    ++published_;
  }

  void report()
  {
    if (!pub_) {
      RCLCPP_INFO(
        get_logger(), "IMU 标定中: %zu/%zu", calibration_count_, kCalibrationSamples);
    } else {
      RCLCPP_INFO(get_logger(), "IMU 姿态桥: %.1f Hz", published_ / 5.0);
    }
    published_ = 0;
  }

  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr report_timer_;
  std::size_t calibration_count_{0};
  std::size_t input_seq_{0};
  std::size_t published_{0};
  std::array<double, 3> acc_residual_sum_{{0.0, 0.0, 0.0}};
  std::array<double, 3> gyro_sum_{{0.0, 0.0, 0.0}};
  std::array<double, 3> acc_bias_{{0.0, 0.0, 0.0}};
  std::array<double, 3> gyro_bias_{{0.0, 0.0, 0.0}};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuAttitudeBridge>());
  rclcpp::shutdown();
  return 0;
}
