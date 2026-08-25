// Copyright 2026 Lim
// SPDX-License-Identifier: MIT

#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "CYdLidar.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ydlidar_g4_ros2/scan_utils.hpp"

namespace ydlidar_g4_ros2
{

class YdlidarG4Node : public rclcpp::Node
{
public:
  YdlidarG4Node()
  : Node("ydlidar_g4_node"), running_(false)
  {
    port_ = declare_parameter<std::string>("port", "/dev/ydlidar_g4");
    frame_id_ = declare_parameter<std::string>("frame_id", "laser_link");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    frequency_ = declare_parameter<double>("frequency", 10.0);
    sample_rate_ = declare_parameter<double>("sample_rate", 9.0);

    if (frequency_ < 5.0 || frequency_ > 12.0) {
      throw std::invalid_argument("frequency must be in the G4 range [5, 12] Hz");
    }
    if (sample_rate_ <= 0.0) {
      throw std::invalid_argument("sample_rate must be positive (kHz)");
    }

    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    configure_lidar();
    start_lidar();
  }

  ~YdlidarG4Node() override
  {
    running_.store(false);
    if (scan_thread_.joinable()) {
      scan_thread_.join();
    }
    lidar_.turnOff();
    lidar_.disconnecting();
    ydlidar::os_shutdown();
  }

private:
  template<typename T>
  void set_option(int property, const T & value, const char * name)
  {
    if (!lidar_.setlidaropt(property, &value, sizeof(T))) {
      throw std::runtime_error(std::string("failed to set YDLidar option: ") + name);
    }
  }

  void configure_lidar()
  {
    ydlidar::os_init();
    if (!lidar_.setlidaropt(LidarPropSerialPort, port_.c_str(), port_.size())) {
      throw std::runtime_error("failed to set YDLidar serial port");
    }

    const int baudrate = 230400;
    const int lidar_type = TYPE_TRIANGLE;
    const int device_type = YDLIDAR_TYPE_SERIAL;
    const int sample_rate = static_cast<int>(std::lround(sample_rate_));
    const int abnormal_check_count = 4;
    const bool disabled = false;
    const bool enabled = true;
    const float min_angle = -180.0F;
    const float max_angle = 180.0F;
    const float min_range = 0.28F;
    const float max_range = 16.0F;
    const float scan_frequency = static_cast<float>(frequency_);

    set_option(LidarPropSerialBaudrate, baudrate, "baudrate");
    set_option(LidarPropLidarType, lidar_type, "lidar_type");
    set_option(LidarPropDeviceType, device_type, "device_type");
    set_option(LidarPropSampleRate, sample_rate, "sample_rate");
    set_option(LidarPropAbnormalCheckCount, abnormal_check_count, "abnormal_check_count");
    set_option(LidarPropFixedResolution, disabled, "fixed_resolution");
    set_option(LidarPropReversion, disabled, "reversion");
    set_option(LidarPropInverted, disabled, "inverted");
    set_option(LidarPropAutoReconnect, enabled, "auto_reconnect");
    set_option(LidarPropSingleChannel, disabled, "single_channel");
    set_option(LidarPropIntenstiy, disabled, "intensity");
    set_option(LidarPropSupportMotorDtrCtrl, disabled, "motor_dtr");
    set_option(LidarPropMinAngle, min_angle, "min_angle");
    set_option(LidarPropMaxAngle, max_angle, "max_angle");
    set_option(LidarPropMinRange, min_range, "min_range");
    set_option(LidarPropMaxRange, max_range, "max_range");
    set_option(LidarPropScanFrequency, scan_frequency, "scan_frequency");
    lidar_.enableGlassNoise(false);
    lidar_.enableSunNoise(false);
    lidar_.setBottomPriority(true);
  }

  void start_lidar()
  {
    if (!lidar_.initialize()) {
      throw std::runtime_error(std::string("G4 initialization failed: ") + lidar_.DescribeError());
    }
    if (!lidar_.turnOn()) {
      throw std::runtime_error(std::string("G4 start failed: ") + lidar_.DescribeError());
    }
    running_.store(true);
    scan_thread_ = std::thread([this]() {scan_loop();});
    RCLCPP_INFO(
      get_logger(), "YDLIDAR G4 started: %s, %.1f kHz, %.1f Hz -> %s",
      port_.c_str(), sample_rate_, frequency_, scan_topic_.c_str());
  }

  void scan_loop()
  {
    LaserScan sdk_scan;
    while (running_.load() && rclcpp::ok()) {
      if (!lidar_.doProcessSimple(sdk_scan)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "G4 scan read failed: %s", lidar_.DescribeError());
        continue;
      }
      publish_scan(sdk_scan);
    }
  }

  void publish_scan(const LaserScan & sdk_scan)
  {
    if (sdk_scan.points.size() < 2) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "G4 scan has fewer than two rays");
      return;
    }
    if (!has_sdk_scan_period(sdk_scan.config.scan_time)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for two complete G4 rotations before publishing");
      return;
    }
    ++complete_scans_seen_;
    if (complete_scans_seen_ < 2) {
      return;
    }

    std::vector<RawPoint> points;
    points.reserve(sdk_scan.points.size());
    for (const auto & point : sdk_scan.points) {
      points.push_back({point.angle, point.range, point.intensity});
    }

    const auto binned = bin_chronological_scan(points);
    const auto now = get_clock()->now();
    const auto timing = resolve_timing(
      sdk_scan.stamp, sdk_scan.config.scan_time, frequency_,
      static_cast<std::uint64_t>(now.nanoseconds()), binned.ranges.size(),
      previous_scan_end_ns_);

    if (timing.used_scan_time_fallback) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "G4 scan period %.3f ms is outside the configured frequency window; using %.3f ms",
        1000.0 * sdk_scan.config.scan_time, 1000.0 * timing.scan_time);
    }
    if (timing.stamp_adjustment_ns > 1'000'000ULL) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "G4 timestamp moved forward %.3f ms to preserve scan chronology",
        static_cast<double>(timing.stamp_adjustment_ns) / 1.0e6);
    }

    sensor_msgs::msg::LaserScan message;
    message.header.stamp = rclcpp::Time(timing.stamp_ns, RCL_SYSTEM_TIME);
    message.header.frame_id = frame_id_;
    message.angle_min = static_cast<float>(binned.angle_first);
    message.angle_max = static_cast<float>(binned.angle_last);
    message.angle_increment = static_cast<float>(binned.angle_increment);
    message.scan_time = static_cast<float>(timing.scan_time);
    message.time_increment = timing.time_increment;
    message.range_min = 0.28F;
    message.range_max = 16.0F;
    message.ranges = binned.ranges;
    message.intensities = binned.intensities;
    publisher_->publish(std::move(message));
    previous_scan_end_ns_ = timing.scan_end_ns;
  }

  std::string port_;
  std::string frame_id_;
  std::string scan_topic_;
  double frequency_;
  double sample_rate_;
  CYdLidar lidar_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  std::atomic<bool> running_;
  std::thread scan_thread_;
  std::size_t complete_scans_seen_{0};
  std::uint64_t previous_scan_end_ns_{0};
};

}  // namespace ydlidar_g4_ros2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ydlidar_g4_ros2::YdlidarG4Node>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("ydlidar_g4_node"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
