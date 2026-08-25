// jdamr_base_driver — JD-AMR Cube 실기 베이스 드라이버.
//
// 구성 (스레드 2개):
//   * 읽기 스레드: 시리얼 → FrameParser → 오도메트리 적분 → odom/TF/IMU 발행.
//     프레임 도착(50Hz) 즉시 발행해 파이프라인 지연을 최소화한다.
//   * 실행기 스레드: cmd_vel 구독 + 20ms 송신 타이머. Nav2 가 어떤 주기로
//     지령을 밀어도 시리얼 쓰기는 50Hz 로 코어레싱된다 (강사원본 C7 수정).
//
// 강사원본 결함 대응 (번호 = JDAMR_Cube_조사기록.md):
//   C1/D1  cmd_vel 의 linear.x + angular.z 를 바퀴별 연속 속도로 — Twist 의미 복원
//   C3     /odom: 유효 쿼터니언, child_frame_id, 시각, 공분산 설정
//   C5     BatteryState 를 실측 전압으로 (Int8 100 더미 폐지)
//   C6     IMU orientation_covariance[0] = -1 (자세 추정 없음을 규약대로 표기)
//   C7     지령 레이트 리밋 (위 코어레싱)
//   C8     포트·보레이트·제원 전부 ROS 파라미터화
//   C9     spin_once busy-loop 재구현 폐지 — 표준 실행기 + 전용 읽기 스레드
//
// 물리 제원의 단일 출처는 이 노드의 파라미터다. wheel_radius / wheel_separation
// 기본값은 URDF 승계값(실측 아님) — 기동 시 경고를 찍고, 실측 후 갈아끼운다.
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "jdamr_base_driver/protocol.hpp"
#include "jdamr_base_driver/serial_port.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

namespace jdamr
{

class BaseDriverNode : public rclcpp::Node
{
public:
  BaseDriverNode()
  : Node("jdamr_base_driver")
  {
    // ── 파라미터 (C8) ──
    port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    baud_ = declare_parameter<int>("baud", 115200);
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.075);
    wheel_separation_ = declare_parameter<double>("wheel_separation", 0.35);
    counts_per_rev_ = declare_parameter<int>("counts_per_rev", 4096);
    // 좌우 유효 반지름 비 (오른쪽/왼쪽). 사출 편차로 두 바퀴 지름이 미세하게
    // 다르면 직진 지령에도 헤딩이 일정 비율로 흐른다 — 미끄러짐이 아니라
    // 기하 상수라 매번 같은 방향으로 휘고, 좌우를 한 값으로 캘리브레이션하면
    // 구조적으로 못 잡는다(UMBmark 가 사각형을 양방향으로 도는 이유).
    // 측정: 직진 L m 에 자이로 대비 헤딩 오차 dtheta 면 ratio ~= 1 + dtheta*b/L.
    wheel_radius_ratio_ = declare_parameter<double>("wheel_radius_ratio", 1.0);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    imu_frame_ = declare_parameter<std::string>("imu_frame", "imu_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.4);

    if (wheel_radius_ == 0.075 || wheel_separation_ == 0.35) {   // 플레이스홀더 기본값 그대로면 경고
      RCLCPP_WARN(get_logger(),
        "wheel_radius=%.4f / wheel_separation=%.4f 는 URDF 승계값(실측 아님). "
        "캘리퍼스·줄자 실측 후 파라미터로 교체할 것 — 오도메트리 스케일이 여기서 나온다.",
        wheel_radius_, wheel_separation_);
    }

    m_per_count_ = 2.0 * M_PI * wheel_radius_ / static_cast<double>(counts_per_rev_);
    // 평균은 보존하고 좌우로만 갈라 준다 — 병진 스케일(캘리브레이션 결과)을
    // 건드리지 않고 회전 편향만 없앤다.
    m_per_count_l_ = m_per_count_ * 2.0 / (1.0 + wheel_radius_ratio_);
    m_per_count_r_ = m_per_count_ * 2.0 * wheel_radius_ratio_ / (1.0 + wheel_radius_ratio_);
    if (std::abs(wheel_radius_ratio_ - 1.0) > 1e-9) {
      RCLCPP_INFO(get_logger(), "좌우 반지름 비 %.5f 적용 (L %.4e / R %.4e m/count)",
        wheel_radius_ratio_, m_per_count_l_, m_per_count_r_);
    }

    // ── 발행자 (재사용 메시지는 멤버로 선할당 — 핫패스 무할당) ──
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      "imu/data_raw", rclcpp::SensorDataQoS());
    mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>(
      "imu/mag", rclcpp::SensorDataQoS());
    batt_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    odom_msg_.header.frame_id = odom_frame_;
    odom_msg_.child_frame_id = base_frame_;          // C3
    // 차동구동 오도메트리의 관측 불가 축(z·roll·pitch)은 큰 값, 관측 축은 작은 값.
    // 실측(UMBmark류) 전의 보수적 초기값 — 융합(EKF) 시 재조정 대상.
    for (int i = 0; i < 36; ++i) {odom_msg_.pose.covariance[i] = 0.0;}
    odom_msg_.pose.covariance[0] = odom_msg_.pose.covariance[7] = 1e-3;
    odom_msg_.pose.covariance[14] = odom_msg_.pose.covariance[21] =
      odom_msg_.pose.covariance[28] = 1e6;
    odom_msg_.pose.covariance[35] = 1e-2;
    odom_msg_.twist.covariance = odom_msg_.pose.covariance;

    imu_msg_.header.frame_id = imu_frame_;
    imu_msg_.orientation_covariance[0] = -1.0;       // C6: 자세 추정 없음
    mag_msg_.header.frame_id = imu_frame_;
    batt_msg_.power_supply_technology =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    batt_msg_.present = true;

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        cmd_v_ = msg->linear.x;
        cmd_w_ = msg->angular.z;
        last_cmd_time_ = now();
      });

    // ── 시리얼 열기 ──
    std::string err;
    if (!serial_.open(port_, baud_, err)) {
      RCLCPP_FATAL(get_logger(), "시리얼 실패: %s", err.c_str());
      throw std::runtime_error(err);
    }
    RCLCPP_INFO(get_logger(), "%s @ %d 연결. m/count=%.6e", port_.c_str(), baud_, m_per_count_);

    send_timer_ = create_wall_timer(20ms, [this]() {send_command();});
    running_ = true;
    read_thread_ = std::thread([this]() {read_loop();});
  }

  ~BaseDriverNode() override
  {
    running_ = false;
    if (read_thread_.joinable()) {read_thread_.join();}
    uint8_t f[kCmdFrameSize];
    serial_.write_all(f, make_stop_cmd(f));   // 종료 시 정지 지령 (이중 안전)
  }

private:
  // ── 송신 경로 (실행기 스레드, 50Hz 고정) ──
  void send_command()
  {
    double v, w;
    rclcpp::Time last;
    {
      std::lock_guard<std::mutex> lk(cmd_mutex_);
      v = cmd_v_;
      w = cmd_w_;
      last = last_cmd_time_;
    }

    const bool stale =
      last.nanoseconds() == 0 || (now() - last).seconds() > cmd_timeout_;
    if (stale) {
      if (!stale_notified_) {
        uint8_t f[kCmdFrameSize];
        serial_.write_all(f, make_stop_cmd(f));   // 1회 명시 정지 후 침묵
        stale_notified_ = true;                   // → 펌웨어 워치독이 이어받음
      }
      return;
    }
    stale_notified_ = false;

    // 차동구동 역기구학 → counts/s
    const double vl = v - w * wheel_separation_ * 0.5;
    const double vr = v + w * wheel_separation_ * 0.5;
    double cl = vl / m_per_count_l_;
    double cr = vr / m_per_count_r_;

    // 포화는 비율 보존으로 — 독립 클램프는 포화 구간에서 곡률을 왜곡한다.
    const double peak = std::max(std::abs(cl), std::abs(cr));
    if (peak > kVelLimitCounts) {
      const double s = kVelLimitCounts / peak;
      cl *= s;
      cr *= s;
    }

    uint8_t f[kCmdFrameSize];
    const size_t n =
      make_velocity_cmd(f, static_cast<int16_t>(cl), static_cast<int16_t>(cr));
    if (!serial_.write_all(f, n)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "시리얼 쓰기 실패");
    }
  }

  // ── 수신 경로 (전용 스레드) ──
  void read_loop()
  {
    uint8_t buf[512];
    while (running_ && rclcpp::ok()) {
      const ssize_t n = serial_.read_some(buf, sizeof(buf));
      if (n < 0) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "시리얼 읽기 오류");
        continue;
      }
      if (n == 0) {continue;}                     // VTIME 타임아웃 — 종료 플래그 확인 주기
      parser_.feed(buf, static_cast<size_t>(n), [this](const State & s) {handle_state(s);});
    }
  }

  void handle_state(const State & s)
  {
    const rclcpp::Time stamp = now();
    const auto receive_time = std::chrono::steady_clock::now();

    if (!have_prev_) {
      prev_ = s;
      last_state_receive_time_ = receive_time;
      have_prev_ = true;
      return;
    }

    // dt 는 펌웨어 순번 기준(주기 20ms 고정)이 USB 도착 지터보다 정확하다.
    uint8_t gap = static_cast<uint8_t>(s.seq - prev_.seq);
    const double receive_gap = std::chrono::duration<double>(
      receive_time - last_state_receive_time_).count();
    if (gap == 0 && receive_gap <= kMaxStateIntegrationGapSeconds) {
      return;                                      // 짧은 중복 프레임 방어
    }
    if (!state_interval_is_integrable(prev_.seq, s.seq, receive_gap)) {
      RCLCPP_WARN(get_logger(),
        "상태 스트림 불연속(seq gap=%u, receive gap=%.3fs) — 해당 구간 적분 생략",
        gap, receive_gap);
      prev_ = s;
      last_state_receive_time_ = receive_time;
      return;
    }
    last_state_receive_time_ = receive_time;
    const double dt = 0.02 * static_cast<double>(gap);

    const double dl = static_cast<double>(s.left_pos - prev_.left_pos) * m_per_count_l_;
    const double dr = static_cast<double>(s.right_pos - prev_.right_pos) * m_per_count_r_;
    prev_ = s;

    const double ds = 0.5 * (dl + dr);
    const double dth = (dr - dl) / wheel_separation_;

    // 정확 원호 적분 — 오일러 근사는 회전 중 계통 오차를 쌓는다.
    if (std::abs(dth) < 1e-9) {
      x_ += ds * std::cos(th_);
      y_ += ds * std::sin(th_);
    } else {
      const double r_arc = ds / dth;
      x_ += r_arc * (std::sin(th_ + dth) - std::sin(th_));
      y_ -= r_arc * (std::cos(th_ + dth) - std::cos(th_));
    }
    th_ = std::atan2(std::sin(th_ + dth), std::cos(th_ + dth));   // 정규화

    // ── /odom (선할당 멤버 재사용) ──
    odom_msg_.header.stamp = stamp;
    odom_msg_.pose.pose.position.x = x_;
    odom_msg_.pose.pose.position.y = y_;
    odom_msg_.pose.pose.orientation.z = std::sin(th_ * 0.5);   // C3: 유효 쿼터니언
    odom_msg_.pose.pose.orientation.w = std::cos(th_ * 0.5);
    odom_msg_.twist.twist.linear.x = ds / dt;
    odom_msg_.twist.twist.angular.z = dth / dt;
    odom_pub_->publish(odom_msg_);

    if (publish_tf_) {
      tf_msg_.header.stamp = stamp;
      tf_msg_.header.frame_id = odom_frame_;
      tf_msg_.child_frame_id = base_frame_;
      tf_msg_.transform.translation.x = x_;
      tf_msg_.transform.translation.y = y_;
      tf_msg_.transform.rotation = odom_msg_.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf_msg_);
    }

    // ── IMU (스케일 규격: 펌웨어 헤더 표) ──
    if (!s.qmi8658_error()) {
      imu_msg_.header.stamp = stamp;
      imu_msg_.linear_acceleration.x = s.accel_mg[0] * kMgToMs2;
      imu_msg_.linear_acceleration.y = s.accel_mg[1] * kMgToMs2;
      imu_msg_.linear_acceleration.z = s.accel_mg[2] * kMgToMs2;
      imu_msg_.angular_velocity.x = s.gyro_cdps[0] * kCdpsToRads;
      imu_msg_.angular_velocity.y = s.gyro_cdps[1] * kCdpsToRads;
      imu_msg_.angular_velocity.z = s.gyro_cdps[2] * kCdpsToRads;
      imu_pub_->publish(imu_msg_);
    }

    if (!s.ak09918_error()) {
      mag_msg_.header.stamp = stamp;
      mag_msg_.magnetic_field.x = s.mag_dut[0] * kDutToTesla;
      mag_msg_.magnetic_field.y = s.mag_dut[1] * kDutToTesla;
      mag_msg_.magnetic_field.z = s.mag_dut[2] * kDutToTesla;
      mag_pub_->publish(mag_msg_);
    }

    // ── 배터리 1Hz + 진단 ──
    if (++frame_count_ % 50 == 0) {
      batt_msg_.header.stamp = stamp;
      batt_msg_.present = !s.ina219_error();
      batt_msg_.voltage = s.ina219_error() ?
        std::numeric_limits<float>::quiet_NaN() : s.batt_mv * 1e-3f;
      batt_pub_->publish(batt_msg_);

      const uint64_t crc = parser_.crc_errors();
      if (crc != last_crc_errors_) {
        RCLCPP_WARN(get_logger(), "CRC 오류 누적 %lu (+%lu) — 케이블·접지 점검",
          crc, crc - last_crc_errors_);
        last_crc_errors_ = crc;
      }
    }
    if (s.watchdog_stopped()) {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
        "펌웨어 워치독 정지 상태");
    }
    if (s.servo_error()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "서보 읽기 실패 플래그 (flags=0x%02X) — 버스·ID 점검", s.flags);
    }
    if (s.qmi8658_error()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "QMI8658 읽기 실패 (flags=0x%02X) — IMU 발행 생략", s.flags);
    }
    if (s.ak09918_error()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "AK09918 읽기 실패 (flags=0x%02X) — 자력계 발행 생략", s.flags);
    }
    if (s.ina219_error()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "INA219 미검출 (flags=0x%02X) — 배터리 전압을 NaN/미장착으로 발행", s.flags);
    }
  }

  // 단위 환산 상수
  static constexpr double kMgToMs2 = 1e-3 * 9.80665;          // mg → m/s²
  static constexpr double kCdpsToRads = 0.01 * M_PI / 180.0;  // 0.01dps → rad/s
  static constexpr double kDutToTesla = 0.1 * 1e-6;           // 0.1uT → T
  static constexpr double kVelLimitCounts = 3400.0;           // 펌웨어 VEL_LIMIT 와 동일

  // 파라미터
  std::string port_, odom_frame_, base_frame_, imu_frame_;
  int baud_{115200}, counts_per_rev_{4096};
  double wheel_radius_{0.075}, wheel_separation_{0.35}, cmd_timeout_{0.4};
  double m_per_count_{0.0};
  double wheel_radius_ratio_{1.0};
  double m_per_count_l_{0.0}, m_per_count_r_{0.0};
  bool publish_tf_{true};

  // 통신
  SerialPort serial_;
  FrameParser parser_;
  std::thread read_thread_;
  std::atomic<bool> running_{false};

  // 지령 상태 (구독 콜백 ↔ 송신 타이머 공유)
  std::mutex cmd_mutex_;
  double cmd_v_{0.0}, cmd_w_{0.0};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  bool stale_notified_{false};

  // 오도메트리 상태 (읽기 스레드 전용 — 락 불필요)
  State prev_{};
  bool have_prev_{false};
  double x_{0.0}, y_{0.0}, th_{0.0};
  uint64_t frame_count_{0}, last_crc_errors_{0};
  std::chrono::steady_clock::time_point last_state_receive_time_{};

  // ROS 인터페이스 (핫패스 메시지는 선할당 멤버)
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr batt_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  nav_msgs::msg::Odometry odom_msg_;
  sensor_msgs::msg::Imu imu_msg_;
  sensor_msgs::msg::MagneticField mag_msg_;
  sensor_msgs::msg::BatteryState batt_msg_;
  geometry_msgs::msg::TransformStamped tf_msg_;
};

}  // namespace jdamr

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<jdamr::BaseDriverNode>());
  } catch (const std::exception & e) {
    fprintf(stderr, "기동 실패: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
