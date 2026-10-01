"""Republish /imu/data_raw with the gyro bias removed, for the odom+IMU EKF.

The bias is learned only while the wheels report no motion (/odom twist exactly zero
for STILL_MIN_S), so turning the robot never feeds the estimate. The output keeps the
sensor axes and declares frame imu_link; the URDF imu_joint gives the base_link->imu_link
rotation (z down, board y forward, board x left; new_base_geometry.yaml imu_rotation_rpy).
"""

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

STILL_MIN_S = 1.0
BIAS_TIME_CONSTANT_S = 5.0
# Stationary gz spread measured 0.0017-0.0068 rad/s (std) on 2026-09-15/16 bags.
GYRO_Z_VARIANCE = 0.005 ** 2


class GyroBias:
    """Per-axis bias from samples taken while the wheels are still."""

    def __init__(self, still_min_s=STILL_MIN_S, tau_s=BIAS_TIME_CONSTANT_S):
        """Start with zero bias; learn after still_min_s of still wheels."""
        self.still_min_s = still_min_s
        self.tau_s = tau_s
        self.still_since = None
        self.bias = [0.0, 0.0, 0.0]
        self.learned_s = 0.0
        self.last_t = None

    def wheels(self, t, moving):
        """Record whether the wheels moved at time t."""
        self.still_since = None if moving else (
            self.still_since if self.still_since is not None else t)

    def gyro(self, t, rates):
        """Return the rates minus the bias, learning it while the wheels are still."""
        dt = 0.0 if self.last_t is None else max(0.0, t - self.last_t)
        self.last_t = t
        if self.still_since is None or t - self.still_since < self.still_min_s or dt == 0.0:
            return [r - b for r, b in zip(rates, self.bias)]
        # Average over the first tau seconds, then an exponential window of tau.
        self.learned_s += dt
        k = dt / min(self.learned_s, self.tau_s)
        self.bias = [b + k * (r - b) for r, b in zip(rates, self.bias)]
        return [r - b for r, b in zip(rates, self.bias)]


class ImuBiasRelay(Node):
    """Subscribe odom and imu/data_raw; publish imu/data."""

    def __init__(self):
        """Create the relay."""
        super().__init__('imu_bias_relay')
        self.frame_id = self.declare_parameter('frame_id', 'imu_link').value
        self.estimator = GyroBias()
        self.publisher = self.create_publisher(Imu, 'imu/data', qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self.on_odom, 20)
        self.create_subscription(Imu, 'imu/data_raw', self.on_imu, qos_profile_sensor_data)
        self.create_timer(5.0, self.report)

    def on_odom(self, msg):
        """Wheels are still when the driver reports zero twist."""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        twist = msg.twist.twist
        self.estimator.wheels(t, twist.linear.x != 0.0 or twist.angular.z != 0.0)

    def on_imu(self, msg):
        """Remove the bias and republish in imu_link."""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        w = msg.angular_velocity
        w.x, w.y, w.z = self.estimator.gyro(t, [w.x, w.y, w.z])
        msg.header.frame_id = self.frame_id
        msg.angular_velocity_covariance[8] = GYRO_Z_VARIANCE
        self.publisher.publish(msg)

    def report(self):
        """Log the current z bias."""
        e = self.estimator
        self.get_logger().info(
            f'gyro bias z {e.bias[2]:+.5f} rad/s from {e.learned_s:.0f} s still')


def main():
    """Run the relay."""
    rclpy.init()
    node = ImuBiasRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
