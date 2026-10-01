"""Read-only: does a fresh participant receive the maps, the laser static TF and /scan.

Run on the Pi (ROS environment sourced): python3 dds_probe.py <name>
Prints one line: PROBE <name> ok|MISSING {counts} <seconds> s
The box observer status is counted for diagnosis only; it is not required.
"""
import sys
import time

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

# The executor's verify_live_maps waits 30 s (restaurant_service.py); an idle Pi
# answered within 1-6 s.
WAIT_S = 30.0
REQUIRED = ('map', 'keepout', 'laser_tf', 'scan')


def main():
    """Subscribe as a new participant and report what arrived."""
    name = sys.argv[1] if len(sys.argv) > 1 else 'probe'
    rclpy.init()
    node = rclpy.create_node('claude_dds_probe_' + name)
    got = {'map': 0, 'keepout': 0, 'laser_tf': 0, 'scan': 0, 'box_status': 0}

    def count(key, n=1):
        got[key] += n

    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    static = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(OccupancyGrid, '/map', lambda _m: count('map'), latched)
    node.create_subscription(OccupancyGrid, '/keepout_filter_mask',
                             lambda _m: count('keepout'), latched)
    node.create_subscription(
        TFMessage, '/tf_static',
        lambda m: count('laser_tf', sum(t.child_frame_id == 'laser_link' for t in m.transforms)),
        static)
    node.create_subscription(LaserScan, '/scan', lambda _m: count('scan'), qos_profile_sensor_data)
    node.create_subscription(String, '/box_parking/perception_status',
                             lambda _m: count('box_status'), 10)
    start = time.time()
    received = False
    while time.time() - start < WAIT_S and not received:
        rclpy.spin_once(node, timeout_sec=0.1)
        received = all(got[key] for key in REQUIRED)
    print('PROBE', name, 'ok' if received else 'MISSING', got,
          round(time.time() - start, 1), 's', flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
