"""Read-only: does a fresh participant receive /map, the laser static TF and /scan.

Run on the Pi (ROS environment sourced): python3 dds_probe.py <name>
Prints one line: PROBE <name> ok|MISSING {counts} <seconds> s
"""
import sys
import time

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

# An idle Pi answered within 1-6 s; the executor itself waits 30 s.
WAIT_S = 20.0


def main():
    """Subscribe as a new participant and report what arrived."""
    name = sys.argv[1] if len(sys.argv) > 1 else 'probe'
    rclpy.init()
    node = rclpy.create_node('claude_dds_probe_' + name)
    got = {'map': 0, 'laser_tf': 0, 'scan': 0}

    def count(key, n=1):
        got[key] += n

    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    static = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(OccupancyGrid, '/map', lambda _m: count('map'), latched)
    node.create_subscription(
        TFMessage, '/tf_static',
        lambda m: count('laser_tf', sum(t.child_frame_id == 'laser_link' for t in m.transforms)),
        static)
    node.create_subscription(LaserScan, '/scan', lambda _m: count('scan'), qos_profile_sensor_data)
    start = time.time()
    while time.time() - start < WAIT_S and not all(got.values()):
        rclpy.spin_once(node, timeout_sec=0.1)
    print('PROBE', name, 'ok' if all(got.values()) else 'MISSING', got,
          round(time.time() - start, 1), 's', flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
