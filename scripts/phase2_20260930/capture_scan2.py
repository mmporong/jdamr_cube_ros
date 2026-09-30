"""Read-only: collect /scan, /odom, /tf_static, /battery_state and print one JSON line to stdout."""
import json, math, sys, time
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, BatteryState
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

N_SCANS = int(sys.argv[1]) if len(sys.argv) > 1 else 40
out = {'scans': [], 'odom': [], 'battery': [], 'tf_static': []}
rclpy.init()
node = rclpy.create_node('claude_readonly_scan_capture')


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def on_scan(m):
    if len(out['scans']) < N_SCANS:
        out['scans'].append({'stamp': stamp(m.header), 'frame': m.header.frame_id,
                             'angle_min': m.angle_min, 'angle_increment': m.angle_increment,
                             'range_min': m.range_min, 'range_max': m.range_max,
                             'ranges': [r if math.isfinite(r) else None for r in m.ranges]})


def on_odom(m):
    if len(out['odom']) < 400:
        p = m.pose.pose
        out['odom'].append({'stamp': stamp(m.header), 'x': p.position.x, 'y': p.position.y,
                            'qz': p.orientation.z, 'qw': p.orientation.w,
                            'v': m.twist.twist.linear.x, 'w': m.twist.twist.angular.z})


def on_batt(m):
    if len(out['battery']) < 5:
        out['battery'].append({'voltage': m.voltage, 'current': m.current,
                               'status': m.power_supply_status})


def on_tf_static(m):
    for t in m.transforms:
        tr, q = t.transform.translation, t.transform.rotation
        out['tf_static'].append({'parent': t.header.frame_id, 'child': t.child_frame_id,
                                 't': [tr.x, tr.y, tr.z], 'q': [q.x, q.y, q.z, q.w]})


latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
node.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data)
node.create_subscription(Odometry, '/odom', on_odom, qos_profile_sensor_data)
node.create_subscription(BatteryState, '/battery_state', on_batt, qos_profile_sensor_data)
node.create_subscription(TFMessage, '/tf_static', on_tf_static, latched)
end = time.monotonic() + 25.0
while time.monotonic() < end and (len(out['scans']) < N_SCANS or not out['battery']):
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node()
rclpy.shutdown()
sys.stdout.write('JSON:' + json.dumps(out) + '\n')
