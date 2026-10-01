"""Read-only: map->base_link TF, /amcl_pose and one /scan in base_link-ish laser frame."""
import json, math, time
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = rclpy.create_node('claude_readonly_pose_probe')
buf = Buffer(); TransformListener(buf, node)
out = {'scan': None, 'amcl': None, 'tf': None, 'laser_tf': None}
def on_scan(m):
    if out['scan'] is None:
        out['scan'] = {'frame': m.header.frame_id, 'angle_min': m.angle_min, 'inc': m.angle_increment,
                       'range_min': m.range_min,
                       'ranges': [r if math.isfinite(r) else None for r in m.ranges]}
def on_amcl(m):
    p = m.pose.pose
    out['amcl'] = [p.position.x, p.position.y, 2 * math.atan2(p.orientation.z, p.orientation.w)]
node.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data)
node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', on_amcl,
                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                    reliability=ReliabilityPolicy.RELIABLE))
end = time.time() + 8
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.1)
    if out['tf'] is None:
        try:
            t = buf.lookup_transform('map', 'base_link', rclpy.time.Time())
            q = t.transform.rotation
            out['tf'] = [t.transform.translation.x, t.transform.translation.y, 2 * math.atan2(q.z, q.w)]
        except Exception:
            pass
    if out['scan'] and out['laser_tf'] is None:
        try:
            t = buf.lookup_transform('base_link', out['scan']['frame'], rclpy.time.Time())
            q = t.transform.rotation
            out['laser_tf'] = [t.transform.translation.x, t.transform.translation.y, 2 * math.atan2(q.z, q.w)]
        except Exception:
            pass
    if all(out.values()):
        break
print('JSON:' + json.dumps(out))
node.destroy_node(); rclpy.shutdown()
