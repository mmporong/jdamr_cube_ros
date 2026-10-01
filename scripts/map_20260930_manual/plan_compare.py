"""Plan each transit leg with GridBased and Lattice on the live session (no motion)."""
import json, math, sys, time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, ComputePathThroughPoses

def pose(n, x, y, yaw):
    p = PoseStamped(); p.header.frame_id = 'map'; p.header.stamp = n.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p

def heading(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])

def chain(start, wps):
    pts = [start] + wps; out = []
    for i, w in enumerate(wps):
        yaw = w[2] if len(w) > 2 and w[2] is not None else heading(pts[i], wps[i + 1] if i + 1 < len(wps) else w)
        out.append((w[0], w[1], yaw))
    return out

D = math.radians
LEGS = {
    'A dock->water obs': ((-0.204, 0.184, 0.0), [(0.496, 0.184, None), (1.06, -1.21, None), (0.64, -1.21, D(180))]),
    'B water->t02 obs': ((0.625, -1.21, D(180)), [(0.689, -1.591, None), (0.689, -2.011, D(-90))]),
    'C water->t01 obs': ((0.625, -1.21, D(180)), [(0.438, -1.817, None), (0.858, -1.817, 0.0)]),
    'D t01->staging': ((0.873, -1.817, 0.0), [(0.496, 0.184, 0.0)]),
    'E t02->staging': ((0.689, -2.026, D(-90)), [(0.496, 0.184, 0.0)]),
}

def main():
    rclpy.init(); n = Node('lattice_plan_compare')
    to_pose = ActionClient(n, ComputePathToPose, 'compute_path_to_pose')
    through = ActionClient(n, ComputePathThroughPoses, 'compute_path_through_poses')
    for c in (to_pose, through):
        if not c.wait_for_server(timeout_sec=10.0):
            print(json.dumps({'error': 'planner action unavailable'})); return
    rows = []
    for name, (start, wps) in LEGS.items():
        goals = chain(start, wps)
        straight = sum(math.dist(a[:2], b[:2]) for a, b in zip([start] + goals, goals))
        for planner in ('GridBased', 'Lattice'):
            if len(goals) == 1:
                g = ComputePathToPose.Goal(); g.goal = pose(n, *goals[0]); client = to_pose
            else:
                g = ComputePathThroughPoses.Goal(); g.goals = [pose(n, *w) for w in goals]; client = through
            g.start = pose(n, *start); g.use_start = True; g.planner_id = planner
            t0 = time.monotonic()
            fut = client.send_goal_async(g)
            rclpy.spin_until_future_complete(n, fut, timeout_sec=10.0)
            h = fut.result(); res_f = h.get_result_async()
            rclpy.spin_until_future_complete(n, res_f, timeout_sec=30.0)
            wall = time.monotonic() - t0
            r = res_f.result().result if res_f.done() else None
            row = {'leg': name, 'planner': planner, 'wall_s': round(wall, 3)}
            if r is None or not r.path.poses:
                row['error'] = getattr(r, 'error_code', None) if r else 'timeout'
            else:
                ps = [(p.pose.position.x, p.pose.position.y, 2 * math.atan2(p.pose.orientation.z, p.pose.orientation.w)) for p in r.path.poses]
                length = sum(math.dist(a[:2], b[:2]) for a, b in zip(ps, ps[1:]))
                end = ps[-1]; gl = goals[-1]
                turn = 0.0
                for a, b in zip(ps, ps[1:]):
                    turn += abs(math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2])))
                row.update(planning_s=round(r.planning_time.sec + r.planning_time.nanosec * 1e-9, 3),
                           length_m=round(length, 3), ratio=round(length / straight, 2),
                           end_err_m=round(math.dist(end[:2], gl[:2]), 3),
                           end_yaw_err_deg=round(math.degrees(math.atan2(math.sin(end[2] - gl[2]), math.cos(end[2] - gl[2]))), 1),
                           path_turn_deg=round(math.degrees(turn), 0), poses=len(ps))
            rows.append(row); print(json.dumps(row), flush=True)
    n.destroy_node(); rclpy.shutdown()

main()
