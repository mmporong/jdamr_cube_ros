"""Fail loudly when a required Nav2 node leaves the ROS graph."""

# A composed container can stay alive after internal nodes leave the graph, so
# process exit alone is insufficient evidence of Nav2 liveness.  This guard is
# a secondary detector beside lifecycle bonds.  It identifies graph loss; it
# does not establish why callbacks, transforms, or bonds stopped.  Historical
# measurements and causal limits live in evaluation/20260904_HANDOFF.md.

import argparse
import sys
import time

import rclpy
from rclpy.node import Node


DEFAULT_REQUIRED = (
    'map_server',
    'amcl',
    'controller_server',
    'planner_server',
    'bt_navigator',
    'collision_monitor',
    'velocity_smoother',
    'keepout_filter_mask_server',
    'keepout_costmap_filter_info_server',
)


def missing_nodes(present, required):
    """Return the required names absent from *present*, order preserved."""
    seen = {name.split('/')[-1] for name in present}
    return [name for name in required if name not in seen]


class Nav2LivenessGuard(Node):
    """Watch the ROS graph and report the first required node that vanishes."""

    def __init__(self, required, grace_s, period_s):
        super().__init__('nav2_liveness_guard')
        self.required = tuple(required)
        self.period_s = period_s
        self.deadline = time.monotonic() + grace_s
        self.ready = False
        self.failure = None
        # A node can flicker out of one discovery sample without being dead,
        # so a single miss is not enough to end a drive.
        self.consecutive_misses = 0
        self.get_logger().info(
            f'watching {len(self.required)} Nav2 nodes; '
            f'grace={grace_s:.0f}s period={period_s:.1f}s')
        self.create_timer(period_s, self._tick)

    def _names(self):
        return [f'{ns.rstrip("/")}/{name}' if ns not in ('', '/') else name
                for name, ns in self.get_node_names_and_namespaces()]

    def _tick(self):
        absent = missing_nodes(self._names(), self.required)
        if not self.ready:
            if not absent:
                self.ready = True
                self.get_logger().info('all required Nav2 nodes present')
            elif time.monotonic() > self.deadline:
                self.failure = (
                    'Nav2 nodes never appeared within the grace period: '
                    + ', '.join(absent))
                raise SystemExit(0)
            return
        if absent:
            self.consecutive_misses += 1
            self.get_logger().warn(
                f'required Nav2 node(s) missing from the graph '
                f'({self.consecutive_misses}/2): ' + ', '.join(absent))
            if self.consecutive_misses >= 2:
                self.failure = (
                    'required Nav2 node(s) vanished from the graph: '
                    + ', '.join(absent))
                raise SystemExit(0)
        else:
            self.consecutive_misses = 0


def main(argv=None):
    """Run the guard; exit non-zero once a required node is gone."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--required', default=','.join(DEFAULT_REQUIRED))
    parser.add_argument('--grace', type=float, default=120.0,
                        help='seconds allowed for the nodes to come up')
    parser.add_argument('--period', type=float, default=5.0)
    args, ros_args = parser.parse_known_args(
        sys.argv[1:] if argv is None else argv)

    rclpy.init(args=ros_args)
    node = Nav2LivenessGuard(
        [name for name in args.required.split(',') if name],
        args.grace, args.period)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        failure = node.failure
        if failure:
            node.get_logger().error(failure)
            print(f'nav2_liveness_guard: {failure}', flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 1 if failure else 0


if __name__ == '__main__':
    sys.exit(main())
