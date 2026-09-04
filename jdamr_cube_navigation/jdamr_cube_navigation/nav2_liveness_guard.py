"""Fail loudly when a required Nav2 node leaves the ROS graph."""

# 2026-09-01: running Nav2 in one composed container let internal nodes
# disappear from the DDS graph while the container process stayed alive.
# Nothing noticed, so the route executor waited on servers that no longer
# existed.  The reaction then was to split every node into its own process so
# launch could see a process die.  That worked as a detector but moved all
# intra-robot traffic onto DDS/UDP, and the measured cost was severe: the
# corridor drive reached 37.6 m composed and stalled at 10 m split, with load
# climbing 5.9 -> 17.8 as the wireless link degraded.
#
# This guard restores the detection without paying that price.  Composition
# keeps intra-process delivery; the guard watches the graph and exits non-zero
# the moment a required node vanishes, which the launch treats as fatal.

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
