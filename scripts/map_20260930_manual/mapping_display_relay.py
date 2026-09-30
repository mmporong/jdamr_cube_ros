"""Temporary mapping display relay: the repo relay plus the live Cartographer /map.

PC:  python3 mapping_display_relay.py import      (LOCALHOST graph on the PC)
Pi:  the import side pipes this file over ssh and runs `python3 - export` against the
     installed jdamr_cube_navigation.rviz_display_relay (no install, no build).
Display only: the export side subscribes; it never publishes on the Pi.
"""
import os
from pathlib import Path
import subprocess
import sys

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

import jdamr_cube_navigation.rviz_display_relay as relay

HOST = 'lim@192.168.0.159'
WORKSPACE = '/home/lim/jdamr_ws'
relay.TOPICS['/map'] = OccupancyGrid
_base_qos = relay.qos


def _qos(topic, *, subscriber=False):
    # Pi side: reliable+volatile matches a volatile or transient-local publisher.
    # PC side: transient-local so RViz (Transient Local) connects at any time.
    if topic == '/map':
        return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                          durability=(DurabilityPolicy.VOLATILE if subscriber
                                      else DurabilityPolicy.TRANSIENT_LOCAL))
    return _base_qos(topic, subscriber=subscriber)


relay.qos = _qos


def run_export():
    rclpy.init(args=[])
    node = relay.DisplayExport()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, BrokenPipeError):
        pass


def run_import():
    remote = relay.ssh_command(HOST, WORKSPACE)
    remote[-1] = remote[-1].replace(
        'python3 -m jdamr_cube_navigation.rviz_display_relay export',
        'python3 - export')
    os.environ.update(ROS_DOMAIN_ID='12', ROS_LOCALHOST_ONLY='0',
                      ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                      FASTDDS_BUILTIN_TRANSPORTS='UDPv4')
    process = subprocess.Popen(remote, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    process.stdin.write(Path(__file__).read_bytes())
    process.stdin.close()
    rclpy.init(args=[])
    node = relay.DisplayImport(process)
    try:
        while rclpy.ok() and not node.finished.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
        node.flush()
        raise RuntimeError(node.failure or 'Pi display connection ended')
    except KeyboardInterrupt:
        pass
    finally:
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()


if __name__ == '__main__':
    run_export() if sys.argv[1:] == ['export'] else run_import()
