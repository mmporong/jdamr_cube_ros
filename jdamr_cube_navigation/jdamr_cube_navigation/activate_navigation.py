"""Activate prepared Nav2 servers after stationary localization is confirmed."""

import argparse
from pathlib import Path
import time

from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
import rclpy

from jdamr_cube_navigation.parking import load_parking_contract
from jdamr_cube_navigation.restaurant_service import ServiceRoute
from jdamr_cube_navigation.service_destinations import load_registry


LOCALIZATION_NODES = (
    'map_server', 'amcl', 'keepout_filter_mask_server',
    'keepout_costmap_filter_info_server')
NAVIGATION_NODES = (
    'controller_server', 'planner_server', 'behavior_server',
    'bt_navigator', 'velocity_smoother', 'collision_monitor')


def require_active(node, names, state_id=State.PRIMARY_STATE_ACTIVE, wait=None):
    """Check lifecycle responses rather than process or graph presence."""
    for name in names:
        client = node.create_client(GetState, '/' + name + '/get_state')
        try:
            if not client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError(name + ' lifecycle service unavailable')
            response = (wait or node._wait)(client.call_async(GetState.Request()), 5.0)
            if response is None or response.current_state.id != state_id:
                raise RuntimeError(name + ' lifecycle state unconfirmed')
        finally:
            node.destroy_client(client)


def rollback_navigation(node, client):
    """Resolve lifecycle reset even when the ordinary request was interrupted."""
    def wait(future, timeout_s):
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not future.done() or future.exception() is not None or future.result() is None:
            raise RuntimeError('navigation rollback response unconfirmed')
        return future.result()

    request = ManageLifecycleNodes.Request()
    request.command = ManageLifecycleNodes.Request.RESET
    response = wait(client.call_async(request), 45.0)
    if not response.success:
        raise RuntimeError('navigation rollback rejected')
    require_active(node, NAVIGATION_NODES,
                   state_id=State.PRIMARY_STATE_UNCONFIGURED, wait=wait)
    node.emit('navigation_activation_rolled_back')


def activate_prepared(node):
    """Send lifecycle startup only; never set a pose or send a motion goal."""
    require_active(node, LOCALIZATION_NODES)
    node.verify_live_maps(require_command_path=False)
    if not node.wait_until_ready(timeout=10.0):
        raise RuntimeError('fresh localization or departure battery unavailable')
    client = node.create_client(
        ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')
    startup_requested = False
    try:
        if node.stop_requested or not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('navigation lifecycle startup unavailable')
        pose, evidence = node.capture_stationary_pose(timeout_s=10.0)
        # Recheck inputs after service discovery; readiness is not a cached grant.
        failure = node._guard_failure(require_fresh_amcl=True)
        if failure or node.stop_requested:
            raise RuntimeError(failure or 'activation interrupted')
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        startup_requested = True
        response = node._wait(client.call_async(request), 45.0)
        if response is None or not response.success:
            raise RuntimeError('navigation lifecycle startup failed')
        require_active(node, NAVIGATION_NODES)
        node.verify_live_maps()
        node.emit('navigation_activated_without_motion',
                  stationary_pose=pose, stationary_evidence=evidence)
    except Exception:
        if startup_requested:
            try:
                rollback_navigation(node, client)
            except Exception as error:
                node.emit('navigation_activation_rollback_unconfirmed',
                          reason=str(error), navigation_ready=False)
                raise RuntimeError('navigation rollback unconfirmed; do not depart') from error
        raise
    finally:
        node.destroy_client(client)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--registry', required=True, type=Path)
    parser.add_argument('--parking-contract', required=True, type=Path)
    parser.add_argument('--log', required=True, type=Path)
    args = parser.parse_args(argv)
    registry = load_registry(args.registry)
    contract = load_parking_contract(args.parking_contract)
    with args.log.open('x', encoding='utf-8') as stream:
        rclpy.init()
        node = ServiceRoute(registry, contract, stream)
        try:
            activate_prepared(node)
            return 0
        except RuntimeError as error:
            node.emit('activation_failed', reason=str(error))
            return 1
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
