"""Submit one explicit bounded parking trial to an already prepared robot."""

import argparse
import math

import rclpy
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from std_srvs.srv import Trigger


def reference_distance(value):
    """Accept only a measured distance inside the trial capture envelope."""
    distance_m = float(value)
    if not math.isfinite(distance_m) or not .65 <= distance_m <= 1.0:
        raise argparse.ArgumentTypeError('reference must be 0.65..1.0 metres')
    return distance_m


def await_result(node, future):
    """Wait a bounded interval without repeating a request."""
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if not future.done():
        raise TimeoutError('service response timed out')
    return future.result()


def start_once(node, reference_m):
    """Configure ephemeral evidence and submit exactly one start request."""
    params = AsyncParameterClient(node, '/box_approach_execution')
    start = node.create_client(Trigger, '/box_parking/start_validation_approach')
    cancel = node.create_client(Trigger, '/box_parking/cancel_approach')
    if not params.wait_for_services(timeout_sec=5.0):
        raise RuntimeError('approach service is not prepared; no motion requested')
    if not start.wait_for_service(timeout_sec=5.0):
        raise RuntimeError('validation start unavailable; no motion requested')
    configured = await_result(node, params.set_parameters([
        Parameter('charger_unplugged_confirmed', value=True),
        Parameter('validation_reference_m', value=reference_m),
    ]))
    if len(configured.results) != 2 or not all(
            result.successful for result in configured.results):
        raise RuntimeError('trial configuration rejected; no motion requested')
    try:
        result = await_result(node, start.call_async(Trigger.Request()))
    except Exception:
        # A lost response is not proof that the robot stayed disarmed.
        # Revoke the token before cancel so a delayed start cannot re-arm.
        try:
            revoked = await_result(node, params.set_parameters([
                Parameter('validation_reference_m', value=0.0)]))
            if len(revoked.results) != 1 or not revoked.results[0].successful:
                raise RuntimeError('reference revocation unconfirmed; state unknown')
        finally:
            if not cancel.wait_for_service(timeout_sec=2.0):
                raise RuntimeError('cancel unavailable; robot stop unconfirmed')
            stopped = await_result(node, cancel.call_async(Trigger.Request()))
            if not stopped.success:
                raise RuntimeError('cancel rejected; robot stop unconfirmed')
        raise
    if not result.success:
        raise RuntimeError(result.message)
    return result.message


def main(args=None):
    """Start only with a new reference and explicit charger confirmation."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference-m', type=reference_distance, required=True)
    parser.add_argument('--charger-unplugged', action='store_true', required=True)
    options = parser.parse_args(args)
    rclpy.init(args=[])
    node = rclpy.create_node('box_parking_start_operator')
    try:
        print(start_once(node, options.reference_m), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
