"""Static graph-contract tests for the evaluation-only G005 launch."""

from pathlib import Path


LAUNCH = Path(__file__).parents[1] / 'launch' / 'g005_frontier_eval.launch.py'


def test_g005_launch_keeps_localization_and_navigation_evaluation_only():
    source = LAUNCH.read_text(encoding='utf-8')
    for required in (
            "executable='g005_ground_truth_localization'",
            "executable='controller_server'",
            "executable='planner_server'",
            "executable='behavior_server'",
            "executable='velocity_smoother'",
            "executable='collision_monitor'",
            "executable='bt_navigator'",
            "'navigate_to_pose_safe_mapping.xml'",
            "('cmd_vel', 'cmd_vel_nav')",
            "('/tf', '/g005/map_to_odom_tf')"):
        assert required in source
    forbidden_nodes = (
        'nav2_amcl', 'cartographer_ros', "executable='map_server'")
    for forbidden in forbidden_nodes:
        assert forbidden not in source


def test_g005_launch_lifecycle_set_is_explicit_and_complete():
    source = LAUNCH.read_text(encoding='utf-8')
    for node_name in (
            'controller_server', 'planner_server', 'behavior_server',
            'velocity_smoother', 'collision_monitor', 'bt_navigator'):
        assert f"'{node_name}'" in source
    assert "'node_names': lifecycle_nodes" in source
    assert "'ground_truth_frame': 'g005_frontier'" in source
    assert 'Generated and hash-sealed G005 Nav2 params' in source
    assert 'default_value=os.path.join' not in source
