"""Start saved-map Nav2 with the registered assets and parking controller."""

from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from jdamr_cube_navigation.docking_stop_profile import apply_docking_stop_profile
from jdamr_cube_navigation.parking import (
    load_parking_contract, parking_controller_overrides,
)
from jdamr_cube_navigation.reverse_parking import (
    reverse_controller_overrides, SERVICE_TRANSIT_MAX_MPS)
from jdamr_cube_navigation.service_destinations import expanded_path, load_registry
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import yaml


def _configure(context):
    registry = load_registry(LaunchConfiguration('registry').perform(context))
    package = Path(get_package_share_directory('jdamr_cube_navigation'))
    source = expanded_path(LaunchConfiguration('params_file').perform(context))
    document = yaml.safe_load(source.read_text(encoding='utf-8'))
    contract_path = LaunchConfiguration(
        'parking_contract', default=str(package / 'config/parking_contract.yaml'))
    contract = load_parking_contract(Path(contract_path.perform(context)))
    controller = parking_controller_overrides(document, contract)
    if (registry.get('home') or {}).get('parking_direction') == 'reverse':
        controller = reverse_controller_overrides(controller)
        smoother = document['velocity_smoother']['ros__parameters']
        smoother['min_velocity'][0] = -contract['desired_linear_mps']
        # Transit no longer runs at the parking contract speed (0.04 m/s cap,
        # 8042a31); the parking controllers bound themselves.
        smoother['max_velocity'][0] = min(
            smoother['max_velocity'][0], SERVICE_TRANSIT_MAX_MPS)
        transit = controller['FollowPath']
        for key in ('desired_linear_vel', 'min_approach_linear_velocity',
                    'regulated_linear_scaling_min_speed'):
            transit[key] = min(transit[key], SERVICE_TRANSIT_MAX_MPS)
    document['controller_server']['ros__parameters'] = controller
    # Transit through several waypoints as one goal (service session only: the
    # stock through-poses tree needs backup/spin servers, so the default is ours).
    navigator = document['bt_navigator']['ros__parameters']
    navigator['navigators'] = [*navigator['navigators'], 'navigate_through_poses']
    navigator['navigate_through_poses'] = {
        'plugin': 'nav2_bt_navigator::NavigateThroughPosesNavigator'}
    navigator['default_nav_through_poses_bt_xml'] = str(
        package / 'behavior_trees/navigate_through_poses_transit.xml')
    if LaunchConfiguration('precision_parking', default='false').perform(context) == 'true':
        geometry_path = (Path(get_package_share_directory('jdamr_cube_description'))
                         / 'config/new_base_geometry.yaml')
        document = apply_docking_stop_profile(
            document, yaml.safe_load(geometry_path.read_text(encoding='utf-8')))
        behavior = document['behavior_server']['ros__parameters']
        behavior.update({
            'max_rotational_vel': contract['rotate_angular_radps'],
            'min_rotational_vel': contract['rotate_angular_radps'] / 2.0,
            'enable_stamped_cmd_vel': False,
        })
    with tempfile.NamedTemporaryFile(
            mode='w', prefix='jdamr_service_', suffix='.yaml',
            encoding='utf-8', delete=False) as stream:
        yaml.safe_dump(document, stream, sort_keys=False)
        generated = Path(stream.name)

    def cleanup(_context):
        generated.unlink(missing_ok=True)
        return []

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(package / 'launch/onboard_nav2_core.launch.py')),
        launch_arguments={
            'map': registry['map']['yaml_path'],
            'keepout_mask': registry['keepout']['yaml_path'],
            'asset_registry': LaunchConfiguration('registry'),
            'params_file': str(generated),
            'navigation_profile': LaunchConfiguration('navigation_profile'),
            'discovery_range': LaunchConfiguration('discovery_range'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true',
            'navigation_autostart': LaunchConfiguration(
                'navigation_autostart', default='true'),
            'precision_parking': LaunchConfiguration('precision_parking', default='false'),
            'enable_box_search': LaunchConfiguration('precision_parking', default='false'),
            'use_composition': LaunchConfiguration(
                'use_composition', default='true'),
            'coordinated_startup': LaunchConfiguration(
                'coordinated_startup', default='true'),
        }.items(),
    )
    box_observer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(
            package / 'launch/depth_box_parking.launch.py')),
        condition=IfCondition(LaunchConfiguration('use_box_observer')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )
    return [RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
            navigation, box_observer]


def generate_launch_description():
    """Load navigation only; named destination execution is a separate command."""
    package = Path(get_package_share_directory('jdamr_cube_navigation'))
    return LaunchDescription([
        DeclareLaunchArgument('precision_parking', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('registry', description='Taught service destination YAML'),
        DeclareLaunchArgument('parking_contract', default_value=str(
            package / 'config/parking_contract.yaml')),
        DeclareLaunchArgument('params_file', default_value=str(
            package / 'config/new_base_nav2_params.yaml')),
        DeclareLaunchArgument(
            'navigation_profile', default_value='new_base_candidate',
            choices=['new_base_candidate', 'new_base_revisit_candidate', 'corridor']),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_composition', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('coordinated_startup', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('navigation_autostart', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument(
            'use_box_observer', default_value='false',
            choices=['true', 'false'],
            description='Start the perception-only RGB-D box observer'),
        DeclareLaunchArgument(
            'discovery_range', default_value='LOCALHOST',
            choices=['LOCALHOST', 'SUBNET'],
            description='Match the physical onboard navigation sensor discovery scope'),
        OpaqueFunction(function=_configure),
    ])
