"""Start saved-map Nav2 with the registered assets and parking controller."""

from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from jdamr_cube_navigation.docking_stop_profile import apply_docking_stop_profile
from jdamr_cube_navigation.parking import (
    load_parking_contract, parking_controller_overrides,
)
from jdamr_cube_navigation.reverse_parking import reverse_controller_overrides
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
        document['velocity_smoother']['ros__parameters']['min_velocity'][0] = (
            -contract['desired_linear_mps'])
    document['controller_server']['ros__parameters'] = controller
    if LaunchConfiguration('precision_parking', default='false').perform(context) == 'true':
        geometry_path = (Path(get_package_share_directory('jdamr_cube_description'))
                         / 'config/new_base_geometry.yaml')
        document = apply_docking_stop_profile(
            document, yaml.safe_load(geometry_path.read_text(encoding='utf-8')))
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
            'precision_parking': LaunchConfiguration('precision_parking', default='false'),
            'use_composition': LaunchConfiguration(
                'use_composition', default='false'),
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
        DeclareLaunchArgument('use_composition', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('coordinated_startup', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument(
            'use_box_observer', default_value='false',
            choices=['true', 'false'],
            description='Start the perception-only RGB-D box observer'),
        DeclareLaunchArgument(
            'discovery_range', default_value='SUBNET',
            choices=['LOCALHOST', 'SUBNET'],
            description='Match the physical onboard navigation sensor discovery scope'),
        OpaqueFunction(function=_configure),
    ])
