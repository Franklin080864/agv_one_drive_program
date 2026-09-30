from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_fault_logging(context):
    if not IfCondition(LaunchConfiguration('enable_fault_logging')).evaluate(context):
        return []

    log_root = Path(
        LaunchConfiguration('fault_log_root').perform(context)
    ).expanduser()
    log_root.mkdir(parents=True, exist_ok=True)
    session_name = datetime.now().strftime('chassis_fault_%Y%m%d_%H%M%S')
    output_directory = log_root / session_name
    cache_size = LaunchConfiguration('fault_cache_size').perform(context)

    topics = [
        '/cmd_vel',
        '/wheel_command',
        '/joy',
        '/to_can_bus',
        '/from_can_bus',
        '/agv2/chassis_telemetry',
        '/agv2/diagnostics',
        '/agv2/log_event',
        '/rosout',
        '/parameter_events',
        '/Battery_SOC_STATE',
    ]
    recorder = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '--snapshot-mode',
            '--max-cache-size', cache_size,
            '-o', str(output_directory),
            *topics,
        ],
        output='screen',
    )
    snapshot_node = Node(
        package='agv2_pkg',
        executable='chassis_fault_snapshot.py',
        name='chassis_fault_snapshot',
        output='screen',
        parameters=[{
            'snapshot_service': '/rosbag2_recorder/snapshot',
            'output_directory': str(output_directory),
            'event_settle_ms': 200,
            'service_timeout_s': 10.0,
        }],
    )
    return [recorder, snapshot_node]


def generate_launch_description():
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('agv2_pkg'),
            'config', 'chassis.yaml',
        ]),
        description='Path to chassis parameter yaml',
    )
    logging_arg = DeclareLaunchArgument(
        'enable_fault_logging',
        default_value='true',
        description='Start passive rosbag snapshot logging',
    )
    log_root_arg = DeclareLaunchArgument(
        'fault_log_root',
        default_value=str(
            Path.home() / 'agv' / 'agv_ws' / 'data' / 'chassis_faults'
        ),
        description='Directory containing passive chassis fault bags',
    )
    cache_size_arg = DeclareLaunchArgument(
        'fault_cache_size',
        default_value='134217728',
        description='Snapshot-mode circular cache size in bytes',
    )

    node = Node(
        package='agv2_pkg',
        executable='agv2_control_node',
        name='agv2_control_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        respawn=True,
        respawn_delay=1.0,
    )

    return LaunchDescription([
        params_arg,
        logging_arg,
        log_root_arg,
        cache_size_arg,
        node,
        OpaqueFunction(function=launch_fault_logging),
    ])
