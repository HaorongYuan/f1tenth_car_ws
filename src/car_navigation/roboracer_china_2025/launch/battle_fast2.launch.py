from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('drive_topic', default_value='/drive'),
        DeclareLaunchArgument('marker_topic', default_value='/battle_fast2/arrow_marker'),
        DeclareLaunchArgument('debug_scan_topic', default_value='/battle_fast2/front_scan'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('mpc_enable', default_value='false'),
        DeclareLaunchArgument('mpc_mode', default_value='off'),
        DeclareLaunchArgument('mpc_timeout_ms', default_value='8.0'),
        DeclareLaunchArgument('wheelbase', default_value='0.25'),
        DeclareLaunchArgument('horizon', default_value='8'),
        DeclareLaunchArgument('dt', default_value='0.1'),
        DeclareLaunchArgument('mpc_steer_candidates', default_value='11'),
        DeclareLaunchArgument('mpc_accel_candidates', default_value='7'),
        DeclareLaunchArgument('mpc_max_speed', default_value='2.0'),
        DeclareLaunchArgument('mpc_min_speed', default_value='0.0'),
        DeclareLaunchArgument('mpc_max_accel', default_value='2.0'),
        DeclareLaunchArgument('mpc_max_decel', default_value='-2.0'),
        DeclareLaunchArgument('follow_speed_cap', default_value='1.0'),
        DeclareLaunchArgument('chaoche_speed_cap', default_value='2.0'),
        DeclareLaunchArgument('final_speed_cap', default_value='4.0'),

        Node(
            package='roboracer_china_2025',
            executable='battle_fast2_node',
            name='battle_fast2_node',
            output='screen',
            parameters=[{
                'scan_topic': LaunchConfiguration('scan_topic'),
                'drive_topic': LaunchConfiguration('drive_topic'),
                'marker_topic': LaunchConfiguration('marker_topic'),
                'debug_scan_topic': LaunchConfiguration('debug_scan_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'mpc_enable': ParameterValue(LaunchConfiguration('mpc_enable'), value_type=bool),
                'mpc_mode': LaunchConfiguration('mpc_mode'),
                'mpc_timeout_ms': ParameterValue(LaunchConfiguration('mpc_timeout_ms'), value_type=float),
                'wheelbase': ParameterValue(LaunchConfiguration('wheelbase'), value_type=float),
                'horizon': ParameterValue(LaunchConfiguration('horizon'), value_type=int),
                'dt': ParameterValue(LaunchConfiguration('dt'), value_type=float),
                'mpc_steer_candidates': ParameterValue(LaunchConfiguration('mpc_steer_candidates'), value_type=int),
                'mpc_accel_candidates': ParameterValue(LaunchConfiguration('mpc_accel_candidates'), value_type=int),
                'mpc_max_speed': ParameterValue(LaunchConfiguration('mpc_max_speed'), value_type=float),
                'mpc_min_speed': ParameterValue(LaunchConfiguration('mpc_min_speed'), value_type=float),
                'mpc_max_accel': ParameterValue(LaunchConfiguration('mpc_max_accel'), value_type=float),
                'mpc_max_decel': ParameterValue(LaunchConfiguration('mpc_max_decel'), value_type=float),
                'follow_speed_cap': ParameterValue(LaunchConfiguration('follow_speed_cap'), value_type=float),
                'chaoche_speed_cap': ParameterValue(LaunchConfiguration('chaoche_speed_cap'), value_type=float),
                'final_speed_cap': ParameterValue(LaunchConfiguration('final_speed_cap'), value_type=float),
            }],
        ),
    ])
