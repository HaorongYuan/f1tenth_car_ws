from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wall_follow_pid_cpp',
            executable='wall_follow_pid_node',
            name='wall_follow_pid_node',
            output='screen',
            parameters=[{
                'kp': 0.8,
                'ki': 0.0,
                'kd': 0.1,
                'integral_limit': 1.0,
                'desired_distance_from_wall': 0.5,
                'lookahead_distance': 3.0,
                'theta_degrees': 60.0,
                'steering_angle_limit': 0.4,
                'max_speed': 3.0,
                'min_speed': 0.5,
                'speed_reduction_gain': 1.0,
                'brake_speed': 1.0,
                'front_distance_threshold': 1.2,
                'filter_distance': 10.0,
            }]
        )
    ])