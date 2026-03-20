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
                'kp': 0.5,
                'ki': 0.0,
                'kd': 0.25,
                'integral_limit': 0.5,
                'desired_distance_from_wall': 0.8,
                'lookahead_distance': 1.2,
                'theta_degrees': 60.0,
                'steering_angle_limit': 0.4,
                'max_speed': 1.2,
                'min_speed': 0.3,
                'speed_reduction_gain': 2.0,
                'brake_speed': 0.2,
                'front_distance_threshold': 0.8,
                'filter_distance': 5.0,
            }]
        )
    ])