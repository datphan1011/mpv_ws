from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mpv',
            executable='mpv_control_node',
            name = 'MPV_Control',
            output='screen',
        ),
        Node(
            package='mpv',
            executable='client_node',
            name='Client',
        ),
        Node(
            package='mpv',
            executable='height_sensor_mpv_node',
            name='height_sensor_mpv'
        ),
        Node(
            package='mpv',
            executable='limit_switch_node',
            name='limit_switch',
        ),
        Node(
            package='mpv',
            executable='lock_stepper_node',
            name='lock_stepper',
        )
    ])
