from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Server Node
        Node(
            package='station',
            executable='server_node',
            name='Server',
            output='screen'
        ),
        # Station Control Node
        Node(
            package='station',
            executable='station_control_node',
            name='StationControl',
            output='screen'
        ),
        # Gripper Node
        Node(
            package='station',
            executable='gripper_node',
            name='Gripper',
            output='screen'
        ),
        # Height Sensor Node
        Node(
            package='station',
            executable='height_sensor_node',
            name='HeightSensorStation',
            output='screen'
        ),
        # QR Code Detection Node
        Node(
            package='station',
            executable='qr_code_node',
            name='QRCodeDetection',
            output='screen'
        ),
        # Lead Screw Node
        Node(
            package='station',
            executable='lead_screw_node',
            name='LeadScrew',
            output='screen'
        ),
        # Limit Switch Node
        Node(
            package='station',
            executable='limitswitch_station_node',
            name='LimitSwitch',
            output='screen'
        ),
        # Linear Actuator Node
        Node(
            package='station',
            executable='linear_actuator_node',
            name='LinearActuator',
            output='screen'
        ),
    ])
