from launch import LaunchDescription
from launch_ros.actions import Node 

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'station',
            executable = 'server_node',
            name = 'Server',
        ),
        Node(
            package = 'station',
            executable = 'station_control_node',
            name = 'StationControl',
            output = 'screen',
        ),
        Node(
            package = 'station',
            executable = 'gripper_node', 
            name = 'Gripper',
        ),
        Node(
            package = 'station',
            executable = 'height_sensor_node',
            name = 'HeightSensorStation',
            output = 'screen',
        ),
        # Node(
        #     package = 'station',
        #     name = 'LeadScrew',
        #     executable = 'lead_screw_node',
        # ),
        Node(
            package = 'station',
            executable = 'limitswitch_station_node',
            name = 'LimitSwitch',
        ),
        Node(
            package = 'station',
            executable = 'linear_actuator_node',
            name = 'LinearActuator',
        ),
    ])