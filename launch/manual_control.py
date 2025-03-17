from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Initialize joy_node for Thrustmaster T.Flight Hotas One
    joy0_node = Node(
        package='joy',
        executable='joy_node',
        name='joy0',
        remappings=[('/joy', '/joy0')],
        parameters=[
            {'device_name': 'Thrustmaster T.Flight Hotas One'}
        ]
    )

    # Initialize joy_node for PS5 Controller
    joy1_node = Node(
        package='joy',
        executable='joy_node',
        name='joy1',
        remappings=[('/joy', '/joy1')],
        parameters=[
            {'device_name': 'PS5 Controller'}
        ]
    )

    drive_node = Node(
        package='irc2025',
        executable='drive.py',
        name='drive_node'
    )

    # Add nodes to the launch description
    ld.add_action(joy0_node)
    ld.add_action(joy1_node)
    ld.add_action(drive_node)
    return ld
