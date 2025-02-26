from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joystick1',
            remappings=[('/joy', '/joy0')],
            parameters=[{
                'dev': '/dev/input/by-id/usb-Thrustmaster_T.Flight_Hotas_One-joystick',
                'coalesce_interval': 0.05
            }]
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joystick2',
            remappings=[('/joy', '/joy1')],
            parameters=[{
                'dev': '/dev/input/by-id/usb-Sony_Interactive_Entertainment_Wireless_Controller-if03-joystick'
            }]
        ),

        Node(
            package='irc2025',
            executable='drive.py',
            name='drive_node'
        )
    ])
