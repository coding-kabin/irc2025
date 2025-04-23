#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point

class AutoDrive(Node): 
    def __init__(self):
        super().__init__('auto_drive')
        self.rover_x = 0.0
        self.rover_z = 0.0

        # Create subscriber for '/rover1' topic
        # QoS depth is 10 (similar to queue_size in ROS1)
        self.subscription = self.create_subscription(
            Point,
            '/rover1',
            self.callback,
            10)

        # Create publisher for '/rover' topic
        self.pub = self.create_publisher(Float32MultiArray, '/rover', 10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('Auto_Drive node has been started.')

    def callback(self, msg):
        """Callback function for the '/rover1' topic subscriber."""
        self.get_logger().info(f'Received Point: x={msg.x}, z={msg.z}')
        self.rover_x = msg.x
        self.rover_z = msg.z

    def timer_callback(self):
        """
        Timer callback executed at 10 Hz.
        Calculates wheel speeds and publishes them.
        """
        wheel_speeds_list = self.move()

        values_msg = Float32MultiArray()
        values_msg.data = wheel_speeds_list

        self.pub.publish(values_msg)
        # self.get_logger().info(f'Publishing wheel speeds: {wheel_speeds_list}') # Optional: Log published data

    def move(self):
        """
        Calculates individual wheel speeds based on rover_x and rover_z.
        The core logic remains the same as the ROS 1 version.
        Returns a list of 6 float values representing wheel speeds.
        """
        velx = self.rover_x
        velz = self.rover_z

        # Clamp velocities between -100 and 100
        velx = max(-100.0, min(100.0, velx))
        velz = max(-100.0, min(100.0, velz))

        # Initialize wheel speeds
        right_wheel_front = 0.0
        left_wheel_front = 0.0
        right_wheel_mid = 0.0
        left_wheel_mid = 0.0
        right_wheel_back = 0.0
        left_wheel_back = 0.0

        # Calculate wheel speeds based on velocity quadrant
        # (Original logic copied, ensure calculations are correct for your robot)
        if velx == 0 and velz == 0:
            pass # Speeds remain 0
        elif velx >= 0 and velz >= 0:
            left_wheel_front = ((velx + velz) / 2 + (velx - velz) / 2) * 0.44
            right_wheel_front = ((velx + velz) / 2 - (velx - velz) / 2) * 0.5
            right_wheel_mid = ((velx + velz) / 2 + (velx - velz) / 2) * 0.501
            left_wheel_mid = ((velx + velz) / 2 - (velx - velz) / 2) * 0.46
            right_wheel_back = ((velx + velz) / 2 + (velx - velz) / 2) * 0.47
            left_wheel_back = ((velx + velz) / 2 - (velx - velz) / 2) * 0.45
        elif velx <= 0 and velz <= 0:
            right_wheel_front = ((velx + velz) / 2 + (velx - velz) / 2) * 0.54
            left_wheel_front = ((velx + velz) / 2 - (velx - velz) / 2) * 0.52
            right_wheel_mid = ((velx + velz) / 2 + (velx - velz) / 2) * 0.57
            left_wheel_mid = ((velx + velz) / 2 - (velx - velz) / 2) * 0.54
            right_wheel_back = ((velx + velz) / 2 + (velx - velz) / 2) * 0.535
            left_wheel_back = ((velx + velz) / 2 - (velx - velz) / 2) * 0.56
        elif velx >= 0 and velz <= 0:
            right_wheel_front = ((velx + velz) / 2 + (velx - velz) / 2) * 0.5
            left_wheel_front = ((velx + velz) / 2 - (velx - velz) / 2) * 0.52
            right_wheel_mid = ((velx + velz) / 2 + (velx - velz) / 2) * 0.501
            left_wheel_mid = ((velx + velz) / 2 - (velx - velz) / 2) * 0.54
            right_wheel_back = ((velx + velz) / 2 + (velx - velz) / 2) * 0.47
            left_wheel_back = ((velx + velz) / 2 - (velx - velz) / 2) * 0.56
        elif velx <= 0 and velz >= 0:
            right_wheel_front = ((velx + velz) / 2 + (velx - velz) / 2) * 0.54
            left_wheel_front = ((velx + velz) / 2 - (velx - velz) / 2) * 0.44
            right_wheel_mid = ((velx + velz) / 2 + (velx - velz) / 2) * 0.57
            left_wheel_mid = ((velx + velz) / 2 - (velx - velz) / 2) * 0.46
            right_wheel_back = ((velx + velz) / 2 + (velx - velz) / 2) * 0.535
            left_wheel_back = ((velx + velz) / 2 - (velx - velz) / 2) * 0.45

        # Return speeds as a list of floats
        wheel_speeds = [
            float(right_wheel_front),
            float(left_wheel_front),
            float(right_wheel_mid),
            float(left_wheel_mid),
            float(right_wheel_back),
            float(left_wheel_back),
        ]
        return wheel_speeds


def main(args=None):
    rclpy.init(args=args)  # Initialize rclpy library

    auto_drive_node = AutoDrive()  # Create the node instance

    try:
        rclpy.spin(auto_drive_node)  # Keep the node running and processing callbacks
    except KeyboardInterrupt:
        pass 
    finally:
        auto_drive_node.destroy_node()
        rclpy.shutdown()  # Shutdown rclpy


if __name__ == '__main__':
    main()
