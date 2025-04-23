#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
import math
import time


class AutoDrive(Node):
    def __init__(self):
        super().__init__('Auto_Drive')
        
        self.rover_x = 0
        self.rover_z = 0
        
        # Create subscription
        self.subscription = self.create_subscription(
            Point,
            '/rover1',
            self.callback,
            10
        )
        
        # Create publisher
        self.pub = self.create_publisher(
            Float32MultiArray,
            '/rover',
            10
        )
        
        # Initialize message
        self.values = Float32MultiArray()
        self.actualValues = [0] * 6
        self.values.data = self.actualValues
        
        # Create timer for periodic publishing (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def callback(self, msg):
        self.rover_x = msg.x
        self.rover_z = msg.z
    
    def timer_callback(self):
        self.actualValues = self.move()
        self.values.data = self.actualValues
        self.pub.publish(self.values)
    
    def move(self):
        velx = self.rover_x
        velz = self.rover_z

        if (velx > 100 or velz > 100):
            velx = 100
            velz = 100
        
        if(velx > 100):
            velx = 100
        if(velx < -100):
            velx = -100
        if(velz > 100):
            velz = 100
        if(velz < -100):
            velz = -100

        if (velx >= 0 and velz >= 0):
            left_wheel_front = ((velx+velz)/2+(velx-velz)/2) * 0.44
            right_wheel_front = ((velx+velz)/2-(velx-velz)/2) * 0.5
            right_wheel_mid = ((velx+velz)/2+(velx-velz)/2) * 0.501
            left_wheel_mid = ((velx+velz)/2-(velx-velz)/2) * 0.46
            right_wheel_back = ((velx+velz)/2+(velx-velz)/2) * 0.47
            left_wheel_back = ((velx+velz)/2-(velx-velz)/2) * 0.45
        
        elif (velx <= 0 and velz <= 0):
            right_wheel_front = ((velx+velz)/2+(velx-velz)/2) * 0.54
            left_wheel_front = ((velx+velz)/2-(velx-velz)/2) * 0.52
            right_wheel_mid = ((velx+velz)/2+(velx-velz)/2) * 0.57
            left_wheel_mid = ((velx+velz)/2-(velx-velz)/2) * 0.54
            right_wheel_back = ((velx+velz)/2+(velx-velz)/2) * 0.535
            left_wheel_back = ((velx+velz)/2-(velx-velz)/2) * 0.56
        
        elif (velx >= 0 and velz <= 0):
            right_wheel_front = ((velx+velz)/2+(velx-velz)/2) * 0.5
            left_wheel_front = ((velx+velz)/2-(velx-velz)/2) * 0.52
            right_wheel_mid = ((velx+velz)/2+(velx-velz)/2) * 0.501
            left_wheel_mid = ((velx+velz)/2-(velx-velz)/2) * 0.54
            right_wheel_back = ((velx+velz)/2+(velx-velz)/2) * 0.47
            left_wheel_back = ((velx+velz)/2-(velx-velz)/2) * 0.56
         
        elif (velx <= 0 and velz >= 0):
            right_wheel_front = ((velx+velz)/2+(velx-velz)/2) * 0.54
            left_wheel_front = ((velx+velz)/2-(velx-velz)/2) * 0.44
            right_wheel_mid = ((velx+velz)/2+(velx-velz)/2) * 0.57
            left_wheel_mid = ((velx+velz)/2-(velx-velz)/2) * 0.46
            right_wheel_back = ((velx+velz)/2+(velx-velz)/2) * 0.535
            left_wheel_back = ((velx+velz)/2-(velx-velz)/2) * 0.45
         
        else:  # velx==0 and velz==0
            right_wheel_front = 0
            left_wheel_front = 0
            right_wheel_mid = 0
            left_wheel_mid = 0
            right_wheel_back = 0
            left_wheel_back = 0

        return (right_wheel_front, left_wheel_front, right_wheel_mid, 
                left_wheel_mid, right_wheel_back, left_wheel_back)


def main(args=None):
    rclpy.init(args=args)
    auto_drive_node = AutoDrive()
    
    try:
        rclpy.spin(auto_drive_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        auto_drive_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
