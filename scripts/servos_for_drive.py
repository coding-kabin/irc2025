#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy


class ServoControl(Node):
    def __init__(self):
        super().__init__('servo_control_node')
        
        # Initialize angles
        self.angle = 90
        self.angle2 = 90
        self.angle3 = 90
        self.angle4 = 90
        self.angle5 = 90
        
        # Initialize previous states
        self.prev_states = {f'prev{i}': 0 for i in range(1, 11)}
        
        # Initialize control array
        self.c = [self.angle, self.angle2, self.angle3, self.angle4, self.angle5]
        
        
        self.pub1 = self.create_publisher(Int16, 'servoc1', 1)
        self.pub2 = self.create_publisher(Int16, 'servoc2', 1)
        self.pub3 = self.create_publisher(Int16, 'servoc3', 1)
        self.pub4 = self.create_publisher(Int16, 'servoc4', 1)
        self.pub5 = self.create_publisher(Int16, 'servoc5', 1)

        
        # Create subscription
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        # Create timer for publishing
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz rate

    def joy_callback(self, data: Joy):
        b = data.axes
        a = data.buttons
        
        # Process D-pad up/down (axis 7)
        if b[7] >= 0.5 and self.angle > 0 and self.prev_states['prev1'] == 0:
            self.angle -= 5
            self.prev_states['prev1'] = 1
        if b[7] <= -0.5 and self.angle < 180 and self.prev_states['prev2'] == 0:
            self.angle += 5
            self.prev_states['prev2'] = 1
            
        # Process D-pad right/left (axis 9)
        if b[9] == -1 and self.angle2 > 0 and self.prev_states['prev3'] == 0:
            self.angle2 -= 5
            self.prev_states['prev3'] = 1
        if b[9] == 1 and self.angle2 < 180 and self.prev_states['prev4'] == 0:
            self.angle2 += 5
            self.prev_states['prev4'] = 1
            
        # Process axis 8
        if b[8] == 1 and self.angle3 > 0 and self.prev_states['prev5'] == 0:
            self.angle3 -= 5
            self.prev_states['prev5'] = 1
        if b[8] == -1 and self.angle3 < 180 and self.prev_states['prev6'] == 0:
            self.angle3 += 5
            self.prev_states['prev6'] = 1
            
        # Process buttons
        if a[1] == 1 and self.angle4 > 0 and self.prev_states['prev7'] == 0:
            self.angle4 -= 5
            self.prev_states['prev7'] = 1
        if a[3] == 1 and self.angle4 < 180 and self.prev_states['prev8'] == 0:
            self.angle4 += 5
            self.prev_states['prev8'] = 1
        if a[9] == 1 and self.angle5 > 0 and self.prev_states['prev9'] == 0:
            self.angle5 -= 5
            self.prev_states['prev9'] = 1
        if a[8] == 1 and self.angle5 < 180 and self.prev_states['prev10'] == 0:
            self.angle5 += 5
            self.prev_states['prev10'] = 1
        
        # Reset previous states
        if b[7] < 0.5:
            self.prev_states['prev1'] = 0
        if b[7] > -0.50:
            self.prev_states['prev2'] = 0
        if b[9] == 0:
            self.prev_states['prev3'] = 0
            self.prev_states['prev4'] = 0
        if b[8] == 0:
            self.prev_states['prev5'] = 0
            self.prev_states['prev6'] = 0
        if a[1] == 0:
            self.prev_states['prev7'] = 0
        if a[3] == 0:
            self.prev_states['prev8'] = 0
        if a[9] == 0:
            self.prev_states['prev9'] = 0
        if a[8] == 0:
            self.prev_states['prev10'] = 0
            
        # Update control array
        self.c = [self.angle, self.angle2, self.angle3, self.angle4, self.angle5]

    def timer_callback(self):
        # Publish all servo positions
        msg1, msg2, msg3, msg4, msg5 = (Int16() for _ in range(5))
        msg1.data = self.c[0]
        msg2.data = self.c[1]
        msg3.data = self.c[2]
        msg4.data = self.c[3]
        msg5.data = self.c[4]
        
        self.pub1.publish(msg1)
        self.pub2.publish(msg2)
        self.pub3.publish(msg3)
        self.pub4.publish(msg4)
        self.pub5.publish(msg5)


def main(args=None):
    rclpy.init(args=args)
    servo_control = ServoControl()
    
    try:
        rclpy.spin(servo_control)
    except KeyboardInterrupt:
        pass
    finally:
        servo_control.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()