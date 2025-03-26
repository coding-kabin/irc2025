#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy

class LD_control(Node):
    def __init__(self):
        super().__init__('ld_control')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'publish', 10)
        self.subscription = self.create_subscription(Joy, '/joy1', self.callback, 10)
        self.c = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #format [pumps,enable,analog_lt,analog_rt,analog_base]
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def callback(self, data):
        b = data.axes
        a = data.buttons
        #gonna have the two vertical lead screws on the two joysticks. the left and right motion of the base will be controlled by the analog triggers auger will be a combination input
        if a[13] == 1:
            self.c[0]=1.0           #one of the pumps
        elif a[14] == 1:
            self.c[0]=2.0           #another pump
        else:
            self.c[0]=0.0

        if b[1]>0.2:              #left trigger
            self.c[2]=(1.2*(b[1]-0.2)) 
        elif b[1]<-0.2:
            self.c[2]=(1.2*(b[1]+0.2))
        else:
            self.c[2]=0.0    
        
        if b[3]>0.2:              #right trigger 
            self.c[3]=(1.2*(b[3]-0.2))
        elif b[3]<-0.2:
            self.c[3]=(1.2*(b[3]+0.2))

        if b[4]>0.8 and b[5]>0.8:
            self.c[4]=0.0
        elif b[5]<0.8:              #base left and right using analog triggers
            self.c[4]=(0.8-b[5])/1.8 
        elif b[4]<0.8:
            self.c[4]=(b[4]-0.8)/1.8

        if a[0] == 1:
            self.c[1]=1.0          #enable the auger X means DRILL
        elif a[1] == 1:
            self.c[1]=2.0          #enable the magnetic spinner O means SPIN
                  #im so sorry. i am too stupid to redo this not from scratch. i will make it work like it used to after vpr the comment in the end is what should be here
        else:
            self.c[1]=0.0
    
    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = self.c
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    ld_control = LD_control()
    rclpy.spin(ld_control)
    ld_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

"""        if data.axes[6] == 1:
            self.c[0] = 1.0
        elif b[7] == 1:
            self.c[0] = 2.0
        elif b[6] == -1:
            self.c[0] = 3.0
        elif a[3] == 1:
            self.c[2] = 1.0
        elif a[2] == 1:
            self.c[2] = 2.0
        elif a[1] == 1:
            self.c[2] = 3.0
        elif a[4] == 1:
            self.c[4] = 1.0
        elif a[5] == 1:
            self.c[4] = 0.0
        elif b[7] == -1:
            self.c[5] = 1.0
        elif a[0] == 1:
            self.c[5] = -1.0

        if b[6] == 0 and b[7] == 0:
            self.c[0] = 0.0
        if a[1] == 0 and a[2] == 0 and a[3] == 0:
            self.c[2] = 0.0
        if b[7] == 0 and a[0] == 0:
            self.c[5] = 0.0

        self.c[6] = -1.0 * b[0]
        self.c[1] = b[4]
        self.c[3] = b[1]"""    