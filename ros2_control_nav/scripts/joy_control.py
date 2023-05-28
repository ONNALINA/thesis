#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import numpy
import time

from sensor_msgs.msg import Joy

from basic_movement import *
from motor_start import *

class Joy_Node(Node):
    def __init__(self):
        super().__init__('base_joy_node')

        timer_cb_group = None
        self.sub_joy = self.create_subscription(Joy,'/joy',self.callback,10)
        self.move_ = self.create_timer(0.1,self.move_wheel,callback_group=timer_cb_group)
    
        Lx = 0.198 # m
        Ly = 0.2043
        self.eqm = numpy.array([[1, -1, -(Lx+Ly)],[1,  1, (Lx+Ly)], [1,  1, -(Lx+Ly)], [1, -1, (Lx+Ly)]])
        self.r = 0.127/2
        self.Vx = float()
        self.Vy = float()
        self.Wz = float()
        self.A0 = float()
        self.A1 = float()
        self.A2 = float()
        self.A3 = float()     
        self.robot_vel = 0
        self.base_vel = 0
        self.a = 0
        self.movevel = float()

    def callback(self,msg):
        
        self.LR = abs(msg.axes[0])
        self.FB = abs(msg.axes[1])

        self.base_vel = [0,0,0,0]
        if msg.buttons[5] != 1:
            if msg.axes[1]  > 0.1 : #front
                self.Vx = -65
                self.Vy = 0
                self.Wz = 0
                self.movevel = self.FB
                self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
                self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
                basic_movement.forward(self)

            if msg.axes[1] < -0.1 : #back
                self.Vx = 65
                self.Vy = 0
                self.Wz = 0

                self.movevel = self.FB
                self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
                self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
                basic_movement.backward(self)

            elif msg.axes[0] > 0.1 : #left
                self.Vx = 0
                self.Vy = -65
                self.Wz = 0
                self.movevel = self.LR
                self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
                self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
                basic_movement.left(self)

            elif msg.axes[0] < -0.1 : #right
                self.Vx = 0
                self.Vy = 65
                self.Wz = 0
                self.movevel = self.LR
                self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
                self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
                basic_movement.right(self)
                
            if msg.axes[0] == 0.0 and msg.axes[1] == 0.0:
                self.Vx = 0
                self.Vy = 0
                self.Wz = 0
                self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
                self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
                basic_movement.stop(self)

    def move_wheel(self):
        a = round((self.base_vel[1])*self.movevel)
        b = round((self.base_vel[2])*self.movevel)
        c = round((self.base_vel[3])*self.movevel)
        d = round((self.base_vel[0])*self.movevel)
        self.base_vel = [self.base_vel[0],self.base_vel[1],self.base_vel[2],self.base_vel[3]]

        packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_MOVING_SPEED, a)   ,packetHandler.write2ByteTxRx(portHandler, DXL4_ID, ADDR_MX_MOVING_SPEED, c)
        packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_ACCELERATION, 5) ,packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_GOAL_ACCELERATION, 5)
        packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_MX_MOVING_SPEED, b)   ,packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_MX_MOVING_SPEED, d)
        packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_GOAL_ACCELERATION, 5) ,packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_ACCELERATION, 5)
        
        self.base_vel = [self.base_vel[0],self.base_vel[1],self.base_vel[2],self.base_vel[3]]
        print(self.base_vel)

def main(args=None):
    rclpy.init(args=args)
    controller = Joy_Node()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()