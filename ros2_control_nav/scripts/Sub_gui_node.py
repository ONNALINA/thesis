#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool,String,Int16

import time
import numpy
import ast

# from basic_movement import *
# from motor_start import *

class Sub_gui_node(Node):
    def __init__(self):
        super().__init__('sub_gui_node')
        self.sub_enable = self.create_subscription(Bool,'/order_enable',self.order_callback,10)
        self.pub_move_store = self.create_publisher(Bool,'/move_to_store',10)
        self.sub_order = self.create_subscription(Int16,'/order_id',self.order_id_callback,10)
        self.pub_move_step = self.create_publisher(String,'/move_step',10)
        self.sub_backward = self.create_subscription(Bool,'/backward',self.backward_callback,10)
        self.pub_move_back = self.create_publisher(String,'/move_backward',10)

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
        self.a  = int()
        self.robot_vel = 0
        self.base_vel = [0,0,0,0]

        self.pos = int()
        self.pos_now = 2
        self.pos_pre = 2

    def order_callback(self,msg:Bool):
        if msg.data == True:
            print('move_store')
            msg.data = True
            self.pub_move_store.publish(msg)
    
    def order_id_callback(self,msg:Int16):
        print(msg.data)

        self.pos_pre = self.pos_now
        print('pos_pre',self.pos_pre)
        if msg.data:
            pos0 = msg.data
            pos1 = pos0 % 3
            if pos1 == 1:
                self.pos = 1
            if pos1 == 2:
                self.pos = 2
            if pos1 == 0:
                self.pos = 3
        self.pos_now = self.pos
        print('pos_now',self.pos_now)
        self.move_step = self.pos_pre-self.pos_now
        print('move_step',self.move_step)

        pos_ = String()
        pos_.data = str(self.move_step)
        self.pub_move_step.publish(pos_)

    def backward_callback(self,msg:String):
        if msg.data == 'backward':
            msgBool = Bool()
            msgBool.data = True
            self.pub_move_back.publish(msgBool)

def main(args=None):
    rclpy.init(args=args)
    controller = Sub_gui_node()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()