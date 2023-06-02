#!/usr/bin/python3

        # ######### Check_list ######### #

        # sub order_enable = True -> // forward = 4
        # home to store //
        # pub station = Store //

        # sub move_step //
        # move store to any id //
        # pub at_order = True //

        # sub order_enable = False //
        # move store to production line // -> backward = 3, left rotate=1, forward = 3,left rotate = 2 forward = 1,leaft rotate = 1 ,forward = 3
        # pub time_for_pick = your time //

        # sub order_done //
        # move producetion line to home  // -> forward = 1,leaft rotate = 1 ,forward = 3
        # pub new_order = True 

        # ######### Check_list ######### #

import rclpy
from rclpy.node import Node

from std_msgs.msg import String,Bool,Int16
import numpy
import time
from basic_movement import *
from motor_start import *

class Move_Step_Node(Node):
    def __init__(self):
        super().__init__('move_step_node')
        self.step = int()
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
        self.base_vel = [0,0,0,0]

        self.sub_enble = self.create_subscription(Bool,'/order_enable',self.enable_callback,10)
        self.sub_order_id = self.create_subscription(String,'/move_step',self.order_id_callback,10)
        self.sub_station_home = self.create_subscription(String,'/station',self.order_done_callback,10)
        
        self.pub_station = self.create_publisher(String,'/station',10)
        self.pub_at_order_ = self.create_publisher(Bool,'/at_order',10)
        self.time_for_pick = self.create_publisher(String,'/time_for_pick',10)
        self.new_order = self.create_publisher(Bool,'/new_order',10)

    def enable_callback(self,msg:Bool):
        if msg.data == True :
            self.move_to_store()
        if msg.data == False :
            self.move_to_productionLine()

    def order_id_callback(self,msg:String):
        if msg.data == '0' :
            self.now()
        if int(msg.data) < 0:
            right_time = (abs(int(msg.data))+(abs(int(msg.data)))/10)+0.1
            self.right()
            time.sleep(right_time)
            self.stop_()
            time.sleep(1)
            self.torque_disable()
            self.now()

        if int(msg.data) > 0:
            left_time = (abs(int(msg.data))+(abs(int(msg.data)))/10)+0.1
            self.left()
            time.sleep(left_time)
            self.stop_()
            time.sleep(1)
            self.torque_disable()
            self.now()

    def order_done_callback(self,msg:String):
        if msg.data == 'Home':
            self.move_to_home()

    def move_to_home(self):
        print('move_to_home')
        time.sleep(1)
        self.right()
        time.sleep(2)
        self.stop_()
        time.sleep(1)
        self.torque_disable()

        time.sleep(1)
        self.forward_()
        time.sleep(10.5)
        self.stop_()
        time.sleep(1)
        self.torque_disable()

        # self.right_rotate()
        # time.sleep(7.5)
        # self.stop_()
        # time.sleep(2)
        # self.torque_disable()

        # time.sleep(1)
        # self.forward_()
        # time.sleep(1)
        # self.stop_()
        # time.sleep(1)
        # self.torque_disable()

        # time.sleep(1)
        # self.left()
        # time.sleep(10)
        # self.stop_()
        # time.sleep(1)
        # self.torque_disable()

        self.right_rotate()
        time.sleep(22.15)
        self.stop_()
        time.sleep(1)
        self.torque_disable()

        msg = Bool()
        msg.data = True
        self.new_order.publish(msg)

    def move_to_store(self):
        print('move_to_store')
        self.forward_()
        time.sleep(6)
        self.stop_()
        time.sleep(1)
        self.torque_disable()

        msgStr = String()
        msgStr.data = 'Store'
        self.pub_station.publish(msgStr)

    def move_to_productionLine(self):
        print('move_to_productionLine')
        self.backward()
        time.sleep(3)
        self.stop_()
        time.sleep(2)
        self.torque_disable()
        time.sleep(1)

        self.left()
        time.sleep(11)
        self.stop_()
        time.sleep(2)
        self.torque_disable()
        time.sleep(1)

        self.right_rotate()
        time.sleep(7.4)
        self.stop_()
        time.sleep(2)
        self.torque_disable()
        time.sleep(1)

        # self.left()
        # time.sleep(1)
        # self.stop_()
        # time.sleep(2)
        # self.torque_disable()
        # time.sleep(1)

        # self.forward_()
        # time.sleep(10)
        # self.stop_()
        # time.sleep(2)
        # self.torque_disable()
        # time.sleep(1)

        # self.right_rotate()
        # time.sleep(14.85)
        # self.stop_()
        # time.sleep(2)
        # self.torque_disable()
        # time.sleep(1)

        msg = String()
        msg.data = 'PL'
        self.pub_station.publish(msg)

    def left_rotate(self):
        self.Vx = 0#-17.9541
        self.Vy = 0
        self.Wz = -18
        self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
        self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
        basic_movement.rotate_2(self)
        self.move_wheel()

    def right_rotate(self): #All CCW
        self.Vx = 0#-17.9541
        self.Vy = 0
        self.Wz = 18
        self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
        self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
        basic_movement.rotate_1(self)
        self.move_wheel()   

    def forward_(self):
        self.Vx = -18#-17.9541
        self.Vy = 0
        self.Wz = 0
        self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
        self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
        basic_movement.forward(self)
        self.move_wheel()

    def backward(self):
        self.Vx = 18   #-17.95
        self.Vy = 0
        self.Wz = 0
        self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
        self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
        basic_movement.backward(self)
        self.move_wheel()

    def stop_(self):
        self.Vx = 0
        self.Vy = 0
        self.Wz = 0
        self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
        self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
        basic_movement.stop(self)
        self.move_wheel()

    def now(self):
        self.forward_()
        time.sleep(1)
        self.stop_()
        time.sleep(1)
        self.torque_disable()

        self.pub_at_order()
        time.sleep(10)

        self.forward_()
        time.sleep(1)
        self.stop_()
        time.sleep(1)
        self.torque_disable()

        time.sleep(15)

        self.backward()
        time.sleep(2)
        self.stop_()
        time.sleep(1)
        self.torque_disable()

    def right(self):
        self.Vx = 0
        self.Vy = 18 #17.95
        self.Wz = 0
        self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
        self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
        basic_movement.right(self)
        self.move_wheel()

    def left(self):
        self.Vx = 0
        self.Vy = -18 #17.95
        self.Wz = 0
        self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
        self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
        basic_movement.left(self)
        self.move_wheel()

    def pub_at_order(self):
        msgBool = Bool()
        msgBool.data = True
        self.pub_at_order_.publish(msgBool)

    def torque_disable(self):
        print('torque_disable')
        packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)

    def move_wheel(self):
        print('move_wheel',self.base_vel)
        packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_MOVING_SPEED, (self.base_vel[1]))  ,packetHandler.write2ByteTxRx(portHandler, DXL4_ID, ADDR_MX_MOVING_SPEED, (self.base_vel[3]))
        packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_ACCELERATION, 10)               ,packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_GOAL_ACCELERATION, 10)

        packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_MX_MOVING_SPEED, (self.base_vel[2]))  ,packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_MX_MOVING_SPEED, (self.base_vel[0]))
        packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_GOAL_ACCELERATION, 10)               ,packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_ACCELERATION, 10)
        self.base_vel = [(self.base_vel[0]),(self.base_vel[1]),(self.base_vel[2]),(self.base_vel[3])]

def main(args=None):
    rclpy.init(args=args)
    controller = Move_Step_Node()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()