#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import time
import numpy
from std_msgs.msg import Bool,String,Int16

from basic_movement import *
from motor_start import *

class move_base_node(Node):
    def __init__(self):
        super().__init__('move_base_node')
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

        self.sub_move_store = self.create_subscription(Bool,'/move_to_store',self.order_callback,10)
        self.pub_list = self.create_publisher(String,'/station',10)
        self.sub_step = self.create_subscription(String,'/move_step',self.move_step,10)
        self.pub_order = self.create_publisher(Bool,'/at_order',10)
        self.sub_move_back = self.create_subscription(String,'/move_backward',self.backward_callback,10)
        self.pub_wait = self.create_publisher(String,'/wait_order',10)
        self.sub_enable = self.create_subscription(Bool,'/order_enable',self.enable_callback,10)

    def enable_callback(self,msg:Bool):
        if msg.data == False:
            msgstation = String()
            msgstation.data = 'walking to home'
            self.pub_list.publish(msgstation)

    def order_callback(self,msg:Bool):
        print('sub')
        if msg.data == True :
            print('forward_start')
            self.Vx = -18#-17.9541
            self.Vy = 0
            self.Wz = 0
            self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
            self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
            basic_movement.forward(self)
            self.move_wheel()
            time.sleep(1)

            # print('stop')
            self.Vx = 0
            self.Vy = 0
            self.Wz = 0
            self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
            self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
            basic_movement.stop(self)
            self.move_wheel()
            time.sleep(2)
            packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)

            msg.data = False
            msgstr = String()
            msgstr.data = 'Store'
            self.pub_list.publish(msgstr)

    def move_step(self,msg:String):
        print(msg.data)
        if msg.data == '0': # now
            print('center')
            self.Vx = -18#-17.95
            self.Vy = 0
            self.Wz = 0
            self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
            self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
            basic_movement.forward(self)
            self.move_wheel()
            time.sleep(1)

            # print('stop')
            self.Vx = 0
            self.Vy = 0
            self.Wz = 0
            self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
            self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
            basic_movement.stop(self)
            self.move_wheel()
            time.sleep(2)
            packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            # -------------------------
            msgBool = Bool()
            msgBool.data = True
            self.pub_order.publish(msgBool)

            time.sleep(10)

            self.Vx = -18#-17.95
            self.Vy = 0
            self.Wz = 0
            self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
            self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
            basic_movement.forward(self)
            self.move_wheel()
            time.sleep(1)

            self.Vx = 0
            self.Vy = 0
            self.Wz = 0
            self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
            self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
            basic_movement.stop(self)
            self.move_wheel()
            time.sleep(2)
            packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)

            # -------------------------
            time.sleep(15) # ***
            self.Vx = 18   #-17.95
            self.Vy = 0
            self.Wz = 0
            self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
            self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
            basic_movement.backward(self)
            self.move_wheel()
            time.sleep(2)

            self.Vx = 0
            self.Vy = 0
            self.Wz = 0
            self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
            self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
            basic_movement.stop(self)
            self.move_wheel()
            time.sleep(2)
            packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)

        if int(msg.data) < 0 : # right
            print('right_move_time',1*abs(int(msg.data)))
            right_time = (abs(int(msg.data))+(abs(int(msg.data)))/10)+0.1
            print(right_time)
            self.Vx = 0
            self.Vy = 18 #17.95
            self.Wz = 0
            self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
            self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
            basic_movement.right(self)
            self.move_wheel()
            time.sleep(right_time)

            # print('stop')
            self.Vx = 0
            self.Vy = 0
            self.Wz = 0
            self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
            self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
            basic_movement.stop(self)
            self.move_wheel()
            time.sleep(1)
            packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            
            time.sleep(1)
            self.Vx = -18#-17.95
            self.Vy = 0
            self.Wz = 0
            self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
            self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
            basic_movement.forward(self)
            self.move_wheel()
            time.sleep(1)

            # print('stop')
            self.Vx = 0
            self.Vy = 0
            self.Wz = 0
            self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
            self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
            basic_movement.stop(self)
            self.move_wheel()
            time.sleep(2)
            packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            # -------------------------
            msgBool = Bool()
            msgBool.data = True
            self.pub_order.publish(msgBool)
            time.sleep(10)

            self.Vx = -18 #-17.95
            self.Vy = 0
            self.Wz = 0
            self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
            self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
            basic_movement.forward(self)
            self.move_wheel()
            time.sleep(1)

            self.Vx = 0
            self.Vy = 0
            self.Wz = 0
            self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
            self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
            basic_movement.stop(self)
            self.move_wheel()
            time.sleep(2)
            packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)

            # -------------------------
            time.sleep(15)
            self.Vx = 18 #-17.95
            self.Vy = 0
            self.Wz = 0
            self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
            self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
            basic_movement.backward(self)
            self.move_wheel()
            time.sleep(2)

            self.Vx = 0
            self.Vy = 0
            self.Wz = 0
            self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
            self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
            basic_movement.stop(self)
            self.move_wheel()
            time.sleep(1)
            packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)

        if int(msg.data) > 0 : #left
            print('left_move_time',1*abs(int(msg.data)))
            left_time = (abs(int(msg.data))+(abs(int(msg.data)))/10)+0.1
            print(left_time)
            self.Vx = 0
            self.Vy = -18 #17.95
            self.Wz = 0
            self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
            self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
            basic_movement.left(self)
            self.move_wheel()
            time.sleep(left_time)

            # print('stop')
            self.Vx = 0
            self.Vy = 0
            self.Wz = 0
            self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
            self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
            basic_movement.stop(self)
            self.move_wheel()
            time.sleep(1)
            packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)

            time.sleep(1)
            self.Vx = -18#-17.95
            self.Vy = 0
            self.Wz = 0
            self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
            self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
            basic_movement.forward(self)
            self.move_wheel()
            time.sleep(1)

            # print('stop')
            self.Vx = 0
            self.Vy = 0
            self.Wz = 0
            self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
            self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
            basic_movement.stop(self)
            self.move_wheel()
            time.sleep(2)
            packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            # -------------------------
            msgBool = Bool()
            msgBool.data = True
            self.pub_order.publish(msgBool)
            time.sleep(10)

            self.Vx = -18#-17.95
            self.Vy = 0
            self.Wz = 0
            self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
            self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
            basic_movement.forward(self)
            self.move_wheel()
            time.sleep(1)

            self.Vx = 0
            self.Vy = 0
            self.Wz = 0
            self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
            self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
            basic_movement.stop(self)
            self.move_wheel()
            time.sleep(2)
            packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)

            # -------------------------
            time.sleep(15)
            self.Vx = 18#-17.95
            self.Vy = 0
            self.Wz = 0
            self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
            self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
            basic_movement.backward(self)
            self.move_wheel()
            time.sleep(2)

            self.Vx = 0
            self.Vy = 0
            self.Wz = 0
            self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
            self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
            basic_movement.stop(self)
            self.move_wheel()
            time.sleep(2)
            packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)

    def move_wheel(self):
        print('move')
        print(self.base_vel)
        packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_MOVING_SPEED, (self.base_vel[1]))  ,packetHandler.write2ByteTxRx(portHandler, DXL4_ID, ADDR_MX_MOVING_SPEED, (self.base_vel[3]))
        packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_ACCELERATION, 10)               ,packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_GOAL_ACCELERATION, 10)

        packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_MX_MOVING_SPEED, (self.base_vel[2]))  ,packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_MX_MOVING_SPEED, (self.base_vel[0]))
        packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_GOAL_ACCELERATION, 10)               ,packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_ACCELERATION, 10)
        self.base_vel = [(self.base_vel[0]),(self.base_vel[1]),(self.base_vel[2]),(self.base_vel[3])]

    def backward_callback(self,msg:String):
        msg.data = 'pick now'
        self.pub_wait.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = move_base_node()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()