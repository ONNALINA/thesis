#!/usr/bin/python3
import os
from motor_start import *

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_MX_MOVING_SPEED, LEN_MX_MOVING_SPEED )

DXL_MINIMUM_POSITION_VALUE  = 100           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 4000 
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE] 

DXL_MINIMUM_SPEED_VALUE = 0       # Goal position
DXL_MAXIMUM_SPEED_VALUE = 1023
dxl_goal_speed = [DXL_MINIMUM_SPEED_VALUE ,DXL_MAXIMUM_SPEED_VALUE]

if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

for id in range(1,5,1):
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d torque has been enable" % id)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id, ADDR_MX_CW_ANGLE_LIMIT, DXL_CW_ANGLE_TO_Z)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id, ADDR_MX_CCW_ANGLE_LIMIT, DXL_CCW_ANGLETO_Z)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d has been successfully set to wheel mode" % id)

import rclpy
from rclpy.node import Node

import numpy
import math

from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan

# Encoder
# imu
# lidar
# amcl

class Mux_Node(Node):
    def __init__(self):
        super().__init__('mux_node')

        timer_cb_group = None
        self.sub_joy = self.create_subscription(Joy,'/joy',self.joy_callback,10)
        self.sub_scan1 = self.create_subscription(LaserScan,'/scan',self.scan1_callback,10) 
        # self.sub_scan2 = self.create_subscription(LaserScan,'/scan',self.scan2_callback,10) 
        self.move_ = self.create_timer(0.1,self.move_wheel,callback_group=timer_cb_group)
        Lx = 0.198 # m
        Ly = 0.2043
        self.eqm = numpy.array([[1, -1, -(Lx+Ly)], [1, 1, (Lx+Ly)], [1, 1, -(Lx+Ly)], [1, -1, (Lx+Ly)]])
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

    def joy_callback(self,msg):
        self.base_vel = [0,0,0,0]

        if msg.axes[5] == -1.0 :
            self.Vx = self.Vx2
            self.Vy = 0
            
            if msg.axes[4] == 0.0 :
                print('back')
                self.Wz = 0
            elif msg.axes[4] == -1.0:
                print('back-L')

                self.Wz = 20
            elif msg.axes[4] == 1.0:
                print('back-R')
                self.Wz = -20

            self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
            self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
            self.A0 = abs(int(self.base_vel[0]))
            self.A1 = abs(int(self.base_vel[1])+1024)
            self.A2 = abs(int(self.base_vel[2]))
            self.A3 = abs(int(self.base_vel[3])+1024)
            self.base_vel = (self.A0,self.A1,self.A2,self.A3)

        elif msg.axes[5] == 1.0 :
            self.Vx = self.Vx1
            self.Vy = 0

            if msg.axes[4] == 0.0 :
                print('front')
                self.Wz = 0
            elif msg.axes[4] == 1.0:
                print('front-R')
                self.Wz = 20
            elif msg.axes[4] == -1.0:
                print('front-L')
                self.Wz = -20
                
            self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
            self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
            self.A0 = abs(int(self.base_vel[0])-1024)
            self.A1 = abs(int(self.base_vel[1]))
            self.A2 = abs(int(self.base_vel[2])-1024)
            self.A3 = abs(int(self.base_vel[3]))
            self.base_vel = (self.A0,self.A1,self.A2,self.A3)

        elif msg.axes[5] == 0.0 and msg.axes[4] == 0.0 and msg.axes[3] == 0.0 :
            print('Stop')
            self.Vy = 0
            self.Wz = 0
            self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
            self.A0 = 0
            self.A1 = 0
            self.A2 = 0
            self.A3 = 0
            self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
            self.robot_vel = self.Vx,self.Vy,self.Wz
            self.base_vel = [self.A0,self.A1,self.A2,self.A3]
            packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_MOVING_SPEED, (self.base_vel[1]))  ,packetHandler.write2ByteTxRx(portHandler, DXL4_ID, ADDR_MX_MOVING_SPEED, (self.base_vel[3]))
            packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_ACCELERATION, 10)               ,packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_GOAL_ACCELERATION, 10)
            packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_MX_MOVING_SPEED, (self.base_vel[2]))  ,packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_MX_MOVING_SPEED, (self.base_vel[0]))
            packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_GOAL_ACCELERATION, 10)               ,packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_ACCELERATION, 10)


        elif msg.axes[2] > 0:
            print('left')
            self.Vx = 0
            self.Vy = -20
            self.Wz = 0
            self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
            self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
            self.A0 = abs(int(self.base_vel[0])-1024)
            self.A1 = abs(int(self.base_vel[1]))
            self.A2 = abs(int(self.base_vel[2])-1024)
            self.A3 = abs(int(self.base_vel[3])+1024)
            self.base_vel = (self.A0,self.A1,self.A2,self.A3)

        elif msg.axes[2] < 0:
            print('right')
            self.Vx = 0
            self.Vy = 20
            self.Wz = 0
            self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
            self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
            self.A0 = abs(int(self.base_vel[0])-1024)
            self.A1 = abs(int(self.base_vel[1])+1024)
            self.A2 = abs(int(self.base_vel[2]))
            self.A3 = abs(int(self.base_vel[3]))
            self.base_vel = (self.A0,self.A1,self.A2,self.A3)
    
        # return self.robot_vel, self.base_vel

    def scan1_callback(self,msg:LaserScan):
        i = 0
        count = int(math.floor(msg.scan_time/msg.time_increment))
        for i in range(0,count):
            degree =(msg.angle_min - msg.angle_increment * i) *(60)

            if (abs(degree) >= 150 and abs(degree) <= 230):
                # print(degree)
                if msg.ranges[5] <= 1 :
                    self.Vx1 = 0
                    self.Vy = 0
                    self.Wz = 0
                
                elif msg.ranges[5] > 1 and msg.ranges[5] <= 1.7:
                    self.Vx1 = -15

                elif msg.ranges[5] > 1.7:
                    self.Vx1 = -30

                self.Vx = self.Vx1
            
            elif (abs(degree) >= 330 and abs(degree) <= 390):
                # print(degree)
                if msg.ranges[500] <= 1 :
                    self.Vx2 = 0
                    self.Vy = 0
                    self.Wz = 0
                
                elif msg.ranges[500] > 1 and msg.ranges[500] <= 1.7:
                    self.Vx2 = 15

                elif msg.ranges[500] > 1.7:
                    self.Vx2 = 30

                self.Vx = self.Vx2

    def move_wheel(self):
        print(self.Vx)
        packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_MOVING_SPEED, (self.base_vel[1]))  ,packetHandler.write2ByteTxRx(portHandler, DXL4_ID, ADDR_MX_MOVING_SPEED, (self.base_vel[3]))
        packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_ACCELERATION, 10)               ,packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_GOAL_ACCELERATION, 10)

        packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_MX_MOVING_SPEED, (self.base_vel[2]))  ,packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_MX_MOVING_SPEED, (self.base_vel[0]))
        packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_GOAL_ACCELERATION, 10)               ,packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_ACCELERATION, 10)
        
        self.base_vel = [(self.base_vel[0]),(self.base_vel[1]),(self.base_vel[2]),(self.base_vel[3])]

def main(args=None):
    rclpy.init(args=args)
    controller = Mux_Node()
    rclpy.spin(controller)
    # controller.destroy_node()
    # rclpy.shutdown()

if __name__=='__main__':
    main()