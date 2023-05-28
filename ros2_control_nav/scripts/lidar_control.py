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
from rplidar import RPLidar
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math

import time

class Lidar_Node(Node):
    def __init__(self):
        super().__init__('lidar_node')
        # timer_cb_group = None
        self.sub_scan = self.create_subscription(LaserScan,'/scan',self.callback,10)  
        
        # self.move_ = self.create_timer(0.1,self.move_wheel,callback_group=timer_cb_group)

        # Lx = 0.198 # m
        # Ly = 0.2043
        # self.eqm = numpy.array([[1, -1, -(Lx+Ly)], [1, 1, (Lx+Ly)], [1, 1, -(Lx+Ly)], [1, -1, (Lx+Ly)]])
        # self.r = 0.127/2
        # self.Vx = float()
        # self.Vy = float()
        # self.Wz = float()
        # self.A0 = float()
        # self.A1 = float()
        # self.A2 = float()
        # self.A3 = float()     
        # self.robot_vel = 0
        # self.base_vel = 0

    def callback(self,msg):
        # range = numpy.array(msg.ranges)
        # angles = numpy.arange(len(msg.ranges)) * msg.angle_increment + msg.angle_min
        # range = msg.ranges < numpy.inf
        # x = msg.ranges[range] * numpy.cos(angles[range])
        # y = msg.ranges[range] * numpy.sin(angles[range])
        # aw_centroid = [[x.mean(),y.mean(),0.,0]]
        # print(aw_centroid)

        i = 0
        count = int(math.floor(msg.scan_time/msg.time_increment))
        # a = int((len(msg.ranges))-1)
        # b = int(len(msg.ranges)/4)
        # c = int(len(msg.ranges)/2)
        # d = int(len(msg.ranges)/len(msg.ranges))
        # print('front: ',msg.ranges[a])
        # print('left: ',msg.ranges[b])
        # print('back: ',msg.ranges[c])
        # print('right: ',msg.ranges[d])
        # # print('right: ',msg.ranges[750])
        # # print('right: ',msg.ranges[750])
        # # print(len(msg.ranges)*1)
        # # print(len(msg.ranges)/2)
        # # print(len(msg.ranges)/len(msg.ranges))
        for i in range (0,count):
            if msg.ranges[i] == None:
                continue
            degree = (msg.angle_min + msg.angle_increment * i)*57.2958
            if(degree >= 90 and degree <= 180):
                degree = degree - 90
            elif(degree >= -180 and degree < 0):
                degree = (180-abs(degree)) + 90
            else:
                degree =  270 + degree
                
            lidar_y = (msg.ranges[i])*math.sin(degree/57.2958)
            lidar_x = (msg.ranges[i])*math.cos(degree/57.2958)
            print('lidar x: ',lidar_x)
            # print('lidar y: ',lidar_y)



def main(args=None):
    rclpy.init(args=args)
    controller = Lidar_Node()
    rclpy.spin(controller)
    # controller.destroy_node()
    # rclpy.shutdown()

if __name__=='__main__':
    main()


            # A = int(count/count)
            # B = int(count/2)
            # C = int(count/3)

            # if(degree >= 90 and degree <= 180):
            #     degree = degree - 90
            #     # print('1',degree, msg.ranges,[A])
            # elif(degree >= -180 and degree < 0):
            #     degree = (180-abs(degree)) + 90
            #     # print('2',degree, msg.ranges,[B])
            # else:
            #     degree =  270 + degree
            #     # print('3',degree, msg.ranges[C])



        # A = 500
        # B = 700
        # C = 800 or 900 or 1000
        # print("value at 0 degree: ",msg.ranges[0])
        # print("value at 90 degree: ",msg.ranges[200])
        # print("value at 180 degree: ",msg.ranges[A])
        # print("value at 270 degree: ",msg.ranges[B])
        # print("value at 360 degree: ",msg.ranges[C])



        # self.pub_scan = self.create_publisher(String,'/scan_to_move',10)

        # self.sub_scan
        # self.pub_scan
        # Bool = False

    # def scan_callback(self,msg:LaserScan):
    #     print('2')
    #     self.state = 0
    #     Bool = True
    #     if Bool == True:
    #         print('state 0')
    #         strtofloat = float(msg.ranges[1])
    #         print('pub',strtofloat)
    #         self.state = 1
    #         self.Bool_true_state()
        
    # def Bool_true_state(self):
    #     msg1 = String()
    #     if self.state == 1:
    #         print('state 1')
    #         msg1.msg = 'set'
    #         self.pub_scan.publish(msg1)
    #         print(msg1)