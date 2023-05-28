#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool,Int16,String

class Manager_node_(Node):

    def __init__(self):
        super().__init__('manager_node')
        
        self.pub_enable = self.create_publisher(Bool,'/order_enable',10)
        self.timer_ = self.create_timer(2.0,self.enable)
        
        # self.sub_station = self.create_subscription(String,'/station',self.station_callback,10)
        # self.pub_id = self.create_publisher(Int16,'/order_id',10)
        # self.timer_ = self.create_timer(2.0,self.station_callback)

        # self.order_done = self.create_publisher(String,'/order_done',10)
        # self.timer_ = self.create_timer(2.0,self.order_done_pub)

    def enable(self):
        msg= Bool()
        msg.data = True
        print('enable:',msg.data)
        self.pub_enable.publish(msg)

    # def station_callback(self):
    #     msg = Int16()
    #     msg.data = 2
    #     print('station:',msg.data)
    #     self.pub_id.publish(msg)

    # def order_done_pub(self):
    #     msg = String()
    #     msg.data = 'order done!'
    #     print('order:',msg.data)
    #     self.order_done.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = Manager_node_()
    rclpy.spin_once(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


            # self.sub_order = self.create_subscription(String,'/at_order',self.at_order,10)
        # self.timer_ = self.create_timer(2.0,self.at_order)
        # self.pub_backward = self.create_publisher(Bool,'/backward',10)
        
        # self.sub_station = self.create_subscription(String,'/station',10)
        # self.sub_wait = self.create_subscription(String,'/wait_order',self.wait_callback,10)
        # self.pub_completed = self.create_publisher(String, '/order_completed',10)