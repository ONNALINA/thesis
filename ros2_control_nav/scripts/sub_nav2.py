#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from map_msgs.msg import ProjectedMap
from geometry_msgs.msg import PoseWithCovariance

class Sub_Nav2(Node):

    def __init__(self):
        super().__init__('sub_nav2_node')

        self.pub_PoseWCov = self.create_publisher(PoseWithCovariance,'/PosWCov',10)
        self.timer_ = self.create_timer(1, self.pub_PoseWithCov)

    def pub_PoseWithCov(self):
        msg = PoseWithCovariance()
        msg.pose.position.x = 1.0
        msg.pose.position.y = 1.0
        self.pub_PoseWCov.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = Sub_Nav2()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()