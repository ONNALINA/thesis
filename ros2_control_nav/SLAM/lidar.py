# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan

# class LidarSubscriber(Node):
#     def __init__(self):
#         super().__init__('lidar_subscriber')
#         self.subscription = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)
#         self.subscription

#     def listener_callback(self, msg):
#         ranges = msg.ranges
#         self.get_logger().info('Received %d range values.' % len(ranges))
#         for i, r in enumerate(ranges):
#             self.get_logger().info('Range[%d]: %f' % (i, r))

# def main(args=None):
#     rclpy.init(args=args)
#     lidar_subscriber = LidarSubscriber()
#     rclpy.spin(lidar_subscriber)
#     lidar_subscriber.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt

import math

class ScanSubscriber(Node):
    def __init__(self):
        super().__init__('scan_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10  # QoS profile depth
        )

    def scan_callback(self, msg):
    # Extract ranges data from the LaserScan message
        ranges = msg.ranges
        num_ranges = len(ranges)
        angle_increment = msg.angle_increment
        start_angle = msg.angle_min

        # Calculate the angle for each range measurement
        angle_degrees = []
        for i in range(num_ranges):
            angle = start_angle + i * angle_increment
            angle_degrees.append(math.degrees(angle))

        # Plot the ranges data using matplotlib
        fig, ax = plt.subplots()
        fig.set_size_inches(5,5)
        print(fig)
        ax.plot(angle_degrees, ranges)
        ax.set_title('LaserScan Ranges')
        ax.set_xlabel('Angle (degrees)')
        ax.set_ylabel('Range')
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = ScanSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()