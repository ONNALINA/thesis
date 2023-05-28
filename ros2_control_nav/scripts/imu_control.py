#!/usr/bin/python3
import serial
import time
import rclpy
import math
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class Arduino_Node(Node):
    def __init__(self):
        super().__init__('arduino_node')

        # MadgwickFilter
        self.beta=1.0
        self.sample_rate=100.0
        self.sample_rate = self.sample_rate
        self.beta = self.beta
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

        # from IDE
        self.arduinoData = serial.Serial('/dev/ttyUSB0', 115200)  # send serial from IDE to Python
        time.sleep(1)

        timer_cb_group = None
        self.gyro_ = self.create_timer(0.1, self.gyro_mpu, callback_group=timer_cb_group)
        print('1')

        self.pub_imu_ = self.create_publisher(Imu, '/Imu_', 100)
        self.transform_broadcaster = TransformBroadcaster(self)

    def gyro_mpu(self):
        if self.arduinoData.in_waiting > 0:
            self.dataPackage = self.arduinoData.readline()   # read data from IDE
            self.dataPackage = str(self.dataPackage, 'utf-8')  # remove b- in front data and \r\n after data
            self.dataPackage = self.dataPackage.strip('\r\n')
            self.splitPackege = self.dataPackage.split(",")  # split data for change str to num
            print(self.splitPackege)
            msg = Imu()
            # header = Header()

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "plane"
            t.transform.translation.x = float(self.splitPackege[0])
            t.transform.translation.y = float(self.splitPackege[1])
            t.transform.translation.z = float(self.splitPackege[2])
            msg.orientation.x = 0.0  # Replace with actual orientation data
            msg.orientation.y = 0.0
            msg.orientation.z = 0.0
            msg.orientation.w = 0.0
            t.transform.rotation.x = msg.orientation.x
            t.transform.rotation.y = msg.orientation.y
            t.transform.rotation.z = msg.orientation.z
            t.transform.rotation.w = msg.orientation.w

            self.transform_broadcaster.sendTransform(t)
            self.pub_imu_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = Arduino_Node()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# #!/usr/bin/python3



# import serial
# import time
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Imu
# from std_msgs.msg import Header
# from builtin_interfaces.msg import Time

# from geometry_msgs.msg import TransformStamped
# from tf2_ros import TransformBroadcaster

# class Arduino_Node(Node):
#     def __init__(self):
#         super().__init__('arduino_node')
#         # from IDE
#         self.arduinoData = serial.Serial('/dev/ttyUSB0',115200)  #send serial from IDE to Python
#         time.sleep(1)

#         timer_cb_group = None
#         self.gyro_ = self.create_timer(0.1,self.gyro_mpu,callback_group=timer_cb_group)
#         print('1')

#         self.pub_imu_ = self.create_publisher(Imu,'/Imu_',100)

#     def gyro_mpu(self):
#         if self.arduinoData.in_waiting > 0:
#             self.dataPackage = self.arduinoData.readline()   #read data from IDE
#             self.dataPackage = str(self.dataPackage,'utf-8') #remove b- in front data and \r\n after data
#             self.dataPackage = self.dataPackage.strip('\r\n')
#             self.splitPackege = self.dataPackage.split(",")  #split data for change str to num
#             print(self.splitPackege)
#             msg = Imu()
#             self.br = TransformBroadcaster()
#             t = TransformStamped()
#             t.header.stamp = self.get_clock().now().to_msg()
#             t.header.frame_id = "plane" 
#             t.transform.translation.x = 0
#             t.transform.translation.y = 0
#             t.transform.translation.z = 0
#             t.transform.rotation.x = msg.orientation.x
#             t.transform.rotation.y = msg.orientation.y
#             t.transform.rotation.z = msg.orientation.z
#             t.transform.rotation.w = msg.orientation.w
#             # msg.header = header
#             # msg.header.stamp = self.get_clock().now().to_msg()
#             # msg.header.frame_id = 'Imu_'

#             # covariance_value = 0.5  # example covariance value
#             # msg.angular_velocity_covariance = [covariance_value] * 9
#             # covariance_value = 0.5  # example covariance value
#             # msg.linear_acceleration_covariance = [covariance_value] * 9
#             # covariance_value = 0.5  # example covariance value
#             # msg.orientation_covariance = [covariance_value] * 9

#             # msg.orientation.x = 0.0 
#             # msg.orientation.y = 0.0 
#             # msg.orientation.z = 0.0
#             # msg.orientation.w = 1.0

#             # msg.angular_velocity.x = float(self.splitPackege[0])
#             # msg.angular_velocity.y = float(self.splitPackege[1])
#             # msg.angular_velocity.z = float(self.splitPackege[2])
#             # msg.linear_acceleration.x = float(self.splitPackege[3])
#             # msg.linear_acceleration.y = float(self.splitPackege[4])
#             # msg.linear_acceleration.z = float(self.splitPackege[5])
#             self.br.sendTransform(t)
#             self.pub_imu_.publish(msg)





# def main(args=None):
#     rclpy.init(args=args)
#     controller = Arduino_Node()
#     rclpy.spin(controller)
#     controller.destroy_node()
#     rclpy.shutdown()

# if __name__=='__main__':
#     main()