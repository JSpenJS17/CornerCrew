# import everything
import rclpy
from rclpy.node import Node
from interfaces.msg import ColorsInfo
from sensor_msgs.msg import LaserScan
import random, time
import random
import time
from ros_robot_controller_msgs.msg import MotorState, MotorsState

class Camera:
    def __init__(self, x, y, size):
        #fix to actually get values
        self.colourXvalue =0
        self.colourYvalue =0
        self.colourSize =0

class Lidar:
    def __init__(self, front, back, right, left):
        #fix to actually get values
        self.frontDisatnce =0
        self.backDistance =0
        self.rightDisatnce =0
        self.leftDisatnce =0

        self.nearestDistance =0 #calculable

class SumoNode(Node):
    def __init__(self, name="ListenerNode"):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        # subscription to camera output
        self.create_subscription(ColorsInfo, '/color_detect/color_info', self.movement_algo, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan_raw', self.movement_algo, 10)
        # self.create_subscription()
        self.get_logger().info(f"Setup Complete")

        self.camera = Camera(None, None, None)
        self.lidar = Lidar(None, None, None, None)


    def movement_algo(self, msg):
        # camera_data = msg.data
        # camera = Camera(msg.data[0].x, msg.data[0].y, msg.data[0].radius) # this is camera data
        self.get_logger().info(f"{msg.intensities[0]}") # print some lidar data for now

        
def main():
    rclpy.init()

    sumo = SumoNode("sumo_node")

    rclpy.spin(sumo)


if __name__ == '__main__':
    main()