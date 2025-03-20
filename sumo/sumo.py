# import everything
import rclpy
from rclpy.node import Node
from interfaces.msg import ColorsInfo
from sensor_msgs.msg import LaserScan
import random
import time
import math
from ros_robot_controller_msgs.msg import MotorState, MotorsState

class Camera:
    def __init__(self, x, y, size):
        #fix to actually get values
        self.colourXvalue =0
        self.colourYvalue =0
        self.colourSize =0

    def set_vals(self, x, y, size):
        self.colourXvalue = x
        self.colourYvalue = y
        self.colourSize   = size

class Lidar:
    def __init__(self, front, back, right, left):
        #fix to actually get values
        # maybe average front/left/back/right?
        self.frontDistance =0
        self.backDistance =0
        self.rightDistance =0
        self.leftDistance =0

        self.nearestDistance =0 #calculable

class SumoNode(Node):
    def __init__(self, name="ListenerNode"):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        # subscription to camera output
        self.create_subscription(ColorsInfo, '/color_detect/color_info', self.movement_algo, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan_raw', self.movement_algo, 10)
        self.get_logger().info(f"Setup Complete")

        self.camera = Camera(None, None, None)
        self.lidar = Lidar(None, None, None, None)
        self.detected_color = False


    def movement_algo(self, msg):
        # self.get_logger().info(f"{msg.intensities[0]}") # print some lidar data for now
        # camera = Camera(msg.data[0].x, msg.data[0].y, msg.data[0].radius) # this is camera data

        # if we got camera data
        if type(msg) == ColorsInfo:
            # check if we found the color
            self.detected_color = len(msg.data) != 0

            # set the camera values accordingly
            if self.detected_color:
                self.camera.set_vals(msg.data[0].x, msg.data[0].y, msg.data[0].radius)
            else:
                self.camera.set_vals(-1, -1, -1)

        # otherwise it's lidar
        else:
            # ranges: 0 to 2pi
            # 0 is dead ahead, so we want -45 degrees to 45 degrees to represent forward
            # increments COUNTERCLOCKWISE from 0
            # so ranges are:
                # 45-135 : left
                # 135-225: back
                # 225-315: right
                # 315-45 : front
            # in radians,
                # .7853982-2.356194 : left
                # 2.356194-3.926991 : back
                # 3.926991-5.497787 : right
                # 5.497787-.7853982 : front
            # the goal is to add together all values associated with these angles and average them
            # then load that average into their respective "bin" in the Lidar class
            
            sums = {
                # side : sum, count    <-- for averaging
                "left" : [0, 0],
                "back" : [0, 0],
                "right": [0, 0],
                "front": [0, 0],
            }

            # start with min angle
            angle = msg.angle_min
            index = 0
            side_str = "front"
            # loop until we exceed max, at which point we're done
            while angle < msg.angle_max and index < len(msg.ranges):
                # angle between 45-135, left side of robot
                if angle >= .7853982 and angle < 2.356194:
                    side_str = "left"

                # angle between 135-225, back of robot
                elif angle >= 2.356194 and angle < 3.926991:
                    side_str = "back"

                # angle between 225-315, right side of robot
                elif angle >= 3.926991 and angle < 5.497787:
                    side_str = "right"
                
                # angle between 315-45
                elif angle >= 5.497787 and angle < msg.angle_max or angle >= msg.angle_min and angle < .7853982:
                    side_str = "front"

                # make sure we're not doing math with nans
                if not math.isnan(msg.ranges[index]):
                    # add to the sum of the side
                    sums[side_str][0] += msg.ranges[index]
                    # and increment the count
                    sums[side_str][1] += 1

                # add on the increment
                angle += msg.angle_increment
                index += 1

            self.lidar.leftDistance  = sums["left"][0] /sums["left"][1]
            self.lidar.backDistance  = sums["back"][0] /sums["back"][1]
            self.lidar.rightDistance = sums["right"][0]/sums["right"][1]
            self.lidar.frontDistance = sums["front"][0]/sums["front"][1]

            # neat printout to make sure it's working
            if self.lidar.leftDistance < .25:
                print("Close left")
            elif self.lidar.backDistance < .25:
                print("Close back")
            elif self.lidar.rightDistance < .25:
                print("Close right")
            elif self.lidar.frontDistance < .25:
                print("Close front")
            else:
                print("Nothing close")

        # now onto the main movement algorithm with our updated information

            
def main():
    rclpy.init()

    sumo = SumoNode("sumo_node")

    rclpy.spin(sumo)


if __name__ == '__main__':
    main()