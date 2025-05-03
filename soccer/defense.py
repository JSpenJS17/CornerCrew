#! /usr/bin/python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import time, signal, sys, math
from interfaces.srv import SetColorDetectParam
from interfaces.msg import ColorDetect, ColorsInfo

soccer = None

class Camera:
    def __init__(self):
        self.colors = {   #  x, y, size
            "purple" : [0, 0, 0],
            "green"  : [0, 0, 0],
            "red"    : [0, 0, 0],
            "orange" : [0, 0, 0],
        }

class Lidar:
    def __init__(self):
        self.front = -1
        self.left = -1
        self.back = -1
        self.right = -1
        self.leftfront = -1
        self.leftback = -1
        self.rightfront = -1
        self.rightback = -1

class SoccerNode(Node):
    def __init__(self):
        super().__init__("PID")
        self.forward_speed = 0.6
        self.turning_speed = 2.0 # rad/s

        # Publisher to cmd_vel for movement
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Data subscriptions
        self.camera_raw = self.create_subscription(ColorsInfo, '/color_detect/color_info', self.camera_callback, 10)
        self.lidar_raw = self.create_subscription(LaserScan, "/scan_raw", self.lidar_callback, 10)
        
        # Variables
        self.l_let = .25  # how close the right wall needs to be to register as "close"
        self.f_let = .3   # how close the front wall needs to be to register as "close"
        self.r_let_CONST = self.l_let
        self.lidar = Lidar()
        self.camera = Camera()
        self.mid_cam = 320 #this shouldn't be changed
        self.kP = 1/(self.mid_cam) #do not increase past 1/(middleOfCamera-lowerBoundForRamming), decrease to make the robot line up with the objective slower


        # Wait for color detector to be up + running
        self.cli = self.create_client(SetColorDetectParam, '/color_detect/set_param')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for color tracking service...')

        # give it a sec for good luck and set the color tracker to "blue"
        time.sleep(1)
        self.send_request(["red", "purple", "orange", "green"])

        print("Initialized!")

        self.create_timer(0.05, self.main)  # 20Hz control loop for main function

    def send_request(self, colors):
        # Sends a request to color_tracker to set color ("blue", "red", etc.)
        msg = SetColorDetectParam.Request()
        for color in colors:
            msg_color = ColorDetect()
            msg_color.color_name = color
            msg_color.detect_type = 'circle'
            msg.data.append(msg_color)
            print(f"Appending {msg_color} to set_param msg")
        future = self.cli.call_async(msg)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def camera_callback(self, msg):
        """
        This will need to be updated for the 3-4 color system!
        """
        # Load in our camera data
        # check if we found the color
        self.detected_color = len(msg.data) != 0

        # set the camera values accordingly
        if self.detected_color:
            for color in range(len(msg.data)):
                # print(msg.data)
                pass
        # else:
        #     self.camera.set_vals(-1, -1, -1)
        
    def lidar_callback(self, msg):
        # Load in our lidar data
        # ranges: 0 to 2pi
        # 0 is dead ahead, so we want -45 degrees to 45 degrees to represent forward
        # increments COUNTERCLOCKWISE from 0
        # the goal is to add together all values associated with these angles and average them
        # then load that average into their respective "bin" in the Lidar class
        sums = {
            # side :      sum, count    <-- for averaging
            "front"     : [0, 0],
            "left"      : [0, 0],
            "back"      : [0, 0],
            "right"     : [0, 0],
            "leftfront" : [0, 0],
            "leftback"  : [0, 0],
            "rightfront": [0, 0],
            "rightback" : [0, 0],
        }

        # start with min angle
        angle = msg.angle_min
        index = 0
        side_str = "front"
        # loop until we exceed max, at which point we're done
        while angle < msg.angle_max and index < len(msg.ranges):
            # find where we are in the circle based
            if angle >= math.radians(0) and angle < math.radians(22.5):
                side_str = "front"
            
            elif angle >= math.radians(45 - 22.5) and angle < math.radians(45 + 22.5):
                side_str = "leftfront"

            elif angle >= math.radians(90 - 22.5) and angle < math.radians(90 + 22.5):
                side_str = "left"
            
            elif angle >= math.radians(135 - 22.5) and angle < math.radians(135 + 22.5):
                side_str = "leftback"

            elif angle >= math.radians(180 - 22.5) and angle < math.radians(180 + 22.5):
                side_str = "back"

            elif angle >= math.radians(225 - 22.5) and angle < math.radians(225 + 22.5):
                side_str = "rightback"

            elif angle >= math.radians(270 - 22.5) and angle < math.radians(270 + 22.5):
                side_str = "right"

            elif angle >= math.radians(315 - 22.5) and angle < math.radians(315 + 22.5):
                side_str = "rightfront"

            else: # between 315 + 22.5 and 360
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

        # load in the data to our lidar data structure
        self.lidar.front =      divNoZero(sums["front"][0], sums["front"][1])
        self.lidar.back =       divNoZero(sums["back"][0], sums["back"][1])
        self.lidar.left =       divNoZero(sums["left"][0], sums["left"][1])
        self.lidar.right =      divNoZero(sums["right"][0], sums["right"][1])
        self.lidar.leftfront =  divNoZero(sums["leftfront"][0], sums["leftfront"][1])
        self.lidar.leftback =   divNoZero(sums["leftback"][0], sums["leftback"][1])
        self.lidar.rightfront = divNoZero(sums["rightfront"][0], sums["rightfront"][1])
        self.lidar.rightback =  divNoZero(sums["rightback"][0], sums["rightback"][1])

    def main(self):
        # Main movement function, run at 20hz
        linear_x = 0.0
        linear_y = 0.0
        angular_z = 0.0

        # print(f"{round(self.lidar.leftfront, 2):.2f} {round(self.lidar.front, 2):.2f} {round(self.lidar.rightfront, 2):.2f}")
        # print(f"{round(self.lidar.left, 2):.2f}       {round(self.lidar.right, 2):.2f}")
        # print(f"{round(self.lidar.leftback, 2):.2f} {round(self.lidar.back, 2):.2f} {round(self.lidar.rightback, 2):.2f}\n")

        # Create and publish Twist message
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        # print(f"{linear_x}, {linear_y}, {angular_z}")
        self.cmd_vel_pub.publish(twist)

    def get_range_at_angle(self, ranges, angle_min, angle_increment, degree):
        # Convert degree to radians
        # this needs to be averages
        desired_angle = math.radians(degree)

        range = 6.25

        top_index = int((desired_angle-math.radians(range))/angle_increment)
        bot_index = int((desired_angle+math.radians(range))/ angle_increment)

        split = False
        if top_index < 0:
            top_index = int((desired_angle-math.radians(range-360))/angle_increment)
            split = True

        if 0 <= bot_index < len(ranges) and 0 <= top_index < len(ranges):
            if split:
                return (Average(ranges[:bot_index]) + Average(ranges[top_index:])) / 2
            else:
                return Average(ranges[top_index:bot_index])
        else:
            return None

    def zero_vel(self):
        # Create and publish Twist message
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)  


def Average(lst):
    # return sum(lst) / len(lst)
    sum = 0
    count = 1
    for i in lst:
        if not np.isnan(i):
            sum += i
            count += 1
    res = sum/count
    # res = np.sum(np.ma.masked_invalid(lst)) / len(lst) #ignore Inf
    res =  res.__float__() # '--' to nan
    if (np.isnan(res)):
        res = 10.0 #assume there is a lot space
    return res

def divNoZero(num, div):
    if div == 0:
        return 0
    else:
        return num/div   

def check_wall(args=None):
    rclpy.init(args=args)
    node = SoccerNode()
    rclpy.spin(node)
    rclpy.shutdown()

def sigint_handler(sig, frame):
    global soccer
    print("Dying!")
    soccer.zero_vel()
    soccer.destroy_node()
    sys.exit(0)

    
def main():
    global soccer
    rclpy.init()
    signal.signal(signal.SIGINT, sigint_handler)

    soccer = SoccerNode()

    rclpy.spin(soccer)

    soccer.destroy_node()

if __name__ == "__main__":
    main()