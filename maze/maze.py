# import everything
import rclpy
from rclpy.node import Node
from interfaces.msg import ColorsInfo
from sensor_msgs.msg import LaserScan
import random, time, math, sys, signal
from ros_robot_controller_msgs.msg import MotorState, MotorsState

frontLeftSpeed = 0
backLeftSpeed = 0
frontRightSpeed = 0
backRightSpeed = 0

maze = None

def setSpeed(fl, bl, fr, br):
    global frontLeftSpeed, backLeftSpeed, frontRightSpeed, backRightSpeed
    maxSpeedConstant = .60
    frontLeftSpeed = fl *maxSpeedConstant
    backLeftSpeed = bl *maxSpeedConstant
    frontRightSpeed = fr *maxSpeedConstant
    backRightSpeed = br *maxSpeedConstant

# Grabbed straight from their code
class MecanumChassis:
    # wheelbase = 0.1368   # 前后轴距(distance between front and real axles)
    # track_width = 0.1446 # 左右轴距(distance between left and right axles)
    # wheel_diameter = 0.065  # 轮子直径(wheel diameter)
    def __init__(self, wheelbase=0.1368, track_width=0.1410, wheel_diameter=0.065):
        self.wheelbase = wheelbase
        self.track_width = track_width
        self.wheel_diameter = wheel_diameter

    def speed_covert(self, speed):
        """
        covert speed m/s to rps/s
        :param speed:
        :return:
        """
        # distance / circumference = rotations per second
        return speed / (math.pi * self.wheel_diameter)

    def set_velocity(self):
        """
        Use polar coordinates to control moving
                    x
        v1 motor1|  ↑  |motor3 v3
          +  y - |     |
        v2 motor2|     |motor4 v4
        :param speed: m/s
        :param direction: Moving direction 0~2pi, 1/2pi<--- ↑ ---> 3/2pi
        :param angular_rate:  The speed at which the chassis rotates rad/sec
        :param fake:
        :return:
        """
        # vx = speed * math.sin(direction)
        # vy = speed * math.cos(direction)
        # vp = angular_rate * (self.wheelbase + self.track_width) / 2
        # v1 = vx - vy - vp
        # v2 = vx + vy - vp
        # v3 = vx + vy + vp
        # v4 = vx - vy + vp
        # v_s = [self.speed_covert(v) for v in [v1, v2, -v3, -v4]]
        motor1 = frontLeftSpeed
        motor2 = backLeftSpeed
        motor3 = frontRightSpeed
        motor4 = backRightSpeed

        v_s = [self.speed_covert(v) for v in [-motor1, -motor2, motor3, motor4]]
        data = []
        for i in range(len(v_s)):
            msg = MotorState()
            msg.id = i + 1
            msg.rps = float(v_s[i])
            data.append(msg)
        
        msg = MotorsState()
        msg.data = data
        return msg

class Camera:
    def __init__(self):
        #fix to actually get values
        self.colourXvalue =0
        self.colourYvalue =0
        self.colourSize =0

    def set_vals(self, x, y, size):
        self.colourXvalue = x
        self.colourYvalue = y
        self.colourSize   = size

class Lidar:
    def __init__(self):
        #fix to actually get values
        # maybe average front/left/back/right?
        self.front = -1
        self.left = -1
        self.back = -1
        self.right = -1

        self.nearestDistance = 0 # calculable

def divNoZero(num, div):
    if div == 0:
        return 0
    else:
        return num/div    

class MazeNode(Node):
    def __init__(self, name="MazeNode"):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        # initialize the camera and lidar data
        self.camera = Camera()
        self.lidar = Lidar()
        self.detected_color = False

        # init motor publisher
        self.motor_pub = self.create_publisher(MotorsState, 'ros_robot_controller/set_motor', 1)
        # init mecanum chassis controller object
        self.mecanum = MecanumChassis()

        # subscription to camera output
        self.create_subscription(ColorsInfo, '/color_detect/color_info', self.update_data, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan_raw', self.update_data, 10)
        
        input("Press enter to start")

    def set_velocity(self):
        # print(f"Setting velocity to: ({frontLeftSpeed}, {backLeftSpeed}, {frontRightSpeed}, {backRightSpeed})")
        speeds = self.mecanum.set_velocity()
        self.motor_pub.publish(speeds)

    def update_data(self, msg):
        # self.get_logger().info(f"{msg.intensities[0]}") # print some lidar data for now
        # camera = Camera(msg.data[0].x, msg.data[0].y, msg.data[0].radius) # this is camera data

        # if we got camera data
        if type(msg) == ColorsInfo:
            """ I don't know how this will look for finding blue """
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
            # the goal is to add together all values associated with these angles and average them
            # then load that average into their respective "bin" in the Lidar class
            
            sums = {
                # side : index, sum, count    <-- for averaging
                "front"     : [0, 0],
                "left"      : [0, 0],
                "back"      : [0, 0],
                "right"     : [0, 0],
                "frontLeft" : [0, 0],
                "frontRight": [0, 0],
                "backLeft"  : [0, 0],
                "backRight" : [0, 0],
            }

            # start with min angle
            angle = msg.angle_min
            index = 0
            side_str = "front"
            spread = 12.25 # degrees of spread in each orthogonal direction
            # i.e. front is [-12.25, 12.25], etc.
            # spread ranges are [0, 45], but 0 is most likely going to skip some values

            # loop until we exceed max, at which point we're done
            while angle < msg.angle_max and index < len(msg.ranges):
                # find where we are in the circle based
                if angle >= math.radians(0) and angle < math.radians(spread):
                    side_str = "front"
                
                elif angle >= math.radians(90 - spread) and angle < math.radians(90 + spread):
                    side_str = "left"
                
                elif angle >= math.radians(180 - spread) and angle < math.radians(180 + spread):
                    side_str = "back"

                elif angle >= math.radians(270 - spread) and angle < math.radians(270 + spread):
                    side_str = "right"

                elif angle >= math.radians(315 + spread) and angle < math.radians(360):
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
            self.lidar.front = divNoZero(sums["front"][0], sums["front"][1])
            self.lidar.back =  divNoZero(sums["back"][0], sums["back"][1])
            self.lidar.left =  divNoZero(sums["left"][0], sums["left"][1])
            self.lidar.right = divNoZero(sums["right"][0], sums["right"][1])

            if self.lidar.front < .25:
                print("Close front")
            elif self.lidar.back < .25:
                print("Close back")
            elif self.lidar.left < .25:
                print("Close left")
            elif self.lidar.right < .25:
                print("Close right")

            self.lidar.nearestDistance = min(
                self.lidar.front,
                self.lidar.back,
                self.lidar.left,
                self.lidar.right,
            )

        # now onto the main movement algorithm with our updated information
        self.move()

        # that will update the speeds, so now we send them to the motors
        self.set_velocity()

    def move(self):
        # make sure lidar and camera data exists
        if self.lidar.front == -1:
            print("No lidar data")
            # return

        ## commented out because we cannot expect to have camera data in Maze
        # elif self.camera.colourXvalue == None:
        #     print("No camera data")
        #     # return

        #set variables
        middleOfCamera = 320 #this shouldn't be changed
        kP = 1/(middleOfCamera) #do not increase past 1/(middleOfCamera-lowerBoundForRamming), decrease to make the robot line up with the other slower
        
        setSpeed(0, 0, 0, 0)
        

            
def sigint_handler(sig, frame):
    print("Exiting...")
    setSpeed(0, 0, 0, 0)
    maze.set_velocity()
    # I think there's a chance that the move() function is called in between now and the next line
    # this would make it so the robot doesn't stop
    maze.destroy_node()
    sys.exit(0)

def main():
    rclpy.init()
    signal.signal(signal.SIGINT, sigint_handler)

    maze = MazeNode("sumo_node")

    rclpy.spin(maze)

    maze.destroy_node()


if __name__ == '__main__':
    main()