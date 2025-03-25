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
        self.frontLeft = -1
        self.frontRight = -1
        self.backLeft = -1
        self.backRight = -1

        self.nearestDistance = 0 # calculable

def divNoZero(num, div):
    if div == 0:
        return 0
    else:
        return num/div    

class SumoNode(Node):
    def __init__(self, name="ListenerNode", die = False):
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

        if die:
            print("Dying!")
            setSpeed(0, 0, 0, 0)
            self.set_velocity()
            return
        
        self.leftOrRight = random.random() < .5
        self.count = 0

        
        

    def set_velocity(self):
        # print(f"Setting velocity to: ({frontLeftSpeed}, {backLeftSpeed}, {frontRightSpeed}, {backRightSpeed})")
        speeds = self.mecanum.set_velocity()
        self.motor_pub.publish(speeds)

    def update_data(self, msg):
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
            # loop until we exceed max, at which point we're done
            while angle < msg.angle_max and index < len(msg.ranges):
                # find where we are in the circle based
                if angle >= math.radians(0) and angle < math.radians(22.5):
                    side_str = "front"
                
                elif angle >= math.radians(45 - 22.5) and angle < math.radians(45 + 22.5):
                    side_str = "frontLeft"

                elif angle >= math.radians(90 - 22.5) and angle < math.radians(90 + 22.5):
                    side_str = "left"
                
                elif angle >= math.radians(135 - 22.5) and angle < math.radians(135 + 22.5):
                    side_str = "backLeft"

                elif angle >= math.radians(180 - 22.5) and angle < math.radians(180 + 22.5):
                    side_str = "back"

                elif angle >= math.radians(225 - 22.5) and angle < math.radians(225 + 22.5):
                    side_str = "backRight"

                elif angle >= math.radians(270 - 22.5) and angle < math.radians(270 + 22.5):
                    side_str = "right"

                elif angle >= math.radians(315 - 22.5) and angle < math.radians(315 + 22.5):
                    side_str = "frontRight"

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
            self.lidar.frontLeft =  divNoZero(sums["frontLeft"][0], sums["frontLeft"][1])
            self.lidar.frontRight = divNoZero(sums["frontRight"][0], sums["frontRight"][1])
            self.lidar.backLeft =   divNoZero(sums["backLeft"][0], sums["backLeft"][1])
            self.lidar.backRight =  divNoZero(sums["backRight"][0], sums["backRight"][1])

            # if self.lidar.front < .25:
            #     print("Close front")
            # elif self.lidar.back < .25:
            #     print("Close back")
            # elif self.lidar.left < .25:
            #     print("Close left")
            # elif self.lidar.right < .25:
            #     print("Close right")
            # elif self.lidar.frontLeft < .25:
            #     print("Close frontLeft")
            # elif self.lidar.frontRight < .25:
            #     print("Close frontRight")
            # elif self.lidar.backLeft < .25:
            #     print("Close backLeft")
            # elif self.lidar.backRight < .25:
            #     print("Close backRight")
            # else:
            #     print("Not close")

            self.lidar.nearestDistance = min(
                # self.lidar.front,
                self.lidar.back,
                self.lidar.left,
                self.lidar.right,
                self.lidar.frontLeft,
                self.lidar.frontRight,
                self.lidar.backLeft,
                self.lidar.backRight,
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

        elif self.camera.colourXvalue == None:
            print("No camera data")
            # return

        #set variables
        middleOfCamera = 320 #this shouldn't be changed
        #lowerBoundForRamming = 0 #increase to make it ram when it is better lined up, decrease it if it needs to ram sooner
        #upperBoundForRamming = middleOfCamera + (middleOfCamera-lowerBoundForRamming) #shouldn't need to be changed
        touchingThreshold = 50 #thershold for how big the orange is to know to ram. Increase to make it wait closer to full send. Decrease to make it full send sooner
        dangerThreshold = .35 #thershold for how close to the wall before the robot is before it moves away. Increase to make it stay further from the wall
        kP = 1/(middleOfCamera) #do not increase past 1/(middleOfCamera-lowerBoundForRamming), decrease to make the robot line up with the other slower
        

        #1st priority is to ram into an opponenet if we are lined up perfectly
        if (self.count < 12):
            print(self.count)
            #pick a random number for whether the robot should circle left or right.
            #main goal is to be unpredicatble
            if self.leftOrRight:
                setSpeed(1, -1, -1, 1)
            else:
                setSpeed(-1, 1, 1, -1)

            self.count += 1

        elif(self.count > 30):
            setSpeed(1, 1, 1, 1)
            self.count -= 1

        #ram straight into the other robot if you are really close
        elif(self.camera.colourSize > touchingThreshold):
            print("Ramming!!!")
            setSpeed(1, 1, 1, 1)
            self.count = 40

        elif(self.camera.colourXvalue > 0 and self.camera.colourXvalue < middleOfCamera * 2):
            print("Strafing towards opponents")
            #using strafing capabilites of mecanum drive to better line up with the target while ramming
            lateralError = self.camera.colourXvalue-middleOfCamera
            print(f"Lateral Error: {lateralError}")
            #execute this if the target is left of the centre 
            if(lateralError < 0):
                #to strafe left slightly, give the front left and back right wheels less power
                setSpeed(1- abs(lateralError) * kP, 1, 1, 1- abs(lateralError) * kP)
            #execute this if the target is right of the centre 
            else:
                #to strafe right slightly, give the back left and front right wheels less power
                setSpeed(1, 1- abs(lateralError) * kP, 1- abs(lateralError) * kP, 1)

        # check for walls
        elif self.lidar.nearestDistance < dangerThreshold:
            if self.lidar.frontLeft == self.lidar.nearestDistance:
                print("frontLeft Danger Threshold hit")
                setSpeed(0, -1, -1, 0)

            elif self.lidar.left == self.lidar.nearestDistance:
                print("left Danger Threshold hit")
                setSpeed(1, -1, -1, 1)

            elif self.lidar.backLeft == self.lidar.nearestDistance:
                print("backLeft Danger Threshold hit")
                setSpeed(1, 0, 0, 1)

            elif self.lidar.back == self.lidar.nearestDistance:
                print("back Danger Threshold hit")
                setSpeed(1, 1, 1, 1)

            elif self.lidar.backRight == self.lidar.nearestDistance:
                print("backRight Danger Threshold hit")
                setSpeed(0, 1, 1, 0)

            elif self.lidar.right == self.lidar.nearestDistance:
                print("Back Danger Threshold hit")
                setSpeed(-1, 1, 1, -1)
            
            elif self.lidar.frontRight == self.lidar.nearestDistance:
                print("Back Danger Threshold hit")
                setSpeed(-1, 0, 0, -1)

            else:
                print("front/frontLeft/frontRight Danger Threshold hit, ignoring")

        #3rd priorty is prepare to ram the other team
        elif(self.camera.colourSize != -1):
            print("Turning to face opponent")
            #object is too far to the left
            if(self.camera.colourXvalue < middleOfCamera):
                #turn to the right
                setSpeed(-1, 1, -1, -1)
            #object too far to the right
            else:
                #turn to the left
                setSpeed(-1, -1, -1, 1)


        #turn to try and find the other robot
        else:
            print("No robot found, turning to find")
            if self.leftOrRight:
                setSpeed(-.5, -.5, .5, .5)
            else:
                setSpeed(.5, .5, -.5, -.5)

            
def sigint_handler(sig, frame):
    sumo = SumoNode("sumo_node_die", die = True)
    sys.exit(0)

def main():
    rclpy.init()
    signal.signal(signal.SIGINT, sigint_handler)

    sumo = SumoNode("sumo_node")

    rclpy.spin(sumo)

    sumo.destroy_node()


if __name__ == '__main__':
    main()