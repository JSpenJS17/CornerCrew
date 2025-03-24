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
    maxSpeedConstant = 0.25
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
        self.create_subscription(ColorsInfo, '/color_detect/color_info', self.update_data, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan_raw', self.update_data, 10)
        self.get_logger().info(f"Setup Complete")

        # initialize the camera and lidar data
        self.camera = Camera(None, None, None)
        self.lidar = Lidar(None, None, None, None)
        self.detected_color = False

        # init motor publisher
        self.motor_pub = self.create_publisher(MotorsState, 'ros_robot_controller/set_motor', 1)
        # init mecanum chassis controller object
        self.mecanum = MecanumChassis()    

        #pick a random number for whether the robot should circle left or right.
        #main goal is to be unpredicatble
        if random.random() < 0.5:
            setSpeed(-1, -1, -1, 1)
        else:
            setSpeed(-1, 1, -1, -1)

        # intial movment to try and dodge out of the opponenets way
        self.set_velocity()
        time.sleep(0.5)

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
            # so ranges are:
                # 45-135 : left
                # 135-225: back
                # 225-315: right
                # 315-45 : front
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
                if angle >= math.radians(45) and angle < math.radians(135):
                    side_str = "left"

                # angle between 135-225, back of robot
                elif angle >= math.radians(135) and angle < math.radians(225):
                    side_str = "back"

                # angle between 225-315, right side of robot
                elif angle >= math.radians(225) and angle < math.radians(315):
                    side_str = "right"
                
                # angle between 315-45 -- since it wraps around, we need to check both ends
                elif angle >= math.radians(315) and angle < msg.angle_max or angle >= msg.angle_min and angle < math.radians(45):
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
            self.lidar.leftDistance  = sums["left"][0] /sums["left"][1]
            self.lidar.backDistance  = sums["back"][0] /sums["back"][1]
            self.lidar.rightDistance = sums["right"][0]/sums["right"][1]
            self.lidar.frontDistance = sums["front"][0]/sums["front"][1]

            # # neat printout to make sure it's working, the .25 is arbitrary and VERY close
            # if self.lidar.leftDistance < .25:
            #     print("Close left")
            # elif self.lidar.backDistance < .25:
            #     print("Close back")
            # elif self.lidar.rightDistance < .25:
            #     print("Close right")
            # elif self.lidar.frontDistance < .25:
            #     print("Close front")
            # else:
            #     print("Nothing close")

        # now onto the main movement algorithm with our updated information
        self.move()

        # that will update the speeds, so now we send them to the motors
        self.set_velocity()

    def move(self):
        # make sure lidar and camera data exists
        if self.lidar.frontDistance == None:
            print("No lidar data")
            return            print(self.lidar.backDistance)

        elif self.camera.colourXvalue == None:
            print("No camera data")
            return

        #set variables
        middleOfCamera = 320 #this shouldn't be changed
        lowerBoundForRamming = 110 #increase to make it ram when it is better lined up, decrease it if it needs to ram sooner
        upperBoundForRamming = middleOfCamera + (middleOfCamera-lowerBoundForRamming) #shouldn't need to be changed
        touchingThreshold = 25 #thershold for how big the orange is to know to ram. Increase to make it wait closer to full send. Decrease to make it full send sooner
        dangerThreshold = .5 #thershold for how close to the wall before the robot is before it moves away. Increase to make it stay further from the wall
        kP = 0.01 #do not increase past 1/(middleOfCamera-lowerBoundForRamming), decrese to make the robot line up with the other slower

        #1st priority is to ram into an opponenet if we are lined up perfectly
        if(self.camera.colourXvalue > lowerBoundForRamming and self.camera.colourXvalue < upperBoundForRamming):
            #ram straight into the other robot if you are really close
            if(self.camera.colourSize > touchingThreshold):
                print("Ramming!!!")
                setSpeed(1, 1, 1, 1)

            else:
                #using strafing capabilites of mecanum drive to better line up with the target while ramming
                lateralError = self.camera.colourXvalue-middleOfCamera

                print("Strafing towards opponents")
                #execute this if the target is left of the centre 
                if(lateralError < 0):
                    #to strafe left slightly, give the front left and back right wheels less power
                    setSpeed(1- abs(lateralError) * kP, 1, 1, 1- abs(lateralError) * kP)
                #execute this if the target is right of the centre 
                else:
                    #to strafe right slightly, give the back left and front right wheels less power
                    setSpeed(1, 1- abs(lateralError) * kP, 1- abs(lateralError) * kP, 1)

        #2nd priorirt check how close to the wall the robot is
        elif(self.lidar.backDistance < dangerThreshold):
            print("Back Danger Threshold hit")
            #drive away from the wall
            setSpeed(1, 1, 1, 1)

        #check for objects to the right
        elif(self.lidar.rightDistance < dangerThreshold):
            print("Right Danger Threshold hit")
            #drive away from the wall
            setSpeed(-1, 1, 1, -1)

        #check for objects to the left
        elif(self.lidar.leftDistance < dangerThreshold):
            print("Left Danger Threshold hit")
            #drive away from the wall
            setSpeed(1, -1, -1, 1)

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
            setSpeed(1, 1, -1, -1)
            
def sigint_handler(sig, frame):
    sumo = SumoNode("sumo_node_die")
    setSpeed(0, 0, 0, 0)
    sumo.set_velocity()
    sys.exit(0)

def main():
    rclpy.init()
    signal.signal(signal.SIGINT, sigint_handler)

    sumo = SumoNode("sumo_node")

    rclpy.spin(sumo)

    sumo.destroy_node()


if __name__ == '__main__':
    main()