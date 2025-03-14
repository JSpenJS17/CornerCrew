import random
import time

frontLeftSpeed =0
backLeftSpeed = 0
frontRightSpeed = 0
backRightSpeed = 0

rightOrLeft = random.random()


def setSpeed(fl, bl, fr, br):
    maxSpeedConstant = 0.61
    fronLeftSpeed = fl *maxSpeedConstant
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

    def set_velocity(self, linear_x, linear_y, angular_z):
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
        motor1 = (frontLeftSpeed)
        motor2 = (backLeftSpeed)
        motor3 = (frontRightSpeed)
        motor4 = (backRightSpeed)

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

class camera:
    def __init__(self):
        #fix to actually get values
        self.colourXvalue =0
        self.colourYvalue =0
        self.colourSize =0

class lidar:
    def __init__(self):
        #fix to actually get values
        self.frontDisatnce =0
        self.backDistance =0
        self.rightDisatnce =0
        self.leftDisatnce =0
        self.nearestDistance =0

def main():


    #set variables
    middleOfCamera = 320
    lowerBoundForRamming = 220
    upperBoundForRamming = 420
    touchingThreshold = 30
    lidarTouchingThreshold =1000
    dangerThreshold = 1000
    kP = 0.01

    #pick a random number for whether the robot should circle left or right.
    #main goal is to be unpredicatble
    
    if rightOrLeft < 0.5:
        setSpeed(-1, -1, -1, 1)

    else:
        setSpeed(-1, 1, -1, -1)

 
    #create camera object
    cam = camera()
    lid = lidar()

    #intial movment to try and dodge out of the opponenets way

    setSpeed(-1, -1, -1, 1)
    time.sleep(0.5)

    while(True):
        #Get camera Data

        #1st priority is to ram into an opponenet if we are lined up perfectly
        if(cam.colourXvalue > lowerBoundForRamming and cam.colourXvalue < upperBoundForRamming):

            #ram straight into the other robot if you are really close
            if(cam.colourSize > touchingThreshold or lid.frontDisatnce < lidarTouchingThreshold):
                setSpeed(1, 1, 1, 1)

            else:
                #using strafing capabilites of mecanum drive to better line up with the target while ramming
                lateralError = cam.colourXvalue-middleOfCamera

                #execute this if the target is left of the centre 
                if(lateralError < 0):
                    #to strafe left slightly, give the front left and back right wheels less power
                    setSpeed(1- abs(lateralError) * kP, 1, 1, 1- abs(lateralError) * kP)
                #execute this if the target is right of the centre 
                else:
                    #to strafe right slightly, give the back left and front right wheels less power
                    setSpeed(1, 1- abs(lateralError) * kP, 1- abs(lateralError) * kP, 1)

        #2nd priorirt check how close to the wall the robot is
        elif(lid.backDistance < dangerThreshold):
            #drive away from the wall
            setSpeed(1, 1, 1, 1)

        #check for objects to the right
        elif(lid.rightDistance < dangerThreshold):
            #drive away from the wall
            setSpeed(-1, 1, 1, -1)

        #check for objects to the left
        elif(lid.leftDistance < dangerThreshold):
            #drive away from the wall
            setSpeed(1, -1, -1, 1)

        #3rd priort is prepare to ram the other team
        elif(cam.colourSize != 0):
            #object is too far to the left
            if(cam.colourXvalue < middleOfCamera):
                #turn to the right
                setSpeed(-1, 1, -1, -1)
            #object too dar to the right
            else:
                #turn to the left
                setSpeed(-1, -1, -1, 1)


        #turn to try and find the other robot
        else:
            setSpeed(1, 1, -1, -1)



        pass

main()