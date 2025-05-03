
import random, time, math, sys, signal


def setSpeed(fl, bl, fr, br):
    global frontLeftSpeed, backLeftSpeed, frontRightSpeed, backRightSpeed
    maxSpeedConstant = .60
    frontLeftSpeed = fl *maxSpeedConstant
    backLeftSpeed = bl *maxSpeedConstant
    frontRightSpeed = fr *maxSpeedConstant
    backRightSpeed = br *maxSpeedConstant

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

def goalie():

    rightGoalThreshold = 0 #needs to be modified
    leftGoalThreshold = 0 #needs to be modified
    middleOfCamera = 320 #this shouldn't be changed
    scoringThreshold = 50 #needs to be modified
    sizeThreshold = 50 #needs to be modified

  
    #drive back into the goal
    setSpeed(-1, -1, -1, -1)
    time.sleep(.5)

    
    #search for bal
    while(1):
        #ensure the ball is in the middle, close, and not obstructed by a robot
        if((redXvalue < middleOfCamera + scoringThreshold and redXvalue > middleOfCamera - scoringThreshold) and (redSizeValue > sizeThreshold) and not(orangeXvalue < middleOfCamera + scoringThreshold and orangeXvalue > middleOfCamera - scoringThreshold) ):
            setSpeed(1, 1, 1, 1)
            time.sleep(1)
            setSpeed(-1, -1, -1, -1)
            time.sleep(1.25)  
            
        elif (redXvalue < middleOfCamera and lidar.leftDistance >  leftGoalThreshold):
            setSpeed(-1, 0.9, 0.9, -1)

        elif (redXvalue > middleOfCamera and lidar.rightDistance >  rightGoalThreshold):
            setSpeed(0.9, -1, -1, 0.9)
        
        else:
            if(lidar.leftDistance > lidar.rightDistance):
                setSpeed(0.9, -1, -1, 0.9)
            else:
                setSpeed(-1, 0.9, 0.9, -1)


        time.sleep(.01)


    

def defence():
    frontGoalThreshold = 0
    backGoalThreshold = 0
  
    #turn to block the goal
    setSpeed(-1, -1, 1, 1)
    time.sleep(.5)

    #strafe into the goal
    setSpeed(-1, 1, 1, -1)
    time.sleep(.5)

    #begin blocking
    setSpeed(-1, -1, -1, -1)

    #bounce to and fro
    while(1):
        if(lidar.frontDistance <  frontGoalThreshold):
            setSpeed(-1, -0.9, -0.9, -1)
 
        elif(lidar.backDistance <  backGoalThreshold):
            setSpeed(0.9, 1, 1, 0.9)

        time.sleep(.01)


def main():
    if(winning):
        defence()
    else:
        goalie()

main()
