
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

def offence():
    pass

def defence():
    
    pass

def main():
    if(winning):
        defence()
    else:
        offence()

main()
