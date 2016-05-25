import sys
import signal
import ctypes
import math
from datetime import datetime
import pixy
from pololu_drv8835_rpi import motors

##### defining PixyCam sensory variables
PIXY_MIN_X = 0
PIXY_MAX_X = 319
PIXY_MIN_Y = 0
PIXY_MAX_Y = 199

PIXY_X_CENTER = ((PIXY_MAX_X-PIXY_MIN_X) / 2)
PIXY_Y_CENTER = ((PIXY_MAX_Y-PIXY_MIN_Y) / 2)
PIXY_RCS_MIN_POS = 0
PIXY_RCS_MAX_POS = 1000
PIXY_RCS_CENTER_POS = ((PIXY_RCS_MAX_POS-PIXY_RCS_MIN_POS) / 2)
BLOCK_BUFFER_SIZE = 10

##### defining PixyCam motor variables
PIXY_RCS_PAN_CHANNEL = 0
PIXY_RCS_TILT_CHANNEL = 1

PAN_PROPORTIONAL_GAIN = 200 #400
PAN_DERIVATIVE_GAIN = 300
TILT_PROPORTIONAL_GAIN = 500
TILT_DERIVATIVE_GAIN = 400

MAX_MOTOR_SPEED = 480
MIN_MOTOR_SPEED = -480

run_flag = 1

# 20ms time interval for 50Hz
dt = 20
# check timeout dt*3
timeout = 0.5
currentTime = datetime.now()
lastTime = datetime.now()
startTime = datetime.now()
waitTime = 5 # in seconds

#### defining motor function variables
# 5% drive is deadband
deadband = 0.05 * MAX_MOTOR_SPEED
# totalDrive is the total power available
totalDrive = MAX_MOTOR_SPEED
# throttle is how much of the totalDrive to use [0~1]
throttle = 0
# differential drive level [0~1]
diffDrive = 0
# this is the drive level allocated for steering [0~1] dynamically modulate
diffDrive = 0
# this is the gain for scaling diffDrive
diffGain = 1
# this ratio determines the steering [-1~1]
bias = 0
# this ratio determines the drive direction and magnitude [-1~1]
advance = 0
# this gain currently modulates the forward drive enhancement
driveGain = 1
# body turning p-gain
h_pgain = 0.7 # 0.5
# body turning d-gain
h_dgain = 0
# define panError
panError = 0

#### defining state estimation variables
# pixyViewV = 47
# pixyViewH = 75
# pixyImgV = 400
# pixyImgH = 640
# pixel to visual angle conversion factor (only rough approximation) (pixyViewV/pixyImgV + pixyViewH/pixyImgH) / 2
pix2ang_factor = 0.117
# reference object one is the pink earplug (~12mm wide)
refSize1 = 12
# reference object two is side post (~50mm tall)
refSize2 = 50
# this is the distance estimation of an object
objectDist = 0
# this is some desired distance to keep (mm)
targetDist = 100
# reference distance; some fix distance to compare the object distance with
refDist = 400

blocks = None

def handle_SIGINT(sig, frame):
    """
    Handle CTRL-C quit by setting run flag to false
    This will break out of main loop and let you close
    pixy gracefully
    """
    global run_flag
    run_flag = False

class Blocks(ctypes.Structure):
    """
    Block structure for use with getting blocks from
    pixy.get_blocks()
    """
    _fields_ = [
        ("type", ctypes.c_uint),
        ("signature", ctypes.c_uint),
        ("x", ctypes.c_uint),
        ("y", ctypes.c_uint),
        ("width", ctypes.c_uint),
        ("height", ctypes.c_uint),
        ("angle", ctypes.c_uint)
    ]

class ServoLoop(object):
    """
    Loop to set pixy pan position
    """
    def __init__(self, pgain, dgain):
        self.m_pos = PIXY_RCS_CENTER_POS
        self.m_prevError = 0x80000000L
        self.m_pgain = pgain
        self.m_dgain = dgain

    def update(self, error):
        if self.m_prevError != 0x80000000:
            vel = (error * self.m_pgain + (error - self.m_prevError) * self.m_dgain) >> 10
            self.m_pos += vel
            if self.m_pos > PIXY_RCS_MAX_POS:
                self.m_pos = PIXY_RCS_MAX_POS
            elif self.m_pos < PIXY_RCS_MIN_POS:
                self.m_pos = PIXY_RCS_MIN_POS
        self.m_prevError = error


class LowPassFilter(object):
    """
    Low Pass Filter
    """
    def __init__(self,tau,dt):
        self.centerX = PIXY_RCS_CENTER_POS
        self.tau = tau
        self.llambda = float(1)/tau
        self.dt = dt
        self.lambdaF = self.llambda*self.dt
        self.lambdaOneF = (1 - self.lambdaF)
        self.previousValue = float(0)
        self.previousError = float(0)

    def calculate(self, givenError):
        #print "givenError is " + str(givenError)
        #print "value is: " + str(self.lambdaOneF)
        #print str(self.lambdaF*(givenError + self.previousError))
        self.currentValue = self.lambdaOneF * self.previousValue + 0.5*self.lambdaF*(givenError + self.previousError)
        #self.currentValue = self.lambdaOneF * self.previousValue + 0.5
        #print "self.lambdaOneF " + str(self.lambdaOneF)
        #print "self.previousValue " + str(self.previousValue)
        self.previousValue = self.currentValue
        self.previousError = givenError


class Gradient(object):
    """
    Gradient
    """
    def __init__(self,tau,alpha,dt):
        self.centerX = PIXY_RCS_CENTER_POS
        self.tau = tau
        self.llambda = float(1)/tau
        self.dt = dt
        self.alpha = alpha
        self.lambdaF = self.llambda*self.dt
        self.lambdaOneF = (1 - self.lambdaF)
        self.u1 = 0
        self.u2 = 0
        self.previousU1 = 0
        self.previousU2 = 0
        self.previousGivenValue = float(0)
        self.uOut = 0

    def calculate(self, givenValue):
        self.u1 = (1-self.llambda*self.dt)*self.previousU1 + 0.5*self.dt*(givenValue + self.previousGivenValue)
        self.u2 = self.llambda*self.dt*self.previousU1 + (1-self.llambda*self.dt)*self.previousU2
        self.uOut = self.alpha*(self.u1 - self.u2)
        self.previousU1 = self.u1
        self.previousU2 = self.u2
        self.previousGivenValue = givenValue
        print "uOut is " + str(self.uOut)
        #print "givenError is " + str(givenError)
        #print "value is: " + str(self.lambdaOneF)
        #print str(self.lambdaF*(givenError + self.previousError))
        #self.currentValue = self.lambdaOneF * self.previousValue + 0.5*self.lambdaF*(givenError + self.previousError)
        #self.currentValue = self.lambdaOneF * self.previousValue + 0.5
        #print "self.lambdaOneF " + str(self.lambdaOneF)
        #print "self.previousValue " + str(self.previousValue)
        #self.previousValue = self.currentValue
        #self.previousError = givenError



# define pan loop
panLoop = ServoLoop(300, 500)

# define LP_error for multiple variables
LPError_x = LowPassFilter(100,dt)
LPError_motorL = LowPassFilter(100,dt)
LPError_motorR = LowPassFilter(100,dt)

# define gradients for movement
Grad_x = Gradient(100,0.01,dt)

def setup():
    """
    One time setup. Inialize pixy and set sigint handler
    """
    global blocks
    pixy_init_status = pixy.pixy_init()
    if pixy_init_status != 0:
        print 'Error: pixy_init() [%d] ' % pixy_init_status
        pixy.pixy_error(pixy_init_status)
        return
    else:
        print "Pixy setup OK"
    blocks = pixy.BlockArray(BLOCK_BUFFER_SIZE)
    signal.signal(signal.SIGINT, handle_SIGINT)

    #pixy.pixy_rcs_set_position(0, 50)
    #pixy.pixy_rcs_set_position(PIXY_RCS_PAN_CHANNEL, 110)

def loop():
    """
    Main loop, Gets blocks from pixy, analyzes target location,
    chooses action for robot and sends instruction to motors
    """
    global blocks, throttle, diffDrive, diffGain, bias, advance, turnError, currentTime, lastTime, objectDist, distError, panError, panError_prev, distError_prev
    currentTime = datetime.now()
    # If no new blocks, don't do anything
    while not pixy.pixy_blocks_are_new() and run_flag:
        pass
    count = pixy.pixy_get_blocks(BLOCK_BUFFER_SIZE, blocks)
    # If negative blocks, something went wrong
    if count < 0:
        print 'Error: pixy_get_blocks() [%d] ' % count
        pixy.pixy_error(count)
        sys.exit(1)
    # if more than one block
    # Check which the largest block's signature and either do target chasing or
    # line following
    if count > 0:
        lastTime = currentTime
        print lastTime

    #print block
    numBlocks = 0
    print "new loop!"	
    greenBlocks = []
    for idx in range(count):
        block_ = blocks[idx]
        #print block_.signature
        if block_.signature == 3 :
            greenBlocks.append(block_)
            #print "X: " + str(block_.x) + " Y: " + str(block_.y) + " width: " + str(block_.width) + " height: " + str(block_.height) + " area: " + str(block_.height*block_.width)
        
    # let's first get the biggest first
    max_area = 0
    area_idx = 0
    furthest_idx = 0
    furthest = 600
    gb_idx = 0
    averageX = 0
    for gb in greenBlocks:
         
        block_area = gb.height*gb.width
        if block_area > max_area:
            max_area = block_area
            area_idx = gb_idx 

        if gb.y < furthest :
            furthest_idx = gb_idx
            furthest = gb.y
        
        averageX = averageX + gb.x 
        gb_idx = gb_idx + 1  



    if gb_idx > 0 : # we don't have green blocks
            
            averageX = averageX / gb_idx

            #print gb_idx
            #print greenBlocks[area_idx].x
          
            furthest_block = greenBlocks[furthest_idx] 
            closest_block = greenBlocks[area_idx]

            singleObjTrack = 0 

            # we select which object to track
            target_block = closest_block 
        #    target_block = furthest_block
            if singleObjTrack == 1:
                panError = PIXY_X_CENTER - target_block.x
                objectDist = refSize1 / (2 * math.tan(math.radians(target_block.width * pix2ang_factor)))
            else :
                panErrorOriginal = PIXY_X_CENTER - averageX
                LPError_x.calculate(PIXY_X_CENTER - averageX)
                
                panError = LPError_x.currentValue
                objectDist = refSize1 / (2 * math.tan(math.radians(target_block.width * pix2ang_factor)))
                Grad_x.calculate(panError)
                pass
            throttle = 0.3
            diffDrive = 1.5* diffGain * abs(float(panError)) / PIXY_X_CENTER
            #diffDrive = 0.6    
            #distError = objectDist - targetDist
            #advance = driveGain * float(distError) / refDist
            advance = 1

            print "panError: " + str(panError) + " objectDist: " + str(objectDist) + " diffDrive: " + str(diffDrive)
            #print objectDist

            panLoopServo.update(panErrorOriginal)
            panLoop.update(panError)
            # Update pixy's pan position
            #pixy.pixy_rcs_set_position(PIXY_RCS_PAN_CHANNEL, panLoop.m_pos)

            # if Pixy sees nothing recognizable, don't move.
            time_difference = currentTime - lastTime
            if time_difference.total_seconds() >= timeout:
                throttle = 0.0
                diffDrive = 1

            print "throttle is " + str(throttle)

            # this is turning to left
            print "panLoop.m_pos: " + str(panLoop.m_pos)
            if panLoop.m_pos > PIXY_RCS_CENTER_POS:
                # should be still int32_t
                turnError = panLoop.m_pos - PIXY_RCS_CENTER_POS
                # <0 is turning left; currently only p-control is implemented
                bias = - float(turnError) / float(PIXY_RCS_CENTER_POS) * h_pgain
            # this is turning to right
            elif panLoop.m_pos < PIXY_RCS_CENTER_POS:
                # should be still int32_t
                turnError = PIXY_RCS_CENTER_POS - panLoop.m_pos
                # >0 is turning left; currently only p-control is implemented
                bias = float(turnError) / float(PIXY_RCS_CENTER_POS) * h_pgain
            
            elapsedTime = currentTime - startTime 
            if elapsedTime.seconds > waitTime:
                advance = 1
            else :
                # we only adjust the beginning
                advance = 0
                pixy.pixy_rcs_set_position(PIXY_RCS_PAN_CHANNEL, panLoopServo.m_pos)
            
            drive()
    else: # no green blocks
        pass






 
    return run_flag

def drive():
    # synDrive is the drive level for going forward or backward (for both wheels)
    synDrive = advance * (1 - diffDrive) * throttle * totalDrive
    leftDiff = bias * diffDrive * throttle * totalDrive
    rightDiff = -bias * diffDrive * throttle * totalDrive

    # construct the drive levels
    LDriveRaw = (synDrive + leftDiff)
    RDriveRaw = (synDrive + rightDiff)

    LPError_motorL.calculate(LDriveRaw)
    LPError_motorR.calculate(RDriveRaw)
    LDrive = LPError_motorL.currentValue
    RDrive = LPError_motorR.currentValue 

    # Make sure that it is outside dead band and less than the max
    if LDrive > deadband:
        if LDrive > MAX_MOTOR_SPEED:
            LDrive = MAX_MOTOR_SPEED
    elif LDrive < -deadband:
        if LDrive < -MAX_MOTOR_SPEED:
            LDrive = -MAX_MOTOR_SPEED
    else:
        LDrive = 0

    if RDrive > deadband:
        if RDrive > MAX_MOTOR_SPEED:
            RDrive = MAX_MOTOR_SPEED
    elif RDrive < -deadband:
        if RDrive < -MAX_MOTOR_SPEED:
            RDrive = -MAX_MOTOR_SPEED
    else:
        RDrive = 0

    # Actually Set the motors
    motors.setSpeeds(int(LDrive), int(RDrive))

if __name__ == '__main__':
    setup()
    while True:
        print 'we run loop'
        ok = loop()
        if not ok:
            print 'not work'
            break
    pixy.pixy_close()
    motors.setSpeeds(0, 0)
    print "Robot Shutdown Completed"



