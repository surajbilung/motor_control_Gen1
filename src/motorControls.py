#!/home/ubuntu/vEnvs/rosPy/bin/python

'''
The following classes and functions control a 4-DOF robotic arm with the option to add a "Tool Head"
for functionality and extra mobility. The 4-DOF will position the robot relative the the "Tool Head"
axis of rotation. What are referred to as "Correction Factors" will be stored locally on the machine
so that the custom tool heads can be postioned correctly without having to be accounted for in every
coordinate.

The arm is positioned by solving two separate 2D problems. First, the base will take the x and y
component of the position vector and rotate itself so that the plane through the arm links will
be coincident with the plane of that line spanned through the z-axis. Once that is done, the x, y
coordinate will be converted into a new line in said plane (xPrime) and that line along with the
z coordinate will define the 2D position vector. With that information as well as the arm lengths
the angle for the main and second arm can be calculated using the law of cosines to create a
triangle that satisfies the required position. The tool arm by default will always be parallel
with the ground.
'''

from math import pi, atan, atan2, acos, sqrt
from time import sleep
import pigpio

### HOME COORDINATES (X, Y, Z) = [2.159, 0.000, 4.920] ###

class axisMotor:

    '''
    The 4 stepper motors to control the arm are managed in the following class. An instance of that
    class is created by feeding the class the GPIO-PIN numbers (BOARD MODE), Gear Ratio on motor,
    and the length of the arm it is positioning. The final method in this class will take the angle
    that it needs to be positioned at as an input. Then it will take it to account the current angle
    it's at and convert the difference of those angles into a number of steps required to move to
    the input. Then it will remember its new location. Methods exisiting outside of the class will
    generate angular inputs for this motor method.
    '''

    def __init__(self, controllerPins, gearRatio, rpm, length):
        self.stepPin = controllerPins[0]
        self.directionPin = controllerPins[1]
        self.gearRatio = gearRatio
        self.rpm = rpm
        self.length = length
        self.memory = [0]
        self.piPins = pigpio.pi()

    def gpioInit(self):
        '''
        Initializes GPIO pins for each respective motor. Each node initializes it's own pins
        '''
        self.piPins.set_mode(self.directionPin, pigpio.OUTPUT)
        self.piPins.set_mode(self.stepPin, pigpio.OUTPUT)

    def directionControl(self, angle):
        '''
        Considers if the new angle is clockwise or counter clockwise from the current position
        then sets the direction pin based on what it sees.
        '''
        steps = ((float(angle) - self.memory[0]) / 360) * 800
        if steps < 0:
            steps = abs(steps)
            self.piPins.write(self.directionPin, 1)
        else:
            self.piPins.write(self.directionPin, 0)
        return steps

    def gpioFreq(self):
        '''
        Determines the time between the rising and falling edges of the psuedo PWM signal.
        '''
        return 1 / ((float(self.rpm) * 800) / 60)

    def moveToAngle(self, angle):
        '''
        This method takes the difference of the current position and desired position and converts
        into steps. If the number is negative the GPIO direction pin will go high to change the
        motors direction. The domain is built to be 0 <= theta <= 360 so the motor will never
        tangle wires. At the end the new position is remembered.
        '''
        steps = self.directionControl(angle * self.gearRatio)
        sleepTime = self.gpioFreq()
        while steps > 0:
            self.piPins.write(self.stepPin, 1)
            sleep(sleepTime / 2)
            self.piPins.write(self.stepPin, 0)
            sleep(sleepTime / 2)
            steps -= 1
        self.memory[0] = angle

def positionVectorDef(xComp, yComp, zComp):
    '''
    Input the x, y, z coordinate of the desired poisiton and the necessary components of that
    vector will be calculated and returned. xPrime describes the horizontal length of the 2D
    vector residing in the plane already achieved by rotating the base.
    '''
    xPrime = sqrt(xComp**2 + yComp**2)
    pvAng = (atan(zComp / xPrime)) * 180 / pi
    length = sqrt(xPrime**2 + zComp**2)
    return length, pvAng

def positionVectorCheck(xComp, yComp, zComp):
    '''
    Returns True if the arm can achieve the components of the position vector. This will also
    avoid any runtime errors of calculating undefined values. The domain is determined by
    the length of the arms.
    '''
    comparisonValue = sqrt(xComp**2 + yComp**2 + zComp**2)
    return 2 <= comparisonValue <= 10

def baseAngle(xComp, yComp): ### I HATE THIS FUNCTION ###
    '''
    Given the x and y components of the position vector, the math.atan2() function will
    return the angle in the domain -180 <= theta <= 180. The desired domain is
    0 <= theta <= 360 so a quadrant correcting constant is passed into the lambda
    function. If the point lies in the Q1 or Q1 the quadrant corrector is 0 since the
    value returned will be between 0 and 180. If the point lies in Q3 or Q4 the quadrant
    corrector is 360 since the value returned will be between -180 and 0. It is like
    measuring forward from 0 to 180 or backward from 360 to 180. Points on quadrantals
    are also accounted for.
    '''
    angleCalc = lambda signCorrection: ((atan2(yComp, xComp)) * 180 / pi) + signCorrection
    if yComp > 0:
        baseAng = angleCalc(0)
    elif yComp < 0:
        baseAng = angleCalc(360)
    elif xComp == 0 and yComp > 0:
        baseAng = 90
    elif xComp == 0 and yComp < 0:
        baseAng = 270
    elif xComp > 0 and yComp == 0:
        baseAng = 0
    elif xComp < 0 and yComp == 0:
        baseAng = 180
    return baseAng

def armAngles(mainArmLength, secArmLength, positionVectorLength, positionVectorAngle):
    '''
    Using the law of cosignes with the length of the main arm, secondary arm, and position vector
    each angle for the arms can be calculated. The tool arm angle is defined by the geometry of
    the triangle and is set to always stay parallel with the ground.
    '''
    [len1, len2, len3] = [mainArmLength, secArmLength, positionVectorLength]
    mainDegInTri = (acos((len3**2 + len1**2 - len2**2) / (2 * len3 * len1))) * 180 / pi
    mainDegFromZ = 90 - mainDegInTri - positionVectorAngle
    secAng = (acos((len1**2 + len2**2 - len3**2) / (2 * len1 * len2))) * 180 / pi
    angleRemainder = 180 - secAng - mainDegInTri
    toolAng = 180 - angleRemainder - positionVectorAngle
    return mainDegFromZ, secAng, toolAng

def coordinateErrorMsg():
    '''
    Error Message when a coordinate is outside the bounds of the arms capabilities.
    '''
    print('''


=========================================
        COORDINATE NOT VALID!!!
    PLEASE CORRECT THE INSTRUCTION...

    ARM RETURNING TO HOME POSITION...
=========================================


    ''')

def genAngles(inputCoor):
    '''
    Takes an x, y, z coordinate and returns all motor angles.
    '''
    [xComp, yComp, zComp] = [float(inputCoor[0]), float(inputCoor[1]), float(inputCoor[2])]
    if positionVectorCheck(xComp, yComp, zComp):
        [pvLen, pvAng] = positionVectorDef(xComp, yComp, zComp)
        baseAng = baseAngle(xComp, yComp)
        [mainAng, secAng, toolAng] = armAngles(7.5, 4.75, pvLen, pvAng)
    else:
        coordinateErrorMsg()
        [pvLen, pvAng] = positionVectorDef(2.159, 0.000, 4.920)
        baseAng = baseAngle(2.159, 0.000)
        [mainAng, secAng, toolAng] = armAngles(7.5, 4.75, pvLen, pvAng)
    return [baseAng, mainAng, secAng, toolAng]
