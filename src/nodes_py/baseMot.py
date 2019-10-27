#!/home/pi/vEnvs/rosPy/bin/python

'''
The base motor node. All motor nodes share identical structure but subscribe to
their respective topics and are defined by unique GPIO Pins.
'''

from motors.msg import DOFArray
from rospy import loginfo, init_node, Subscriber, spin
from motorControls import axisMotor

#home ang = 0

baseMot = axisMotor((17, 18), 1, 60, None)

def baseAngCall(data):
    '''
    Logs the angle calculated by the master and feeds it to the base motor.
    '''
    loginfo(data.baseAng)
    baseAng = data.baseAng
    baseMot.moveToAngle(baseAng)

def baseMotNode():
    '''
    Subscriber Node for the base motor.
    '''
    init_node('baseMotor', anonymous=True)
    Subscriber('motAngs', DOFArray, baseAngCall)
    spin()

if __name__ == '__main__':
    baseMot.gpioInit()
    baseMotNode()
