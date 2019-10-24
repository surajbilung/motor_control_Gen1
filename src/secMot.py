#!/home/ubuntu/vEnvs/rosPy/bin/python

'''
The second motor node. All motor nodes share identical structure but subscribe to
their respective topics and are defined by unique GPIO Pins.
'''

from rospy import loginfo, init_node, Subscriber, spin
from motors.msg import DOFArray
from motorControls import axisMotor

secMot = axisMotor((19, 16), 1, 60, 4.75)

def secAngCall(data):
    '''
    Logs the angle calculated by the master and feeds it to the second motor.
    '''
    loginfo(data.secAng)
    secAng = data.secAng
    secMot.moveToAngle(secAng)


def secondaryMotNode():
    '''
    Subscriber Node for the second motor.
    '''
    init_node('secondaryMotor', anonymous=True)
    Subscriber('motAngs', DOFArray, secAngCall)
    spin()

if __name__ == '__main__':
    secMot.gpioInit()
    secondaryMotNode()
