#!/home/pi/vEnvs/rosPy/bin/python

'''
The base motor node. All motor nodes share identical structure but subscribe to
their respective topics and are defined by unique GPIO Pins.
'''

from rospy import loginfo, init_node, Subscriber, spin
from std_msgs.msg import Float32MultiArray
from motorControls import axisMotor

#home ang = 0

baseMot = axisMotor((17, 18), 1, 60, None)

def baseAngCall(data):
    '''
    Logs the angle calculated by the master and feeds it to the base motor.
    '''
    loginfo(data.data[0])
    baseAng = data.data[0]
    baseMot.moveToAngle(baseAng)

def baseMotNode():
    '''
    Subscriber Node for the base motor.
    '''
    init_node('baseMotor', anonymous=True)
    Subscriber('motAngs', Float32MultiArray, baseAngCall)
    spin()

if __name__ == '__main__':
    baseMot.gpioInit()
    baseMotNode()
