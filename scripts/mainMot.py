#!/home/pi/vEnvs/rosPy/bin/python

'''
The main motor node. All motor nodes share identical structure but subscribe to
their respective topics and are defined by unique GPIO Pins.
'''

from rospy import loginfo, init_node, Subscriber, spin
from std_msgs.msg import Float32MultiArray
from motorControls import axisMotor

#home ang = -15.3943910599

mainMot = axisMotor((22, 23), 1, 60, 7.5)

def mainAngCall(data):
    '''
    Logs the angle calculated by the master and feeds it to the main motor.
    '''
    loginfo(data.data[1])
    mainAng = data.data[1]
    mainMot.moveToAngle(mainAng)

def mainMotNode():
    '''
    Subscriber Node for the main motor.
    '''
    init_node('mainMotor', anonymous=True)
    Subscriber('motAngs', Float32MultiArray, mainAngCall)
    spin()

if __name__ == '__main__':
    mainMot.gpioInit()
    mainMotNode()
