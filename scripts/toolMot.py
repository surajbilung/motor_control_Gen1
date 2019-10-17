#!/home/pi/vEnvs/rosPy/bin/python

'''
The tool motor node. All motor nodes share identical structure but subscribe to
their respective topics and are defined by unique GPIO Pins.
'''

from rospy import loginfo, init_node, Subscriber, spin
from std_msgs.msg import Float32MultiArray
from motorControls import axisMotor

toolMot = axisMotor((26, 20), 1, 60, None)

def toolAngCall(data):
    '''
    Logs the angle calculated by the master and feeds it to the tool motor.
    '''
    loginfo(data.data[3])
    toolAng = data.data[3]
    toolMot.moveToAngle(toolAng)

def toolMotNode():
    '''
    Subscriber Node for the tool motor.
    '''
    init_node('toolMotor', anonymous=True)
    Subscriber('motAngs', Float32MultiArray, toolAngCall)
    spin()

if __name__ == '__main__':
    toolMot.gpioInit()
    toolMotNode()
