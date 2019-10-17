#!/home/pi/vEnvs/rosPy/bin/python

'''
Master ROS Node. Responsible for solving arm angles and distributing them to each arm.
This node publishes to 4 separate topics in the float format just to keep messages simple.
Once the arm nodes receive their instructions as a float, it will only be converted to an integer
once the actual amount of steps are calculated.
'''

from pandas import DataFrame, read_csv
from motorControls import genAngles
from rospy import Publisher, init_node, Rate, loginfo, ROSInterruptException, is_shutdown
from std_msgs.msg import Float32MultiArray

#HOME#
dataFrame = DataFrame(read_csv("~/catkin_ws/src/motors/scripts/testCoordinates.csv", sep=","))

#WORK#
#dataFrame = DataFrame(read_csv("U:\\Documents\\GitCode\\Robot-Arm\\testCoordinates.csv", sep=","))

def motorMaster():
    '''
    The Master Node defined as a function. Using a multiarray message, the 4 angles are calculated
    by the functions brought in from motorControls and then published to a single topic. Each motor
    pulls its respective command from that multiarray.
    '''
    angPub = Publisher('motAngs', Float32MultiArray, queue_size=10)
    init_node('master', anonymous=True)

    while not is_shutdown():
        for i in dataFrame.values:
            angles = genAngles(list(i))
            rate = Rate(.5)
            loginfo(angles)
            angPub.publish(data=angles)
            rate.sleep()

if __name__ == '__main__':
    try:
        motorMaster()
    except ROSInterruptException:
        pass
