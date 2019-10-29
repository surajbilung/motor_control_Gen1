#include "ros/ros.h"
#include "motors/DOFArray.h"
#include "AxisMotor/AxisMotor.h"
#include <string>

#include <iostream>
using namespace std;

AxisMotor secMot(19, 16, 1, 60, 4.75);

void callback(const motors::DOFArray::ConstPtr& msg)
{
    float str = msg->secAng;
    ROS_INFO("[%s]\n", to_string(str));
    secMot.moveToAngle(msg->secAng);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "secMot");
    ros::NodeHandle node;
    ros::Subscriber chat_top_sub = node.subscribe("motAngs", 1000, callback);
    ros::spin();

    return 0;
}