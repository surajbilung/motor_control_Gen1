#include "ros/ros.h"
#include "motors/DOFArray.h"
#include "AxisMotor/AxisMotor.h"
#include <sstream>

#include <iostream>
using namespace std;

AxisMotor toolMot(26, 20, 1, 60, 0);

void callback(const motors::DOFArray::ConstPtr& msg)
{
    float str = msg->toolAng;
    ROS_INFO("[%s]\n", to_string(str));
    toolMot.moveToAngle(msg->toolAng);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "tooMot");
    ros::NodeHandle node;
    ros::Subscriber chat_top_sub = node.subscribe("motAngs", 1000, callback);
    ros::spin();

    return 0;
}