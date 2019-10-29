#include "ros/ros.h"
#include "motors/DOFArray.h"
#include "AxisMotor/AxisMotor.h"
#include <string>

#include <iostream>
using namespace std;

AxisMotor baseMot(17, 18, 1, 60, 0);

void callback(const motors::DOFArray::ConstPtr& msg)
{
    float str = msg->baseAng;
    ROS_INFO("[%s]\n", to_string(str));
    baseMot.moveToAngle(msg->baseAng);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "baseMot");
    ros::NodeHandle node;
    ros::Subscriber chat_top_sub = node.subscribe("motAngs", 1000, callback);
    ros::spin();

    return 0;
}