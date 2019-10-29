#include "ros/ros.h"
#include "motors/DOFArray.h"
#include "AxisMotor/AxisMotor.h"
#include <string>

#include <iostream>
using namespace std;

AxisMotor mainMot(22, 23, 1, 60, 7.5);

void callback(const motors::DOFArray::ConstPtr& msg)
{
    float str = msg->mainAng;
    ROS_INFO("[%s]\n", to_string(str));
    mainMot.moveToAngle(msg->mainAng);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "mainMot");
    ros::NodeHandle node;
    ros::Subscriber chat_top_sub = node.subscribe("motAngs", 1000, callback);
    ros::spin();

    return 0;
}