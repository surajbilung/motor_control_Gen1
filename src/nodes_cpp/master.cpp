#include "ros/ros.h"
#include "motors/DOFArray.h"
#include "DataHandle/DataHandle.h"
#include <sstream>

int xCoor[5] = {0, 2, 0, -2, 0};
int yCoor[5] = {2, 0, -2, 0, 2};
int zCoor[5] = {2, 2, 2, 2, 2};

int main(int argc, char **argv) {
    
    // Initiate a new ROS Node named Talker
    ros::init(argc, argv, "master");
    // Create a node handle (Reference for new Node)
    ros::NodeHandle node;
    // Create Publisher with Topic chat_top that will
    // send a string msg
    ros::Publisher motAngs = node.advertise<motors::DOFArray>("motAngs", 1000);
    // Use ros::Rate to set the frequency of advertising
    ros::Rate rate(2); // One message per second

    // Spin until keyboard interrupt (Ctrl + C)
    while (ros::ok()) {
        // Create a new msg (String) and create a string for the data
        motors::DOFArray msg;
        // Publish the message
        motAngs.publish(msg);
        // Allows ROS to process incoming messages
        ros::spinOnce();
        // Sleep for the cycle set by Rate class
        rate.sleep();
        }

    return 0;
}