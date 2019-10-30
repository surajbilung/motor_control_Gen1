#include "ros/ros.h"
#include "motors/DOFArray.h"
#include "DataHandle/DataHandle.h"
#include <sstream>

int main(int argc, char **argv) 
{
    
    // Initiate a new ROS Node named Talker
    ros::init(argc, argv, "master");
    // Create a node handle (Reference for new Node)
    ros::NodeHandle node;
    // Create Publisher with Topic chat_top that will
    // send a string msg
    ros::Publisher motAngs = node.advertise<motors::DOFArray>("motAngs", 1000);
    // Use ros::Rate to set the frequency of advertising
    ros::Rate rate(0.5); // One message per second

    while (ros::ok()) // Spin until keyboard interrupt (Ctrl + C)
    {
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