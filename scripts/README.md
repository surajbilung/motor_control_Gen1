# Robot-Arm
Raspberry Pi Robotic Arm Program

This project uses Python and ROS on a Raspberry Pi to control the base, first arm, second arm, and tool arm motors of a robotic arm. Each
motor is controlled using an A4988 motor driver and is set to a constant step interval of 1/4 step resolution. The design intent is to
feed the software a list of coordinates and have the arm solve for what angles to position the motors at so that the head is now located
in that position. The functions will reference a file of "correction factors" which will take into account the geometry and positional
requirements of whatever tool head is attached to the arm.

This is my first ROS project and, while it doesn't utilize a lot of ROS' built in features, it has been great practice for developing my
workspaces, getting nodes to communicate, and managing make and launch files. I encourage you to clone this repo and play around if you
are looking to learn ROS or if you want to try and break the code to give me some improvement points. THANKS!
