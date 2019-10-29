#include <iostream>
#include <thread>
#include <chrono>
#include <math.h>

#include "AxisMotor.h"

using namespace std;
/*
A Motor will be defined by the GPIO pins for control, the gear ratio of the local axis, the rpm
of the motor, and the length of the local arm. The methods will compound into a greater method.

The motor will move the the angle passed to the moveToAngle method. If you are at 90 degrees and 
feed the function 180 the motor will move 90 degrees counterclockwise.
*/

//Constructor
AxisMotor::AxisMotor(int stepPin, int directionPin, float gearRatio, float rpm, float length){
    this->stepPin = stepPin;
    this->directionPin = directionPin;
    this->gearRatio = gearRatio;
    this->rpm = rpm;
    this->length = length;
}

//Destructor
AxisMotor::~AxisMotor(void){}

//Initialize the GPIO Pins
void AxisMotor::gpioInit(){
    cout << "Direction Pin : " << directionPin << endl;
    cout << "Step Pin : " << stepPin << endl;
}

//Determine the amount of steps from the angle as well as the direction
int AxisMotor::motStepAndDir(float angle){
    int steps = ((angle - memory) / 360) * 800;
    if (steps < 0){
        steps = abs(steps);
        cout << "Moving " << steps << ", Clockwise" << endl;
    }
    else{
        cout << "Moving " << steps << ", CounterClockwise" << endl;
    }
    return steps;
}

//Determining the time between rising and falling edges on pusedo pwm
float AxisMotor::gpioFreq(){
    return 1 / ((rpm * 800) / 60);
}

//Moves TO angle passed to the method then remembers it's current angle
void AxisMotor::moveToAngle(float angle){
    int steps = motStepAndDir(angle * gearRatio);
    int sleepTime = gpioFreq() * 1000;
    while (steps > 0)
    {
        //step high
        this_thread::sleep_for(chrono::milliseconds(sleepTime));
        //step low
        this_thread::sleep_for(chrono::milliseconds(sleepTime));
        steps = steps - 1;
    }
    memory = angle;
    this_thread::sleep_for(chrono::milliseconds(1000));
}