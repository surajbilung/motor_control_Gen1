#ifndef AXISMOTOR_H_
#define AXISMOTOR_H_

#include <iostream>
#include <math.h>

using namespace std;

class AxisMotor{
private:

    int stepPin;
    int directionPin;
    float gearRatio;
    float rpm;
    float length;
    float memory = 0;

public:

    AxisMotor(int stepPin, int directionPin, float gearRatio, float rpm, float length);
    
    void gpioInit();
    int motStepAndDir(float angle);
    float gpioFreq(void);
    void moveToAngle(float angle);

    ~AxisMotor();
};

#endif /* AXISMOTOR_H_ */