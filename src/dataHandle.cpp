#include <iostream>
#include <math.h>

using namespace std;

class dataHandle {

public:

    float baseAngle(float xComp, float yComp){
       return (atan2(yComp, xComp)) * 180 / M_PI;
    }
    void positionVectorDef(float xComp, float yComp, float zComp, float *length, float *pvAng){
        float xPrime = sqrt(pow(xComp, 2) + pow(yComp, 2));
        *pvAng = (atan(zComp / xPrime)) * 180 / M_PI;
        *length = sqrt(pow(xPrime, 2) + pow(zComp, 2));
    }
    bool positionVectorCheck(float xComp, float yComp, float zComp){
        float comparisonValue = sqrt(pow(xComp, 2) + pow(yComp, 2) + pow(zComp, 2));
        return 2 <= comparisonValue <= 10;
    }
};

int main() {
    dataHandle point;

    float length, pvAng;

    float x = point.baseAngle(5.0, 12.0);
    point.positionVectorDef(4, 4, 4, &length, &pvAng);
    cout << "Base Angle: " << x << endl;
    cout << "PV Length: " << length << " PV Ang: " << pvAng << endl;
}