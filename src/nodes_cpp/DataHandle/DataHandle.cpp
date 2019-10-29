#include <iostream>
#include <math.h>

#include "DataHandle.h"

using namespace std;

DataHandle::DataHandle(float xComp, float yComp, float zComp){
    this->xComp = xComp;
    this->yComp = yComp;
    this->zComp = zComp;
}

float DataHandle::baseAngle(){
    return (atan2(yComp, xComp)) * 180 / M_PI;
}

void DataHandle::positionVectorDef(float *length, float *pvAng){
    float xPrime = sqrt(pow(xComp, 2) + pow(yComp, 2));
    *pvAng = (atan(zComp / xPrime)) * 180 / M_PI;
    *length = sqrt(pow(xPrime, 2) + pow(zComp, 2));
}

bool DataHandle::positionVectorCheck(){
    float comparisonValue = sqrt(pow(xComp, 2) + pow(yComp, 2) + pow(zComp, 2));
    return 2 <= comparisonValue <= 10;
    }

DataHandle::~DataHandle(void){
    cout << "Destroying Class ... " << endl;
}