#ifndef DATAHANDLE_H_
#define DATAHANDLE_H_

class DataHandle{
private:
    float xComp, yComp, zComp;
public:
    DataHandle(float xComp, float yComp, float zComp);

    float baseAngle();
    void positionVectorDef(float *length, float *pvAng);
    bool positionVectorCheck();
    
    ~DataHandle(void);
};

#endif /* DATAHANDLE_H_ */