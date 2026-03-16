#pragma once

#include <Servo.h>

class Joint {
public:
    Joint(int off, int pin);
    void init();
    void writeDeg(int pos);

    inline float getPos() {return jAngle;}; 
private:
    Servo servo;
    int offset;
    int pin;

    float jAngle;
};