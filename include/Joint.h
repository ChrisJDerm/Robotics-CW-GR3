#pragma once

#include <Servo.h>

class Joint {
public:
    Joint(int off, int pin);
    void init();
    void writeDeg(int pos);
private:
    Servo servo;
    int offset;
    int pin;
};