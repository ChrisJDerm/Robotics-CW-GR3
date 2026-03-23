#include <Arduino.h>
#include "Joint.h"

Joint::Joint(int off, int pin) {
    this->offset = off;
    this->pin = pin;
}

void Joint::init(){
    this->servo.attach(pin);
}

void Joint::writeDeg(int pos){
    this->jAngle = pos;
    servo.write(pos + offset);
}