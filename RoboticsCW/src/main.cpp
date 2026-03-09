#include <Arduino.h>
#include <Servo.h>
#include "Robot.h"
#include "Kinematics.h"

Robot* robot = new Robot();

void setup() {
  Serial.begin(115200);
  robot->init();
  delay(100);
}

void loop() {
  robot->moveIK(0, 0, 26, true);
}