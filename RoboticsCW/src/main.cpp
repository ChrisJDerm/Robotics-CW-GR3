#include <Arduino.h>
#include <Servo.h>
#include "Robot.h"

Robot* robot = new Robot();

void setup() {
  Serial.begin(115200);
  robot->init();
  delay(100);
}

void loop() {
  robot->jointMove(30, 45, 60, true);
}