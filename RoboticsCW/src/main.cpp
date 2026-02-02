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
  // robot->potMove();
  robot->jointMove(0, 90, 90, true);
}