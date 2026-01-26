#include <Arduino.h>
#include <Servo.h>
#include "Robot.h"

Robot* robot = new Robot();

void setup() {
  Serial.begin(115200);
  robot->init();
  delay(3000); // Wait 5 seconds before going into loop()
}

void loop() {
  robot->potMove();
}