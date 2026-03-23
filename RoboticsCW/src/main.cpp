#include <Arduino.h>
#include <Servo.h>
#include "Robot.h"
#include "Kinematics.h"

Robot* robot = new Robot();

void setup() {
  Serial.begin(115200);
  robot->init();
  delay(100);
  pinMode(7, INPUT);
}

void loop() {
  if (digitalRead(7) == LOW)
  {
    robot->pickAndPlace({-8.7, -20.6, -5}, {13.8, -14.1, -5}, 5, 1000);
  }
}