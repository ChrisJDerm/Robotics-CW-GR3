#include <Arduino.h>
#include <Servo.h>
#include "Robot.h"
#include "Kinematics.h"

#include <TimerOne.h>

Robot* robot = new Robot();

void setup() {
  Serial.begin(115200);
  robot->init();
  delay(100);

  // Timer1.attachInterrupt(robot->trajTimerCallback);

  robot->jointMove(90, 90, 90, false);
  delay(1000);
  robot->generateTrajectory({15, -10, 5}, 5);
  robot->runTrajectory(false);
}

void loop() {
}