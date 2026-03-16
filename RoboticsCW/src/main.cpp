#include <Arduino.h>
#include <Servo.h>
#include "Robot.h"
#include "Kinematics.h"

Robot* robot = new Robot();

void setup() {
  Serial.begin(115200);
  robot->init();
  delay(100);

  robot->jointMove(90, 90, 90, true);
  delay(1000);
  robot->generateTrajectory({0, -13.5, -8}, 5);
  robot->runTrajectory(true);
  delay(1000);
  robot->grpMove(false);
  delay(1000);
  robot->generateTrajectory(Kinematics::Forward({90, 90, 90}), 5);
  robot->runTrajectory(false); 
  delay(1000);
  robot->generateTrajectory({15, 0, -5}, 5);
  robot->runTrajectory(false);
  delay(1000);
  robot->grpMove(true);
  delay(1000);
  robot->generateTrajectory(Kinematics::Forward({90, 90, 90}), 5);
  robot->runTrajectory(true); 
}

void loop() {
}