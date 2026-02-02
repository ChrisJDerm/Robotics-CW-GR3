#include "Arduino.h"
#include "Robot.h"
#include "Kinematics.h"

#define POT1 A1
#define POT2 A2
#define POT3 A3

#define JOINT1_PIN 2
#define JOINT2_PIN 3
#define JOINT3_PIN 4
#define GRIPPER_PIN 11

#define OFF1 0
#define OFF2 0
#define OFF3 0

#define GRP_OPEN 110
#define GRP_CLOSE 180

Robot::Robot() :
    J1(OFF1, JOINT1_PIN),
    J2(OFF2, JOINT2_PIN),
    J3(OFF3, JOINT3_PIN)
{}

void Robot::init(){
    J1.init();
    J2.init();
    J3.init();
    this->Grp.attach(GRIPPER_PIN);
}

void Robot::jointMove(int Pos1, int Pos2, int Pos3, bool PosGRP){
    J1.writeDeg(Pos1);
    J2.writeDeg(Pos2);
    J3.writeDeg(Pos3);
    grpMove(PosGRP);
}   

void Robot::potMove(){
    int ang1 = map(analogRead(POT1), 0, 1023, 0, 180);
    int ang2 = map(analogRead(POT2), 0, 1023, 0, 180);
    int ang3 = map(analogRead(POT3), 0, 1023, 0, 180);

    Robot::jointMove(ang1, ang2, ang3, true);
}

void Robot::grpMove(bool PosGRP){
    Grp.write(PosGRP ? GRP_OPEN : GRP_CLOSE);
}
