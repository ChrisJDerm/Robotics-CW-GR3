#include "Robot.h"
#include <TimerOne.h>

#define POT1 A1
#define POT2 A2
#define POT3 A3

#define JOINT1_PIN 2
#define JOINT2_PIN 3
#define JOINT3_PIN 4
#define GRIPPER_PIN 11

#define OFF1 6
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

void Robot::jointMove(int theta1, int theta2, int theta3, bool PosGRP){
    J1.writeDeg(theta1);
    J2.writeDeg(theta2);
    J3.writeDeg(theta3);
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

void Robot::moveIK(float x, float y, float z, bool PosGRP){
    Kinematics::Vec3 joints = Kinematics::Inverse(Kinematics::Vec3{x, y, z});
    jointMove(joints.a, joints.b, joints.c, PosGRP);
}

void Robot::potMoveIK(bool PosGRP){
    int x = map(analogRead(POT1), 0, 1023, -25, 25);
    int y = map(analogRead(POT2), 0, 1023, -25, 25);
    int z = map(analogRead(POT3), 0, 1023, -5, 25);

    moveIK(x, y, z, PosGRP);
}

Kinematics::Vec3 Robot::getCurrentJoints(){
    Kinematics::Vec3 joints = {J1.getPos(), J2.getPos(), J3.getPos()};
    return joints;
}

Kinematics::Vec3 Robot::getCurrentPos(){
    Kinematics::Vec3 pose = Kinematics::Forward(getCurrentJoints());
    return pose;
}

void Robot::generateTrajectory(Kinematics::Vec3 end, float tf) {

    Kinematics::Vec3 start = getCurrentPos();

    const float dt = 0.1f;
    traj.tf = tf;
    traj.numPoints = (int)(tf / dt) + 1;

    if (traj.numPoints > 100) {
        traj.numPoints = 100;
    }

    float ax0 = start.a;
    float ax1 = 0.0f;
    float ax2 = 3.0f * (end.a - start.a) / (tf * tf);
    float ax3 = -2.0f * (end.a - start.a) / (tf * tf * tf);

    float ay0 = start.b;
    float ay1 = 0.0f;
    float ay2 = 3.0f * (end.b - start.b) / (tf * tf);
    float ay3 = -2.0f * (end.b - start.b) / (tf * tf * tf);

    float az0 = start.c;
    float az1 = 0.0f;
    float az2 = 3.0f * (end.c - start.c) / (tf * tf);
    float az3 = -2.0f * (end.c - start.c) / (tf * tf * tf);

    for (int i = 0; i < traj.numPoints; i++) {
        float t = i * dt;

        if (t > tf) {
            t = tf;
        }

        traj.points[i].a = ax0 + ax1 * t + ax2 * t * t + ax3 * t * t * t;
        traj.points[i].b = ay0 + ay1 * t + ay2 * t * t + ay3 * t * t * t;
        traj.points[i].c = az0 + az1 * t + az2 * t * t + az3 * t * t * t;
    }
}

void Robot::runTrajectory(bool PosGrp){
    unsigned long start = millis();
    unsigned long now = start;
    while (now - start <= traj.tf*1000.0 + 200)
    {
        now = millis();
        index = (now - start) / 100;
        float x = traj.points[index].a;
        float y = traj.points[index].b;
        float z = traj.points[index].c;

        Kinematics::Vec3 joints = Kinematics::Inverse(traj.points[index]);

        jointMove(joints.a, joints.b, joints.c, PosGrp);
        Serial.print("X: ");
        Serial.print(x);
        Serial.print(", Y: ");
        Serial.print(y);
        Serial.print(", Z: ");
        Serial.println(z);
    }

    // jointMove(Kinematics::Inverse(traj.points[index]));
}