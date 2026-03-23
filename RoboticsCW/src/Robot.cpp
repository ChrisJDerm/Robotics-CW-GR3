#include "Robot.h"
#include <TimerOne.h>

#define POT1 A1
#define POT2 A2
#define POT3 A3

#define JOINT1_PIN 2
#define JOINT2_PIN 3
#define JOINT3_PIN 4
#define GRIPPER_PIN 11

#define OFF1 8
#define OFF2 0
#define OFF3 0

#define GRP_OPEN 90
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
    joints = {J1.getPos(), J2.getPos(), J3.getPos()};
    pose = Kinematics::Forward(joints);
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
    return joints;
}

Kinematics::Vec3 Robot::getCurrentPos(){
    return pose;
}

void Robot::generateTrajectory(Kinematics::Vec3 end, float tf) {

    tf = constrain(tf, 0, 5);

    Kinematics::Vec3 start = getCurrentPos();

    Serial.print("Start Pose - X: ");
    Serial.print(start.a);
    Serial.print(", Y: ");
    Serial.print(start.b);
    Serial.print(", Z: ");
    Serial.println(start.c);

    const float dt = 0.05f;
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
    int index = 0;
    while (now - start <= traj.tf*1000.0)
    {
        now = millis();
        if (index < traj.numPoints)
        {
            index = (now - start) / 50;
            float x = traj.points[index].a;
            float y = traj.points[index].b;
            float z = traj.points[index].c;

            moveIK(x, y, z, PosGrp);
        } else {
            break;
        }
    }
    moveIK(traj.points[traj.numPoints - 1].a, 
        traj.points[traj.numPoints - 1].b, 
        traj.points[traj.numPoints - 1].c, 
        PosGrp);
}

void Robot::linearMove(Kinematics::Vec3 end, float tf, bool PosGrp){
    generateTrajectory(end, tf);
    runTrajectory(PosGrp);
}

void Robot::pickAndPlace(Kinematics::Vec3 pick, Kinematics::Vec3 place, 
                                int pathTime, int delayTime){

    // Home                                
    jointMove(90, 90, 90, true);
    delay(delayTime);

    // Move to pick                                
    linearMove(pick, pathTime, true);
    delay(delayTime);

    // Pick                                
    grpMove(false);
    delay(delayTime);

    // Home                                
    linearMove(Kinematics::Forward({90, 90, 90}), pathTime, false);
    delay(delayTime);

    // Move to place                                
    linearMove(place, pathTime, false);
    delay(delayTime);

    // Place                                
    grpMove(true);
    delay(delayTime);

    // Clear place vertically                 
    linearMove({place.a, place.b, place.c + 5}, pathTime/2, true);
    delay(delayTime);

    //Home
    linearMove(Kinematics::Forward({90, 90, 90}), pathTime, true);
}