#pragma once

#include "Arduino.h"
#include "Joint.h"
#include "Kinematics.h"

class Robot {
public:

    struct Trajectory
    {
        Kinematics::Vec3 points[100];
        int numPoints;
        float tf; //seconds
    };

    Robot();

    void init();
    void jointMove(int theta1, int theta2, int theta3, bool PosGRP);
    // void servoWrite(Servo joint, int pos);
    void potMove();
    void grpMove(bool PosGRP);
    void moveIK(float x, float y, float z, bool PosGRP);
    void potMoveIK(bool PosGRP);

    Kinematics::Vec3 getCurrentJoints();
    Kinematics::Vec3 getCurrentPos();

    void generateTrajectory(Kinematics::Vec3 end, float tf);

    void runTrajectory(bool PosGrp);

    void linearMove(Kinematics::Vec3 end, float tf, bool PosGrp);

    void pickAndPlace(Kinematics::Vec3 pick, Kinematics::Vec3 place, 
                                int pathTime, int delayTime);

private:
    Joint J1;
    Joint J2;
    Joint J3;
    Servo Grp;

    Kinematics::Vec3 joints;
    Kinematics::Vec3 pose;

    Trajectory traj;
};