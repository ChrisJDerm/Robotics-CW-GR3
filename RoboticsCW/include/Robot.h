#pragma once

#include "Joint.h"

class Robot {
public:
    Robot();

    void init();
    void jointMove(int theta1, int theta2, int theta3, bool PosGRP);
    // void servoWrite(Servo joint, int pos);
    void potMove();
    void grpMove(bool PosGRP);
    void moveIK(int x, int y, int z, bool PosGRP);
    void potMoveIK(bool PosGRP);

private:
    Joint J1;
    Joint J2;
    Joint J3;
    Servo Grp;
};