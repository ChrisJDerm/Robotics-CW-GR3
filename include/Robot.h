#pragma once

#include "Joint.h"

class Robot {
public:
    Robot();

    void init();
    void jointMove(int Pos1, int Pos2, int Pos3, bool PosGRP);
    void servoWrite(Servo joint, int pos);
    void potMove();
    void grpMove(bool PosGRP);

private:
    Joint J1;
    Joint J2;
    Joint J3;
    Servo Grp;

    int GRP_OPEN;
    int GRP_CLOSE;
};