#include <Arduino.h>
#include "Kinematics.h"

Kinematics::Vec3 Kinematics::Forward(float th1, float th2, float th3){
    Kinematics::Vec3 joints;
    joints.a = constrain(th1, 0, 180);
    joints.b = constrain(th2, 0, 180);
    joints.c = constrain(th3, 0, 180);
    return Kinematics::Forward(joints);
}

Kinematics::Vec3 Kinematics::Forward(Kinematics::Vec3 joints){
    const float t1 = deg2Rad(joints.a);
    const float t2 = deg2Rad(joints.b);
    const float t3 = deg2Rad(joints.c);

    const float s1  = sinf(t1);
    const float c1  = cosf(t1);
    const float s2  = sinf(t2);
    const float c2  = cosf(t2);
    const float s23 = sinf(t2 + t3);
    const float c23 = cosf(t2 + t3);

    Kinematics::Vec3 toolPose;

    toolPose.a = (L1 * s1) + (L2 * c1 * c2) + (L3 * c1 * s23);   // x
    toolPose.b = (-L1 * c1) + (L2 * s1 * c2) + (L3 * s1 * s23);  // y
    toolPose.c = (L2 * s2) - (L3 * c23);                         // z

    return toolPose;
}