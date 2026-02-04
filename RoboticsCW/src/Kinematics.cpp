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

    const float s1 = sinf(t1);
    const float c1 = cosf(t1);
    const float s2 = sinf(t2);
    const float c2 = cosf(t2);

    const float s3p = sinf(t3 + M_PI_2);
    const float c3p = cosf(t3 + M_PI_2);

    Kinematics::Vec3 toolPose;

    toolPose.a =
          L3 * ( c1*c2*s3p - c1*c3p*s2 )
        + L2 * c1*c2
        - L3 * ( c2*s1*s3p - c3p*s1*s2 )
        - L2 * c2*s1;

    toolPose.b =
          L3 * ( c2*s1*s3p - c3p*s1*s2 )
        + L2 * s1*c2
        + L3 * ( c1*c2*s3p - c1*c3p*s2 )
        + L2 * c1*c2;

    toolPose.c =
          L3 * ( c2*c3p + s2*s3p )
        + L2 * s2;

    return toolPose;
}
