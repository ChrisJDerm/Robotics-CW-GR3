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

    const float s1 = sinf(t1), c1 = cosf(t1);
    const float s2 = sinf(t2), c2 = cosf(t2);
    const float s3 = sinf(t3), c3 = cosf(t3);

    Kinematics::Vec3 toolPose;

    // x = L2*c1*c2*c3 + L2*c1*s2*s3 + L1*c1*c2
    toolPose.a = L2*c1*c2*c3 + L2*c1*s2*s3 + L1*c1*c2;

    // y = L2*s1*c2*c3 + L2*s1*s2*s3 + L1*s1*c2
    toolPose.b = L2*s1*c2*c3 + L2*s1*s2*s3 + L1*s1*c2;

    // z = -L2*s2*c3 + L2*c2*s3 - L1*s2
    toolPose.c = -L2*s2*c3 + L2*c2*s3 - L1*s2;

    return toolPose;
}

Kinematics::Vec3 Kinematics::Inverse(Kinematics::Vec3 toolPose){
    const float x  = toolPose.a;
    const float yp = toolPose.b;   // y'
    const float zp = toolPose.c;   // z'

    // theta1 from new-frame relation y' = -s1 * r, x = c1 * r
    const float t1 = atan2f(-yp, x);

    // Planar reduction (u,v) = (r, z')
    const float u = sqrtf(x*x + yp*yp);
    const float v = zp;

    const float d = sqrtf(u*u + v*v);

    // Circle-circle intersection parameters
    const float a_len = (L1*L1 - L2*L2 + d*d) / (2.0f * d);
    const float h2    = L1*L1 - a_len*a_len;
    const float h     = sqrtf(fmaxf(h2, 0.0f));

    // Point along the line from origin to target
    const float pu = a_len * u / d;
    const float pv = a_len * v / d;

    // Perpendicular offset (choose this branch as "sol1")
    const float ou = -h * v / d;
    const float ov =  h * u / d;

    // Elbow point P1 (sol1 branch)
    const float u1 = pu + ou;
    const float v1 = pv + ov;

    // Recover theta2 directly from elbow point (this is the corrected part)
    const float t2 = atan2f(v1, u1);

    // Direction of link2 in (u,v)
    const float alpha = atan2f(v - v1, u - u1);

    // For your NEW-frame FK: theta3 = theta2 - alpha
    const float t3 = t2 - alpha;

    Kinematics::Vec3 joints;
    joints.a = rad2Deg(t1);
    joints.b = rad2Deg(t2);
    joints.c = rad2Deg(t3);

    return joints;
}