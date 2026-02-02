#pragma once

class Kinematics {

public:

    struct Vec3 {
        float a;
        float b;
        float c;
    };

    // Static calculation functions
    static Vec3 Forward(float th1, float th2, float th3);
    static Vec3 Forward(Vec3 joints);

    static Vec3 Inverse(Vec3 pose);

    static inline float deg2Rad(float deg){
        return deg * DEG_TO_RAD;
    }

    static inline float rad2Deg(float rad){
        return rad * RAD_TO_DEG;
    }

private:
    Kinematics() = delete;

    // Joint distances
    static constexpr float L0 = 2;
    static constexpr float L1 = 5;
    static constexpr float L2 = 5;
    static constexpr float L3 = 5;
};