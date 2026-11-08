#pragma once
struct ServoData{
    float servo1;
    float servo2;
    float servo3;
};

bool solveIK(float x, float z, float yz);

extern ServoData servos;