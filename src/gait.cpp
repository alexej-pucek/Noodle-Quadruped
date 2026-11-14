#include <Arduino.h>
#include <cmath>
#define _USE_MATH_DEFINES
#include "ik.h"
#include "utils.h"
#include "gait.h"
bool swing{};
bool stance{};

float stepLength(unsigned long stepTime){
    float phase{(millis() % stepTime) / (float)stepTime};
    return phase;
}

float bezierCurve(float t, float P0, float P1, float P2, float P3){
    float Pt0{(1.0f-t)*(1.0f-t)*(1.0f-t)*P0};
    float Pt1{3.0f*((1.0f-t)*(1.0f-t))*t*P1};
    float Pt2{3.0f*(1.0f-t)*(t*t)*P2};
    float Pt3{t*t*t*P3};
    return {Pt0 + Pt1 + Pt2 + Pt3};
}

float linearInterpolation(float t, float xstart, float xend){
    return xstart + (xend - xstart) * t;
}