#include <Arduino.h>
#include <cmath>
#define _USE_MATH_DEFINES
#include "ik.h"
#include "utils.h"
const float x{};
const float y{};
const float d{}; // Y-axis Y rotation pivot point offset
const float z{};
const float b{}; // Y-axis Z rotation pivot point offset
const float L2{};
const float L3{};
const float L4{};
const float s{};
const float L5{};
const float Sx{};
const float Sz{};
const float L7{};
const float L6{};

// Y Rotation Servo
float a{atan2f((y + d), (z + b))};
float h{sqrtf((y * y) + (z * z))};
float O3 = a;

// X, Z Rotation Servo1
float g{atan2f(h,x)};
float D{sqrtf((x*x) + (h*h))};
float i{arccosf(L2, D, (L3 -L4))};
float O1 = g + i;

//X, Z Rotation Servo2
float beta{arccosf(L2, (L3 -L4), D)};
float p{360 - radToDeg(beta + s + O1)};
float Kx{ L5 * cosf(p)};
float Kz{ L5 * sinf(p)};
float E{sqrtf(sqrf(Kx - Sx) + (Kz - Sz))};
float l{atan2f((Kz - Sz), (Kx - Sx))};
float n{arccosf(L7, E, L6)};
float O2 = l + n;// new Z height for other calculations



