#include <Arduino.h>
#include <cmath>
#define _USE_MATH_DEFINES
#include "ik.h"
#include "utils.h"

const float y{};
const float d{}; // Y-axis Y rotation pivot point offset
const float z{};
const float b{}; // Y-axis Z rotation pivot point offset

// Y Rotation Servo
float a{atanf((y + d)/ (z + b))};
float h{sqrtf((y * y) + (z * z))}; // new Z height for other calculations



