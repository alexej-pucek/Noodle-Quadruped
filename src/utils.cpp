#include <Arduino.h>
#include <cmath>
#define _USE_MATH_DEFINES
#include "utils.h"

float pi{M_PI};

float radToDeg(float rad)
{
	return 180 / pi * rad;
}

float readFloat() {
    while (true) {
        if (Serial.available() > 0) {
            String input = Serial.readStringUntil('\n');
            input.trim();
            
            if (input.length() > 0) {
                return input.toFloat();
            }
        }
    }
}