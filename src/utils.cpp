#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
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

 /*void setServoAngle(uint8_t channel, int angle){
    angle = {constrain(angle,0,180)};
    int pulse{map(angle, 0, 180, SERVOMIN, SERVOMAX)};
    pwm.setPWM(channel, 0, pulse);
}*/