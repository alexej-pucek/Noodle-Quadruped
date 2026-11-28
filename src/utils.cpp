#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <cmath>
#define _USE_MATH_DEFINES
#include "utils.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

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

void setServoAngle(uint8_t channel, int angle){
    angle = {constrain(angle,0,180)};
    int pulse{map(angle, 0, 180, SERVOMIN, SERVOMAX)};
    pwm.setPWM(channel, 0, pulse);
}

float clamp(float val){
    if(val > 1.0f){
        return 1.0f;
    }
    if(val < -1.0f){
        return -1.0f;
    }
    return val;
}

float arccosf(float adj1, float adj2, float hyp){
    return acosf(clamp(((adj1 * adj1) + (adj2 * adj2) - ((hyp) * (hyp))) / (2.0f * adj1 * adj2)));
}

float sqrf(float a){
    return (a * a);
}