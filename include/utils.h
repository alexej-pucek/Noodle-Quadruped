#pragma once

#define SERVOMIN 110
#define SERVOMAX 510
#define SERVO_FREQUENCY 50

// extern Adafruit_PWMServoDriver pwm;

extern bool swing;
extern bool stance;

void setServoAngle(uint8_t channel, int angle);
float radToDeg(float rad);
float readFloat();