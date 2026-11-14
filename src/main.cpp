#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <cmath>
#define _USE_MATH_DEFINES
#include "ik.h"
#include "utils.h"
#include "lidar.h"
#include "gait.h"

unsigned long stepTime{1000}; //step length in milliseconds
float t{stepLength(stepTime)};
/*float x{};
float yz{};
float z{};*/
float y{0};
float P0x{}; // Step P0 Swing, Stance X-axis
float P1x{}; // Step P1 Swing X-axis
float P2x{}; // Step P2 Swing X-axis
float P3x{}; // Step P3 Swing, Stance X-axis
float P0z{}; // Step P0 Swing Z-axis
float P1z{}; // Step P1 Swing Z-axis
float P2z{}; // Step P2 Swing Z-axis
float P3z{}; // Step P3 Swing Z-axis

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

void setup() {
  Serial.begin(115200);
	// Serial1.begin(115200, SERIAL_8N1, 5,4);
  delay(1000);
  Serial.println("ESP32 Connected");
  // lidarsetup();
  /*pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQUENCY);
  delay(10);*/
}

void loop() {
	lidarloop();
  
	/*Serial.println("X:");
	x = readFloat();
	Serial.println("Y: ");
	yz = readFloat();
	Serial.println("Z: ");
	z = readFloat();
		
	if (solveIK(x,z,yz)){
		Serial.print( "Servo1: ");
		Serial.println( servos.servo1);
		Serial.print( "Servo2: ");
		Serial.println( servos.servo2);
		Serial.print( "Servo3: ");
		Serial.println( servos.servo3);
	}
		
	else{
		Serial.println("Unable to calculate, try different.\n");
	}*/
	
	t = stepLength(stepTime);
	if(t < 0.5){
		float tB{t * 2}; 
		swing = {solveIK(bezierCurve(tB, P0x, P1x, P2x, P3x),bezierCurve(tB, P0z, P1z, P2z, P3z), y)};
		if(swing){
			Serial.print("Swing Servo1: ");
			Serial.println(servos.servo1);
			//setServoAngle(0,servos.servo1);
			Serial.print("Swing Servo2: ");
			Serial.println(servos.servo2);
			//setServoAngle(1,servos.servo2);
			Serial.print("Swing Servo3: ");
			Serial.println(servos.servo3);
			//setServoAngle(2,servos.servo3);
		}

	}
	else if(t >= 0.5){
		float tI{(t - 0.5) * 2};
		stance = {solveIK(linearInterpolation(tI,P3x, P0x), P0z, y)};
		if(stance){
			Serial.print("Stance Servo1: ");
			Serial.println(servos.servo1);
			//setServoAngle(0,servos.servo1);
			Serial.print("Stance Servo2: ");
			Serial.println(servos.servo2);
			//setServoAngle(1,servos.servo2);
			Serial.print("Stance Servo3: ");
			Serial.println(servos.servo3);
			//setServoAngle(2,servos.servo3);
		}
	}
}
