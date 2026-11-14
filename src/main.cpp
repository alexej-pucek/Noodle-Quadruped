#include <Arduino.h>
#include <cmath>
#define _USE_MATH_DEFINES
#include "ik.h"
#include "utils.h"
#include "lidar.h"
#include "gait.h"

unsigned long stepTime{1000}; //step length in milliseconds
float t{stepLength(stepTime)};
float x{};
float yz{};
float z{};
float P0x{};
float P1x{};
float P2x{};
float P3x{};
float P0z{};
float P1z{};
float P2z{};
float P3z{};

void setup() {
  Serial1.begin(115200, SERIAL_8N1, 5,4);
  delay(1000);
  Serial.println("ESP32 Connected");
  lidarsetup();
}

void loop() {
	lidarloop();
  
	Serial.println("X:");
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
	}
	
	t = stepLength(stepTime);
	if(t < 0.5){
		float tB{t * 2}; 
		swing = {solveIK(bezierCurve(tB, P0x, P1x, P2x, P3x),bezierCurve(tB, P0z, P1z, P2z, P3z), yz)};
		if(swing){
			//write angles to servos
		}

	}
	else if(t >= 0.5){
		float tI{(t - 0.5) * 2};
		stance = {solveIK(linearInterpolation(tI,P3x, P0x), P0z, yz)};
		if(stance){
			//write angles to servos
		}
	}
}
