#include <Arduino.h>
#include <cmath>
#define _USE_MATH_DEFINES
#include "ik.h"
#include "utils.h"

float x{};
float yz{};
float z{};

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32 Connected");
}

void loop() {
  while (true)
	{
		Serial.println("X:");
		x = readFloat();
		Serial.println("Y: ");
		yz = readFloat();
		Serial.println("Z: ");
		z = readFloat();
		
		if (solveIK(x,z,yz))
		{
			Serial.print( "Servo1: ");
			Serial.println( servos.servo1);
			Serial.print( "Servo2: ");
			Serial.println( servos.servo2);
			Serial.print( "Servo3: ");
			Serial.println( servos.servo3);
		}
		
		else
		{
			Serial.println("Unable to calculate, try different.\n");
		}
	}
}
