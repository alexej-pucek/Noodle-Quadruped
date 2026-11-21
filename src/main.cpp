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
float P0x{60}; // Step P0 Swing, Stance X-axis
float P1x{15}; // Step P1 Swing X-axis
float P2x{-5}; // Step P2 Swing X-axis
float P3x{-50}; // Step P3 Swing, Stance X-axis
float P0z{140}; // Step P0 Swing Z-axis
float P1z{120}; // Step P1 Swing Z-axis
float P2z{120}; // Step P2 Swing Z-axis
float P3z{140}; // Step P3 Swing Z-axis
float SwX{};
float StX{};
float SwZ{};
float StZ{};


void setup() {
  Serial.begin(115200);
	// Serial1.begin(115200, SERIAL_8N1, 5,4);
  delay(1000);
  Serial.println("ESP32 Connected");
  // lidarsetup();
  Wire.begin(4,5);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQUENCY);
  delay(10);
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
		Serial.println("Swing");
		float tB{t * 2};
		SwX = bezierCurve(tB, P0x, P1x, P2x, P3x);
		SwZ = bezierCurve(tB, P0z, P1z, P2z, P3z);
		swing = {solveIK(SwX,SwZ, y)};
		if(swing == false){
			Serial.println("FALSEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE");
			Serial.print("Swing: ");
			Serial.print(SwX);
			Serial.print("   ");
			Serial.println(SwZ);
		}
		if(swing){
			
			// Front Right Leg
			Serial.print("Swing: ");
			Serial.print(SwX);
			Serial.print("   ");
			Serial.println(SwZ);
			Serial.print("Swing Servo1: ");
			Serial.println( 80.0f - servos.servo1);
			setServoAngle(0, 80.0f - servos.servo1);
			Serial.print("Swing Servo2: ");
			Serial.println(servos.servo2 + 90.0f);
			setServoAngle(1,servos.servo2 + 90);
			Serial.print("Swing Servo3: ");
			Serial.println(servos.servo3);
			setServoAngle(2,servos.servo3);
			
			// Front Left Leg
			Serial.print("Swing: ");
			Serial.print(SwX);
			Serial.print("   ");
			Serial.println(SwZ);
			Serial.print("Swing Servo1: ");
			Serial.println( 200.0f - servos.servo1);
			setServoAngle(3, 200.0f - servos.servo1);
			Serial.print("Swing Servo2: ");
			Serial.println( 90.0f - servos.servo2);
			setServoAngle(4, 90.0f - servos.servo2);
			Serial.print("Swing Servo3: ");
			Serial.println(servos.servo3);
			setServoAngle(5,servos.servo3);

			// Back Right Leg
			Serial.print("Swing: ");
			Serial.print(SwX);
			Serial.print("   ");
			Serial.println(SwZ);
			Serial.print("Swing Servo1: ");
			Serial.println( 80.0f - servos.servo1);
			setServoAngle(6, 80.0f - servos.servo1);
			Serial.print("Swing Servo2: ");
			Serial.println(servos.servo2 + 90.0f);
			setServoAngle(7,servos.servo2 + 90);
			Serial.print("Swing Servo3: ");
			Serial.println(servos.servo3);
			setServoAngle(8,servos.servo3);

			// Back Left Leg
			Serial.print("Swing: ");
			Serial.print(SwX);
			Serial.print("   ");
			Serial.println(SwZ);
			Serial.print("Swing Servo1: ");
			Serial.println( 80.0f - servos.servo1);
			setServoAngle(9, 80.0f - servos.servo1);
			Serial.print("Swing Servo2: ");
			Serial.println(servos.servo2 + 90.0f);
			setServoAngle(10,servos.servo2 + 90);
			Serial.print("Swing Servo3: ");
			Serial.println(servos.servo3);
			setServoAngle(11,servos.servo3);



		}

	}
	else if(t >= 0.5){
		Serial.println("Stance");
		float tI{(t - 0.5) * 2};
		StX = linearInterpolation(tI,P3x, P0x);
		StZ = P0z;
		stance = {solveIK(StX, StZ, y)};
		if(stance == false){
			Serial.println("FALSEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE");
			Serial.print("Stance: ");
			Serial.print(StX);
			Serial.print("   ");
			Serial.println(StZ);
		}
		if(stance){
			
			// Front Right Leg
			Serial.print("Stance: ");
			Serial.print(StX);
			Serial.print("   ");
			Serial.println(StZ);
			Serial.print("Stance Servo1: ");
			Serial.println(80.0f - servos.servo1);
			setServoAngle(0, 80.0f - servos.servo1);
			Serial.print("Stance Servo2: ");
			Serial.println(servos.servo2 + 90);
			setServoAngle(1,servos.servo2 + 90.0f);
			Serial.print("Stance Servo3: ");
			Serial.println(servos.servo3);
			setServoAngle(2,servos.servo3);

			// Front Left Leg
			Serial.print("Stance: ");
			Serial.print(StX);
			Serial.print("   ");
			Serial.println(StZ);
			Serial.print("Stance Servo1: ");
			Serial.println(200.0f - servos.servo1);
			setServoAngle(3, 200.0f - servos.servo1);
			Serial.print("Stance Servo2: ");
			Serial.println( 90.0f - servos.servo2);
			setServoAngle(4, 90.0f - servos.servo2);
			Serial.print("Stance Servo3: ");
			Serial.println(servos.servo3);
			setServoAngle(5,servos.servo3);

			// Back Right Leg
			Serial.print("Stance: ");
			Serial.print(StX);
			Serial.print("   ");
			Serial.println(StZ);
			Serial.print("Stance Servo1: ");
			Serial.println(200.0f - servos.servo1);
			setServoAngle(6, 200.0f - servos.servo1);
			Serial.print("Stance Servo2: ");
			Serial.println( 90.0f - servos.servo2);
			setServoAngle(7, 90.0f - servos.servo2);
			Serial.print("Stance Servo3: ");
			Serial.println(servos.servo3);
			setServoAngle(8,servos.servo3);

			// Back Left Leg
			Serial.print("Stance: ");
			Serial.print(StX);
			Serial.print("   ");
			Serial.println(StZ);
			Serial.print("Stance Servo1: ");
			Serial.println(200.0f - servos.servo1);
			setServoAngle(9, 200.0f - servos.servo1);
			Serial.print("Stance Servo2: ");
			Serial.println( 90.0f - servos.servo2);
			setServoAngle(10, 90.0f - servos.servo2);
			Serial.print("Stance Servo3: ");
			Serial.println(servos.servo3);
			setServoAngle(11,servos.servo3);
		}
	}
}
