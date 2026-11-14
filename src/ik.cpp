#include <Arduino.h>
#include <cmath>
#define _USE_MATH_DEFINES
#include "ik.h"
#include "utils.h"

// Parallelogram Linkage Leg Parameters

const float Dy{34.7f};
const float R1{ 40.105f };
const float R2{ 27.113f };
const float R5{ 30.0f };
const float L1{ 99.988f };
const float L2{ 100.0f };
const float L3{ 130.0f };
const float Sx{ 22.0f };
const float Sz{ 20.3f };
const float Tr{ asin( (14.314f / 2) / 30.0f) };

ServoData servos{0.0f, 0.0f, 0.0f};

bool solveIK(float x, float z, float yz)
{
	// Y-axis movement
	float z1{ sqrt((yz * yz) + (z * z)) };
	float kl{ atan(yz / z) };
	float k{ radToDeg(kl) + 90 }; // Y Servo Angle in Degrees
	
	// Servo1(Bottom X,Z-axis) Movement
	float D{ sqrt((x * x) + (z1 * z1)) };
	
	if (D > 150){
		return false; // Leg Reach Checke
	}

	float a{ acos(((L2 * L2) + ((L3 - 30) * (L3 - 30)) - (D * D)) / (2 * L2 * (L3 - 30))) };
	
	float b{ atan2(z1,x) };
	
	float y{ acos(((D * D) + (L2 * L2) - ((L3 - 30) * (L3 - 30))) / (2 * D * L2)) };
	
	float g{ b + y }; // Servo1 (Bottom) Angle in Radians	

	// Servo2 (Top X,Z-axis) Movement
	float Kx{ L2 * cos(g) };
	
	float Kz{ L2 * sin(g) };
	
	float n{ a + Tr };
	
	float Tx{ R5 * cos(n) };
	
	float Tz{ R5 * sin(n) };
	
	float B{ sqrt(((Tx - Sx) * (Tx - Sx)) + ((Tz - Sz) * (Tz - Sz))) };
	
	float o{ acos(((R1 * R1) + (B * B) - (R2 * R2)) / (2 * R1 * B)) };
	
	float f{ atan2(Tx, Tz) };
	
	float q{ f + o }; // Servo2 (Top) Angle in Radians
	
	
	if (radToDeg(g) < -90.0f || radToDeg(g) > 90.0f || radToDeg(q) < -90.0f || radToDeg(q) > 90.0f){
		return false;
	}

	if (std::isnan(g) || std::isnan(q) || std::isnan(k) ){
		Serial.println("Error at calculating IK, value NaN appeared.");
		return false;
	}

	if (radToDeg(g) - radToDeg(q) >= 60){
		return false;
	}
	
	servos.servo1 = radToDeg(g);
	servos.servo2 = radToDeg(q);
	servos.servo3 = k;

	return true;
}
