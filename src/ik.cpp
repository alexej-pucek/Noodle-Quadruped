#include <Arduino.h>
#include <cmath>
#define _USE_MATH_DEFINES
#include "ik.h"
#include "utils.h"

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
	float z1{ sqrt((yz * yz) + (z * z)) };
	float kl{ atan(yz / z) };
	float k{ radToDeg(kl) + 90 };
	
	float D{ sqrt((x * x) + (z1 * z1)) };
	//std::cout << "D: " << D << '\n';
	
	if (D > 150)
	{
		return false;
	}

	float a{ acos(((L2 * L2) + ((L3 - 30) * (L3 - 30)) - (D * D)) / (2 * L2 * (L3 - 30))) };
	//std::cout << "a: " << a << '\n';
	float b{ atan2(z1,x) };
	//std::cout << "b: " << b << '\n';
	float y{ acos(((D * D) + (L2 * L2) - ((L3 - 30) * (L3 - 30))) / (2 * D * L2)) };
	//std::cout << "y: " << y << '\n';
	float g{ b + y };
	//std::cout << "g: " << g << '\n';

	float Kx{ L2 * cos(g) };
	//std::cout << "Kx: " << Kx << '\n';
	float Kz{ L2 * sin(g) };
	//std::cout << "Kz: " << Kz << '\n';
	float n{ a + Tr };
	//std::cout << "n: " << n << '\n';
	float Tx{ R5 * cos(n) };
	//std::cout << "Tx: " << Tx << '\n';
	float Tz{ R5 * sin(n) };
	//std::cout << "Tz: " << Tz << '\n';
	float B{ sqrt(((Tx - Sx) * (Tx - Sx)) + ((Tz - Sz) * (Tz - Sz))) };
	//std::cout << "B: " << B << '\n';
	float o{ acos(((R1 * R1) + (B * B) - (R2 * R2)) / (2 * R1 * B)) };
	//std::cout << "o: " << o << '\n';
	float f{ atan2(Tx, Tz) };
	//std::cout << "f: " << f << '\n';
	float q{ f + o };
	//std::cout << "q: " << q << '\n';
	
	if (g < -90.0f || g > 90.0f || q < -90.0f || q > 90.0f)
	{
		return false;
	}

	if (g - q >= 60)
	{
		return false;
	}
	
	servos.servo1 = radToDeg(g);
	servos.servo2 = radToDeg(q);
	servos.servo3 = k;

	return true;
}



