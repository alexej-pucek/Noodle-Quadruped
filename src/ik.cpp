#include <Arduino.h>
#include <cmath>
#define _USE_MATH_DEFINES

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

float radToDeg(float rad)
{
	return 180 / pi * rad;
}

float pi{M_PI};
float Servo1{};
float Servo2{};
float Servo3{};
float x{};
float z{};
float yz{};

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
	


	Servo1 = radToDeg(g);
	Servo2 = radToDeg(q);
	Servo3 = k;
	return true;
}

int main()
{
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
			std::cout << "Servo1: " << Servo1 << " Servo2: " << Servo2 << "Servo3: " << Servo3 << '\n';
		}
		
		else
		{
			std::cout << "Unable to calculate, try different.\n";
		}
	}
}

