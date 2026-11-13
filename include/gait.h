#pragma once

float stepLength(unsigned long stepTime);
float bezierCurve(float t, float P0, float P1, float P2, float P3);
float linearInterpolation(float t, float xstart, float xend);