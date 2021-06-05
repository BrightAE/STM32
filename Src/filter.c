#include "filter.h"

float angle;
float a;

float ComplementaryFilter(float acc, float gyro, float dt) {
	a = 0.98;
	angle = a * (angle + gyro * dt) + (1 - a) * acc;
	return angle;
}
