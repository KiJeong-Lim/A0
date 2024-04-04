#ifndef CAPSTONE_HPP
#define CAPSTONE_HPP

#include <cmath>
#include "mbed.h"

#define PI 3.14159265359f
#define CAN_ID 0x01

float fmaxf(float x, float y);
float fminf(float x, float y);
float fmaxf3(float x, float y, float z);
float fminf3(float x, float y, float z);
void limit_norm(float *x, float *y, float limit);
int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);

struct UCh8 { unsigned char data[8]; };

#endif
