#ifndef _KALMAN_H_
#define _KALMAN_H_

#include "sys.h"

float Kalman_Filter_X(float Accel,float Gyro);
float Kalman_Filter_Y(float Accel,float Gyro);
float yijiehubu(float angle_m, float gyro_m);
float Erjielvbo(float angle_m,float gyro_m);
#endif
