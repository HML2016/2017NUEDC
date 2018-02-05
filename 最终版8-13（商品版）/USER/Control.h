#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "sys.h"

typedef struct
{
	float sum_x;
    float sum_y;
	float temp_x;//中间变量 加速度计拟合
	float temp_y;
	float kalman_x;//卡尔曼滤波后角度
	float kalman_y;
	float speed_x;//角速度
	float speed_y;
	float hubu_x;//互补滤波后角度
	float hubu_y;
	float Offset_x;//偏差
	float Offset_y;
	float Goal_x;//理论上目标角度
	float Goal_y;
	float Current_x;//当前测量到的角度
	float Current_y;
}ANGLE_t;

typedef struct
{
	float Kp_x;
	float Ki_x;
	float Kd_x;
	float Kp_y;
	float Ki_y;
	float Kd_y;
	float Err_x;
	float Err_x_Pre;
	float Err_y;
	float Err_y_Pre;
	float ErrP_sum_x;
	float ErrI_sum_x;
    float ErrD_sum_x;
	float ErrP_sum_y;
	float ErrI_sum_y;
	float ErrD_sum_y;
	float ErrI_max_x;
	float ErrI_max_y;
	float ErrD_x;
	float ErrD_y;
	float Out_x;
	float Out_y;
}PID_t;

typedef struct
{
	int16_t num1;//1号电机PWM值
	int16_t num2;
	int16_t num3;
	int16_t num4;
}MOTOR_t;

extern ANGLE_t Angle;
extern MOTOR_t Motor;
extern PID_t Pid;
extern PID_t Pidv;
extern u8 posA,posB,posC,posD;
extern u32 StarTime,NowTime,NT;
extern float positionX,positionY;

void Para_Init(void);
void Angle_Calculate(void);
void Question1(void);
void Question2(void);
void Question3(void);
void Question4(void);
void Question5(void);
void Question6(void);
void Question7(void);
void Question8(void);
void Route_plan(u8 last_area,u8 next_area);



#endif
