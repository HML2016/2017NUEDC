#ifndef __PWM_H
#define	__PWM_H	   
#include "sys.h"

void PWM_Init(void);
void TIM4_PWM_Init(u16 Hz);
void TIM3_PWM_Init(u16 Hz);
void Motor_Out(int16_t pwm_tem1,int16_t pwm_tem2,int16_t pwm_tem3,int16_t pwm_tem4);
#endif
