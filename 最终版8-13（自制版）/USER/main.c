#include "sys.h"
#include "scheduler.h"
#include "DJI_RC.h"
#include "delay.h"
#include "usart.h"
#include "timer.h"
#include "led.h"
#include "oled.h"
#include "pwm.h"
#include "Encoder.h"
#include "MPU6050.h"
#include "Pixy.h"
#include "Control.h"
#include "imu.h"
#include "BLDC.h"
#include "servo.h"
#include "key.h"

u8 Flag_star=0;
int t=0;
u8 flag0=1;
u16 Time=0;
int main(void)
{ 
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);     //初始化延时函数
	Usart3_Init(500000);	 //初始化数传串口
	//MPU6050_Initialize();//初始化MPU6050 20Hz低通滤波 成功返回0
	//Quad_Encoder_Configuration();//初始化编码器
	//RC_Init();           //初始化遥控器
	Servo_Init(); //初始舵机 100Hz 
	Pixy_Uart_Init();//初始化PIXY串口接收
	PWM_Init();//初始化PWM波
	LED_Init();					//初始化LED
 	OLED_Init();				//初始化OLED
	SysTick_Configuration();//初始化滴答定时器 1ms中断
	Servo1_Setspeed(Servo1_mid);//中值1200-1630-2060
	Servo2_Setspeed(1750);//1670/1750//1790
	Para_Init();
	OLED_show_init();
	while(1) 
	{	
		Duty_Loop();	
	}
}
