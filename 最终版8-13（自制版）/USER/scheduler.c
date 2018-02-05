/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：scheduler.c
 * 描述    ：任务调度
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/
#include "scheduler.h"
#include "mymath.h"
#include "time.h"
#include "timer.h"
#include "pwm.h"
#include "Encoder.h"
#include "imu.h"
#include "key.h"
#include "MPU6050.h"
#include "Control.h"
#include "Pixy.h"
#include "data_transfer.h"
#include "usart.h"
#include "GPIO_Test.h"
#include "led.h" 
#include "oled.h"
#include "servo.h"
extern u16 Time;
s16 loop_cnt;
loop_t loop;
u8 mode=5;

void Loop_check()  //TIME INTTERRUPT
{
	loop.time++; //u16
	loop.cnt_2ms++;
	loop.cnt_5ms++;
	loop.cnt_10ms++;
	loop.cnt_20ms++;
	loop.cnt_50ms++;
	loop.cnt_100ms++;
	loop.cnt_200ms++;
	
	if( loop.check_flag == 1)
	{
		loop.err_flag ++;     //每累加一次，证明代码在预定周期内没有跑完。
	}
	else
	{	
		loop.check_flag = 1;	//该标志位在循环的最后被清零
	}
	
}

/******************************** 任 务 说 明 ***********************************/

/********************************************************************************/
void Duty_1ms()
{
	
	ANO_DT_Data_Exchange();	//数传通信定时调用
}

/******************************** 任 务 说 明 ***********************************/

/********************************************************************************/

void Duty_2ms()
{
}

/******************************** 任 务 说 明 ***********************************/

/********************************************************************************/
void Duty_5ms()
{
	
}

/******************************** 任 务 说 明 ***********************************/

/********************************************************************************/
void Duty_10ms()
{

}

/******************************** 任 务 说 明 ***********************************/

/********************************************************************************/
void Duty_20ms()
{
	if(Flag_star==1)
		{
			switch(mode)
			{
				case 1: Question1();break;
				case 2: Question2();break;
				case 3: Question3();break;
				case 4: Question4();break;
				case 5: Question5();break;
				case 6: Question6();break;
				case 7: Question7();break;
				case 8: Question8();break;
				default:break;
			}
		}
}

/******************************** 任 务 说 明 ***********************************/

/********************************************************************************/
void Duty_50ms()
{
	if(Flag_star==1)
	{
		LED_GRE=!LED_GRE;//正常运行指示灯
	Servo1_Setspeed(Pidv.Out_x);//执行输出
	Servo2_Setspeed(Pidv.Out_y);//执行输出
	}
}
void Duty_100ms()
{
	Time++;
	OLED_show();
}
void Duty_200ms()
{
	
		if(KEY!=1)
		{
			Flag_star=1;
			StarTime=Get_Systime()/1000;//获取开始时间 单位 ms
		}
	if(Flag_star==0)
	{
		if(KEY_add!=1) mode++;
		if(KEY_minus!=1) mode--;
		if(KEY_reset!=1) mode=0;
	}
}

void Duty_Loop()   					//最短任务周期为1ms，总的代码执行时间需要小于1ms。
{

	if( loop.check_flag == 1 )
	{
		Duty_1ms();							//周期1ms的任务
		
		if( loop.cnt_2ms >= 2 )
		{
			loop.cnt_2ms = 0;
			Duty_2ms();						//周期2ms的任务
		}
		if( loop.cnt_5ms >= 5 )
		{
			loop.cnt_5ms = 0;
			Duty_5ms();						//周期5ms的任务
		}
		if( loop.cnt_10ms >= 10 )
		{
			loop.cnt_10ms = 0;
			Duty_10ms();					//周期10ms的任务
		}
		if( loop.cnt_20ms >= 20 )
		{
			loop.cnt_20ms = 0;
			Duty_20ms();					//周期20ms的任务
		}
		if( loop.cnt_50ms >= 50 )
		{
			loop.cnt_50ms = 0;
			Duty_50ms();					//周期50ms的任务
		}
		if( loop.cnt_100ms >= 100 )
		{
			loop.cnt_100ms = 0;
			Duty_100ms();					//周期100ms的任务
		}
		if( loop.cnt_200ms >= 200 )
		{
			loop.cnt_200ms = 0;
			Duty_200ms();					//周期200ms的任务
		}
		loop.check_flag = 0;		//循环运行完毕标志
	}
}
