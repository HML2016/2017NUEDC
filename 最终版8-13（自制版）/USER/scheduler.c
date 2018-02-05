/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * ����   �������ƴ�
 * �ļ���  ��scheduler.c
 * ����    ���������
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
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
		loop.err_flag ++;     //ÿ�ۼ�һ�Σ�֤��������Ԥ��������û�����ꡣ
	}
	else
	{	
		loop.check_flag = 1;	//�ñ�־λ��ѭ�����������
	}
	
}

/******************************** �� �� ˵ �� ***********************************/

/********************************************************************************/
void Duty_1ms()
{
	
	ANO_DT_Data_Exchange();	//����ͨ�Ŷ�ʱ����
}

/******************************** �� �� ˵ �� ***********************************/

/********************************************************************************/

void Duty_2ms()
{
}

/******************************** �� �� ˵ �� ***********************************/

/********************************************************************************/
void Duty_5ms()
{
	
}

/******************************** �� �� ˵ �� ***********************************/

/********************************************************************************/
void Duty_10ms()
{

}

/******************************** �� �� ˵ �� ***********************************/

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

/******************************** �� �� ˵ �� ***********************************/

/********************************************************************************/
void Duty_50ms()
{
	if(Flag_star==1)
	{
		LED_GRE=!LED_GRE;//��������ָʾ��
	Servo1_Setspeed(Pidv.Out_x);//ִ�����
	Servo2_Setspeed(Pidv.Out_y);//ִ�����
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
			StarTime=Get_Systime()/1000;//��ȡ��ʼʱ�� ��λ ms
		}
	if(Flag_star==0)
	{
		if(KEY_add!=1) mode++;
		if(KEY_minus!=1) mode--;
		if(KEY_reset!=1) mode=0;
	}
}

void Duty_Loop()   					//�����������Ϊ1ms���ܵĴ���ִ��ʱ����ҪС��1ms��
{

	if( loop.check_flag == 1 )
	{
		Duty_1ms();							//����1ms������
		
		if( loop.cnt_2ms >= 2 )
		{
			loop.cnt_2ms = 0;
			Duty_2ms();						//����2ms������
		}
		if( loop.cnt_5ms >= 5 )
		{
			loop.cnt_5ms = 0;
			Duty_5ms();						//����5ms������
		}
		if( loop.cnt_10ms >= 10 )
		{
			loop.cnt_10ms = 0;
			Duty_10ms();					//����10ms������
		}
		if( loop.cnt_20ms >= 20 )
		{
			loop.cnt_20ms = 0;
			Duty_20ms();					//����20ms������
		}
		if( loop.cnt_50ms >= 50 )
		{
			loop.cnt_50ms = 0;
			Duty_50ms();					//����50ms������
		}
		if( loop.cnt_100ms >= 100 )
		{
			loop.cnt_100ms = 0;
			Duty_100ms();					//����100ms������
		}
		if( loop.cnt_200ms >= 200 )
		{
			loop.cnt_200ms = 0;
			Duty_200ms();					//����200ms������
		}
		loop.check_flag = 0;		//ѭ��������ϱ�־
	}
}
