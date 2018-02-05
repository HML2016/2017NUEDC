#include "Control.h"
#include "pwm.h"
#include "scheduler.h"
#include "Kalman.h"
#include "MPU6050.h"
#include "Pixy.h"
#include "timer.h"
#include "servo.h"
#include "fuzzy.h"
#include "delay.h"
#include "key.h"

#define Servo1_max 1950//2035
#define Servo1_min 1350//1500
#define Servo2_max 1950//2035
#define Servo2_min 1400//1500

extern u16 Time;
ANGLE_t Angle;//定义角度结构体
MOTOR_t Motor;//定义电机输出结构体
PID_t Pid; 
PID_t Pidv;//速度环PID
float Delt_error;
float Delta_P;
u8 Length;
u32 StarTime,NowTime,NT;//系统时间 单位 ms
u32 time;
float Previous_Error[15];

void Angle_Calculate(void)//角度融合
{
	u8 i;
	for(i=0;i<3;i++)
	{
		MPU6050_getMotion6();//读取MPU6050原始值
		Angle.sum_x+=atan(MPU6050_Raw_Data.Accel_X/MPU6050_Raw_Data.Accel_Z)*57.2958f-Angle.Offset_x;
		Angle.sum_y+=atan(MPU6050_Raw_Data.Accel_Y/MPU6050_Raw_Data.Accel_Z)*57.2958f-Angle.Offset_y;
	}
	
	Angle.temp_x=Angle.sum_x/3.0f;
	Angle.temp_y=Angle.sum_y/3.0f;
	
	Angle.sum_x=Angle.sum_y=0;
	
	Angle.kalman_x=Kalman_Filter_X(Angle.temp_x,-MPU6050_Raw_Data.Gyro_Y);//卡尔曼滤波
	Angle.kalman_y=Kalman_Filter_Y(Angle.temp_y,MPU6050_Raw_Data.Gyro_X);
//	Angle.hubu_x=yijiehubu(Angle.temp_x,-MPU6050_Raw_Data.Gyro_Y);//一阶互补
//	Angle.hubu_y=yijiehubu(Angle.temp_y,MPU6050_Raw_Data.Gyro_X);
	
}
void Para_Init(void)//PID参数初始化
{
	/******************* X方向********************************/
	//速度环参数初始化
	Pidv.Kp_x=12;//抑制超调
	Pidv.Ki_x=2;//加快动态响应
	Pidv.Kd_x=8;//减慢速度 减小稳差
	Pidv.ErrI_max_x=200;	
	//  位置环参数初始化 
	Pid.Kp_x=10;
	Pid.Ki_x=1;
	Pid.Kd_x=100;
	Pid.ErrI_max_x=200;

	/******************* Y方向********************************/
		//速度环参数初始化
	Pidv.Kp_y=12;//抑制超调
	Pidv.Ki_y=2;//加快动态响应
	Pidv.Kd_y=8;//减慢速度 减小稳差
	Pidv.ErrI_max_y=200;	
	//  位置环参数初始化
  Pid.Kp_y=10;
	Pid.Ki_y=1;
	Pid.Kd_y=100;
	Pid.ErrI_max_y=200;

}
/*------------------------------------------
 函数功能:PID计算
 函数说明:坐标位置
------------------------------------------*/
void PID_Deal(u8 goalX,u8 goalY,u8 Stop)
{
	static u8 t1;
	static u8 Calculate_Length;
	static float Delta_D;	
	static u8 count=0;
	static float Now_err;
    static float Last_err_x,Last_err_y;
	static float Pre_err_x,Pre_err_y;
	static u8 Last_PosX=0,Last_PosY=0;
	
	Angle.Goal_x=goalX;//目标值
	Angle.Goal_y=goalY;//目标值
	
	count++;
	if(count>=200)
	{
		count=0;
		Pid.ErrI_sum_x=0;
		Pidv.ErrI_sum_x=0;
		Pid.ErrI_sum_y=0;
		Pidv.ErrI_sum_y=0;
	}//10 2 20  10 1 100
	/******************* X方向********************************/
	
	//位置环PID
	Pid.Err_x=Pixy_Color_Inf.Pixy_Color_PosX_Raw -Angle.Goal_x;//位置环误差
	Pid.ErrD_x=Pid.Err_x-Pid.Err_x_Pre;
	Pid.ErrI_sum_x=(Pid.ErrI_sum_x+Pid.Err_x)/2;//积分
	Pid.ErrI_sum_x =Pid.Ki_x*Pid.ErrI_sum_x;
	Pid.Out_x=-Pid.Kp_x*Pid.Err_x-Pid.ErrI_sum_x-Pid.Kd_x*Pid.ErrD_x;//位置环输出
	Pid.Err_x_Pre=Pid.Err_x;
	//速度环PID
	Pidv.Err_x=Pixy_Color_Inf.Pixy_Color_PosX_Raw-Last_PosX;
	Pidv.ErrP_sum_x=Pidv.Kp_x*Pidv.Err_x;	
	Pidv.ErrD_sum_x=(Pidv.Err_x-Last_err_x)*Pidv.Kd_x;
	Pidv.Out_x=Pidv.ErrP_sum_x-Pidv.ErrD_sum_x;
	Last_err_x=Pidv.Err_x;
	Last_PosX=Pixy_Color_Inf.Pixy_Color_PosX;
	
	Pidv.Out_x=Servo1_mid-4*Pidv.Out_x+Pid.Out_x;//速度环输出
	if(Pidv.Out_x >= Servo1_max)//输出限幅
		Pidv.Out_x = Servo1_max;
	if(Pidv.Out_x <= Servo1_min)
		Pidv.Out_x =Servo1_min;
	/******************* Y方向********************************/
	
	//位置环PID
	Pid.Err_y=Pixy_Color_Inf.Pixy_Color_PosY_Raw -Angle.Goal_y;//位置环误差
	Pid.ErrD_y=Pid.Err_y-Pid.Err_y_Pre;
	Pid.ErrI_sum_y=(Pid.ErrI_sum_y+Pid.Err_y)/2;//积分
	Pid.ErrI_sum_y =Pid.Ki_y*Pid.ErrI_sum_y;
	Pid.Out_y=-Pid.Kp_y*Pid.Err_y-Pid.ErrI_sum_y -Pid.Kd_y*Pid.ErrD_y;//位置环输出
	Pid.Err_y_Pre=Pid.Err_y;
	//速度环PID
	Pidv.Err_y=Pixy_Color_Inf.Pixy_Color_PosY_Raw-Last_PosY;
	Pidv.ErrP_sum_y=Pidv.Kp_y*Pidv.Err_y;	
	Pidv.ErrD_sum_y=(Pidv.Err_y-Last_err_y)*Pidv.Kd_y;
	Pidv.Out_y=Pidv.ErrP_sum_y-Pidv.ErrD_sum_y;
	Last_err_y=Pidv.Err_y;
	Last_PosY=Pixy_Color_Inf.Pixy_Color_PosY;

	Pidv.Out_y=Servo2_mid-4*Pidv.Out_y+Pid.Out_y;//速度环输出
	if(Pidv.Out_y >= Servo2_max)//输出限幅
		Pidv.Out_y = Servo2_max;
	if(Pidv.Out_y <= Servo2_min)
		Pidv.Out_y =Servo2_min;
	
	if((fabs(Pid.Err_x)<=9) && (fabs(Pid.Err_y)<=9) && Stop==1)
	{
		Pidv.Out_y=Servo2_mid;
	    Pidv.Out_x=Servo1_mid;
	}
}
void Question1(void)
{
	PID_Deal(Pixy_Color_Inf.Pixy_Color_PosX_Raw,Pixy_Color_Inf.Pixy_Color_PosY_Raw,1);
}
u8 Hole1_x=232,Hole1_y=171;
u8 Hole2_x=165,Hole2_y=172;
u8 Hole3_x=96,Hole3_y=168;
u8 Hole4_x=235,Hole4_y=101;
u8 Hole5_x=164,Hole5_y=99;
u8 Hole6_x=95,Hole6_y=99;
u8 Hole7_x=235,Hole7_y=34;
u8 Hole8_x=167,Hole8_y=30;
u8 Hole9_x=98,Hole9_y=33;
u8 HoleX[]={0,232,165,96,235,164,95,235,167,98
};
u8 HoleY[]={0,171,172,168,101,99,99,34,30,33
};
void Question2(void)
{
	if(fabs(Pixy_Color_Inf.Pixy_Color_PosX_Raw-Hole1_x)<=8 && 
		fabs(Pixy_Color_Inf.Pixy_Color_PosY_Raw-Hole1_y)<=8 )
		PID_Deal(Hole1_x-20,Hole1_y,0);
	else
		PID_Deal(Hole5_x-2,Hole5_y-2,1);
}

u8 flag1_4=0;
void Question3(void)
{
	static u8 time;
	if(flag1_4!=1)
	{
		PID_Deal(Hole4_x,Hole4_y,1);
		if(fabs(Pixy_Color_Inf.Pixy_Color_PosX_Raw-Hole4_x)<=7&& 
			fabs(Pixy_Color_Inf.Pixy_Color_PosY_Raw-Hole4_y)<=7)
			time++;
		else
			time=0;
		if(time>=150)//20ms周期 执行100次 约等于2s
			flag1_4=1;
	}
	else
		PID_Deal(Hole5_x-2,Hole5_y-2,1);
		
}
u8 Hole9_x_1=205,Hole9_y_1=100;
u8 Hole9_x_2=164,Hole9_y_2=64;
void Question4(void)
{
	if(fabs(Pixy_Color_Inf.Pixy_Color_PosX_Raw-Hole1_x)<=8 && 
		fabs(Pixy_Color_Inf.Pixy_Color_PosY_Raw-Hole1_y)<=8 )
	  PID_Deal(Hole1_x-15,Hole1_y,0);
	else if(Pixy_Color_Inf.Pixy_Color_PosX_Raw>Hole9_x_1+2&&Pixy_Color_Inf.Pixy_Color_PosX_Raw>Hole9_y_1)
	  PID_Deal(Hole9_x_1+5,Hole9_y_1,0);
	else if(Pixy_Color_Inf.Pixy_Color_PosX_Raw>Hole9_x_2+5&&Pixy_Color_Inf.Pixy_Color_PosX_Raw>Hole9_y_2)
	  PID_Deal(Hole9_x_2-5,Hole9_y_2-5,0);
	else PID_Deal(Hole9_x-3,Hole9_y-1,1);
}
u8 flag5_1=0,flag5_2=2,flag5_3=2;
void Question5(void)
{
if(flag5_1!=1)
{
	PID_Deal(Hole2_x-4,Hole2_y,0);
	if(fabs(Pixy_Color_Inf.Pixy_Color_PosX_Raw-Hole2_x)<=6 && 
			fabs(Pixy_Color_Inf.Pixy_Color_PosY_Raw-Hole2_y)<=5)
	{ 
		flag5_1=1;
	  flag5_2=1;
	}
}
	if(flag5_2==1)
	{
		PID_Deal(Hole6_x-3,Hole6_y,0);
		if(fabs(Pixy_Color_Inf.Pixy_Color_PosX_Raw-Hole6_x)<=6 && 
			fabs(Pixy_Color_Inf.Pixy_Color_PosY_Raw-Hole6_y)<=6)
		{
			flag5_2=0;
		  flag5_3=1;
		}
	}
	if(flag5_3==1)
		PID_Deal(Hole9_x-3,Hole9_y-3,1);
}
u8 PosX[9]={227,158,88,230,157,85,230,158,86};//9??X??
u8 PosY[9]={174,177,174,105,105,104,34,33,34};	//9??Y??
u8 adcX[4]={158,121,158,194};//4?adc???X??
u8 adcY[4]={141,105,69,105};	
u8 posA,posB,posC,posD;
u8 flagAB=1,flagBC=0,flagCD=0;
u8 flag1,flag2,flag3,flag4;
u8 flag6_1=0;
u8 flag6_2=0;
u8 flag6_3=0;
u8 flag6_4=0;
u8 flag6_5=0;
u8 Ball[4];
void Question6(void)
{
	static int ball=0;
	static int i=0;
	if(flag6_1==0)
	{
		while(KEY_add==0) 
		{
			ball++;
			while(KEY_add==0); 
		}
		while(KEY_minus==0) 
		{
			ball--;
			while(KEY_minus==0); 
		}
		while(KEY_reset==0)
		{
			Ball[i]=ball;
		  i++;
			while(KEY_reset==0);
		}
		OLED_ShowNum(36,0, ball,4,12);
		OLED_ShowNum(36000,16, i,4,12);	//显示通道值
		OLED_Refresh_Gram();
		if(i>=4)
		flag6_1=1;
	}
	else{	
		if(fabs(Pixy_Color_Inf.Pixy_Color_PosX_Raw-HoleX[Ball[0]])>=6 && 
			fabs(Pixy_Color_Inf.Pixy_Color_PosY_Raw-HoleY[Ball[0]])>=6&& flag6_2==0)
		PID_Deal(HoleX[Ball[0]],HoleY[Ball[0]],0);
		else if(fabs(Pixy_Color_Inf.Pixy_Color_PosX_Raw-HoleX[Ball[1]])>=6 && 
			fabs(Pixy_Color_Inf.Pixy_Color_PosY_Raw-HoleY[Ball[1]])>=6&& flag6_3==0)
		{
			PID_Deal(HoleX[Ball[1]],HoleY[Ball[1]],0);
			flag6_2=1;
		}
		else if(fabs(Pixy_Color_Inf.Pixy_Color_PosX_Raw-HoleX[Ball[2]])>=6 && 
			fabs(Pixy_Color_Inf.Pixy_Color_PosY_Raw-HoleY[Ball[2]])>=6&& flag6_4==0)
		{
			PID_Deal(HoleX[Ball[2]],HoleY[Ball[2]],0);
			flag6_3=1;
		}
		else if(fabs(Pixy_Color_Inf.Pixy_Color_PosX_Raw-HoleX[Ball[3]])>=6 && 
			fabs(Pixy_Color_Inf.Pixy_Color_PosY_Raw-HoleY[Ball[3]])>=6&& flag6_5==0)
		{
			PID_Deal(HoleX[Ball[3]],HoleY[Ball[3]],1);
			flag6_4=1;
		}
	}
}
u16 T=100;//100 40  45
u8 R=40;
float positionX,positionY;
u8 flag7_1=0;
u8 flag7_2=0;
u8 flag7_3=0;
u8 flag7_4=0;
void Question7(void)
{
	static u16 TIME=0;
  if(fabs(Pixy_Color_Inf.Pixy_Color_PosX_Raw-Hole4_x)<=4&&fabs(Pixy_Color_Inf.Pixy_Color_PosY_Raw-Hole4_y)<=5&& (flag7_1==0))
	{
		PID_Deal(Hole5_x,Hole5_y,0);
	}
	else if(flag7_2==0)
	{
		flag7_1=1;
		if(flag7_3==0)
			PID_Deal(Hole5_x+30,Hole5_y,0);
		if(Time/100%2==0)
			flag7_3=1;
		if(	flag7_3==1)
		{TIME++;
		  positionX=Hole5_x+R*cos(Time*2*Pi/T);
      positionY=Hole5_y+R*sin((Time+45)*2*Pi/T);
		  PID_Deal(positionX,positionY,1);
		}
		if(TIME>=2000)
		{
			flag7_4=1;
			flag7_2=1;
		}
	}
	else if(flag7_4==1&&flag7_2==1)
		PID_Deal(Hole9_x-4,Hole9_y-3,1);
}

void Question8(void)
{
	PID_Deal(Pixy_Color_Inf.Pixy_Color_PosX_b,Pixy_Color_Inf.Pixy_Color_PosY_b,0);
}
void Route_plan(u8 last_area,u8 next_area)
{
	if(last_area==1||last_area==2)//A?????		
	{
		flag1=flag4=1;
		if(next_area==1||next_area==2)//B?????
			PID_Deal(PosX[next_area],PosY[next_area],1);	
		else if(next_area==3||next_area==4)//B?????
		{
			if(flag1==1)
			{
				PID_Deal(adcX[0],adcY[0],1);
				if(fabs(Pixy_Color_Inf.Pixy_Color_PosX_Raw-adcX[0])<=8 && fabs(Pixy_Color_Inf.Pixy_Color_PosY_Raw-adcY[0])<=8)//?????
					flag1=0;
			}
			else
				PID_Deal(PosX[next_area],PosY[next_area],1);
		}
		else if(next_area==5||next_area==6)//B?????
		{
			if(flag1==1)
			{
				PID_Deal(adcX[0],adcY[0],1);
				if(fabs(Pixy_Color_Inf.Pixy_Color_PosX_Raw-adcX[0])<=8 && fabs(Pixy_Color_Inf.Pixy_Color_PosY_Raw-adcY[0])<=8)//?????
				{ 
					flag1=0;
					flag2=1;
				}
			}
			else if(flag2==1)
			{
				PID_Deal(adcX[1],adcY[1],1);
				if(fabs(Pixy_Color_Inf.Pixy_Color_PosX_Raw-adcX[1])<=8 && fabs(Pixy_Color_Inf.Pixy_Color_PosY_Raw-adcY[1])<=8)//?????
					flag2=0;
			}
			else
				PID_Deal(PosX[next_area],PosY[next_area],1);
		}
					
		else if(next_area==7||next_area==8)//B?????
		{
			if(flag4==1)
			{
				PID_Deal(adcX[2],adcY[2],1);
				if(fabs(Pixy_Color_Inf.Pixy_Color_PosX_Raw-adcX[2])<=8 && fabs(Pixy_Color_Inf.Pixy_Color_PosY_Raw-adcY[2])<=8)//?????
					flag4=0;
			}
			else
				PID_Deal(PosX[next_area],PosY[next_area],1);
		}
		else //B????
			PID_Deal(PosX[next_area],PosY[next_area],1);
	}
	else if(last_area==3||last_area==4)//A?????
	{
		flag1=flag3=1;
		if(next_area==1||next_area==2)//B?????
		{
			if(flag1==1)
			{
				PID_Deal(adcX[0],adcY[0],1);
				if(fabs(Pixy_Color_Inf.Pixy_Color_PosX_Raw-adcX[0])<=8 && fabs(Pixy_Color_Inf.Pixy_Color_PosY_Raw-adcY[0])<=8)//?????
					flag1=0;
			}
			else
				PID_Deal(PosX[next_area],PosY[next_area],1);
		}
					
		else if(next_area==3||next_area==4)//B?????
			PID_Deal(PosX[next_area],PosY[next_area],1);	
		else if(next_area==5||next_area==6)//B?????
		{
			if(flag2==1)
			{
				PID_Deal(adcX[1],adcY[1],1);
				if(fabs(Pixy_Color_Inf.Pixy_Color_PosX_Raw-adcX[1])<=8 && fabs(Pixy_Color_Inf.Pixy_Color_PosY_Raw-adcY[1])<=8)//?????
					flag2=0;
			}
			else
				PID_Deal(PosX[next_area],PosY[next_area],1);			
		}	
		else if(next_area==7||next_area==8)//B?????
		{
			if(flag2==1)
			{
				PID_Deal(adcX[1],adcY[1],1);
				if(fabs(Pixy_Color_Inf.Pixy_Color_PosX_Raw-adcX[1])<=8 && fabs(Pixy_Color_Inf.Pixy_Color_PosY_Raw-adcY[1])<=8)//?????
				{ 
					flag2=0;
					flag3=1;
				}
			}
			else if(flag3==1)
			{
				PID_Deal(adcX[2],adcY[2],1);
				if(fabs(Pixy_Color_Inf.Pixy_Color_PosX_Raw-adcX[2])<=8 && fabs(Pixy_Color_Inf.Pixy_Color_PosY_Raw-adcY[2])<=8)//?????
							 flag3=0;
			}
			else
				PID_Deal(PosX[next_area],PosY[next_area],1);
		}
		else //B????
			PID_Deal(PosX[next_area],PosY[next_area],1);	
			}
	else if(last_area==5||last_area==6)//A?????
	{
		flag2=flag3=1;
		if(next_area==1||next_area==2)//B?????
		{
			if(flag2==1)
			{
				PID_Deal(adcX[1],adcY[1],1);
				if(fabs(Pixy_Color_Inf.Pixy_Color_PosX_Raw-adcX[1])<=8 && fabs(Pixy_Color_Inf.Pixy_Color_PosY_Raw-adcY[1])<=8)//?????
				{ 
					 flag2=0;
					 flag1=1;
				}
			}
			else if(flag1==1)
			{
				PID_Deal(adcX[0],adcY[0],1);
				if(fabs(Pixy_Color_Inf.Pixy_Color_PosX_Raw-adcX[0])<=8 && fabs(Pixy_Color_Inf.Pixy_Color_PosY_Raw-adcY[0])<=8)//?????
					flag1=0;
			}
			else
				PID_Deal(PosX[next_area],PosY[next_area],1);
		}
					
		else if(next_area==3||next_area==4)//B?????
		{
			if(flag2==1)
			{
				PID_Deal(adcX[1],adcY[1],1);
				if(fabs(Pixy_Color_Inf.Pixy_Color_PosX_Raw-adcX[1])<=8 && fabs(Pixy_Color_Inf.Pixy_Color_PosY_Raw-adcY[1])<=8)//?????
					flag2=0;
			}
			else
				PID_Deal(PosX[next_area],PosY[next_area],1);	
			}				
		else if(next_area==5||next_area==6)//B?????
			PID_Deal(PosX[next_area],PosY[next_area],1);
		else if(next_area==7||next_area==8)//B?????
		{
			if(flag4==1)
			{
				PID_Deal(adcX[3],adcY[3],1);
				if(fabs(Pixy_Color_Inf.Pixy_Color_PosX_Raw-adcX[3])<=8 && fabs(Pixy_Color_Inf.Pixy_Color_PosY_Raw-adcY[3])<=8)//?????
					flag4=0;
			}
			else
				PID_Deal(PosX[next_area],PosY[next_area],1);	
		}				
		else //B????
			PID_Deal(PosX[next_area],PosY[next_area],1);	
	}			
	else//A?????
	{
		flag3=flag4=1;
		if(next_area==1||next_area==2)//B?????
		{
			if(flag4==1)
			{
				PID_Deal(adcX[3],adcY[3],1);
				if(fabs(Pixy_Color_Inf.Pixy_Color_PosX_Raw-adcX[3])<=8 && fabs(Pixy_Color_Inf.Pixy_Color_PosY_Raw-adcY[3])<=8)//?????
					flag4=0;
			}
			else
				PID_Deal(PosX[next_area],PosY[next_area],1);
		}
					
		else if(next_area==3||next_area==4)//B?????
		{
			if(flag4==1)
			{
				PID_Deal(adcX[3],adcY[3],1);
				if(fabs(Pixy_Color_Inf.Pixy_Color_PosX_Raw-adcX[3])<=8 && fabs(Pixy_Color_Inf.Pixy_Color_PosY_Raw-adcY[3])<=8)//?????
				{ 
					flag4=0;
					flag1=1;
				}
			}
			else if(flag1==1)
			{
				PID_Deal(adcX[0],adcY[0],1);
				if(fabs(Pixy_Color_Inf.Pixy_Color_PosX_Raw-adcX[0])<=8 && fabs(Pixy_Color_Inf.Pixy_Color_PosY_Raw-adcY[0])<=8)//?????
					flag1=0;
			}
			else
				PID_Deal(PosX[next_area],PosY[next_area],1);
		}	
		else if(next_area==5||next_area==6)//B?????
		{
			if(flag3==1)
			{
				PID_Deal(adcX[2],adcY[2],1);
				if(fabs(Pixy_Color_Inf.Pixy_Color_PosX_Raw-adcX[2])<=8 && fabs(Pixy_Color_Inf.Pixy_Color_PosY_Raw-adcY[2])<=8)//?????
					flag3=0;
			}
			else
				PID_Deal(PosX[next_area],PosY[next_area],1);			
		}	
		else if(next_area==7||next_area==8)//B?????
			 PID_Deal(PosX[next_area],PosY[next_area],1);
		else //B????
			PID_Deal(PosX[next_area],PosY[next_area],1);	
	}			
}
