#include "pwm.h"

#define ACCURACY 10000 //u16(2500/0.25) //accuracy
#define INIT_DUTY 0 //死区
#define PWM_RADIO 10//(8000 - 4000)/1000.0

void PWM_Init(void)
{
	TIM4_PWM_Init(1000);
	//TIM3_PWM_Init(1000);
}

void TIM4_PWM_Init(u16 Hz)
{		 					 
	u16 PrescalerValue = 0;
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  	//TIM4时钟使能
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//使能PORTB时钟	
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4); //GPIOB6复用为定时器4
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_TIM4);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;           
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure);              //初始化PB
	
	PrescalerValue = (uint16_t) ( ( SystemCoreClock/2) / (ACCURACY*Hz) ) - 1;
	
	TIM_TimeBaseStructure.TIM_Prescaler=PrescalerValue;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=ACCURACY;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=0; 
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);//初始化定时器4
	
	//初始化TIM4 Channel1 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 CH1
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);  
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);  
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);  
	
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM4在CCR1上的预装载寄存器
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
 
    TIM_ARRPreloadConfig(TIM4,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM4, ENABLE);  //使能TIM4									  
} 

void TIM3_PWM_Init(u16 Hz)
{		 					 
	u16 PrescalerValue = 0;
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  	//TIM8时钟使能
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//使能PORTC时钟	
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM3); //GPIOC复用为定时器8
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM3);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure);              //初始化PC
	
	PrescalerValue = (uint16_t) ( ( SystemCoreClock/2) / (ACCURACY*Hz) ) - 1;
	
	TIM_TimeBaseStructure.TIM_Prescaler=PrescalerValue;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=ACCURACY;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=0;    //设置时钟分割TDTS=Tck_tim
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//初始化定时器8
	
	//初始化TIM8 Channel PWM	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	//TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Set;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM8 CH1
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);  
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);  
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);  
	
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM8在CCR1上的预装载寄存器
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
 
    TIM_ARRPreloadConfig(TIM3,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM3, ENABLE);  //使能TIM8									  
} 
/*********************************************************************************
 函数功能:设置PWM占空比
 函数说明:定时器寄存器比较值
*********************************************************************************/
void Motor_Out(int16_t pwm_tem1,int16_t pwm_tem2,int16_t pwm_tem3,int16_t pwm_tem4)
{

	if(pwm_tem1>=0)
	{
		TIM4->CCR1 = PWM_RADIO *( pwm_tem1 ) + INIT_DUTY;				//1
		TIM4->CCR2 = 0;				//2	
	}
	else
	{
		TIM4->CCR1 = 0;				//3
		TIM4->CCR2 = PWM_RADIO *( pwm_tem1 ) + INIT_DUTY;				//4
	}
	if(pwm_tem2>=0)
	{
		TIM4->CCR3 = PWM_RADIO *( pwm_tem2 ) + INIT_DUTY;				
		TIM4->CCR4 = 0;				
	}
	else
	{
		TIM4->CCR3 = 0;				
		TIM4->CCR4 = PWM_RADIO *( pwm_tem2) + INIT_DUTY;				
	}
	if(pwm_tem3>=0)
	{
		TIM3->CCR1 = PWM_RADIO *( pwm_tem3 ) + INIT_DUTY;				
		TIM3->CCR2 = 0;					
	}
	else
	{
		TIM3->CCR1 = 0;				
		TIM3->CCR2 = PWM_RADIO *( pwm_tem3 ) + INIT_DUTY;				
	}
	if(pwm_tem4>=0)
	{
		TIM3->CCR3 = PWM_RADIO *( pwm_tem4 ) + INIT_DUTY;				
		TIM3->CCR4 = 0;					
	}
	else
	{
		TIM3->CCR3 = 0;				
		TIM3->CCR4 = PWM_RADIO *( pwm_tem4 ) + INIT_DUTY;				
	}
		
}
