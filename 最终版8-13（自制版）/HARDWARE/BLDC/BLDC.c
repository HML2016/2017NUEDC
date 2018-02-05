#include "BLDC.h"

void BLDC_Init(void)
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  	//TIM8时钟使能
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//使能PORTC时钟	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM3); //GPIOC复用为定时器3
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure);              //初始化PC
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure);              //初始化PC
	GPIO_SetBits(GPIOB,GPIO_Pin_13);
		
	TIM_TimeBaseStructure.TIM_Prescaler=84-1;  //定时器分频 TIM3时钟为84MHz
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=10000;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=0;    //设置时钟分割TDTS=Tck_tim
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//初始化定时器8
	
	//初始化TIM8 Channel PWM	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM8 CH1
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM8在CCR1上的预装载寄存器
 
    TIM_ARRPreloadConfig(TIM3,ENABLE);//ARPE使能 								  
} 
void BLDC_Star(void)
{
	TIM_Cmd(TIM3, ENABLE);  //使能TIM3	
}
void BLDC_Stop(void)
{
	TIM_Cmd(TIM3, DISABLE);  //使能TIM3	
}
void BLDC_Setspeed(u8 Hz)
{
	TIM3->ARR=100*Hz;
	TIM3->CCR1=50*Hz;
}
