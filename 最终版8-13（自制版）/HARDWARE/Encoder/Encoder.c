#include "Encoder.h"

ENCODER Encoder;
/**********************************
	Encoder1  `	TIM3 	PA6
				TIM3 	PA7
	Encoder2 	TIM12	PB14  TIM12不支持正交编码功能
				TIM12	PB15
************************************/


void Quad_Encoder_Configuration(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&gpio);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3);
	
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);
	TIM3->CNT = 0x7fff;
    TIM_Cmd(TIM3,ENABLE);

}

 
void GetQuadEncoderDiff(void)
{
    int32_t cnt = 0; 
	cnt = (TIM3->CNT)-0x7fff;
	//TIM3->CNT = 0x7fff;    	
    Encoder.Num_a=cnt;
}
