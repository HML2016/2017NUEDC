#include "led.h" 
#include "delay.h"
/**********************************************
RED  PA1
GRE  PA2
***********************************************/

//初始化PA1和PA2为输出口.并使能这两个口的时钟		    
//LED IO初始化
void LED_Init(void)
{    	 
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOF时钟

  //GPIOF9,F10初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
	
  GPIO_SetBits(GPIOA,GPIO_Pin_6 | GPIO_Pin_7);//GPIOF9,F10设置高，灯灭

}
 void LED_mpu6050_ok(void)
 {
	LED_GRE=1;
	delay_ms(100);
	LED_GRE=0;
	delay_ms(100);
	LED_GRE=1;
	delay_ms(500); 
	LED_GRE=0;
 }
 void LED_mpu6050_error(void)
 {
	LED_RED=1;
	delay_ms(200);
	LED_RED=0;
	delay_ms(200);
	LED_RED=1;
	delay_ms(200); 
	LED_RED=0;
 }

