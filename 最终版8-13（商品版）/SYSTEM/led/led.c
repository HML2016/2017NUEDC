#include "led.h" 
#include "delay.h"
/**********************************************
RED  PA1
GRE  PA2
***********************************************/

//��ʼ��PA1��PA2Ϊ�����.��ʹ���������ڵ�ʱ��		    
//LED IO��ʼ��
void LED_Init(void)
{    	 
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOFʱ��

  //GPIOF9,F10��ʼ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��
	
  GPIO_SetBits(GPIOA,GPIO_Pin_6 | GPIO_Pin_7);//GPIOF9,F10���øߣ�����

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

