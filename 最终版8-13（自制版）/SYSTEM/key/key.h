#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
									  
////////////////////////////////////////////////////////////////////////////////// 	 

/*����ķ�ʽ��ͨ��ֱ�Ӳ����⺯����ʽ��ȡIO*/
#define KEY 		GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) //PE4
#define KEY_add 		GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_10)
#define KEY_minus		GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_11)
#define KEY_reset		GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_12)
#define KEY_ok          GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13)
/*���淽ʽ��ͨ��λ��������ʽ��ȡIO*/
/*
#define KEY 		PAin(0)   	//PE4
*/


#define KEY0_PRES 	1
#define KEY1_PRES	2
#define KEY2_PRES	3
#define WKUP_PRES   4

void KEY_Init(void);	//IO��ʼ��
u8 KEY_Scan(u8);  		//����ɨ�躯��	

#endif
