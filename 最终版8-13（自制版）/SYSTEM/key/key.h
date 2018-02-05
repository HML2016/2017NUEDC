#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
									  
////////////////////////////////////////////////////////////////////////////////// 	 

/*下面的方式是通过直接操作库函数方式读取IO*/
#define KEY 		GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) //PE4
#define KEY_add 		GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_10)
#define KEY_minus		GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_11)
#define KEY_reset		GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_12)
#define KEY_ok          GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13)
/*下面方式是通过位带操作方式读取IO*/
/*
#define KEY 		PAin(0)   	//PE4
*/


#define KEY0_PRES 	1
#define KEY1_PRES	2
#define KEY2_PRES	3
#define WKUP_PRES   4

void KEY_Init(void);	//IO初始化
u8 KEY_Scan(u8);  		//按键扫描函数	

#endif
