#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 

extern u8 Rx_Buf[];
void Usart3_Init(u32 br_num);
void Usart3_IRQ(void);
void Usart3_Send(unsigned char *DataToSend ,u8 data_num);
#endif


