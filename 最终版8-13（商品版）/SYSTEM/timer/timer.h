#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"

void  SysTick_Configuration(void);
uint32_t Get_Systime(void);
void TIM2_Int_Init(u32 arr,u16 psc);
#endif
