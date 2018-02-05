#ifndef __BLDC_H
#define	__BLDC_H	   
#include "sys.h"

#define BLDC_DirA PBout(13)

void BLDC_Init(void);
void BLDC_Star(void);
void BLDC_Stop(void);
void BLDC_Setspeed(u8 Hz);
#endif
