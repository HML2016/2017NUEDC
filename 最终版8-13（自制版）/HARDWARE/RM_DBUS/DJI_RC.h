#ifndef __DJI_RC_H
#define	__DJI_RC_H	   
#include "sys.h"
#define  RC_Value	1024

typedef struct
{
	struct
	{
	uint16_t ch1;
	uint16_t ch2;
	uint16_t ch3;
	uint16_t ch4;
	uint8_t s1;
	uint8_t s2;
	}rc;
	struct
	{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t press_l;
	uint8_t press_r;
	}mouse;
	struct
	{
	uint16_t v;
	}key;
}RC_Ctl_t;

extern RC_Ctl_t RC_Ctl;
void RC_Init(void);
#endif
