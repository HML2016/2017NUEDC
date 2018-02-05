#ifndef _IIC_SW_H
#define	_IIC_SW_H

#include "sys.h"
#include "time.h"

void IIC_GPIO_Init(void);
int IIC_WriteData(u8 dev_addr,u8 reg_addr,u8 data);
int IIC_ReadData(u8 dev_addr,u8 reg_addr,u8 *pdata,u8 count);
void HEAT_Configuration(void);

#endif
