#ifndef __PIXY_H
#define __PIXY_H	 
#include "sys.h"


typedef struct Pixy_Color//单色块位置大小信息
{
	u16 Pixy_Color_Sig;//1-7 for normal signatures
	float Pixy_Color_PosX;  //0 to 319
	float Pixy_Color_PosY;  //0 to 319
	float Pixy_Color_Width; //1 to 320
	float Pixy_Color_Height;//1 to 320
	u16 Pixy_Color_Sig_Raw;//1-7 for normal signatures
	u16 Pixy_Color_PosX_Raw;  //0 to 319
	u16 Pixy_Color_PosY_Raw;  //0 to 319
	u16 Pixy_Color_Width_Raw; //1 to 320
	u16 Pixy_Color_Height_Raw;//1 to 320
	u16 Pixy_Color_Sig_b;//1-7 for normal signatures
	u16 Pixy_Color_PosX_b;  //0 to 319
	u16 Pixy_Color_PosY_b;  //0 to 319
	u16 Pixy_Color_Width_b; //1 to 320
	u16 Pixy_Color_Height_b;//1 to 320
}Pixy_Color;


typedef struct Pixy_ColorCode//色条位置大小信息
{
	u16 Pixy_ColorCode_Sig;//Same as follow
	u16 Pixy_ColorCode_PosX;
	u16 Pixy_ColorCode_PosY;
	u16 Pixy_ColorCode_Width;
	u16 Pixy_ColorCode_Height;
	u16 Pixy_ColorCode_Angle;//The angle of the object detected object only when the detected object is a color code
}Pixy_ColorCode;

extern Pixy_Color Pixy_Color_Inf;

 void Pixy_Uart_Init(void);
void Pixy_DataSave(int16_t s,int16_t x,int16_t y,int16_t w,int16_t h) ;
 void Pixy_SPI_Init(void);
 void Pixy_SPI_SendData(u8 data1);
 void Pixy_SPI_ReadData(void);


	
#endif
