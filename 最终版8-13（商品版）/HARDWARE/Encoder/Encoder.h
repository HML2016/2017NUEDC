#ifndef __ENCODER_H
#define	__ENCODER_H	   
#include "sys.h"
typedef struct
{
	int32_t Num_a;
	int32_t Num_b;
}ENCODER;

extern ENCODER Encoder;

void Encoder_configuration(void);
void Quad_Encoder_Configuration(void);
void GetQuadEncoderDiff(void);

#endif
