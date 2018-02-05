/*************************************ʹ��˵��***********************************/
//����ʹ��CMUCAM5(Pixy)����
//STM32F407�ײ����
//�ṩ�����жϽ����SPI��ѯ�������
//�����ˣ�����
//QQ:1596311719
//2015/9/20
/****************************************END*************************************/

#include "Pixy.h"
#include "led.h"
#include "Control.h"
#include "timer.h"

Pixy_Color Pixy_Color_Inf;//�ṹ������
u8 Raw_Data[40];//ԭʼ���ݴ����Ϊ�⴮׼��
u8 counter;//������


/**********************STM32F4XX UART1��ʼ������********************************/
//PA9��PA10����ΪUART1
//����λ8��ֹͣλ1��������λ,������19200
//ʹ�ô����жϷ�ʽ,��ʹ�ÿ���ע�͵���6�д���
/*******************************END*********************************************/
void Pixy_Uart_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1
	
	//USART1�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = 500000;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
    USART_Init(USART1, &USART_InitStructure); //��ʼ������1
	
    USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1 
	
	USART_ClearFlag(USART1, USART_FLAG_TC);
	
	//ʹ���жϷ�ʽ
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//��������ж�
	//Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
}

/***********************STM32F4XX UART1�жϴ�����******************************/
//˼��:
//����40����Ч����
//�������Ұ�ͷ55aa��55aa��
//�ҵ�����Ч���ݸ���Pixy_Color_Inf�ṹ��

//ע�⣺
//1.���ڽ��հ��������£�
//          ��ͷ     55 AA 55(56) AA
//        ��У��     xx xx
//      ��ɫ����     xx xx
//         X����     xx xx
//         Y����     xx xx
//         width     xx xx
//        height     xx xx
//    CCģʽ�Ƕ�     xx xx

//2.���16bit������8bit�ȷ��͵ĸ�λ,
//  ���͵ĵ�λ
/*******************************END*********************************************/
void USART1_IRQHandler(void)//Pixy_Uart_ReadData
{
	int i;
	
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  
	{
		Raw_Data[counter] = USART_ReceiveData(USART1);
		counter++;
		if(counter == 40)
		{
			  counter=0;//���»�����ͷ
			 
				for(i=0;i<40;i++)//�������һ������
				{
					if((Raw_Data[i] == 0x55)&&(Raw_Data[i+1] == 0xaa)&&(Raw_Data[i+2] == 0x55)&&(Raw_Data[i+3] == 0xaa))
						{
							LED_RED=!LED_RED;//�ж�ָʾ��
							//4.5У�鲻����ȥ �޶���Ұ��Χ
							if((Raw_Data[i+8]  + Raw_Data[i+9]*256)>=57 && (Raw_Data[i+8]  + Raw_Data[i+9]*256)<=261 && 
								(Raw_Data[i+10] + Raw_Data[i+11]*256)>=0 && (Raw_Data[i+10] + Raw_Data[i+11]*256)<=192 &&  (Raw_Data[i+6]  + Raw_Data[i+7]*256)==1)
							{
							Pixy_Color_Inf.Pixy_Color_Sig_Raw    = Raw_Data[i+6]  + Raw_Data[i+7]*256;
							Pixy_Color_Inf.Pixy_Color_PosX_Raw    = Raw_Data[i+8]  + Raw_Data[i+9]*256;
							Pixy_Color_Inf.Pixy_Color_PosY_Raw    = Raw_Data[i+10] + Raw_Data[i+11]*256;
							Pixy_Color_Inf.Pixy_Color_Width_Raw   = Raw_Data[i+12] + Raw_Data[i+13]*256;
							Pixy_Color_Inf.Pixy_Color_Height_Raw  = Raw_Data[i+14] + Raw_Data[i+15]*256;
							}
							else if ((Raw_Data[i+8]  + Raw_Data[i+9]*256)>=57 && (Raw_Data[i+8]  + Raw_Data[i+9]*256)<=261 && (Raw_Data[i+10] + Raw_Data[i+11]*256)>=0 
								&& (Raw_Data[i+10] + Raw_Data[i+11]*256)<=192 &&  (Raw_Data[i+6]  + Raw_Data[i+7]*256)==2)
							{
								Pixy_Color_Inf.Pixy_Color_Sig_b   = Raw_Data[i+6]  + Raw_Data[i+7]*256;
							Pixy_Color_Inf.Pixy_Color_PosX_b    = Raw_Data[i+8]  + Raw_Data[i+9]*256;
							Pixy_Color_Inf.Pixy_Color_PosY_b    = Raw_Data[i+10] + Raw_Data[i+11]*256;
							Pixy_Color_Inf.Pixy_Color_Width_b  = Raw_Data[i+12] + Raw_Data[i+13]*256;
							Pixy_Color_Inf.Pixy_Color_Height_b  = Raw_Data[i+14] + Raw_Data[i+15]*256;
							}
							Pixy_DataSave(Pixy_Color_Inf.Pixy_Color_Sig_Raw ,Pixy_Color_Inf.Pixy_Color_PosX_Raw ,Pixy_Color_Inf.Pixy_Color_PosY_Raw ,
							Pixy_Color_Inf.Pixy_Color_Width_Raw ,Pixy_Color_Inf.Pixy_Color_Height_Raw );
							break;//����forѭ��
							}
							
						}
				}
		}
	}
int16_t Pixy_FIFO[5][10] = {0};
void Pixy_DataSave(int16_t s,int16_t x,int16_t y,int16_t w,int16_t h) //[0]-[9]Ϊ���10������ [10]Ϊ10�����ݵ�ƽ��ֵ
{
	uint8_t i = 0;
	int32_t sum=0;
	
	for(i=1;i<10;i++)
	{
		Pixy_FIFO[0][i-1]=Pixy_FIFO[0][i];
		Pixy_FIFO[1][i-1]=Pixy_FIFO[1][i];
		Pixy_FIFO[2][i-1]=Pixy_FIFO[2][i];
		Pixy_FIFO[3][i-1]=Pixy_FIFO[3][i];
		Pixy_FIFO[4][i-1]=Pixy_FIFO[4][i];
	}
	
	Pixy_FIFO[0][9]=s;//���µ����ݷ��õ� ���ݵ������
	Pixy_FIFO[1][9]=x;
	Pixy_FIFO[2][9]=y;
	Pixy_FIFO[3][9]=w;
	Pixy_FIFO[4][9]=h;
	
	for(i=0;i<10;i++)//��ǰ����ĺϣ���ȡƽ��ֵ
	{	
		 sum+=Pixy_FIFO[0][i];
	}
	Pixy_Color_Inf.Pixy_Color_Sig=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=Pixy_FIFO[1][i];
	}
	Pixy_Color_Inf.Pixy_Color_PosX=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=Pixy_FIFO[2][i];
	}
	Pixy_Color_Inf.Pixy_Color_PosY=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=Pixy_FIFO[3][i];
	}
	Pixy_Color_Inf.Pixy_Color_Width=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=Pixy_FIFO[4][i];
	}
	Pixy_Color_Inf.Pixy_Color_Height=sum/10;
}

/*************************STM32F4XX SPI3��ʼ������*******************************/
//PC10��PC11��PC12����ΪSPI3
//PC9��ʼ��ΪƬѡ��
//��Ƶʹʱ�ӵ���1MƵ��
//ʹ�÷��жϷ�ʽ
/*******************************END*********************************************/
//void Pixy_SPI_Init(void)
//{
//	GPIO_InitTypeDef  GPIO_InitStructure;
//  SPI_InitTypeDef  SPI_InitStructure;
//	
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��GPIOCʱ��
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);//ʹ��SPI3ʱ��
// 
//  //GPIOFC10,11,12��ʼ������ΪSPI3
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;//PC10~12���ù������	
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//50MHz
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//  GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��
//	
//	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_SPI3); //PC10����Ϊ SPI3
//	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_SPI3); //PC11����Ϊ SPI3
//	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_SPI3); //PC12����Ϊ SPI3
//	
//	//PC9��ʼ��ΪƬѡ
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
//	GPIO_Init(GPIOC,&GPIO_InitStructure); //��ʼ��PC9
//	GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_SET);//��ʼ����
//	
//	
//	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
//	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
//	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
//	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//����ͬ��ʱ�ӵĿ���״̬Ϊ�͵�ƽ
//	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
//	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
//	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
//	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
//	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ
//	SPI_Init(SPI3, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���

//	SPI_Cmd(SPI1, ENABLE); //ʹ��SPI����	 
//}

//void Pixy_SPI_SendData(u8 data1)
//{
//	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}//�ȴ���������  
//	SPI_I2S_SendData(SPI1, data1); //ͨ������SPIx����һ��byte  ����
//}

///***************************STM32F4XX SPI3������*****************************/
////˼��:
////����32����Ч����
////�������Ұ�ͷaa55
////�ҵ�����Ч���ݸ���Pixy_Color_Inf�ṹ��

////ע�⣺
////1.���ڽ��հ��������£�
////          ��ͷ     AA 55
////        ��У��     xx xx
////      ��ɫ����     xx xx
////         X����     xx xx
////         Y����     xx xx
////         width     xx xx
////        height     xx xx
////    CCģʽ�Ƕ�     xx xx

////2.���16bit������8bit�ȷ��͵ĵ�λ,
////  ���͵ĸ�λ

////�²ۣ���Ȼ��UART������һ��
////      ��ʹ������~~~~~~      
///*******************************END*********************************************/
//void Pixy_SPI_ReadData(void)
//{
//	int i;
//	
//	GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_RESET);//Ƭѡ���ͣ�����
//	
//	for(i=0;i<32;i++)
//	{
//		while(SPI_GetFlagStatus(SPI3, SPI_FLAG_TXE) == RESET);
//		Raw_Data[i] = SPI_ReceiveData(SPI3);
//	}
//		for(i=0;i<32;i++)//�������һ������
//		{
//			if((Raw_Data[i] == 0xaa)&&(Raw_Data[i+1] == 0x55))
//			{
//				//4.5У�鲻����ȥ
//				Pixy_Color_Inf.Pixy_Color_Sig    = Raw_Data[i+7]  + Raw_Data[i+6]*256;
//				Pixy_Color_Inf.Pixy_Color_PosX   = Raw_Data[i+9]  + Raw_Data[i+8]*256;
//				Pixy_Color_Inf.Pixy_Color_PosY   = Raw_Data[i+11] + Raw_Data[i+10]*256;
//				Pixy_Color_Inf.Pixy_Color_Width  = Raw_Data[i+13] + Raw_Data[i+12]*256;
//				Pixy_Color_Inf.Pixy_Color_Height = Raw_Data[i+15] + Raw_Data[i+14]*256;
//				break;//����forѭ��
//			}
//		}
//		GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_SET);
////		delay_ms(10);
//}	


/*************************************************END********************************************/
