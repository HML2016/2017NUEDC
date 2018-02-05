/*************************************使用说明***********************************/
//配套使用CMUCAM5(Pixy)解码
//STM32F407亲测可用
//提供串口中断解码和SPI查询处理代码
//整理人：明眸
//QQ:1596311719
//2015/9/20
/****************************************END*************************************/

#include "Pixy.h"
#include "led.h"
#include "Control.h"
#include "timer.h"

Pixy_Color Pixy_Color_Inf;//结构体例化
u8 Raw_Data[40];//原始数据大包，为解串准备
u8 counter;//计数用


/**********************STM32F4XX UART1初始化程序********************************/
//PA9，PA10复用为UART1
//数据位8，停止位1，无其它位,波特率19200
//使用串口中断方式,不使用可以注释掉后6行代码
/*******************************END*********************************************/
void Pixy_Uart_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = 500000;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
    USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
    USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART1, USART_FLAG_TC);
	
	//使用中断方式
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断
	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}

/***********************STM32F4XX UART1中断处理函数******************************/
//思想:
//接收40个有效数据
//在其中找包头55aa，55aa，
//找到后有效数据更新Pixy_Color_Inf结构体

//注意：
//1.串口接收包码流如下：
//          包头     55 AA 55(56) AA
//        和校验     xx xx
//      颜色代号     xx xx
//         X坐标     xx xx
//         Y坐标     xx xx
//         width     xx xx
//        height     xx xx
//    CC模式角度     xx xx

//2.组成16bit的两个8bit先发送的高位,
//  后发送的低位
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
			  counter=0;//重新回数组头
			 
				for(i=0;i<40;i++)//计算更新一次坐标
				{
					if((Raw_Data[i] == 0x55)&&(Raw_Data[i+1] == 0xaa)&&(Raw_Data[i+2] == 0x55)&&(Raw_Data[i+3] == 0xaa))
						{
							LED_RED=!LED_RED;//中断指示灯
							//4.5校验不用舍去 限定视野范围
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
							break;//跳出for循环
							}
							
						}
				}
		}
	}
int16_t Pixy_FIFO[5][10] = {0};
void Pixy_DataSave(int16_t s,int16_t x,int16_t y,int16_t w,int16_t h) //[0]-[9]为最近10次数据 [10]为10次数据的平均值
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
	
	Pixy_FIFO[0][9]=s;//将新的数据放置到 数据的最后面
	Pixy_FIFO[1][9]=x;
	Pixy_FIFO[2][9]=y;
	Pixy_FIFO[3][9]=w;
	Pixy_FIFO[4][9]=h;
	
	for(i=0;i<10;i++)//求当前数组的合，再取平均值
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

/*************************STM32F4XX SPI3初始化程序*******************************/
//PC10，PC11，PC12复用为SPI3
//PC9初始化为片选线
//分频使时钟低于1M频率
//使用非中断方式
/*******************************END*********************************************/
//void Pixy_SPI_Init(void)
//{
//	GPIO_InitTypeDef  GPIO_InitStructure;
//  SPI_InitTypeDef  SPI_InitStructure;
//	
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOC时钟
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);//使能SPI3时钟
// 
//  //GPIOFC10,11,12初始化复用为SPI3
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;//PC10~12复用功能输出	
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//50MHz
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化
//	
//	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_SPI3); //PC10复用为 SPI3
//	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_SPI3); //PC11复用为 SPI3
//	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_SPI3); //PC12复用为 SPI3
//	
//	//PC9初始化为片选
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
//	GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化PC9
//	GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_SET);//初始化高
//	
//	
//	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
//	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
//	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
//	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//串行同步时钟的空闲状态为低电平
//	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
//	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
//	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;		//定义波特率预分频的值:波特率预分频值为256
//	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
//	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
//	SPI_Init(SPI3, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器

//	SPI_Cmd(SPI1, ENABLE); //使能SPI外设	 
//}

//void Pixy_SPI_SendData(u8 data1)
//{
//	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}//等待发送区空  
//	SPI_I2S_SendData(SPI1, data1); //通过外设SPIx发送一个byte  数据
//}

///***************************STM32F4XX SPI3处理函数*****************************/
////思想:
////接收32个有效数据
////在其中找包头aa55
////找到后有效数据更新Pixy_Color_Inf结构体

////注意：
////1.串口接收包码流如下：
////          包头     AA 55
////        和校验     xx xx
////      颜色代号     xx xx
////         X坐标     xx xx
////         Y坐标     xx xx
////         width     xx xx
////        height     xx xx
////    CC模式角度     xx xx

////2.组成16bit的两个8bit先发送的低位,
////  后发送的高位

////吐槽：竟然与UART包还不一样
////      惨痛的人生~~~~~~      
///*******************************END*********************************************/
//void Pixy_SPI_ReadData(void)
//{
//	int i;
//	
//	GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_RESET);//片选拉低，接收
//	
//	for(i=0;i<32;i++)
//	{
//		while(SPI_GetFlagStatus(SPI3, SPI_FLAG_TXE) == RESET);
//		Raw_Data[i] = SPI_ReceiveData(SPI3);
//	}
//		for(i=0;i<32;i++)//计算更新一次坐标
//		{
//			if((Raw_Data[i] == 0xaa)&&(Raw_Data[i+1] == 0x55))
//			{
//				//4.5校验不用舍去
//				Pixy_Color_Inf.Pixy_Color_Sig    = Raw_Data[i+7]  + Raw_Data[i+6]*256;
//				Pixy_Color_Inf.Pixy_Color_PosX   = Raw_Data[i+9]  + Raw_Data[i+8]*256;
//				Pixy_Color_Inf.Pixy_Color_PosY   = Raw_Data[i+11] + Raw_Data[i+10]*256;
//				Pixy_Color_Inf.Pixy_Color_Width  = Raw_Data[i+13] + Raw_Data[i+12]*256;
//				Pixy_Color_Inf.Pixy_Color_Height = Raw_Data[i+15] + Raw_Data[i+14]*256;
//				break;//跳出for循环
//			}
//		}
//		GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_SET);
////		delay_ms(10);
//}	


/*************************************************END********************************************/
