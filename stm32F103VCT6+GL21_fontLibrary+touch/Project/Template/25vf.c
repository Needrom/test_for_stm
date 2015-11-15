#include "stm32f10x.h"
#include "25vf.h"


//SPI初始化
void spi_init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure; 
  SPI_InitTypeDef   SPI_InitStructure; 

  //SPI1 Periph clock enable 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE); 
  //Configure SPI1 pins: SCK, MISO and MOSI 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_7; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;   //复用推挽输出
  GPIO_Init(GPIOA,&GPIO_InitStructure); 

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   //上拉输入
  GPIO_Init(GPIOA,&GPIO_InitStructure);	


  // SPI1 Config  
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; 
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master; 
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; 
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; 
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; 
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;   //SPI_NSS_Hard
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; 
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; 
  SPI_InitStructure.SPI_CRCPolynomial = 7; 
  SPI_Init(SPI1,&SPI_InitStructure); 
  // SPI1 enable  
  SPI_Cmd(SPI1,ENABLE); 
}

/**********************************************************************/
unsigned char SPI_WriteByte(unsigned char data) 
{ 
 unsigned char Data = 0; 

   //Wait until the transmit buffer is empty 
  while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE)==RESET); 
  // Send the byte  
  SPI_I2S_SendData(SPI1,data); 

   //Wait until a data is received 
  while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE)==RESET); 
  // Get the received data 
  Data = SPI_I2S_ReceiveData(SPI1); 

  // Return the shifted data 
  return Data; 
}  




//===========================================
void Read_25(unsigned int Dst,unsigned char *buff,unsigned char num)
{
	unsigned char i;
 	CE_25_0;
	SPI_WriteByte(0X03);
	SPI_WriteByte(Dst>>16);	
	SPI_WriteByte(Dst>>8);	
	SPI_WriteByte(Dst);
	for(i=0;i<num;i++)
		*(buff+i)=SPI_WriteByte(0);			
	CE_25_1;
}
