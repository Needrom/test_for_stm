#include "stm32f10x.h"
#include "touch.h"
#include "25vf.h"


void SpiDelay(unsigned int DelayCnt)
{
 unsigned int i;
 for(i=0;i<DelayCnt;i++);
}


void touch_Init(void)
{
    CE_25_1;
	TP_CS_1; 
}
u16 TPReadX(void)
{ 
   u16 x=0;
   TP_CS_0;
   SpiDelay(10);
   SPI_WriteByte(0x90);
   SpiDelay(10);
   x=SPI_WriteByte(0x00);
   x<<=8;
   x+=SPI_WriteByte(0x00);
   SpiDelay(10);
   TP_CS_1; 
   x = x>>3;
   return (x);
}

u16 TPReadY(void)
{
 u16 y=0;
  TP_CS_0;
  SpiDelay(10);
  SPI_WriteByte(0xD0);
  SpiDelay(10);
  y=SPI_WriteByte(0x00);
  y<<=8;
  y+=SPI_WriteByte(0x00);
  SpiDelay(10);
  TP_CS_1;
  y = y>>3; 
  return (y);
}
/**************************************************************************************
* 名    称: 
* 功    能:
* 参    数:
* 返 回 值:
*
* 修改历史:
*   版本    日期     作者     
*   ----------------------------------------------------
*   1.0   2008.8.13  孙逸洁       www.http://shop35330111.taobao.com
**************************************************************************************/
 void touch_GetAdXY(unsigned int *x,unsigned int *y) 
{
    *x=TPReadX();
    *y=TPReadY();
}





