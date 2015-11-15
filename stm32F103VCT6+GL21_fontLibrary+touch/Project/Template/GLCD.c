/*----------------------------------------------------------------------------
 * Name:    GLCD.c
 * Purpose: MCBSTM32E low level Graphic LCD (320x240 pixels) functions
 * Version: V1.00
 * Note(s):
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2009 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------
 * History:
 *          V1.00 Initial Version
 *----------------------------------------------------------------------------*/

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f10x.h"

#include "glcd.h"
#include "font.h"
#include "zimo.h"
#include "25vf.h"
/*********************** Hardware specific configuration **********************/

/*------------------------- Speed dependant settings -------------------------*/

/* If processor works on high frequency delay has to be increased, it can be 
   increased by factor 2^N by this constant                                   */
#define DELAY_2N    18

/*---------------------- Graphic LCD size definitions ------------------------*/


#define BPP         16                  /* Bits per pixel                     */
#define BYPP        ((BPP+7)/8)         /* Bytes per pixel                    */

/*--------------- Graphic LCD interface hardware definitions -----------------*/

/* Note: LCD /CS is CE1 - Bank 1 of NOR/SRAM Bank 1~4 */
//#define LCD_BASE        (0x60000000UL | 0x00000000UL)
//#define LCD_REG16  (*((volatile unsigned short *)(LCD_BASE  )))		// RS(A16)=0 
//#define LCD_DAT16  (*((volatile unsigned short *)(LCD_BASE+2)))		// RS(A16)=1

// RS=A16;
#define LCD_REG16  (*((volatile unsigned short *)(0x60000000)))		// RS(A16)=0 
#define LCD_DAT16  (*((volatile unsigned short *)(0x60020000)))		// RS(A16)=1


/*---------------------------- Global variables ------------------------------*/

/******************************************************************************/
static volatile unsigned short TextColor = Black, BackColor = White;
static volatile unsigned short DriverCode;



unsigned char H_V;		//������ѡ��
unsigned char U_D;		//����ɨ��ѡ��
unsigned char L_R;		//����ɨ��ѡ��
unsigned char CMD_CTR_PWM;
unsigned char CMD_CTR_DISP;
unsigned char CMD_CTR_WRITE;
unsigned char CMD_CTR_POINT;
#define CMD_CTR_REG	   (H_V<<10)+(U_D<<9)+(L_R<<8)+(CMD_CTR_PWM<<5)+(CMD_CTR_DISP<<3)+(CMD_CTR_WRITE<<2)+CMD_CTR_POINT//����������


 
/************************ Local auxiliary functions ***************************/

/*******************************************************************************
* Delay in while loop cycles                                                   *
*   Parameter:    cnt:    number of while cycles to delay                      *
*   Return:                                                                    *
*******************************************************************************/

static void delay (int cnt) {

  cnt <<= DELAY_2N;
  while (cnt--);
}


/*******************************************************************************
* Write command to LCD controller                                              *
*   Parameter:    c:      command to be written                                *
*   Return:                                                                    *
*******************************************************************************/

//static 
__inline void wr_cmd (unsigned char c) {

  LCD_REG16 = c;
}


/*******************************************************************************
* Write data to LCD controller                                                 *
*   Parameter:    c:      data to be written                                   *
*   Return:                                                                    *
*******************************************************************************/

//static
 __inline void wr_dat (unsigned short c) {

  LCD_DAT16 = c;
}


/*******************************************************************************
* Read data from LCD controller                                                *
*   Parameter:                                                                 *
*   Return:               read data                                            *
*******************************************************************************/

//static
__inline unsigned short rd_dat (void) {

  return (LCD_DAT16);                                    /* return value */
}

/*******************************************************************************
* Write to LCD register                                                        *
*   Parameter:    reg:    register to be read                                  *
*                 val:    value to write to register                           *
*******************************************************************************/

//static
__inline void wr_reg (unsigned char reg, unsigned short val) {

  wr_cmd(reg);
  wr_dat(val);
}


/*******************************************************************************
* Read from LCD register                                                       *
*   Parameter:    reg:    register to be read                                  *
*   Return:               value read from register                             *
*******************************************************************************/

//static
unsigned short rd_reg (unsigned char reg) {

  wr_cmd(reg);
  return (rd_dat());
}

/****************************************************************************
* ��    �ƣ�void FYlcd_set_ctrl(unsigned short num)
* ��    �ܣ�����ϵ��Һ��д�������
* ��ڲ���������������
* ���ڲ�������
* ˵    ����
* ���÷�����FYlcd_set_ctrl(0x03e0);
****************************************************************************/
void FYlcd_set_ctrl(unsigned short num)
{
   	wr_reg(CMD_CTR, 0x5aa5);	  // д�������֮ǰ��д��У����
   	wr_reg(CMD_CTR, num);
   	wr_cmd(CMD_Data);
}
/****************************************************************************
* ��    �ƣ�void black_light(unsigned char liangdu)
* ��    �ܣ� ���Ʊ�������
* ��ڲ���������ֵ��0-7��
* ���ڲ�������
* ˵    ����
* ���÷�����black_light(7);
****************************************************************************/
void FY_lcd_BackLight(unsigned char liangdu)
{
	if(liangdu<8)CMD_CTR_PWM=liangdu;
	FYlcd_set_ctrl(CMD_CTR_REG);
//	unsigned short i;
//	i=rd_reg (CMD_Ctrl);//i=0x03e0;  	  //
//	i&=(~(7<<5));
//	i|=(liangdu&0x07)<<5;	 
//  FYlcd_set_ctrl(i);
}

/************************ Exported functions **********************************/

/*******************************************************************************
* Initialize the Graphic LCD controller                                        *
*   Parameter:                                                                 *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_init (void) { 
unsigned int i;
/* Configure the LCD Control pins --------------------------------------------*/
  RCC->APB2ENR |= 0x000001FD;                         /* enable GPIOA,..G, AFIO clock */

  /* PD.00(D2),  PD.01(D3),  PD.04(NOE), PD.05(NWE), PD.07(NE1)*/ 
  GPIOD->CRL &= ~0xF0FF00FF;                          /* clear Bits */
  GPIOD->CRL |=  0xB0BB00BB;                          /* alternate function output Push-pull 50MHz */
  /* PD.08(D13), PD.09(D14), PD.10(D15), PD.11(RS), PD.14(D0), PD.15(D1) */
  GPIOD->CRH &= ~0xFF00FFFF;                          /* clear Bits */
  GPIOD->CRH |=  0xBB00BBBB;                          /* alternate function output Push-pull 50MHz */
   
  /* PE.07(D4) */ 
  GPIOE->CRL &= ~0xF0000000;                          /* clear Bits */
  GPIOE->CRL |=  0xB0000000;                          /* alternate function output Push-pull 50MHz */
  /* PE.08(D5), PE.09(D6),  PE.10(D7), PE.11(D8), PE.12(D9), PE.13(D10), PE.14(D11), PE.15(D12) */
  GPIOE->CRH &= ~0xFFFFFFFF;                          /* clear Bits */
  GPIOE->CRH |=  0xBBBBBBBB;                          /* alternate function output Push-pull 50MHz */

  /* PC.06(RST) */ 
  GPIOC->CRL &= ~0x0F000000;                          /* clear Bits */
  GPIOC->CRL |=  0x0B000000;                          /* alternate function output Push-pull 50MHz */
  GPIOC->ODR |=  0x40;						

/*-- FSMC Configuration ------------------------------------------------------*/
/*----------------------- SRAM Bank 4 ----------------------------------------*/
  RCC->AHBENR  |= (1<<8);                             /* enable FSMC clock */

  FSMC_Bank1->BTCR[FSMC_Bank1_NORSRAM1+1] =           /* Bank1 NOR/SRAM timing register configuration */
                                         (0 << 28) |  /* FSMC AccessMode A */
                                         (0 << 24) |  /* Data Latency */
                                         (0 << 20) |  /* CLK Division */
                                         (0 << 16) |  /* Bus Turnaround Duration */
                                         (1 <<  8) |  /* Data SetUp Time */
                                         (0 <<  4) |  /* Address Hold Time */
                                         (0 <<  0);   /* Address SetUp Time */
  FSMC_Bank1->BTCR[FSMC_Bank1_NORSRAM1  ] =           /* Control register */
                                         (0 << 19) |  /* Write burst disabled */
                                         (0 << 15) |  /* Async wait disabled */
                                         (0 << 14) |  /* Extended mode disabled */
                                         (0 << 13) |  /* NWAIT signal is disabled */ 
                                         (1 << 12) |  /* Write operation enabled */
                                         (0 << 11) |  /* NWAIT signal is active one data cycle before wait state */
                                         (0 << 10) |  /* Wrapped burst mode disabled */
                                         (1 <<  9) |  /* Wait signal polarity active low */
                                         (0 <<  8) |  /* Burst access mode disabled */
                                         (1 <<  4) |  /* Memory data  bus LCD_XSIZE is 16 bits */
                                         (0 <<  2) |  /* Memory type is SRAM */
                                         (0 <<  1) |  /* Address/Data Multiplexing disable */
                                         (1 <<  0);   /* Memory Bank enable */
 H_V=0;		//������ѡ��
 U_D=1;		//����ɨ��ѡ��
 L_R=1;		//����ɨ��ѡ��
  for(i=8;i!=0;i--)
  {
    FY_lcd_BackLight(i-1);	
  	delay(5);
  }
  	FY_lcd_BackLight(7);

  wr_reg(CMD_Yaddr_Start, 0);                      /* Horizontal GRAM Start Address      */
  wr_reg(CMD_Yaddr_End, LCD_YSIZE-1);                  /* Horizontal GRAM End   Address (-1) */
  wr_reg(CMD_Xaddr_Start, 0);                      /* Vertical   GRAM Start Address      */
  wr_reg(CMD_Xaddr_End, LCD_XSIZE-1);                  /* Vertical   GRAM End   Address (-1) */
  wr_reg(CMD_Xaddr, 0);
  wr_reg(CMD_Yaddr, 0);
  wr_cmd(CMD_Data);
}

/****************************************************************************
* ��    �ƣ�void FY_lcd_SetCursor(unsigned short x,unsigned short y)
* ��    �ܣ�������Ļ����
* ��ڲ�����x      ������
*           y      ������
* ���ڲ�������
* ˵    ����
* ���÷�����FY_lcd_SetCursor(10,10);
****************************************************************************/
void FY_lcd_SetCursor(unsigned int x,unsigned int y)
{
  wr_reg(CMD_Xaddr, x);
  wr_reg(CMD_Yaddr, y);
}

/****************************************************************************
* ��    �ƣ�void FY_lcd_SetWindows(unsigned short StartX,unsigned short StartY,unsigned short EndX,unsigned short EndY)
* ��    �ܣ����ô�������
* ��ڲ�����StartX     ����ʼ����
*           StartY     ����ʼ����
*           EndX       �н�������
*           EndY       �н�������
* ���ڲ�������
* ˵    ����
* ���÷�����FY_lcd_SetWindows(0,0,100,100)��
****************************************************************************/
void FY_lcd_SetWindows(unsigned short StartX,unsigned short StartY,unsigned short EndX,unsigned short EndY)
{
  FY_lcd_SetCursor(StartX,StartY);
  wr_reg(CMD_Yaddr_Start, StartY);
  wr_reg(CMD_Yaddr_End,EndY );

  wr_reg(CMD_Xaddr_Start,StartX);
  wr_reg(CMD_Xaddr_End,EndX);
}

/****************************************************************************
* ��    �ƣ�void FY_lcd_Clear(unsigned short dat)
* ��    �ܣ�����Ļ����ָ������ɫ��������������� 0xffff
* ��ڲ�����dat      ���ֵ
* ���ڲ�������
* ˵    ����
* ���÷�����FY_lcd_Clear(0xffff);
****************************************************************************/
void FY_lcd_Clear(unsigned short dat)
{
unsigned int i;
  wr_reg(CMD_Yaddr_Start, 0);                      /* Horizontal GRAM Start Address      */
  wr_reg(CMD_Yaddr_End, LCD_YSIZE-1);                  /* Horizontal GRAM End   Address (-1) */
  wr_reg(CMD_Xaddr_Start, 0);                      /* Vertical   GRAM Start Address      */
  wr_reg(CMD_Xaddr_End, LCD_XSIZE-1); 

  wr_reg(CMD_Xaddr, 0);
  wr_reg(CMD_Yaddr, 0);
  wr_cmd(CMD_Data);
  for(i = 0; i < (LCD_XSIZE*LCD_YSIZE); i++)
    wr_dat(dat);
}

/****************************************************************************
* ��    �ƣ�unsigned short FY_lcd_GetPoint(unsigned short x,unsigned short y)
* ��    �ܣ���ȡָ���������ɫֵ
* ��ڲ�����x      ������
*           y      ������
* ���ڲ�������ǰ������ɫֵ
* ˵    ����
* ���÷�����i=FY_lcd_GetPoint(10,10);
****************************************************************************/
unsigned short FY_lcd_GetPoint(unsigned short x,unsigned short y)
{
  	FY_lcd_SetCursor(x,y);
	wr_cmd(0x22);
	return rd_dat();
	//return FY_lcd_BGR2RGB(rd_dat());
}

/****************************************************************************
* ��    �ƣ�void FY_lcd_SetPoint(unsigned short x,unsigned short y,unsigned short point)
* ��    �ܣ���ָ�����껭��
* ��ڲ�����x      ������
*           y      ������
*           point  �����ɫ
* ���ڲ�������
* ˵    ����
* ���÷�����FY_lcd_SetPoint(10,10,0x0fe0);
****************************************************************************/
void FY_lcd_SetPoint(unsigned short x,unsigned short y,unsigned short point)
{
  if ( (x>LCD_XSIZE)||(y>LCD_YSIZE) ) return;
  FY_lcd_SetCursor(x,y);
  wr_cmd(CMD_Data);
  wr_dat(point);
}

/*******************************************************************************
* Set foreground color                                                         *
*   Parameter:    color:  color for clearing display                           *
*   Return:                                                                    *
*******************************************************************************/
void GLCD_setTextColor(unsigned short color) {
  TextColor = color;
}


/*******************************************************************************
* Set background color                                                         *
*   Parameter:    color:  color for clearing display                           *
*   Return:                                                                    *
*******************************************************************************/
void GLCD_setBackColor(unsigned short color) {
  BackColor = color;
}


/*******************************************************************************
* Clear display                                                                *
*   Parameter:    color:  color for clearing display                           *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_clear (unsigned short color) {
  unsigned int   i;
  wr_reg(0x50, 0);                      /* Horizontal GRAM Start Address      */
  wr_reg(0x51, LCD_YSIZE-1);                  /* Horizontal GRAM End   Address (-1) */
  wr_reg(0x52, 0);                      /* Vertical   GRAM Start Address      */
  wr_reg(0x53, LCD_XSIZE-1); 

  wr_reg(CMD_Xaddr, 0);
  wr_reg(CMD_Yaddr, 0);
  wr_cmd(CMD_Data);
  for(i = 0; i < (LCD_XSIZE*LCD_YSIZE); i++)
    wr_dat(color);
}
 /**************************************************************************************
* ��    ��: DrawSingleAscii
* ��    ��: ��ָ����λ�õ����ַ�
* ��    ��: x           : x����
*                       y         : y����
*               LineColor : �ַ�����ɫ
*           FillColor   : �ַ�������ɫ
* �� �� ֵ: ��
*
* �޸���ʷ:
*   �汾    ����     ����     
*   ----------------------------------------------------
*   1.0   2008.8.13  ���ݽ�       www.http://shop35330111.taobao.com
**************************************************************************************/
void DrawSingleAscii(unsigned int x, unsigned int y, unsigned char *pAscii, unsigned int LineColor,unsigned int FillColor, unsigned char Mod)
{
    unsigned char i, j;
    unsigned char str;
    unsigned int OffSet;

    OffSet = (*pAscii - 32)*16;

    for (i=0;i<16;i++)
    {
        FY_lcd_SetCursor(x,y+i);
        wr_cmd(CMD_Data);
        str = *(AsciiLib + OffSet + i);  
        for (j=0;j<8;j++)
        {
            if ( str & (0x80>>j) )     //0x80>>j
            {
                wr_dat((unsigned int)(LineColor&0xffff));
            }
            else
            {
                if (NORMAL == Mod) 
                    wr_dat((unsigned int)(FillColor&0xffff));
                else
                {
                    FY_lcd_SetCursor(x+j+1,y+i);
                    wr_cmd(CMD_Data);  
                }
            }               
        } 
    }
}
/**************************************************************************************
* ��    ��: DrawSingleHz
* ��    ��: ��ָ����λ����ʾ����
* ��    ��: x           : x����
*                       y       : y����
*               LineColor : ���ֵ���ɫ
*           FillColor   : ���ֱ�����ɫ
* �� �� ֵ: ��
*
* �޸���ʷ:
*   �汾    ����     ����     
*   ----------------------------------------------------
*   1.0   2008.8.13  ���ݽ�       www.http://shop35330111.taobao.com
**************************************************************************************/
void DrawSingleHz(unsigned int x, unsigned int y, unsigned char *pHz,unsigned int LineColor,unsigned int FillColor, unsigned char Mod)
{
    unsigned char c1,c2;
    unsigned char i, k;
    unsigned int str;
    unsigned int OffSet=0;
	unsigned char hz_mo[32];	//��ʱ�洢����ģ�Ļ���
	c1=*pHz; 
	c2=*(pHz+1);

    if(c1>=0xA1 && c1 <= 0xAB && c2>=0xa1)//������,ȫ��
    {
		OffSet = (c1 - 0xA1) * 94 + (c2 - 0xA1);
		OffSet =OffSet *32 + CUTS1516ZF_ADDR;
    } 	
    else if(c1>=0xb0 && c1 <= 0xf7 && c2>=0xa1)//GBK˫�ֽ�2�� 6768
    {
    	OffSet= (c1 - 0xB0) * 94 + (c2 - 0xA1);
		OffSet = OffSet*32 + JFLS1516HZ_ADDR;   
	}
	Read_25(OffSet,&hz_mo[0],32);
    for (i=0;i<16;i++)
    {
        FY_lcd_SetCursor(x,y+i);		 
        wr_cmd(CMD_Data);
		str =(unsigned int)hz_mo[i*2]<<8|hz_mo[i*2+1];
        for (k=0;k<16;k++)
        {
            if ( str & (0x8000>>k) )     //0x8000>>k
            {
                wr_dat(LineColor);
            }
            else
            {
                if (NORMAL == Mod) 
                     wr_dat(FillColor);
                else
                {
                    FY_lcd_SetCursor(x+k+1,y+i);
                    wr_cmd(CMD_Data);  
                }
            }               
        } 
    }
}

/**************************************************************************************
* ��    ��: DrawString
* ��    ��: ��ָ����λ����ʾ����ַ�
* ��    ��: x           : x����
*                       y         : y����
*               LineColor : �ַ�����ɫ
*           FillColor   : �ַ�������ɫ
* �� �� ֵ: ��
*
* �޸���ʷ:
*   �汾    ����     ����     
*   ----------------------------------------------------
*   1.0   2008.8.13  ���ݽ�       www.http://shop35330111.taobao.com
**************************************************************************************/
void DrawString(unsigned int x, unsigned int y, unsigned char *pStr, unsigned int LineColor,unsigned int FillColor, unsigned char Mod)
{

    while(1)
    {
        if (*pStr == 0)
        {
            return;
        }

        if (*pStr > 0x80)           //����
        {
			DrawSingleHz(x, y, pStr,LineColor, FillColor,Mod);
            x += HZ_column;
            pStr += 2;              
        }
        else                        //Ӣ���ַ�
        {
            DrawSingleAscii(x, y, pStr, LineColor, FillColor, Mod);
            x += 8;
            pStr += 1;              
        }
    }   
}


void DispNum(unsigned int x, unsigned int y, unsigned short num)
{
    unsigned char str[5];

    str[0] = num/1000+0x30;
    str[1] = (num%1000)/100+0x30;
    str[2] = (num%1000)%100/10+0x30;
    str[3] = (num%1000)%100%10+0x30;
    str[4] = '\0';

    DrawString(x, y, str, Red, Yellow, NORMAL);
}
/*******************************************************************************
* Draw character on given position (line, coloum                               *
*   Parameter:     x :        horizontal position                              *
*                  y :        vertical position                                *
*                  c*:        pointer to color value                           *
*   Return:                                                                    *
*******************************************************************************/
void GLCD_drawChar(unsigned int x, unsigned int y, unsigned short *c) {
  unsigned int index = 0;
  int  i = 0;
  unsigned int Xaddress = 0;
   
  Xaddress = x;
  
/*  wr_reg(0x20, Xaddress);
  wr_reg(0x21, y);	*/

  wr_reg(CMD_Yaddr, Xaddress);
  wr_reg(CMD_Xaddr, y);

  for(index = 0; index < 24; index++)
  {
    wr_cmd(CMD_Data);;              /* Prepare to write GRAM */
//    for(i = 15; i >= 0; i--)
    for(i = 0; i < 16; i++)
    {
      if((c[index] & (1 << i)) == 0x00) {
        wr_dat(BackColor);
      } else {
        wr_dat(TextColor);
      }
    }
    Xaddress++;
  	wr_reg(CMD_Yaddr, Xaddress);
  	wr_reg(CMD_Xaddr, y);
  	}
}


/*******************************************************************************
* Disply character on given line                                               *
*   Parameter:     c :        ascii character                                  *
*                  ln:        line number                                      *
*   Return:                                                                    *
*******************************************************************************/
void GLCD_displayChar(unsigned int ln, unsigned int col, unsigned char  c) {
  c -= 32;
  GLCD_drawChar(ln, col, &ASCII_Table[c * 24]);							//ӳ�䵽ASCII������
}


/*******************************************************************************
* Disply string on given line                                                  *
*   Parameter:     s*:        pointer to string                                *
*                  ln:        line number                                      *
*   Return:                                                                    *
*******************************************************************************/
void GLCD_displayStringLn(unsigned int ln, unsigned char *s) {
  unsigned int i = 0;
  //unsigned int refcolumn = (LCD_XSIZE/*-1*/)-16;
  unsigned int refcolumn =0;
  while ((*s != 0) & (i < (LCD_XSIZE/16)))                   /* write the string character by character on lCD */
  {
    GLCD_displayChar(ln, refcolumn, *s);         /* Display one character on LCD */
//    refcolumn -= 16;                             /* Decrement the column position by 16 */
    refcolumn += 16;                             /* Decrement the column position by 16 */
    s++;                                         /* Point on the next character */
    i++;                                         /* Increment the character counter */
  }
}


/*******************************************************************************
* Clear given line                                                             *
*   Parameter:     ln:        line number                                      *
*   Return:                                                                    *
*******************************************************************************/
void GLCD_clearLn(unsigned int ln) {
	
	
  GLCD_displayStringLn(ln, "                    ");
}


/*******************************************************************************
* Display graphical bitmap image at position x horizontally and y vertically   *
* (This function is optimized for 16 bits per pixel format, it has to be       *
*  adapted for any other bits per pixel format)                                *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*                   w:        LCD_XSIZE of bitmap                                  *
*                   h:        LCD_YSIZE of bitmap                                 *
*                   bitmap:   address at which the bitmap data resides         *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_bitmap (unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned char *bitmap) {
  unsigned int   i;
  unsigned int   len = w*h;
  unsigned short *bitmap_ptr = (unsigned short *)bitmap;

  wr_reg(0x50, y);                      /* Horizontal GRAM Start Address      */
  wr_reg(0x51, y+h-1);                  /* Horizontal GRAM End   Address (-1) */
  wr_reg(0x52, x);                      /* Vertical   GRAM Start Address      */
  wr_reg(0x53, x+w-1);                  /* Vertical   GRAM End   Address (-1) */

  wr_reg(0x20, y);
  wr_reg(0x21, x);

  wr_cmd(CMD_Data);
  for (i = 0; i < len; i++) {
    wr_dat(*bitmap_ptr++);
  }
}

/****************************************************************************
* ��    �ƣ�void ili9320_Test()
* ��    �ܣ�����Һ����
* ��ڲ�������
* ���ڲ�������
* ˵    ������ʾ����������Һ�����Ƿ���������
* ���÷�����ili9320_Test();
****************************************************************************/
void FY_lcd_Test()
{
  unsigned char  R_data,G_data,B_data;
  unsigned int i,j;
	wr_cmd(CMD_Data);
    R_data=0;G_data=0;B_data=0;     
    for(j=0;j<20;j++)//��ɫ��ǿ��
    {
        for(i=0;i<LCD_XSIZE;i++)
            {R_data=i/(LCD_XSIZE/32);wr_dat(R_data<<11|G_data<<5|B_data);
			}
    }
    R_data=0x1f;G_data=0x3f;B_data=0x1f;
    for(j=0;j<20;j++)
    {
        for(i=0;i<LCD_XSIZE;i++)
            {
            G_data=0x3f-(i/(LCD_XSIZE/64));
            B_data=0x1f-(i/(LCD_XSIZE/32));
            wr_dat(R_data<<11|G_data<<5|B_data);
			}
    }
//----------------------------------
    R_data=0;G_data=0;B_data=0;
    for(j=0;j<20;j++)//��ɫ��ǿ��
    {
        for(i=0;i<LCD_XSIZE;i++)
            {G_data=i/(LCD_XSIZE/64);
			wr_dat(R_data<<11|G_data<<5|B_data);
			}
    }

    R_data=0x1f;G_data=0x3f;B_data=0x1f;
    for(j=0;j<20;j++)
    {
        for(i=0;i<LCD_XSIZE;i++)
            {
            R_data=0x1f-(i/(LCD_XSIZE/32));
            B_data=0x1f-(i/(LCD_XSIZE/32));
            wr_dat(R_data<<11|G_data<<5|B_data);
		}
    }
//----------------------------------
 
    R_data=0;G_data=0;B_data=0;
    for(j=0;j<20;j++)//��ɫ��ǿ��
    {
        for(i=0;i<LCD_XSIZE;i++)
            {B_data=i/(LCD_XSIZE/32);wr_dat(R_data<<11|G_data<<5|B_data);
			}
    } 

    B_data=0; 
    R_data=0x1f;G_data=0x3f;B_data=0x1f;

    for(j=0;j<20;j++)
    {
        for(i=0;i<LCD_XSIZE;i++)
            {
            G_data=0x3f-(i/(LCD_XSIZE/64));
            R_data=0x1f-(i/(LCD_XSIZE/32));
            wr_dat(R_data<<11|G_data<<5|B_data);
		}
    }	  
}
/******************************************************************************/
