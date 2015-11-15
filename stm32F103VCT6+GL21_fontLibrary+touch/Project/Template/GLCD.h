/*----------------------------------------------------------------------------
 * Name:    GLCD.h
 * Purpose: MCBSTM32E low level Graphic LCD (320x240 pixels) prototypes
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

#ifndef _GLCD_H
#define _GLCD_H

/*------------------------------------------------------------------------------
  color coding.
  GLCD is coded:   15..11 red, 10..5 green, 4..0 blue  (unsigned short)  GLCD_R5, GLCD_G6, GLCD_B5   
  original coding: 17..12 red, 11..6 green, 5..0 blue                    ORG_R6,  ORG_G6,  ORG_B6

  ORG_R1..5 = GLCD_R0..4,  ORG_R0 = GLCD_R4
  ORG_G0..5 = GLCD_G0..5,
  ORG_B1..5 = GLCD_B0..4,  ORG_B0 = GLCD_B4
 *------------------------------------------------------------------------------*/
#define	LCD_RST 		GPIO_Pin_1
#define	LCD_RST_0		GPIO_ResetBits(GPIOA, LCD_RST)
#define	LCD_RST_1		GPIO_SetBits  (GPIOA, LCD_RST)
// 触摸屏片选
#define	T_CS			GPIO_Pin_14
#define	T_CS_0			GPIO_ResetBits(GPIOG, T_CS)
#define	T_CS_1			GPIO_SetBits  (GPIOG, T_CS)
// 触摸屏中断
#define	T_INT			GPIO_Pin_14
#define T_INT_IN        GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3)



#define LCD_XSIZE      800                 /* Screen Width (in pixels)           *///320
#define LCD_YSIZE      480                 /* Screen Hight (in pixels)           *///240                            
/* GLCD RGB color definitions */
#define Black           0x0000		/*   0,   0,   0 */
#define Navy            0x000F      /*   0,   0, 128 */
#define DarkGreen       0x03E0      /*   0, 128,   0 */
#define DarkCyan        0x03EF      /*   0, 128, 128 */
#define Maroon          0x7800      /* 128,   0,   0 */
#define Purple          0x780F      /* 128,   0, 128 */
#define Olive           0x7BE0      /* 128, 128,   0 */
#define LightGrey       0xC618      /* 192, 192, 192 */
#define DarkGrey        0x7BEF      /* 128, 128, 128 */
#define Blue            0x001F      /*   0,   0, 255 */
#define Green           0x07E0      /*   0, 255,   0 */
#define Cyan            0x07FF      /*   0, 255, 255 */
#define Red             0xF800      /* 255,   0,   0 */
#define Magenta         0xF81F      /* 255,   0, 255 */
#define Yellow          0xFFE0      /* 255, 255, 0   */
#define White           0xFFFF      /* 255, 255, 255 */

#define Line0                0
#define Line1               24
#define Line2               48
#define Line3               72
#define Line4               96
#define Line5              120
#define Line6              144
#define Line7              168
#define Line8              192
#define Line9              216


#define CMD_FRONT  		0x2
#define CMD_BACK   		0x3
#define CMD_IO_Port		0x4
#define CMD_CTR	    	0x5

#define CMD_Xaddr 		0x21
#define CMD_Yaddr 		0x20
#define CMD_Data    	0x22
#define CMD_Yaddr_Start 0x50
#define CMD_Yaddr_End 	0x51
#define CMD_Xaddr_Start 0x52
#define CMD_Xaddr_End 	0x53

#define BANK0 		0x0
#define BANK1 		0x1
#define BANK0_TOP 	0x2
#define BANK0_1 	0x3

#define _1_point  0    
#define _2D_Clear  1           //2d 加速       
#define _M_point  2           //多点写入，字体显示的时候不带背景颜色
#define _8_point  3           //8点写入，字体显示的时候有背景颜色

#define	HZ_column  16 //汉字字体大小
#define TRANSP  1           //字体显示的时候不带背景颜色
#define NORMAL  0           //字体显示的时候有背景颜色

extern void GLCD_init           (void);
extern void GLCD_init           (void);
extern void GLCD_clear          (unsigned short color);
extern void GLCD_setTextColor   (unsigned short color);
extern void GLCD_setBackColor   (unsigned short color);
extern void GLCD_displayChar    (unsigned int ln, unsigned int col, unsigned char  c);
extern void GLCD_displayStringLn(unsigned int ln, unsigned char *s);
extern void GLCD_bitmap         (unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned char *bitmap);
extern void GLCD_clearLn        (unsigned int ln);
extern void GLCD_putPixel       (unsigned int x, unsigned int y);
extern void FY_lcd_SetCursor	(unsigned int x,unsigned int y);
extern void FY_lcd_SetWindows	(unsigned short StartX,unsigned short StartY,unsigned short EndX,unsigned short EndY)	;
extern void FY_lcd_Clear		(unsigned short dat) ;
extern unsigned short FY_lcd_GetPoint		(unsigned short x,unsigned short y);
extern void FY_lcd_SetPoint		(unsigned short x,unsigned short y,unsigned short point);
extern void GLCD_setTextColor	(unsigned short color);
extern void GLCD_setBackColor	(unsigned short color);
extern void GLCD_clear 			(unsigned short color);
extern void DrawSingleAscii		(unsigned int x, unsigned int y, unsigned char *pAscii, unsigned int LineColor,unsigned int FillColor, unsigned char Mod);
extern void DrawSingleHz		(unsigned int x, unsigned int y, unsigned char *pHz,unsigned int LineColor,unsigned int FillColor, unsigned char Mod);
extern void DrawString			(unsigned int x, unsigned int y, unsigned char *pStr, unsigned int LineColor,unsigned int FillColor, unsigned char Mod);
extern void DispNum(unsigned int x, unsigned int y, unsigned short num);
extern void FY_lcd_SetCursor	(unsigned int x,unsigned int y);
extern void FY_lcd_Test       	(void);

extern  __inline void wr_dat 	(unsigned short c);
extern  __inline void wr_cmd 	(unsigned char c); 
extern  __inline unsigned short rd_dat (void);
extern  __inline void wr_reg 	(unsigned char reg, unsigned short val);
#endif /* _GLCD_H */
