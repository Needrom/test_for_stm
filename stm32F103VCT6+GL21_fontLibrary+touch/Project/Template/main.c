/**
  ******************************************************************************
  * @file    FSMC/NAND/main.c 
  * @author  MCD Application Team
  * @version V3.1.0
  * @date    06/19/2009
  * @brief   Main program body 
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "glcd.h"
#include "25vf.h"
#include "touch.h"
/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	//SS
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 ;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		 //推挽输出
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

  	/* Touch Int (PB.0) as input floating */
  	GPIO_InitStructure.GPIO_Pin = touch_INT_pin;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;			//floating
  	GPIO_Init(GPIOB, &GPIO_InitStructure);
  	/* Flash Cs (PB.1)   */
	GPIO_InitStructure.GPIO_Pin =  CE_25 ;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		 //推挽输出
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);
}


int main(void)
{
	unsigned int T_x ;
    unsigned int T_y ;
  /* System Clocks Configuration */
  RCC_Configuration();
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC 
         | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE  
         | RCC_APB2Periph_AFIO, ENABLE);
  	spi_init();
  	GPIO_Config();
	touch_Init() ;
   	GLCD_init ();
   	GLCD_clear(Black);
   	GLCD_clear(Blue);
   	GLCD_displayStringLn(Line1, "   MCBSTM32E Demo   ");
   	GLCD_displayStringLn(Line2, "      SD_File       ");
   	GLCD_displayStringLn(Line3, "    www.keil.com    ");	
	FY_lcd_SetCursor(0,120);
   	FY_lcd_Test();
	delay_ms(2000);
	DrawString(0, 184, "提供Keil C51演示程序",Yellow,Red,NORMAL);
  	while(1)
	{
		if (touch_INT==0)
        {
            touch_GetAdXY(&T_x, &T_y);
            DispNum(20, 20, T_x);
            DispNum(20, 40, T_y);
        }	  
   	} 
}

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void RCC_Configuration(void)
{
  /* Setup the microcontroller system. Initialize the Embedded Flash Interface,
     initialize the PLL and update the SystemFrequency variable. */
  SystemInit();
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
/**
  * @}
  */ 

/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
