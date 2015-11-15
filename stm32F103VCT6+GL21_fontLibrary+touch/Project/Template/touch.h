#define	TP_CS			GPIO_Pin_4
#define	TP_CS_0			GPIO_ResetBits(GPIOA, TP_CS)
#define	TP_CS_1			GPIO_SetBits  (GPIOA, TP_CS)

#define	touch_INT_pin		GPIO_Pin_0
#define touch_INT           GPIO_ReadInputDataBit(GPIOB, touch_INT_pin)

void SpiDelay(unsigned int DelayCnt);
void touch_Init(void);
u16 TPReadX(void);
u16 TPReadY(void) ;
void touch_GetAdXY(unsigned int *x,unsigned int *y) ;




