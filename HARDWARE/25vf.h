#define	CE_25			GPIO_Pin_1
#define	CE_25_0			GPIO_ResetBits(GPIOB, CE_25)
#define	CE_25_1			GPIO_SetBits  (GPIOB, CE_25)
extern unsigned char SPI_WriteByte(unsigned char data);
extern void spi_init(void);
extern void Read_25(unsigned int Dst,unsigned char *buff,unsigned char num);
//------------------------------------------------------
//  在GT21L16S2W芯片中数据的地址
//------------------------------------------------------
#define ASC0808D2HZ_ADDR  	( 0x66c0 ) 		//7*8 ascii code
#define ASC0812M2ZF_ADDR ( 0x66d40 )   		//6*12 ascii code
#define GBEX0816ZF_ADDR  243648   	   		//8*16 ascii code

#define ZF1112B2ZF_ADDR ( 0x3cf80 )	   		//12*12 12点字符
#define HZ1112B2HZ_ADDR  ( 0x3cf80+376*24 )	//12*12 12点汉字

#define CUTS1516ZF_ADDR  0x00  				//16*16 16点字符
#define JFLS1516HZ_ADDR  27072  			//16*16 16点汉字
