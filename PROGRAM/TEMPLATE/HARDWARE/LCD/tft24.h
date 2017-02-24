#ifndef _TFT24_H
#define _TFT24_H
#include "stm32l4xx.h"
#include "delay.h"

#define TFT24_DATAGPIO GPIOB
#define TFT24_CTRLGPIO GPIOC

#define TFT24_DATA_GPIO_CLOEN __HAL_RCC_GPIOB_CLK_ENABLE()
#define TFT24_CTRL_GPIO_CLOEN __HAL_RCC_GPIOC_CLK_ENABLE()

#define TFT24_WR_CLR	TFT24_CTRLGPIO->BSRR=1<<23		//P25д���ƽŵ͵�ƽ
#define TFT24_WR_SET	TFT24_CTRLGPIO->BSRR=1<<7		//P25д���ƽŸߵ�ƽ

#define TFT24_RS_CLR	TFT24_CTRLGPIO->BSRR=1<<24		//P26��������ѡ��˵͵�ƽ
#define TFT24_RS_SET	TFT24_CTRLGPIO->BSRR=1<<8		//P26��������ѡ��˸ߵ�ƽ

#define TFT24_CS_CLR 	TFT24_CTRLGPIO->BSRR=1<<25		//P27Ƭѡ�ŵ͵�ƽ
#define TFT24_CS_SET 	TFT24_CTRLGPIO->BSRR=1<<9			//P27Ƭѡ�Ÿߵ�ƽ

#define TFT24_RD_CLR 	TFT24_CTRLGPIO->BSRR=1<<22    //P32�����ƽŵ͵�ƽ
#define TFT24_RD_SET 	TFT24_CTRLGPIO->BSRR=1<<6    	//P32�����ƽŸߵ�ƽ

#define	TFT24_RST_CLR TFT24_CTRLGPIO->BSRR=1<<20		//P33��λ�ŵ͵�ƽ
#define	TFT24_RST_SET TFT24_CTRLGPIO->BSRR=1<<4			//P33��λ�Ÿߵ�ƽ

#define TFT24_DATA(x) TFT24_DATAGPIO->ODR=x  				//16λ����

//#define TFT24_XMAX 	175		          								//����TFT���Ĵ�С
//#define TFT24_YMAX 	219
#define TFT24_XMAX 	240	          								//����TFT���Ĵ�С
#define TFT24_YMAX 	320

#define WHITE 			0xFFFF
#define BLACK 			0x0000
#define RED       	0xF800
#define GREEN    		0x07E0	
#define BLUE 				0x001F 
#define YELLOW   		0xFFE0
#define GRAY  			0X8430 
#define BRED  			0XF81F	
#define GRED  			0XFFE0	
#define GBLUE 			0X07FF	
#define MAGENTA   	0xF81F  
#define CYAN      	0x7FFF	
#define BROWN 			0XBC40  
#define BRRED 			0XFC07  
#define DARKBLUE  	0X01CF	
#define LIGHTBLUE 	0X7D7C	
#define GRAYBLUE  	0X5458  
#define LIGHTGREEN 	0X841F  
#define LIGHTGRAY 	0XEF5B  
#define LGRAY 			0XC618  
#define LGRAYBLUE   0XA651  
#define LBBLUE      0X2B12  

void TFT24_WriteCmd(int cmd);
void TFT24_WriteData(int dat);
void TFT24_WriteCmdData(int cmd,int dat);
void TFT24_GPIO_Init();
void TFT24_Init();
void TFT24_SetWindow(int xStart,int yStart,int xEnd,int yEnd);
void TFT24_ClearScreen(int color);//ˢ������
void GUI_Dot(int x,int y,int color);//����������ɫ����
void TFT24_Line(int xStart,int yStart, char xEnd,int yEnd,int color);//���ߺ���
void TFT24_DrawRectangle(int x1,int y1,int x2,int y2,int color);
void TFT24_Write_Char816(int x,int y,int n,int wordColor,int backColor);//��ʾASCII��
void TFT24_Write_CnChar1616(int x,int y,int n,int wordColor,int backColor);//��ʾ����
void TFT24_Write_CnChar3232(int x,int y,int n,int wordColor,int backColor,const unsigned char *p);//��ʾ����
void LCD_Drawbmp(int x,int y,int width,int height,const unsigned char *p);

#endif
