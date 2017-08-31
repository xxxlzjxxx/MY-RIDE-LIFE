#ifndef _TFT24_H
#define _TFT24_H
#include "stm32l4xx.h"
#include "delay.h"

#define TFT24_DATAGPIO GPIOB
#define TFT24_CTRLGPIO GPIOC

#define TFT24_DATA_GPIO_CLOEN __HAL_RCC_GPIOB_CLK_ENABLE()
#define TFT24_CTRL_GPIO_CLOEN __HAL_RCC_GPIOC_CLK_ENABLE()

#define TFT24_WR_CLR	TFT24_CTRLGPIO->BSRR=1<<16		//P25写控制脚低电平
#define TFT24_WR_SET	TFT24_CTRLGPIO->BSRR=1<<0			//P25写控制脚高电平

#define TFT24_RS_CLR	TFT24_CTRLGPIO->BSRR=1<<17		//P26数据命令选择端低电平
#define TFT24_RS_SET	TFT24_CTRLGPIO->BSRR=1<<1			//P26数据命令选择端高电平

#define TFT24_CS_CLR 	TFT24_CTRLGPIO->BSRR=1<<18		//P27片选脚低电平
#define TFT24_CS_SET 	TFT24_CTRLGPIO->BSRR=1<<2			//P27片选脚高电平

#define TFT24_RD_CLR 	TFT24_CTRLGPIO->BSRR=1<<19    //P32读控制脚低电平
#define TFT24_RD_SET 	TFT24_CTRLGPIO->BSRR=1<<3    	//P32读控制脚高电平

#define	TFT24_RST_CLR TFT24_CTRLGPIO->BSRR=1<<20		//P33复位脚低电平
#define	TFT24_RST_SET TFT24_CTRLGPIO->BSRR=1<<4			//P33复位脚高电平

#define TFT24_DATA(x) TFT24_DATAGPIO->ODR=x  				//16位数据

#define TFT24_XMAX 	175		          								//设置TFT屏的大小
#define TFT24_YMAX 	219

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

void TFT24_WriteCmd(int cmd)
{
	TFT24_CS_CLR;			  //打开片选
	TFT24_RS_CLR;			  //选择命令
	TFT24_DATA(cmd);
	TFT24_WR_CLR;			  //写入时序
	TFT24_WR_SET;
	TFT24_CS_SET;			  //关闭片选
}

void TFT24_WriteData(int dat)
{
	TFT24_CS_CLR;			  //打开片选
	TFT24_RS_SET;			  //选择发送数据
	TFT24_DATA(dat);
	TFT24_WR_CLR;			  //写入时序
	TFT24_WR_SET;
	TFT24_CS_SET;			  //关闭片选
}

void TFT24_WriteCmdData(int cmd,int dat)
{
	TFT24_WriteCmd(cmd);
	TFT24_WriteData(dat);
}

void TFT24_GPIO_Init()
{
	TFT24_DATA_GPIO_CLOEN;
	TFT24_CTRL_GPIO_CLOEN;
  GPIO_InitStruct.Mode=GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull=GPIO_PULLUP;
  GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_VERY_HIGH;
	
	GPIO_InitStruct.Pin=GPIO_PIN_All;
  HAL_GPIO_Init(TFT24_DATAGPIO,&GPIO_InitStruct);
	
	GPIO_InitStruct.Pin=GPIO_PIN_0|GPIO_PIN_1
	|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  HAL_GPIO_Init(TFT24_CTRLGPIO,&GPIO_InitStruct);
}

void TFT24_Init()
{
	TFT24_GPIO_Init();
	TFT24_RST_CLR;
	Delay_us(500);
	TFT24_RST_SET;
	Delay_us(500);
	TFT24_CS_CLR;
 	TFT24_WriteCmdData(0x0000,0x0001);
	TFT24_WriteCmdData(0x0007,0x0000);			
	TFT24_WriteCmdData(0x0012,0x0000);       
	TFT24_WriteCmdData(0x00A4,0x0001);
	Delay_ms(5);
//--------GAMMA SETTINGS---------
	TFT24_WriteCmdData(0x0008,0x0808); 
	TFT24_WriteCmdData(0x0018,0x0001);			
	TFT24_WriteCmdData(0x0010,0x11B0);			
	TFT24_WriteCmdData(0x0011,0x0000);
	TFT24_WriteCmdData(0x0012,0x1115);
	TFT24_WriteCmdData(0x0013,0x8B0B);
	TFT24_WriteCmdData(0x0012,0x1135);			
	TFT24_WriteCmdData(0x0014,0x8000);
	TFT24_WriteCmdData(0x0001,0x0100);
	TFT24_WriteCmdData(0x0002,0x0700); 
	TFT24_WriteCmdData(0x0003,0x1030);
	TFT24_WriteCmdData(0x0070,0x1B00);

	TFT24_WriteCmdData(0x0071,0x0001);
	TFT24_WriteCmdData(0x0090,0x0002);           
	TFT24_WriteCmdData(0x0091,0x0000);
	TFT24_WriteCmdData(0x0092,0x0001);
	TFT24_WriteCmdData(0x0007,0x0001);
	Delay_ms(5);
	TFT24_WriteCmdData(0x0007,0x0021);
	Delay_ms(5);
	TFT24_WriteCmdData(0x0012,0x1135);                  
	TFT24_WriteCmdData(0x0007,0x0233);                 			
	TFT24_WriteCmd(0x0022);
}

void TFT24_SetWindow(int xStart,int yStart,int xEnd,int yEnd)
{
	TFT24_WriteCmdData(0x0050,xStart);	 //水平线起始
	TFT24_WriteCmdData(0x0052,yStart);	 //垂直线起始
	TFT24_WriteCmdData(0x0051,xEnd);	 //水平线结束
	TFT24_WriteCmdData(0x0053,yEnd);	 //垂直线结束

	TFT24_WriteCmdData(0x0020,xEnd);
	TFT24_WriteCmdData(0x0021,yEnd);

	TFT24_WriteCmd(0x0022);	
}

void TFT24_ClearScreen(int color)//刷屏函数
{
	int i,j;
	TFT24_SetWindow(0,0,TFT24_XMAX,TFT24_YMAX);	 //作用区域
  	for(i=0;i<TFT24_XMAX+1;i++)
			for(j=0;j<TFT24_YMAX+1;j++)
				TFT24_WriteData(color);	
}

void GUI_Dot(int x,int y,int color)
//单个像素上色函数
{  
	int i;
	TFT24_SetWindow(x-1,y-1,x,y);  //单个像素
	for(i=0;i<16;i++)
		TFT24_WriteData(color);
}

void TFT24_Line(int xStart,int yStart, 
char xEnd,int yEnd,int color)
//画线函数
{
	int t;  
	int xerr=0,yerr=0,delta_x,delta_y,distance;  
	int incx,incy;  
	int row,col;  
	delta_x=xEnd-xStart;//计算坐标增量  
	delta_y=yEnd-yStart;  
	col=xStart;  
	row=yStart;  
	if (delta_x>0)incx=1;//设置单步方向  
	else    
	{  
	    if(delta_x==0)incx=0;//垂直线  
	    else 
		{
			incx=-1;
			delta_x=-delta_x;
		}  
	}  
	if(delta_y>0)incy=1;  
	else  
	{  
	    if (delta_y==0)incy=0;//水平线  
	    else 
		{
			incy = -1;
			delta_y=-delta_y;
		}  
	}  
	if (delta_x>delta_y)
	{ 
		distance=delta_x;//选取基本增量坐标轴  
	}
	else
	{
		distance=delta_y; 
	} 	
	for (t=0;t<=distance+1;t++)  
	{                                     //画线输出  
	    GUI_Dot(col,row,color);
	    xerr+=delta_x;  
	    yerr+=delta_y;  
	  	if(xerr>distance)  
	    {  
	        xerr-=distance;  
	        col+=incx;  
	    }  
	    if(yerr>distance)  
	    {  
	        yerr-=distance;  
	        row+=incy;  
	    }  
	}  
}

void TFT24_DrawRectangle(int x1,
int y1,int x2,int y2,int color)
{
	TFT24_Line(x1,y1,x2,y1,color);
	TFT24_Line(x1,y1,x1,y2,color);
	TFT24_Line(x1,y2,x2,y2,color);
	TFT24_Line(x2,y1,x2,y2,color);
} 

void TFT24_Write_Char816(int x,int y,int n,
int wordColor,int backColor)//显示ASCII码
{  
	int i,j,temp;
	n*=16;
	TFT24_SetWindow(x,y,x+7,y+15);
	for(i=n;i<n+16;i++)
	{
		//temp=read_24c256(i);
		for(j=0;j<8;j++)
		{
		if((temp&0x80)==0x80)
		TFT24_WriteData(wordColor);
		else 
		TFT24_WriteData(backColor);
		temp<<=1;
		}
	}
}

void TFT24_Write_CnChar1616(int x,int y,int n,
int wordColor,int backColor)//显示汉字
{  
	int i,j,temp;
	n*=32;
	TFT24_SetWindow(x,y,x+15,y+15);
	for(i=n;i<n+32;i++)
	{
		//temp=read_24c256(i+0x1000);
		for(j=0;j<8;j++)
		{
		if((temp&0x80)==0x80)
		TFT24_WriteData(wordColor);
		else 
		TFT24_WriteData(backColor);
		temp<<=1;
		}
	}
}

void TFT24_Write_CnChar3232(int x,int y,int n,
int wordColor,int backColor,
const unsigned char *p)//显示汉字
{  
	int i,j;
	char temp;
	n*=128;
	TFT24_SetWindow(x,y,x+31,y+31);
	for(i=n;i<n+128;i++)
	{	
		temp=p[i];
		for(j=0;j<8;j++)
		{
		if((temp&0x80)==0x80)
		TFT24_WriteData(wordColor);
		else 
		TFT24_WriteData(backColor);
		temp<<=1;
		}
	}
}

void LCD_Drawbmp(int x,int y,
int width,int height,
const unsigned char *p)
{
  int i,picH,picL;
	TFT24_SetWindow(x,y,x+width-1,y+height-1);
  for(i=0;i<width*height+4;i++)
	{	
	 	picL=*(p+i*2);	
		picH=*(p+i*2+1);				
		TFT24_WriteData(picH<<8|picL);  						
	}	
}

#endif
