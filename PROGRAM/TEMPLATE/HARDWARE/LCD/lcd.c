#include "stm32l4xx.h"
#include "delay.h"
#include "tft24.h"

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
    GPIO_InitTypeDef GPIO_InitStruct;
    
	TFT24_DATA_GPIO_CLOEN;
	TFT24_CTRL_GPIO_CLOEN;
    GPIO_InitStruct.Mode=GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull=GPIO_PULLUP;
    GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_VERY_HIGH;
	
	GPIO_InitStruct.Pin=GPIO_PIN_All;
    HAL_GPIO_Init(TFT24_DATAGPIO,&GPIO_InitStruct);
	
	GPIO_InitStruct.Pin=GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_4;
    HAL_GPIO_Init(TFT24_CTRLGPIO,&GPIO_InitStruct);
}

void TFT24_Init()
{
	TFT24_GPIO_Init();
    
    TFT24_WriteCmdData(0x00E5, 0x8000); // Set the Vcore voltage and this setting is must.
    TFT24_WriteCmdData(0x0000, 0x0001); // Start internal OSC.
    TFT24_WriteCmdData(0x0001, 0x0100); // set SS and SM bit
    TFT24_WriteCmdData(0x0002, 0x0700); // set 1 line inversion
    TFT24_WriteCmdData(0x0003, 0x0030); // set GRAM write direction and BGR=0.
    TFT24_WriteCmdData(0x0004, 0x0000); // Resize register
    TFT24_WriteCmdData(0x0008, 0x0202); // set the back porch and front porch
    TFT24_WriteCmdData(0x0009, 0x0000); // set non-display area refresh cycle ISC[3:0]
    TFT24_WriteCmdData(0x000A, 0x0000); // FMARK function
    TFT24_WriteCmdData(0x000C, 0x0000); // RGB interface setting
    TFT24_WriteCmdData(0x000D, 0x0000); // Frame marker Position
    TFT24_WriteCmdData(0x000F, 0x0000); // RGB interface polarity
    //*************Power On sequence ****************//
    TFT24_WriteCmdData(0x0010, 0x0000); // SAP, BT[3:0], AP, DSTB, SLP, STB
    TFT24_WriteCmdData(0x0011, 0x0007); // DC1[2:0], DC0[2:0], VC[2:0]
    TFT24_WriteCmdData(0x0012, 0x0000); // VREG1OUT voltage
    TFT24_WriteCmdData(0x0013, 0x0000); // VDV[4:0] for VCOM amplitude
    delay_ms(200); // Dis-charge capacitor power voltage
    TFT24_WriteCmdData(0x0010, 0x17B0); // SAP, BT[3:0], AP, DSTB, SLP, STB
    TFT24_WriteCmdData(0x0011, 0x0037); // DC1[2:0], DC0[2:0], VC[2:0]
    delay_ms(50); // Delay 50ms
    TFT24_WriteCmdData(0x0012, 0x013E); // VREG1OUT voltage
    delay_ms(50); // Delay 50ms
    TFT24_WriteCmdData(0x0013, 0x1F00); // VDV[4:0] for VCOM amplitude
    TFT24_WriteCmdData(0x0029, 0x0013); // VCM[4:0] for VCOMH
    delay_ms(50);
    TFT24_WriteCmdData(0x0020, 0x0000); // GRAM horizontal Address
    TFT24_WriteCmdData(0x0021, 0x0000); // GRAM Vertical Address
    // ----------- Adjust the Gamma Curve ----------//
    TFT24_WriteCmdData(0x0030, 0x0000);
    TFT24_WriteCmdData(0x0031, 0x0404);
    TFT24_WriteCmdData(0x0032, 0x0404);
    TFT24_WriteCmdData(0x0035, 0x0004);
    TFT24_WriteCmdData(0x0036, 0x0404);
    TFT24_WriteCmdData(0x0037, 0x0404);
    TFT24_WriteCmdData(0x0038, 0x0404);
    TFT24_WriteCmdData(0x0039, 0x0707);
    TFT24_WriteCmdData(0x003C, 0x0500);
    TFT24_WriteCmdData(0x003D, 0x0607);
    //------------------ Set GRAM area ---------------//
    TFT24_WriteCmdData(0x0050, 0x0000); // Horizontal GRAM Start Address

    TFT24_WriteCmdData(0x0051, 0x00EF); // Horizontal GRAM End Address
    TFT24_WriteCmdData(0x0052, 0x0000); // Vertical GRAM Start Address
    TFT24_WriteCmdData(0x0053, 0x013F); // Vertical GRAM Start Address
    TFT24_WriteCmdData(0x0060, 0x2700); // Gate Scan Line
    TFT24_WriteCmdData(0x0061, 0x0001); // NDL,VLE, REV
    TFT24_WriteCmdData(0x006A, 0x0000); // set scrolling line
    //-------------- Partial Display Control ---------//
    TFT24_WriteCmdData(0x0080, 0x0000);
    TFT24_WriteCmdData(0x0081, 0x0000);
    TFT24_WriteCmdData(0x0082, 0x0000);
    TFT24_WriteCmdData(0x0083, 0x0000);
    TFT24_WriteCmdData(0x0084, 0x0000);
    TFT24_WriteCmdData(0x0085, 0x0000);
    //-------------- Panel Control -------------------//
    TFT24_WriteCmdData(0x0090, 0x0010);
    TFT24_WriteCmdData(0x0092, 0x0000);
    TFT24_WriteCmdData(0x0093, 0x0003);
    TFT24_WriteCmdData(0x0095, 0x0110);
    TFT24_WriteCmdData(0x0097, 0x0000);
    TFT24_WriteCmdData(0x0098, 0x0000);
    TFT24_WriteCmdData(0x0007, 0x0173); // 262K color and display ON

        
    //	Write_Cmd_Data(0x0022);//
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

