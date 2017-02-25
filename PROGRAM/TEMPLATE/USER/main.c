/**
  ******************************************************************************
  * @file    Templates/Src/main.c 
  * @author  MCD Application Team
  * @version V1.6.0
  * @date    28-October-2016
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "usmart.h"
#include "RTC.h"
//#include "adc.h"
//#include "dac.h"
#include "lcd.h"
//#include "font.h"
//#include "pic.h"
//#include "24cxx.h"
#include "w25qxx.h"
/** @addtogroup STM32L4xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(u32 plln, u32 pllm, u32 pllr, u32 pllp,u32 pllq);

//extern const unsigned char gImage_pic[61258];
const u8 TEXT_Buffer[]={"NUCLEO-L476RG IIC TEST"};//Ҫд�뵽24c02���ַ�������
#define SIZE sizeof(TEXT_Buffer)
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
//����ΪRTC����ʹ��
    RTC_TimeTypeDef RTC_TimeStruct;
    RTC_DateTypeDef RTC_DateStruct;
//    u8 tbuf[40];
//    u16 adcx;
//����ΪDAC����ʹ��
//	float temp;
//    u8 t=0;	 
//	u16 dacval=0;
//	u8 key;
//����ΪLCD����ʵ��ʹ�ñ���
//    u8 x=0;
//	u8 lcd_id[12];
//����ΪEEPROM����ʹ��
    u8 key;
	u16 i=0;
	u8 datatemp[SIZE];
    u32 nFLASH_SIZE;
//**********************************************************************************
    HAL_Init(); //��ʼ�� HAL ��
  /* Configure the System clock to have a frequency of 40 MHz */
    SystemClock_Config(1, 20, 4, 7, 2);
    delay_init(40);                //��ʼ����ʱ����
//    TFT24_Init();                     //��ʼ��LCD
    uart_init(115200);              //��ʼ��USART
    printf(">>system reset.\r\n");
    usmart_dev.init(40); 		    //��ʼ��USMART
    LED_Init();                     //��ʼ��LED
    KEY_Init();                     //��ʼ������   
//    MY_ADC_Init();                  //��ʼ��ADC1ͨ��9  
    RTC_Init();                     //��ʼ��RTC 
    RTC_Set_WakeUp(RTC_WAKEUPCLOCK_CK_SPRE_16BITS,0); //����WAKE UP�ж�,1�����ж�һ�� 
//    AT24CXX_Init();				    //��ʼ��IIC
    W25QXX_Init();				    //W25QXX��ʼ��
    
//    POINT_COLOR=RED; 
//	sprintf((char*)lcd_id,"LCD ID:%04X",lcddev.id);//��LCD ID��ӡ��lcd_id���顣    
	while(W25QXX_ReadID()!=W25Q128)//��ⲻ��
	{
		printf(">>Check Failed!   please check!\r\n");
		delay_ms(500);
		LED1_Toggle;//DS0��˸
	}
    printf(">>ready!\r\n");
    nFLASH_SIZE=32*1024*1024;	//FLASH ��СΪ32M�ֽ�
    while(1)
    {
        HAL_RTC_GetTime(&RTC_Handler,&RTC_TimeStruct,RTC_FORMAT_BIN);
        printf(">>Time:%02d:%02d:%02d ",RTC_TimeStruct.Hours,RTC_TimeStruct.Minutes,RTC_TimeStruct.Seconds); 	
        HAL_RTC_GetDate(&RTC_Handler,&RTC_DateStruct,RTC_FORMAT_BIN);
        printf("Date:20%02d-%02d-%02d ",RTC_DateStruct.Year,RTC_DateStruct.Month,RTC_DateStruct.Date); 	
        printf("Week:%d  \r\n" ,RTC_DateStruct.WeekDay);
        
		key=KEY_Scan(0);
		if(key==KEY1_PRES)//KEY1����,д��24C02
		{    
			printf("Start Write ....");
//			AT24CXX_Write(0,(u8*)TEXT_Buffer,SIZE);
            W25QXX_Write((u8*)TEXT_Buffer,nFLASH_SIZE-100,SIZE);		//�ӵ�����100����ַ����ʼ,д��SIZE���ȵ�����
			printf("Write Finished!");//��ʾ�������
		}
		if(key==KEY0_PRES)//KEY0����,��ȡ�ַ�������ʾ
		{
			printf("Start Read .... ");
//			AT24CXX_Read(0,datatemp,SIZE);
            W25QXX_Read(datatemp,nFLASH_SIZE-100,SIZE);					//�ӵ�����100����ַ����ʼ,����SIZE���ֽ�
			printf("The Data Readed Is:  ");//��ʾ�������
			//LCD_ShowString(30,190,200,16,16,datatemp);//��ʾ�������ַ���
            for(i = 0; i < SIZE; i++)
            {
                printf("%c", datatemp[i]);
            }
		}
		i++;
		delay_ms(10);
		if(i==20)
		{
			LED0_Toggle;//��ʾϵͳ��������	
			i=0;
		}
	}  
} 
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 1
  *            PLL_N                          = 20
  *            PLL_R                          = 4
  *            PLL_P                          = 7
  *            PLL_Q                          = 2
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
//void Stm32_Clock_Init(u32 plln,u32 pllm,u32 pllp,u32 pllq)
void SystemClock_Config(u32 pllm, u32 plln, u32 pllr, u32 pllp,u32 pllq)
{
//    HAL_StatusTypeDef ret = HAL_OK;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;
    __HAL_RCC_PWR_CLK_ENABLE(); //ʹ�� PWR ʱ��
//����ΪMSIʱ�ӵ�����
//    /* MSI is enabled after System reset, activate PLL with MSI as source */
//    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
//    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
//    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
//    RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
//    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
//    RCC_OscInitStruct.PLL.PLLM = 1;
//    RCC_OscInitStruct.PLL.PLLN = 40;
//    RCC_OscInitStruct.PLL.PLLR = 2;
//    RCC_OscInitStruct.PLL.PLLP = 7;
//    RCC_OscInitStruct.PLL.PLLQ = 4;
    
    //������������������õ�ѹ�������ѹ�����Ա�������δ�����Ƶ�ʹ���
    //ʱʹ�����빦��ʵ��ƽ�⣬�˹���ֻ�� STM32F42xx �� STM32F43xx �����У�
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /* HSE is enabled after System reset, activate PLL with HSE as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = pllm;
    RCC_OscInitStruct.PLL.PLLN = plln;
    RCC_OscInitStruct.PLL.PLLR = pllr;
    RCC_OscInitStruct.PLL.PLLP = pllp;
    RCC_OscInitStruct.PLL.PLLQ = pllq;      
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        /* Initialization Error */
        while(1);
    }

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
    /* Initialization Error */
        while(1);
    }
    
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        while(1);
    }

//    /**Configure the main internal regulator output voltage 
//    */
//    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
//    {
//        while(1);
//    }

//    /**Configure the Systick interrupt time 
//    */
//    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

//    /**Configure the Systick 
//    */
//    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

//    /* SysTick_IRQn interrupt configuration */
//    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);    
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
