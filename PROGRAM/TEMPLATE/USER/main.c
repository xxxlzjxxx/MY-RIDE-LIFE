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
#include "usart3.h"	   
#include "gps.h"
#include "string.h"
//#include "24l01.h"
//#include "adc.h"
//#include "dac.h"
//#include "lcd.h"
//#include "font.h"
//#include "pic.h"
//#include "24cxx.h"
//#include "w25qxx.h"
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

u8 USART1_TX_BUF[USART3_MAX_RECV_LEN]; 					//����1,���ͻ�����
nmea_msg gpsx; 											//GPS��Ϣ
__align(4) u8 dtbuf[50];   								//��ӡ������
const u8*fixmode_tbl[4]={"Fail","Fail"," 2D "," 3D "};	//fix mode�ַ���

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
    //����ΪGPS����ʹ�ñ���
	u16 i,rxlen;
	u16 lenx;
	u8 key=0XFF;
	u8 upload=0;    
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
//    u8 key;
//	u16 i=0;
//	u8 datatemp[SIZE];
//    u32 nFLASH_SIZE;
    //����Ϊ����NRF24L01ʹ�ñ���
//    u8 key,mode;
//	u16 t=0;			 
//	u8 tmp_buf[33];
//**********************************************************************************
    HAL_Init(); //��ʼ�� HAL ��
  /* Configure the System clock to have a frequency of 40 MHz */
//    SystemClock_Config(1, 10, 4, 7, 2);       //80MHz
    SystemClock_Config(1, 10, 4, 7, 2);       //40MHz
    delay_init(80);                //��ʼ����ʱ����

    uart_init(115200);              //��ʼ��USART
    USART2_init(38400);
    printf(">>system reset.\r\n");
    usmart_dev.init(40); 		    //��ʼ��USMART
    LED_Init();                     //��ʼ��LED
    KEY_Init();                     //��ʼ������
//    LCD_Init();                     //��ʼ��LCD
    
    RTC_Init();                     //��ʼ��RTC 
    RTC_Set_WakeUp(RTC_WAKEUPCLOCK_CK_SPRE_16BITS,0); //����WAKE UP�ж�,1�����ж�һ�� 
//    AT24CXX_Init();				    //��ʼ��IIC
//    W25QXX_Init();				    //W25QXX��ʼ��
//     NRF24L01_Init();    		    //��ʼ��NRF24L01   

	if(Ublox_Cfg_Rate(1000,1)!=0)	//���ö�λ��Ϣ�����ٶ�Ϊ1000ms,˳���ж�GPSģ���Ƿ���λ. 
	{
   		printf("NEO-6M Setting.........     ");
		while((Ublox_Cfg_Rate(1000,1)!=0)&&key)	//�����ж�,ֱ�����Լ�鵽NEO-6M,�����ݱ���ɹ�
		{
			USART3_Init(9600);				//��ʼ������2������Ϊ9600(EEPROMû�б������ݵ�ʱ��,������Ϊ9600.)
	  		Ublox_Cfg_Prt(38400);			//��������ģ��Ĳ�����Ϊ38400
			USART3_Init(38400);				//��ʼ������2������Ϊ38400 
			Ublox_Cfg_Tp(1000000,100000,1);	//����PPSΪ1�������1��,������Ϊ100ms	    
			key=Ublox_Cfg_Cfg_Save();		//��������  
		}	  					 
	   	printf("NEO-6M Set Done!!\r\n");
		delay_ms(500); 
	}

    printf(">>ready!\r\n");
    while(1)
    {
        HAL_RTC_GetTime(&RTC_Handler,&RTC_TimeStruct,RTC_FORMAT_BIN);
        printf(">>Time:%02d:%02d:%02d ",RTC_TimeStruct.Hours,RTC_TimeStruct.Minutes,RTC_TimeStruct.Seconds); 	
        HAL_RTC_GetDate(&RTC_Handler,&RTC_DateStruct,RTC_FORMAT_BIN);
        printf("Date:20%02d-%02d-%02d ",RTC_DateStruct.Year,RTC_DateStruct.Month,RTC_DateStruct.Date); 	
        printf("Week:%d  \r\n" ,RTC_DateStruct.WeekDay);

		delay_ms(1);
		if(USART3_RX_STA & 0X8000)		//���յ�һ��������
		{
			rxlen=USART3_RX_STA&0X7FFF;	//�õ����ݳ���
			for(i=0;i<rxlen;i++)
                USART1_TX_BUF[i]=USART3_RX_BUF[i];	   
 			USART3_RX_STA=0;		   	//������һ�ν���
			USART1_TX_BUF[i]=0;			//�Զ���ӽ�����
			GPS_Analysis(&gpsx,(u8*)USART1_TX_BUF);//�����ַ���
			Gps_Msg_Show();				//��ʾ��Ϣ	
			if(upload)
                printf("\r\n%s\r\n",USART1_TX_BUF);//���ͽ��յ������ݵ�����1
 		}
		key=KEY_Scan(0);
		if(key==KEY0_PRES)
		{
			upload=!upload;
			if(upload)
                printf("NMEA Data Upload:ON     \r\n");
			else 
                printf("NMEA Data Upload:OFF   \r\n");
 		}
		if((lenx%500)==0)
            LED0_Toggle; 	    				 
		lenx++;
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
