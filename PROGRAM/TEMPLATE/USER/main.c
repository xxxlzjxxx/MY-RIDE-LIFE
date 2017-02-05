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
#include "adc.h"
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

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
    RTC_TimeTypeDef RTC_TimeStruct;
    RTC_DateTypeDef RTC_DateStruct;
    u16 adcx;
	float temp;
    HAL_Init(); //初始化 HAL 库
  /* Configure the System clock to have a frequency of 80 MHz */
    SystemClock_Config(1, 20, 2, 7, 2);
    delay_init(80);                //初始化延时函数
    uart_init(115200);              //初始化USART
    usmart_dev.init(80); 		    //初始化USMART
    LED_Init();                     //初始化LED
    MY_ADC_Init();                  //初始化ADC1通道9  
    RTC_Init();                     //初始化RTC 
    RTC_Set_WakeUp(RTC_WAKEUPCLOCK_CK_SPRE_16BITS,0); //配置WAKE UP中断,1秒钟中断一次  		   
    while(1)
    {
//		t++;
//		if((t%100)==0)	//每1000ms更新一次显示数据
//		{
        HAL_RTC_GetTime(&RTC_Handler,&RTC_TimeStruct,RTC_FORMAT_BIN);
        printf("Time:%02d:%02d:%02d  ",RTC_TimeStruct.Hours,RTC_TimeStruct.Minutes,RTC_TimeStruct.Seconds); 
//			LCD_ShowString(30,140,210,16,16,tbuf);	
        HAL_RTC_GetDate(&RTC_Handler,&RTC_DateStruct,RTC_FORMAT_BIN);
        printf("Date:20%02d-%02d-%02d  ",RTC_DateStruct.Year,RTC_DateStruct.Month,RTC_DateStruct.Date); 
//			LCD_ShowString(30,160,210,16,16,tbuf);	
        printf("Week:%d\t",RTC_DateStruct.WeekDay); 
//			LCD_ShowString(30,180,210,16,16,tbuf);
        adcx=Get_Adc_Average(ADC_CHANNEL_TEMPSENSOR,10);//获取通道5的转换值，20次取平均
        printf("ADC1_TEMPSENSOR_VALUE: %d " , adcx);
        temp=(((float)adcx*(3.3/4096)) - 0.76) / 0.0025 + 25;          //获取计算后的带小数的实际电压值，比如3.1111
        printf("VALUE: %.3f\r\n",temp);
        
        LED0_Toggle;
//		} 
//		if((t%200)==0)LED0_Toggle;	//每200ms,翻转一次LED0 
        delay_ms(1000);
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
  *            PLL_R                          = 2
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
    __HAL_RCC_PWR_CLK_ENABLE(); //使能 PWR 时钟
//以下为MSI时钟的配置
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
    
    //下面这个设置用来设置调压器输出电压级别，以便在器件未以最大频率工作
    //时使性能与功耗实现平衡，此功能只有 STM32F42xx 和 STM32F43xx 器件有，
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
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
