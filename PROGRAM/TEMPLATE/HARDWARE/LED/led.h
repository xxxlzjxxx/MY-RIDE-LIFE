#ifndef __LED_H
#define __LED_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
//ALIENTEKս��STM32������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) �������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
//#define LED0 PAout(5)	// PA5
//#define LED1 PAout(6)	// PA6	

#define LED0_Toggle HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5) 	// PA5
#define LED1_Toggle HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_6)	// PA6	

void LED_Init(void);//��ʼ��

		 				    
#endif