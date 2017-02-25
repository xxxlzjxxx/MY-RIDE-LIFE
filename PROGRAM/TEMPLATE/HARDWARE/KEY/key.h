#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK miniSTM32������
//������������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/3
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////   	 


//#define KEY0 PCin(5)   	
//#define KEY1 PAin(15)	 
//#define WK_UP  PAin(0)	 
 

/*����ķ�ʽ��ͨ��ֱ�Ӳ��� HAL �⺯����ʽ��ȡ IO*/
#define KEY0 HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_5) //KEY0 ���� PC5
#define KEY1 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15) //KEY1 ���� PA15
#define KEY2 HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13) //KEY2 ���� PC13     ---->��ӦNUCLEO-L476�е�USER��
#define WK_UP HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) //WKUP ���� PA0
///*����ķ�ʽ��ͨ��λ��������ʽ��ȡ IO*/
//#define KEY0 PHin(3) //KEY0 ���� PH3
//#define KEY1 PHin(2) //KEY1 ���� PH2
//#define KEY2 PCin(13)//KEY2 ���� PC13
//#define WK_UP PAin(0) //WKUP ���� PA0
#define KEY0_PRES 0
#define KEY1_PRES 1
#define KEY2_PRES 3
#define WKUP_PRES 4  

void KEY_Init(void);//IO��ʼ��
u8 KEY_Scan(u8 mode);  	//����ɨ�躯��					    
#endif
