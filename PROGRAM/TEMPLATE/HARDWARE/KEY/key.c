#include "key.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
//ALIENTEK Mini STM32������
//�������� ��������		   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2014/3/06
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) �������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									   
//////////////////////////////////////////////////////////////////////////////////	 
 	    
//������ʼ������ 
//PA15��PC5 ���ó�����
void KEY_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_GPIOA_CLK_ENABLE(); //���� GPIOA ʱ��
    __HAL_RCC_GPIOC_CLK_ENABLE(); //���� GPIOC ʱ��
    
    GPIO_Initure.Pin=GPIO_PIN_0; //PA0
    GPIO_Initure.Mode=GPIO_MODE_INPUT; //����
    GPIO_Initure.Pull=GPIO_PULLDOWN; //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH; //����
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
    
    GPIO_Initure.Pin=GPIO_PIN_13; //PC13
    GPIO_Initure.Mode=GPIO_MODE_INPUT; //����
    GPIO_Initure.Pull=GPIO_PULLUP; //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH; //����
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);
} 
//������������
//���ذ���ֵ
//mode:0,��֧��������;1,֧��������;
//����ֵ��
//0��û���κΰ�������
//WKUP_PRES��WK_UP���� 
//ע��˺�������Ӧ���ȼ�,KEY0>KEY1>WK_UP!!
u8 KEY_Scan(u8 mode)
{	 
    static u8 key_up=1;     //�����ɿ���־
    if(mode==1)
    {
        key_up=1;    //֧������
    }
    if(key_up&&(KEY2==0||WK_UP==1))
    {
        delay_ms(10);
        key_up=0;
        if(KEY2==0)  
            return KEY2_PRES;
        else if(WK_UP==1) 
            return WKUP_PRES;          
    }
    else if(KEY2==1&&WK_UP==0)
    {
        key_up=1;
    }
    return 0;   //�ް�������
}