#include "spi.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F429������
//SPI��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2016/16/16
//�汾��V1.1
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
//����˵��
//V1.1 20160116
//������SPI1��Ӧ�ô���
////////////////////////////////////////////////////////////////////////////////// 	 

//SPI_HandleTypeDef SPI5_Handler;  //SPI5���
SPI_HandleTypeDef SPI1_Handler;  //SPI1���

//������SPIģ��ĳ�ʼ�����룬���ó�����ģʽ 						  
//SPI�ڳ�ʼ��
//�������Ƕ�SPI5�ĳ�ʼ��
//void SPI5_Init(void)
//{
//    SPI5_Handler.Instance=SPI5;                         //SP5
//    SPI5_Handler.Init.Mode=SPI_MODE_MASTER;             //����SPI����ģʽ������Ϊ��ģʽ
//    SPI5_Handler.Init.Direction=SPI_DIRECTION_2LINES;   //����SPI�������˫�������ģʽ:SPI����Ϊ˫��ģʽ
//    SPI5_Handler.Init.DataSize=SPI_DATASIZE_8BIT;       //����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
//    SPI5_Handler.Init.CLKPolarity=SPI_POLARITY_HIGH;    //����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
//    SPI5_Handler.Init.CLKPhase=SPI_PHASE_2EDGE;         //����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
//    SPI5_Handler.Init.NSS=SPI_NSS_SOFT;                 //NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
//    SPI5_Handler.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_256;//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
//    SPI5_Handler.Init.FirstBit=SPI_FIRSTBIT_MSB;        //ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
//    SPI5_Handler.Init.TIMode=SPI_TIMODE_DISABLE;        //�ر�TIģʽ
//    SPI5_Handler.Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE;//�ر�Ӳ��CRCУ��
//    SPI5_Handler.Init.CRCPolynomial=7;                  //CRCֵ����Ķ���ʽ
//    HAL_SPI_Init(&SPI5_Handler);
//    
//    __HAL_SPI_ENABLE(&SPI5_Handler);                    //ʹ��SPI5
//    SPI5_ReadWriteByte(0Xff);                           //��������
//}

//������SPIģ��ĳ�ʼ�����룬���ó�����ģʽ 						  
//SPI�ڳ�ʼ��
//�������Ƕ�SPI1�ĳ�ʼ��
void SPI1_Init(void)
{
    SPI1_Handler.Instance=SPI1;                      //SP2
    SPI1_Handler.Init.Mode=SPI_MODE_MASTER;          //����SPI����ģʽ������Ϊ��ģʽ
    SPI1_Handler.Init.Direction=SPI_DIRECTION_2LINES;//����SPI�������˫�������ģʽ:SPI����Ϊ˫��ģʽ
    SPI1_Handler.Init.DataSize=SPI_DATASIZE_8BIT;    //����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
    SPI1_Handler.Init.CLKPolarity=SPI_POLARITY_HIGH; //����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
    SPI1_Handler.Init.CLKPhase=SPI_PHASE_2EDGE;      //����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
    SPI1_Handler.Init.NSS=SPI_NSS_SOFT;              //NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
    SPI1_Handler.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_256;//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
    SPI1_Handler.Init.FirstBit=SPI_FIRSTBIT_MSB;     //ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
    SPI1_Handler.Init.TIMode=SPI_TIMODE_DISABLE;     //�ر�TIģʽ
    SPI1_Handler.Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE;//�ر�Ӳ��CRCУ��
    SPI1_Handler.Init.CRCPolynomial=7;               //CRCֵ����Ķ���ʽ
    HAL_SPI_Init(&SPI1_Handler);
    
    __HAL_SPI_ENABLE(&SPI1_Handler);                 //ʹ��SPI1
    SPI1_ReadWriteByte(0Xff);                        //��������
}
//SPI5�ײ�������ʱ��ʹ�ܣ���������
//�˺����ᱻHAL_SPI_Init()����
//hspi:SPI���
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_GPIOA_CLK_ENABLE();       //ʹ��GPIOFʱ��
//    __HAL_RCC_SPI5_CLK_ENABLE();        //ʹ��SPI5ʱ��
    __HAL_RCC_SPI1_CLK_ENABLE();        //ʹ��SPI1ʱ��
    
    //SPI5���ų�ʼ��PF7,8,9
//    GPIO_Initure.Pin=GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
//    GPIO_Initure.Mode=GPIO_MODE_AF_PP;              //�����������
//    GPIO_Initure.Pull=GPIO_PULLUP;                  //����
//    GPIO_Initure.Speed=GPIO_SPEED_FAST;             //����            
//    GPIO_Initure.Alternate=GPIO_AF5_SPI5;           //����ΪSPI5
//    HAL_GPIO_Init(GPIOF,&GPIO_Initure);
    
    //SPI1���ų�ʼ��PB13,14,15
    GPIO_Initure.Pin=GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;   
    GPIO_Initure.Alternate=GPIO_AF5_SPI1;           //��
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;              //�����������
    GPIO_Initure.Pull=GPIO_PULLUP;                  //����
    GPIO_Initure.Speed=GPIO_SPEED_FAST;             //���� ��ΪSPI1
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);             //��ʼ��
}

//SPI�ٶ����ú���
//SPI�ٶ�=fAPB1/��Ƶϵ��
//@ref SPI_BaudRate_Prescaler:SPI_BAUDRATEPRESCALER_2~SPI_BAUDRATEPRESCALER_2 256
//fAPB1ʱ��һ��Ϊ45Mhz��
//void SPI5_SetSpeed(u8 SPI_BaudRatePrescaler)
//{
//    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));//�ж���Ч��
//    __HAL_SPI_DISABLE(&SPI5_Handler);            //�ر�SPI
//    SPI5_Handler.Instance->CR1&=0XFFC7;          //λ3-5���㣬�������ò�����
//    SPI5_Handler.Instance->CR1|=SPI_BaudRatePrescaler;//����SPI�ٶ�
//    __HAL_SPI_ENABLE(&SPI5_Handler);             //ʹ��SPI
//    
//}

//SPI�ٶ����ú���
//SPI�ٶ�=fAPB1/��Ƶϵ��
//@ref SPI_BaudRate_Prescaler:SPI_BAUDRATEPRESCALER_2~SPI_BAUDRATEPRESCALER_2 256
//fAPB1ʱ��һ��Ϊ45Mhz��
void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));//�ж���Ч��
    __HAL_SPI_DISABLE(&SPI1_Handler);            //�ر�SPI
    SPI1_Handler.Instance->CR1&=0XFFC7;          //λ3-5���㣬�������ò�����
    SPI1_Handler.Instance->CR1|=SPI_BaudRatePrescaler;//����SPI�ٶ�
    __HAL_SPI_ENABLE(&SPI1_Handler);             //ʹ��SPI
    
}

//SPI5 ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
//u8 SPI5_ReadWriteByte(u8 TxData)
//{
//    u8 Rxdata;
//    HAL_SPI_TransmitReceive(&SPI5_Handler,&TxData,&Rxdata,1, 1000);       
// 	return Rxdata;          		    //�����յ�������		
//}

//SPI1 ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
u8 SPI1_ReadWriteByte(u8 TxData)
{
    u8 Rxdata;
    HAL_SPI_TransmitReceive(&SPI1_Handler,&TxData,&Rxdata,1, 1000);       
 	return Rxdata;          		    //�����յ�������		
}
