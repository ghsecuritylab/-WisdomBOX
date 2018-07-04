/*******************************************************************************
// 说明: 头文件声明
*******************************************************************************/
#include "ADS1230.h"
#include "stm32f4xx_hal.h"
#include "spi.h"
#include <stdint.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"


void delay_us(uint16_t us)
{
//	osDelay(us);
	uint16_t i;
	for (i = 0; i < (us*1000); i++)
	{
		asm("NOP");
	} 
}

/*****************************************************************************
函数名称：ReadAD(void)
功    能：读AD
入口参数：无
返回参数：AD的转换结果，为long型
使用资源：无
******************************************************************************/

int16_t _raw[2];
int32_t ADdatatemp = 0; 
int32_t ReadAD(void)
{
//	int32_t ADdatatemp = 0; 
////	int16_t _raw[2];
//	HAL_SPI_Receive(&hspi2, (uint8_t *)&ADdatatemp,2, 100);
////	printf("0:%d\n", _raw[0]);
////	printf("1:%d\n", _raw[1]);
////	ADdatatemp = _raw[0] >> 16;
////	ADdatatemp |= _raw[1]&0xffff0000;
////			ADdatatemp <<= 12;
////	ADdatatemp &= 0x000fffff;
//	return ADdatatemp;
	
/********************************************************************************/	
////		int32_t ADdatatemp = 0; 
//		for (uint8_t i = 0; i < 20; i++)            //获取20位数据
//			{
//				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET); 
//				delay_us(1);
//				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);  
//				ADdatatemp = ADdatatemp << 1;
//				if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14))                     //AD输出的数据位是1？
//					{
//						ADdatatemp++;
//					}
//			}    
//	//	for (uint8_t i = 0; i < 4; i++)             //为下一次转换准备
//	//		{
//	//			HAL_GPIO_WritePin(adc_out_GPIO_Port, adc_out_Pin, GPIO_PIN_SET);  
//	//			 delay_us(1);
//	//			HAL_GPIO_WritePin(adc_out_GPIO_Port, adc_out_Pin, GPIO_PIN_RESET);                          //ADS_OUT &= ~ADS_CLK_BIT;
//	//		}
////		ADdatatemp <<= 4;
//	ADdatatemp &= 0x000fffff;
//	return (ADdatatemp);
	
/********************************************************************************/	
//	int32_t ADdatatemp=0;
//	for (int8_t a = 0; a < 20; a++) {
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET); 
//		osDelay(1);
//		ADdatatemp <<= 1;
//		ADdatatemp |=  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
////		osDelay(1);
//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);  
////		osDelay(1);
//	}
//	ADdatatemp <<= 6;
//	ADdatatemp &= 0x000fffff;
//	return ADdatatemp;
}


/*****************************************************************************
函数名称：void OffsetAD()
功    能：一个补偿函数，补偿AD1230的标尺误差
入口参数：无
返回参数：无
使用资源：无
******************************************************************************/
void OffsetAD()
{
	uint8_t time = 10;
	while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)||time--)
	{
		time--;
		if (time==0)
		{
			printf("adc初始化错\n");
		}
		osDelay(100);
	}
//	HAL_SPI_Transmit(&hspi2, 0, 2, 100);
////	for (int8_t a = 0; a < 26; a++) {
////		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);  
////		osDelay(1);
////		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);  
////		osDelay(1);
////	} 
}
void wakeUp()
{
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET); 
}

void powerDown()
{
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET); 
}

/*****************************************************************************
函数名称：void InitADline(void)
功    能：初始化AD控制线
入口参数：无
返回参数：无
使用资源：port2
******************************************************************************/
void InitADline(void)
{
//	GPIO_InitTypeDef GPIO_InitStruct;
//	
//	GPIO_InitStruct.Pin = GPIO_PIN_10 ;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//	/*Configure GPIO pin : PtPin */
//	GPIO_InitStruct.Pin = GPIO_PIN_14;
//	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    powerDown();
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);  
	osDelay(100);
	wakeUp();
	
	osDelay(100);
	OffsetAD();

    //启动转换
}