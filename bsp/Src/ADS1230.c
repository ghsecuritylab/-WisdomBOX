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
#include "math.h"


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

uint8_t _raw[5];
//int32_t ADdatatemp = 0; 

void filter(int32_t * adcsum)
{
	int32_t  value_buff;
	char count;
	int32_t sum = 0;
	HAL_SPI_MspDeInit(&hspi2);
	osDelay(10);
	InitADgpio();
	osDelay(10);
	ReadAD(adcsum);
	//		for (count = 0; count < 4; count++)
	//		{
	//			osDelay(200);
	//			ReadAD(&value_buff);
	//			if (value_buff>25000)
	//			{
	//				osDelay(200);
	//				ReadAD(&value_buff);
	//			}
	//			sum = sum  +   value_buff;
	//		
	//		}
		*adcsum = *adcsum - 100;
	if (*adcsum<0)
	{
		*adcsum = 0;
	}
		*adcsum = *adcsum*(0.53 - (0.000095**adcsum));     // sum * 0.25;
	HAL_SPI_MspInit(&hspi2);
}
void ReadAD(int32_t * ADdatatemp)
{

 
//	    HAL_SPI_Transmit(&hspi2, (uint8_t *) 0, 1, 100);
//		HAL_SPI_Receive(&hspi2, (uint8_t *)_raw, 5, 100);
	//////	printf("0:%d\n", _raw[0]);
	//////	printf("1:%d\n", _raw[1]);
	//////	ADdatatemp = _raw[0] >> 16;
	//////	ADdatatemp |= _raw[1]&0xffff0000;
//	 *ADdatatemp = ~*ADdatatemp;
			// *ADdatatemp >>= 16;
	//	*ADdatatemp &= 0x000fffff;

	
	/********************************************************************************/	
	
	for(uint8_t i = 0 ; i < 20 ; i++)            //获取20位数据
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET); 
		delay_us(1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);  
		*ADdatatemp = * ADdatatemp << 1;
		*ADdatatemp |=  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
	}
				
//	for (uint8_t i = 0; i < 4; i++)             //为下一次转换准备
//		{
//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);  
//			delay_us(1);
//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);                             //ADS_OUT &= ~ADS_CLK_BIT;
//		}
	*ADdatatemp &= 0x000fffff;

	

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
	while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)&&time--)
	{
		time--;
		if (time==0)
		{
			printf("adc初始化错\n");
		}
		osDelay(100);
	}
//	HAL_SPI_Transmit(&hspi2,(uint8_t *) 0xff, 3, 100);
	for (int8_t a = 0; a < 26; a++) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);  
		delay_us(1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);  
		delay_us(1);
	} 
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
void InitADgpio(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PtPin */
	GPIO_InitStruct.Pin = GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
}
void InitADline(void)
{
	

    powerDown();
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);  
	osDelay(50);
	wakeUp();
	
	osDelay(50);
	OffsetAD();
	osDelay(100);

    //启动转换
}