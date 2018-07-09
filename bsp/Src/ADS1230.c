
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




/*****************************************************************************
函数名称：ReadAD(void)
功    能：读AD
入口参数：无
返回参数：AD的转换结果，为long型
使用资源：无
******************************************************************************/

int32_t  offset;

uint8_t filter(int32_t * adcsum)
{
	int32_t  value_buff;
	//	osDelay(500);
		if(ReadAD(&value_buff)) 
	{
		*adcsum = 0xffff;
		return 1;
	}
	
//	if (value_buff==0)
//	{
//		printf("read0\n");
//	}
//	printf("adc:%d\n", value_buff);

		*adcsum = value_buff - offset;
		if (*adcsum<0)
	{
		*adcsum = 0;
	}
//	*adcsum =	5 * (*adcsum / ((1L << 20) *64 * (0.0005 / 1000)));
//	*adcsum =	 (*adcsum / 42)*20;
	*adcsum = *adcsum*(1.2 - (0.000448**adcsum));

	return 0;	
}

uint8_t  ReadAD(int32_t * ADdatatemp)
{
	uint16_t Timeout = 5000;
	uint32_t tickstart = 0U;
	int16_t  _raw[2];
	tickstart = HAL_GetTick();
	while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14))
	{
			/* Timeout management */
			if ((Timeout == 0U) || ((Timeout != HAL_MAX_DELAY) && ((HAL_GetTick() - tickstart) >=  Timeout)))
			{
			   return 1;
			}
		osDelay(1);
	}
	HAL_SPI_Receive(&hspi2, (uint8_t*)_raw, 2, 100);
				     *ADdatatemp = 0;
					 *ADdatatemp |= _raw[0]&0xffff;
					*ADdatatemp <<= 16;
					*ADdatatemp |= (_raw[1]) & 0xffff;
					*ADdatatemp /= 1L << 12;
	return 0;	
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
//	HAL_SPI_Transmit(&hspi2, (uint8_t *) 0, 1, 100);
	uint16_t Timeout= 5000;
	uint32_t tickstart = 0U;
	tickstart = HAL_GetTick();
	while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14))
	{
		/* Timeout management */
		if ((Timeout == 0U) || ((Timeout != HAL_MAX_DELAY) && ((HAL_GetTick() - tickstart) >=  Timeout)))
		{
			printf("adctimout\n");
			break;
		}
		osDelay(1);
	}
	HAL_SPI_Transmit(&hspi2,(uint8_t *) 0, 2, 100);
	osDelay(50);
	ReadAD(&offset);
	ReadAD(&offset);
	if (offset>1000)
	{
		offset = 0;
	}
	printf("offset:%d\n", offset);
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
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);  
	osDelay(100);
	wakeUp();
	
	osDelay(50);
	OffsetAD();
	osDelay(100);
    //启动转换
}