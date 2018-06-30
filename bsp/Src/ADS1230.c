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

/**
 * @brief  Initializes DWT_Clock_Cycle_Count for DWT_Delay_us function
 * @return Error DWT counter
 *         1: clock cycle counter not started
 *         0: clock cycle counter works
 */
uint32_t DWT_Delay_Init(void) {
	/* Disable TRC */
	CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;  // ~0x01000000;
	/* Enable TRC */
	CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk;  // 0x01000000;

	/* Disable clock cycle counter */
	DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;  //~0x00000001;
	/* Enable  clock cycle counter */
	DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk;  //0x00000001;

	/* Reset the clock cycle counter value */
	DWT->CYCCNT = 0;

	/* 3 NO OPERATION instructions */
	__ASM volatile("NOP");
	__ASM volatile("NOP");
	__ASM volatile("NOP");

	/* Check if clock cycle counter has started */
	if (DWT->CYCCNT)
	{
		return 0; /*clock cycle counter started*/
	}
	else
	{
		return 1; /*clock cycle counter not started*/
	}
}
/*****************************************************************************
函数名称：ReadAD(void)
功    能：读AD
入口参数：无
返回参数：AD的转换结果，为long型
使用资源：无
******************************************************************************/


int32_t ReadAD(void)
{
	int32_t ADdatatemp = 0; 
	int16_t _raw;
	HAL_SPI_Receive(&hspi2, (uint8_t *)& ADdatatemp, 2, 100);
//	ADdatatemp = _raw << 16;
//	ADdatatemp |= _raw;
	//	int32_t ADdatatemp = 0; 
	//	for (uint8_t i = 0; i < 20; i++)            //获取20位数据
	//		{
	//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);   
	//			delay_us(1);
	//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);   
	//			ADdatatemp = ADdatatemp << 1;
	//			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4))                     //AD输出的数据位是1？
	//				{
	//					ADdatatemp++;
	//				}
	//		}    
	////	for (uint8_t i = 0; i < 4; i++)             //为下一次转换准备
	////		{
	////			HAL_GPIO_WritePin(adc_out_GPIO_Port, adc_out_Pin, GPIO_PIN_SET);  
	////			 delay_us(1);
	////			HAL_GPIO_WritePin(adc_out_GPIO_Port, adc_out_Pin, GPIO_PIN_RESET);                          //ADS_OUT &= ~ADS_CLK_BIT;
	////		}
//		ADdatatemp <<= 6;
	ADdatatemp &= 0x000fffff;
	return (ADdatatemp);
//	int32_t _raw;
//	for (int8_t a = 0; a < 20; a++) {
//		HAL_GPIO_WritePin(adc_out_GPIO_Port, adc_out_Pin, GPIO_PIN_SET);  
//		delay_us(1);
//		_raw <<= 1;
//		_raw |=  HAL_GPIO_ReadPin(adc_in_GPIO_Port, adc_in_Pin);
//		//delay_us(1);
//		HAL_GPIO_WritePin(adc_out_GPIO_Port, adc_out_Pin, GPIO_PIN_RESET);  
//		delay_us(1);
//	}
//	_raw <<= 6;
//	_raw &= 0x000fffff;
//	return _raw;
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
	uint8_t time = 20;
	while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)&&time)
	{
		time--;
		if (time==0)
		{
			printf("adc初始化错\n");
		}
		osDelay(100);
	}
	HAL_SPI_Transmit(&hspi2, 0, 2, 100);
//	for (int8_t a = 0; a < 26; a++) {
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);  
//		delay_us(2);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);  
//		delay_us(2);
//	} 
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
	//	if (DWT_Delay_Init())
	//	{
	//		Error_Handler(); /* Call Error Handler */
	//	}
    powerDown();
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);  
	osDelay(100);
	wakeUp();
	
	osDelay(100);
	OffsetAD();

    //启动转换
}