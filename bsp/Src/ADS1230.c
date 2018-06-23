/*******************************************************************************
// ˵��: ͷ�ļ�����
*******************************************************************************/
#include "ADS1230.h"
#include "stm32f7xx_hal.h"
#include "spi.h"
#include <stdint.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"


void delay_us(uint16_t us)
{
	uint16_t i;
	for (i = 0; i < (us*200); i++)
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
�������ƣ�ReadAD(void)
��    �ܣ���AD
��ڲ�������
���ز�����AD��ת�������Ϊlong��
ʹ����Դ����
******************************************************************************/

int32_t _raw;

int32_t ReadAD(void)
{
	for (int8_t a = 0; a < 20; a++) {
		HAL_GPIO_WritePin(spi_sck_GPIO_Port, spi_sck_Pin, GPIO_PIN_SET);  
		delay_us(1);
		_raw <<= 1;
		_raw |= HAL_GPIO_ReadPin(spi_miso_GPIO_Port, spi_miso_Pin);
		delay_us(1);
		HAL_GPIO_WritePin(spi_sck_GPIO_Port, spi_sck_Pin, GPIO_PIN_RESET);  
		delay_us(2);
	}
	_raw <<= 12;
	return _raw;
}


/*****************************************************************************
�������ƣ�void OffsetAD()
��    �ܣ�һ����������������AD1230�ı�����
��ڲ�������
���ز�������
ʹ����Դ����
******************************************************************************/
void OffsetAD()
{
	for (int8_t a = 0; a < 26; a++) {
		HAL_GPIO_WritePin(spi_sck_GPIO_Port, spi_sck_Pin, GPIO_PIN_SET);  
		delay_us(2);
		HAL_GPIO_WritePin(spi_sck_GPIO_Port, spi_sck_Pin, GPIO_PIN_RESET);  
		delay_us(2);
	} 
}
void wakeUp()
{
	HAL_GPIO_WritePin(spi2_cs_GPIO_Port, spi2_cs_Pin, GPIO_PIN_SET); 
}

void powerDown()
{
	HAL_GPIO_WritePin(spi2_cs_GPIO_Port, spi2_cs_Pin, GPIO_PIN_RESET); 
}

/*****************************************************************************
�������ƣ�void InitADline(void)
��    �ܣ���ʼ��AD������
��ڲ�������
���ز�������
ʹ����Դ��port2
******************************************************************************/
void InitADline(void)
{
//	if (DWT_Delay_Init())
//	{
//		Error_Handler(); /* Call Error Handler */
//	}
	wakeUp();
	osDelay(100);
	OffsetAD();

    //����ת��
}