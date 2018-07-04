
#include "cat1023.h"
#include "i2c.h"
#include "cmsis_os.h"


uint8_t I2C_EEPROM_WriteBuffer(uint16_t Reg, uint8_t *pBuffer, uint16_t Length)
{
	uint8_t time = 10;
	/* Check if the EEPROM is ready for a new operation */
//	HAL_I2C_IsDeviceReady(&hi2c3, CAT1023_ADDR_WRITE, 100, 300);

	

	/* Write EEPROM_PAGESIZE */
	if(HAL_I2C_Mem_Write(&hi2c3, (uint16_t)CAT1023_ADDR_WRITE, Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)pBuffer, Length, 100) != HAL_OK)
	{
		/* Writing process Error */
		Error_Handler();
	}

	while ((HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY) || time)
	{
		time--;
		osDelay(10);
	}
	return 0;
}

uint8_t I2C_EEPROM_ReadBuffer(uint16_t Reg, uint8_t *pBuffer, uint16_t Length)
{
	uint8_t time = 10;
	/* Check if the EEPROM is ready for a new operation */
//	HAL_I2C_IsDeviceReady(&hi2c3, CAT1023_ADDR_READ, 100, 300);
//	while ((HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY) && time--)
//	{
//		osDelay(10);
//	}
	
	
	/* Write EEPROM_PAGESIZE */
	if(HAL_I2C_Mem_Read(&hi2c3, (uint16_t)CAT1023_ADDR_READ, Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)pBuffer, Length, 100) != HAL_OK)
	{
		/* Writing process Error */
		Error_Handler();
	}

	while ((HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY) && time--)
	{
		osDelay(10);
	}
	return 0;
}