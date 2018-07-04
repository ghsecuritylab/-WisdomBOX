
#include "recod.h"
#include "DAC.h"
#include "R8025t.h"
#include "cat1023.h"
#include <memory.h>


///////////////////////////////////////////////////
uint8_t	 mode_flage=0;
STDATETIME set_time; 
uint8_t timeflage[7];
void decoding (uint8_t * data)
{
	
	struct    tm *now_ptm;
	time_t     timep;

	
	uint16_t dac;
	
	if (data[0] == 0 && data[1] == 0x06 && data[2] == 0) //写入
	{
		
	
		switch (data[3])
		{
			
		case setrelay1:
			if (data[4] == 0&&data[5] == 0) 	HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_RESET);
			else	HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_SET);
			break;
		case setrelay2:
			if (data [4]==0&&data[5] == 0) 	HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, GPIO_PIN_RESET);
			else	HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, GPIO_PIN_SET);
			break;
			
		case setdac1:
			dac = ((uint16_t)data[4] << 8) | ((uint16_t)(data[5])); 
			dac = dac * 3.055;
			if (dac > 3055) dac = 3055;
			spi1_dac_write_chb(dac);
			//spi1_dac_write_cha(dac);	
			
			break;
		case setdac2:
			dac = ((uint16_t)data[4] << 8) | ((uint16_t)(data[5])); 
			dac = dac * 3.055;
			if (dac > 3055) dac = 3055;
//			spi1_dac_write_chb(dac);
			spi1_dac_write_cha(dac);	
			
			break;
		case set_mode:
			if (data[5] == 0)
			{
				mode_flage = 0;
			}
			else if (data[5] ==0x01)
			{
				mode_flage = 1;
			}
			
			break;
		case set:
			if (data[5]==0x01)
			{
				RtcSetoneTime(&set_time,timeflage);
				memset(timeflage, 0, 1);
			}
			
			break;
		case set_Year:
			set_time.year    = ( uint8_t ) data[5]; 
			timeflage[6] = 1;
			break;
		case set_Month:
			set_time.month    = (uint8_t) data[5]; 
			timeflage[5] = 1;
			break;
		case set_Day:
			printf("%d\n", data[5]);
			set_time.day    =    (uint8_t)  data[5];  
			timeflage[4] = 1;
			break;
		case set_Week:
			
			set_time.week    =  (uint8_t)  data[5]; 
			timeflage[3] = 1;

			break;
		case set_Hour:
			
			set_time.hour    =   (uint8_t)data[5];  
			timeflage[2] = 1;

			break;
		case set_Minute :
			
			set_time.minute    =   (uint8_t) data[5];  
			timeflage[1] = 1;

			break;
		case set_Second:
			set_time.second    =  (uint8_t) data[5]; 
			timeflage[0] = 1;
			

			break;
		case setip1:
			IP_ADDRESS[0] = data[5];
			IP_ADDRESS[1] = data[4];
			
			
			break;
		case setip2:
			IP_ADDRESS[2] = data[5];
			IP_ADDRESS[3] = data[4];
			
			

			break;
		case setip3:
			GATEWAY_ADDRESS[0] = data[5];
			GATEWAY_ADDRESS[1] = data[4];
			
			

			break;
		case setip4:
			GATEWAY_ADDRESS[2] = data[5];
			GATEWAY_ADDRESS[3] = data[4];
			
			

			break;
		default:
			break;
		}	
	}
	if (data[0] == 0 && data[1] == 0x40)
	{
		if (data[3] == 0x02)
		{
			for (uint8_t i = 0; i < 4; i++)
			{
				printf("ip:%d\n", IP_ADDRESS[i]);
				
			}
			for (uint8_t i = 0; i < 4; i++)
			{
				printf("%d\n", GATEWAY_ADDRESS[i]);
				
			}
			
			I2C_EEPROM_WriteBuffer(0x00, IP_ADDRESS, 4);
			I2C_EEPROM_WriteBuffer(0x04, GATEWAY_ADDRESS, 4);
			
		}
		else if (data[3]==0x03)
		{
			__set_FAULTMASK(1);
			HAL_NVIC_SystemReset();
		}
		
	}
	
	if (data[0] == 0 && data[1] == 0x10)
	{
		if (data[3]==0x01)
		{
			if (data[7] == 0&& data[8] == 0)
			{
				HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_RESET);
			}
			else
			{
				HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_SET);
			}
			
			
		}
		if (data[5]==0x02)
		{
			if (data[9] == 0&& data[10] == 0)
			{
				HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, GPIO_PIN_RESET);
			}
			else
			{
				HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, GPIO_PIN_SET);
			}
			
		}
		
	}
	

}