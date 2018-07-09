
#include "recod.h"
#include "DAC.h"
#include "R8025t.h"
#include "cat1023.h"
#include <memory.h>
#include "spi.h"
#include "ADS1230.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
///////////////////////////////////////////////////

STDATETIME set_time; 
uint8_t timeflage[7];
uint8_t gpiostatus1= 0;
u_int16_t  dacstatusa= 0;
u_int16_t  dacstatusb= 0;
//uint8_t setip_flage = 0;
void decoding(uint8_t * data, uint8_t * error)
{
	STDATETIME time;
	struct    tm *now_ptm;
	time_t     timep;
	int32_t   adcvule;
	uint16_t   adc;
	uint8_t    timeingflage[24] =  { 0 };
	uint8_t  addr = 0,flageaddr=0 ;
	uint16_t dac;
	uint8_t	 mode_flage = 0;
	*error = 0;
	uint8_t setip_flage = 0;
	if (data[0] == 0 && data[1] == 0x06 && data[2] == 0) //写入
		{
			if (data[3] >= 0x47&&data[3] <= 0x5f)
			{
				addr = (data[3] - 0x47) * 2;
				flageaddr = (data[3] - 0x47);
				//			printf("data:%d addr:%d  ", (data[3]-0x47),addr);
				//			dacbuff[addr] = ((uint8_t)data[4] << 8) | ((uint8_t)(data[5])); 
				//			printf("dac:%d\n", dacbuff[addr]);
							timeingflage[flageaddr] = 1;
			
				I2C_EEPROM_WriteBuffer((EE_timeaddr + addr), (uint8_t*)& data[4], 1);
				I2C_EEPROM_WriteBuffer((EE_timeaddr + (addr + 1)), (uint8_t*)& data[5], 1);
				I2C_EEPROM_WriteBuffer((EE_timeflageaddr + flageaddr), (uint8_t*) &timeingflage[flageaddr], 1);
			}
	
			switch (data[3])
			{
			
			case setrelay1:
				if (data[4] == 0&&data[5] == 0) 
				{
					HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_RESET);
					gpiostatus1 = 0;
				}
				else	
				{
					HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_SET);
					gpiostatus1 = 1;
				}
				break;
			case setrelay2:
				if (data[4] == 0&&data[5] == 0) 	HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, GPIO_PIN_RESET);
				else	HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, GPIO_PIN_SET);
				break;
			
			case setdac1:
				dac = ((uint16_t)data[4] << 8) | ((uint16_t)(data[5])); 
				if (dac > 1000)
				{
					dac = 1000;
				}
				dacstatusa = dac;
				dac = dac * 3.055;
				if (dac > 3055) dac = 3055;
				spi1_dac_write_chb(dac);
				//spi1_dac_write_cha(dac);	
			
				break;
			case setdac2:
				dac = ((uint16_t)data[4] << 8) | ((uint16_t)(data[5])); 
				if (dac > 1000)
				{
					dac = 1000;
				}
				printf("adc:%d", dac);
				dacstatusb = dac;
				printf("adcs:%d", dacstatusb);
				dac = dac * 3.055;
				if (dac > 3055) dac = 3055;
				spi1_dac_write_chb(dac);
				//			spi1_dac_write_cha(dac);	
			
							break;
			case set_mode:
				if (data[5] == 0)
				{
					mode_flage = 0;
					I2C_EEPROM_WriteBuffer(EE_modeflageaddr, &mode_flage, 1);
				}
				else if (data[5] == 0x01)
				{
					mode_flage = 1;
					I2C_EEPROM_WriteBuffer(EE_modeflageaddr, &mode_flage, 1);
				}
			
				break;
			case set:
				if (data[5] == 0x01)
				{
					RtcSetoneTime(&set_time, timeflage);
					memset(timeflage, 0, 7);
				}
			
				break;
			case set_Year:
			
				set_time.year    = (uint8_t) data[5]; 
			
				timeflage[6] = 1;
				break;
			case set_Month:
				set_time.month    = (uint8_t) data[5]; 
				timeflage[5] = 1;
				break;
			case set_Day:
				set_time.day    =    (uint8_t)  data[5];  
				printf("data5:%d \n", set_time.day);
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
			
		
			
			default:*error = 1;
				break;
			}	
		}
	else if (data[0] == 0 && data[1] == 0x40)
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
			setip_flage = 1;
			I2C_EEPROM_WriteBuffer(EE_ipaddr, IP_ADDRESS, 4);
			I2C_EEPROM_WriteBuffer(EE_ipaddr + 4, GATEWAY_ADDRESS, 4);
			I2C_EEPROM_WriteBuffer(EE_setipflageaddr, &setip_flage, 1);
			
		}
		else if (data[3] == 0x03)
		{
			__set_FAULTMASK(1);
			HAL_NVIC_SystemReset();
		}
		else
		{
			*error = 1;
		}
		
	}
	
	else if (data[0] == 0 && data[1] == 0x10)
	{
		if (data[3] == 0x01)
		{
			if (data[7] == 0 && data[8] == 0)
			{
				HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_RESET);
				gpiostatus1 = 0;
			}
			else
			{
				HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_SET);
				gpiostatus1 = 1;
			}
			
			
		}
		if (data[5] == 0x02)
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
		
		else
		{
			*error = 1;
		}
		
	}
	else if (data[0] == 0 && data[1] == 0x03 &&data[2] == 0)
	{
		switch (data[3])
		{
			
		case 0x01:
			if (data[5] == 1) data[5] = gpiostatus1;
			
			break;
		case 0x0a:
			if (data[5] == 1)//读条光
			{
				data[4] = dacstatusa >> 8;
				data[5] = dacstatusa & 0xff;	
			}

			
			break;
		case 0x0b:
			if (data[5] == 1)
			{
//				printf("adcb:%d", dacstatusb);
				data[4] = dacstatusb >> 8;
				data[5] = dacstatusb & 0xff;
			}
			
			break;
		case 0x26:
			if (data[5] == 1)//读取电流
			{

				//				taskDISABLE_INTERRUPTS(); 
								filter(&adcvule);
				//				taskENABLE_INTERRUPTS(); 
								adc = (uint16_t)adcvule;

				data[4] = adc >> 8;
				data[5] = adc & 0xff;
			}
			
			break;
		case 0x28:
			UpdateDateTime(&time);
			data[5] = time.hour;
			break;
		case 0x29:
			UpdateDateTime(&time);
			data[5] = time.minute;
			
			break;
		case 0x2a:
			UpdateDateTime(&time);
			data[5] = time.second;
			
			break;
		case 0x2c:
			UpdateDateTime(&time);
			data[5] = time.year;
			
			break;
		case 0x2d:
			UpdateDateTime(&time);
			data[5] = time.month;
			
			break;
		case 0x2e:
			UpdateDateTime(&time);
			data[5] = time.day;
			
			break;
		case 0x2F:
			UpdateDateTime(&time);
			data[5] = time.week;
			
			break;
		default:*error = 1;
			break;
		}
	
	}
	else
	{
		*error = 1;
	}
	
}