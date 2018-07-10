﻿/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     

/* ------------------------ LWIP includes --------------------------------- */
#include "lwip/api.h"
#include "lwip/tcpip.h"
#include "lwip/memp.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip.h"
#include "netif/ethernet.h"
/* ------------------------ FreeModbus includes --------------------------- */
#include "mb.h"
#include "iwdg.h"
/* ------------------------ Project includes ------------------------------ */
#include "stm32f4xx_nucleo_144.h"
#include "usart.h"
#include "gpio.h"
#include "iwdg.h"
#include "DAC.h"
#include "R8025t.h"
#include "Relay.h"
#include "ADS1230.h"
#include "recod.h"
#include <memory.h>
#include "crc.h"
#include "eeprom.h"
#include "cat1023.h"
#include "mbcrc.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId mainMB_TASKHandle;
osThreadId monitorTaskHandle;
osThreadId uMBpoll_taskHandle;
osThreadId periodTaskHandle;
osThreadId socket_clienTasHandle;
osThreadId socketTaskHandle;
osThreadId recvTaskHandle;
osMutexId MBholdingMutexHandle;

/* USER CODE BEGIN Variables */

//#define mainMB_TASK_PRIORITY    ( tskIDLE_PRIORITY + 3 )
#define PROG                    "FreeModbus"
#define REG_INPUT_START         1000
#define REG_INPUT_NREGS         4
#define REG_HOLDING_START       1
#define REG_HOLDING_NREGS       130

/* ----------------------- Static variables ---------------------------------*/

//struct netif gnetif; /* network interface structure */


static USHORT   usRegInputStart = REG_INPUT_START;
static USHORT   usRegInputBuf[REG_INPUT_NREGS];
static USHORT   usRegHoldingStart = REG_HOLDING_START;
static USHORT   usRegHoldingBuf[REG_HOLDING_NREGS];
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void vMBServerTask(void const * argument);
void monitor(void const * argument);
void uMBpoll(void const * argument);
void period(void const * argument);
void socket_clien(void const * argument);
void socketsever(void const * argument);
void socket1(void const * argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
extern STDATETIME stDateTime;
uint8_t User_notification(struct netif *netif);
int NetworkConnect(Network* n, char* addr, char* port);
int FreeRTOS_read(Network*, unsigned char*, int, int);
int FreeRTOS_write(Network*, unsigned char*, int, int);
void FreeRTOS_disconnect(Network*);

void 
rede_adc();

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Create the mutex(es) */
  /* definition and creation of MBholdingMutex */
  osMutexDef(MBholdingMutex);
  MBholdingMutexHandle = osMutexCreate(osMutex(MBholdingMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityHigh, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of mainMB_TASK */
//  osThreadDef(mainMB_TASK, vMBServerTask, osPriorityAboveNormal, 0, 512);
//  mainMB_TASKHandle = osThreadCreate(osThread(mainMB_TASK), NULL);

  /* definition and creation of monitorTask */
  osThreadDef(monitorTask, monitor, osPriorityNormal, 0, 128);
  monitorTaskHandle = osThreadCreate(osThread(monitorTask), NULL);

  /* definition and creation of uMBpoll_task */
//  osThreadDef(uMBpoll_task, uMBpoll, osPriorityNormal, 0, 512);
//  uMBpoll_taskHandle = osThreadCreate(osThread(uMBpoll_task), NULL);

  /* definition and creation of periodTask */
  osThreadDef(periodTask, period, osPriorityBelowNormal, 0, 256);
  periodTaskHandle = osThreadCreate(osThread(periodTask), NULL);

  /* definition and creation of socket_clienTas */
  osThreadDef(socket_clienTas, socket_clien, osPriorityAboveNormal, 0, 512);
  socket_clienTasHandle = osThreadCreate(osThread(socket_clienTas), NULL);

  /* definition and creation of socketTask */
  osThreadDef(socketTask, socketsever, osPriorityAboveNormal, 0, 512);
  socketTaskHandle = osThreadCreate(osThread(socketTask), NULL);

  /* definition and creation of recvTask */
  osThreadDef(recvTask, socket1, osPriorityNormal, 0, 128);
  recvTaskHandle = osThreadCreate(osThread(recvTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */


		 /* USER CODE BEGIN StartDefaultTask */
	
	printf("systeminit...");
	BSP_LED_On(LED_RED);
	EEinit();
	tcpip_init(NULL, NULL);
	MX_LWIP_Init();
	
	uint16_t Timeout = 5000;
	uint32_t tickstart = 0U;
	tickstart = HAL_GetTick();
	uint32_t  phyreg = 0U;

	
	if (User_notification(&gnetif))
	{
		do
		{
			HAL_ETH_ReadPHYRegister(&heth, PHY_BSR, &phyreg);
			HAL_Delay(50);
			BSP_LED_Toggle(LED_RED);
		//	HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
			/* Check for the Timeout */
			if ((HAL_GetTick() - tickstart) > Timeout)
			{
				/* In case of write timeout */	
			}
		} while (((phyreg & PHY_LINKED_STATUS) != PHY_LINKED_STATUS)) ;
		MX_LWIP_Init();
	}
//	taskDISABLE_INTERRUPTS(); 	
//	
//	I2C_EEPROM_WriteBuffer(EE_timeaddr, a, 5);
//	
//	I2C_EEPROM_ReadBuffer(EE_timeaddr, eeprombuff, 48);
//	taskENABLE_INTERRUPTS();
	BSP_LED_Off(LED_RED);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	bsp_init();
	/* definition and creation of socketTask */
//	  osThreadDef(socketTask, socketsever, osPriorityAboveNormal, 0, 512);
//	  socketTaskHandle = osThreadCreate(osThread(socketTask), NULL);
  /* Infinite loop */
  for(;;)
  {
	  
	  vTaskDelete(NULL);
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* vMBServerTask function */
void vMBServerTask(void const * argument)
{
  /* USER CODE BEGIN vMBServerTask */

	eMBErrorCode    xStatus;
	
	/* Infinite loop */
	for (;;)
	{
		if (eMBTCPInit(MB_TCP_PORT_USE_DEFAULT) != MB_ENOERR)
		{
			//  fprintf(stderr, "%s: can't initialize modbus stack!\r\n", PROG);
		}
		else if (eMBEnable() != MB_ENOERR)
		{
			//  fprintf(stderr, "%s: can't enable modbus stack!\r\n", PROG);
		}
		else
		{
			do
			{
				osDelay(10);
				xSemaphoreTake(MBholdingMutexHandle, (TickType_t)10);
				xStatus = eMBPoll();
			} while (xStatus == MB_ENOERR);
		}
		/* An error occured. Maybe we can restart. */
		(void)eMBDisable();
		(void)eMBClose();
		osDelay(10);
	}
  /* USER CODE END vMBServerTask */
}

/* monitor function */
void monitor(void const * argument)
{
  /* USER CODE BEGIN monitor */
	MX_IWDG_Init();
//	
	static portTickType xLastWakeTime;  
	const portTickType xFrequency = pdMS_TO_TICKS(1000);  
   
	
	xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(WDI_GPIO_Port, WDI_Pin);
//	  HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
//	  rede_adc();
	  
	  HAL_IWDG_Refresh(&hiwdg);
	  vTaskDelayUntil(&xLastWakeTime, xFrequency); 
  }
  /* USER CODE END monitor */
}

/* uMBpoll function */
void uMBpoll(void const * argument)
{
  /* USER CODE BEGIN uMBpoll */
	uint8_t Relay;
	uint8_t DI;
	uint16_t dac;
	/* Infinite loop */
	for (;;)
	{
	
		if (xSemaphoreTake(MBholdingMutexHandle, (TickType_t)portMAX_DELAY) == pdTRUE)
		{
			/********************************************************************************/		
					/*set relay*/
			for (Relay = 0; Relay < 8; Relay++)
			{
				DI = usRegHoldingBuf[Relay];
				relaycontrol(Relay, DI);	 
			}
		
			/*set 0-10v */
			
			dac = usRegHoldingBuf[12];
			dac = dac * 3.055;
			if (dac > 3055) dac = 3055;
			spi1_dac_write_chb(dac);
			spi1_dac_write_cha(dac);
			
							/************************************************/
									xSemaphoreGive(MBholdingMutexHandle);
			osDelay(10);
		}
		osDelay(10);
	}
  /* USER CODE END uMBpoll */
}

/* period function */
void period(void const * argument)
{
  /* USER CODE BEGIN period */
  /* Infinite loop */
	STDATETIME time;
	uint8_t  time_buf[50];
	uint8_t 	flage[26];
	uint16_t dac;
	uint8_t modeflage;
	
	
	I2C_EEPROM_ReadBuffer(EE_timeaddr, time_buf, 48);
//	
	I2C_EEPROM_ReadBuffer(EE_timeflageaddr, flage, 24);
//	for (u_int8_t i = 0; i < 24; i++)
//	{
//		printf("dac%2d:%d  ", i, flage[i]);
//	}
	I2C_EEPROM_ReadBuffer(EE_modeflageaddr, &modeflage, 1);
//	printf("mode:%d\n", modeflage);
	
  for(;;)
  {
	  UpdateDateTime(&time);
	  if (modeflage == 0)
	  {
		  if (time.minute == 0)
		  {
			  if (flage[time.hour] == 1)
			  {
				  dac = (uint16_t)((time_buf[time.hour * 2] << 8) | time_buf[(time.hour * 2) + 1]); 
				  
				  printf("time:%d dac:%d  \n", time.hour, dac);
				  if (dac > 1000)
				  {
					  dac = 1000;
				  }
				  dac = dac * 3.055;
				  if (dac > 3055) dac = 3055;
				  spi1_dac_write_chb(dac);
				  spi1_dac_write_cha(dac);
			  }
		  }
	  }
//	  sprintf((char *)time_buf,
//		  "%04d-%02d-%02d %02d:%02d:%02d",
//		  time.year + 2000, 
//		  time.month,
//		  time.day,
//		  time.hour,
//		  time.minute,
//		  time.second);
//	  
//	  printf("%s\n", time_buf);
//	  rede_adc();
	  
    osDelay(1000);
  }
  /* USER CODE END period */
}

/* socket_clien function */
void socket_clien(void const * argument)
{
  /* USER CODE BEGIN socket_clien */
	u_int8_t buflen = 32;
	u_int8_t ret, error;
	u_int16_t crc;
	unsigned char recv_buffer[buflen];
	
	uint8_t rc = 0;
	Network network;
		
	char* address = "192.168.1.92";
	char* port = "1992";
	if ((rc = NetworkConnect(&network, address, port)) != 0)
		printf("Return code from network connect is %d\n", rc);
  /* Infinite loop */
  for(;;)
  {
	  if ((ret = FreeRTOS_read(&network, recv_buffer, buflen, 100)) > 0)
	  {
		  crc = (recv_buffer[(ret - 1)] << 8) | (recv_buffer[(ret - 2)]);

		  if (crc != usMBCRC16(recv_buffer, (ret - 2)))
		  {
			  FreeRTOS_disconnect(&network);
			  break;
		  }
		  decoding(recv_buffer, &error);
		  if (error == 1)
		  {
			  FreeRTOS_disconnect(&network);
			  break;
		  }
		  crc = usMBCRC16(recv_buffer, (ret - 2));
		  recv_buffer[(ret - 1)] = crc >> 8;
		  recv_buffer[(ret - 2)] = crc & 0xff;
	  FreeRTOS_write(&network,recv_buffer, ret,100);
		  memset(recv_buffer, 0, sizeof(recv_buffer));
	  }
	  BSP_LED_Toggle(LED_RED);
    osDelay(100);
  }
  /* USER CODE END socket_clien */
}

/* socketsever function */
void socketsever(void const * argument)
{
  /* USER CODE BEGIN socketsever */
	__IO uint32_t uwCRCValue = 0;
	u_int8_t buflen = 32;
	u_int8_t ret, error;
	u_int16_t crc;
	unsigned char recv_buffer[buflen];
	int sock, newconn, size;
	struct sockaddr_in address, remotehost;

	/* create a TCP socket */
	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) 
	{
		printf("can not create socket");
		return;
	}
  
	/* bind to port 80 at any interface */
	address.sin_family = AF_INET;
	address.sin_port = htons(PORT);
	address.sin_addr.s_addr = INADDR_ANY;
	if (bind(sock, (struct sockaddr *)&address, sizeof(address)) < 0)
	{
		printf("can not bind socket");
		close(sock);
		return;
	}

	/* listen for connections (TCP listen backlog = 3) */
	listen(sock, 1); 
	size = sizeof(remotehost);

	/* Infinite loop */
	for (;;)
	{
		newconn = accept(sock, (struct sockaddr *)&remotehost, (socklen_t *)&size);
	//	printf("mewcon:%d", newconn);

		while ((ret =  read(newconn, recv_buffer, buflen)) > 0)
		{
			crc = (recv_buffer[(ret-1)] << 8 )|( recv_buffer[(ret-2)]);

			if (crc != usMBCRC16(recv_buffer, (ret - 2)))
			{
				close(newconn);
				break;
			}
			decoding(recv_buffer, &error);
			if (error==1)
			{
				close(newconn);
				break;
			}
			crc = usMBCRC16(recv_buffer, (ret - 2));
			recv_buffer[(ret - 1)] = crc >> 8;
			recv_buffer[(ret - 2)] = crc & 0xff;
			write(newconn, recv_buffer, ret);
			memset(recv_buffer, 0, sizeof(recv_buffer));
		}
		close(newconn);
	    osDelay(1);
	}
  /* USER CODE END socketsever */
}

/* socket1 function */
void socket1(void const * argument)
{
  /* USER CODE BEGIN socket1 */
	//char RECEVbuff[8] =  { 0};
	uint8_t zero = 0;
	 
////		I2C_EEPROM_WriteBuffer(EE_ipaddr, &zero, 20);
////	I2C_EEPROM_WriteBuffer(EE_timeaddr, &zero, 24);
////	I2C_EEPROM_WriteBuffer(EE_timeaddr + 24, &zero, 24);
////	I2C_EEPROM_WriteBuffer(EE_timeflageaddr, &zero, 24);
////	I2C_EEPROM_WriteBuffer(EE_modeflageaddr, &zero, 10);
////	I2C_EEPROM_WriteBuffer(EE_setipflageaddr, &zero, 10);
//	printf("restore");
  /* Infinite loop */
  for(;;)
  {
	
	  if (huart1.RxXferCount < 32)
	  {
		  printf("rxlen:%d\n", huart1.RxXferCount);
		   //RxXferCount 告诉我们剩余空间大小，如果剩余空间和总空间不一样，则说明中断收到数据了。	
		  HAL_GPIO_WritePin(_485DIR_GPIO_Port, _485DIR_Pin, GPIO_PIN_SET);
	  HAL_UART_Transmit(&huart1, (uint8_t *)aRxBuffer, 8, 100);    //发送接收到得数据
		  HAL_GPIO_WritePin(_485DIR_GPIO_Port, _485DIR_Pin, GPIO_PIN_RESET);
		
		  huart1.pRxBuffPtr -= (32 - huart1.RxXferCount);   //将接收指针放回到开始，否则会一直增加哦
      huart1.RxXferCount = 32;   //修改剩余空间，否则会一直进入这里


	  		  if(strncmp((char const *)aRxBuffer, "$restore", 8) == 0)
	  		  {
		  		  printf("\nstart restore\n");
		  		  HAL_IWDG_Refresh(&hiwdg);
		  		  taskDISABLE_INTERRUPTS(); 
		  		
		  		  			
	  			  I2C_EEPROM_WriteBuffer(EE_ipaddr, &zero, 12);
	  	         osDelay(10);
		  			  I2C_EEPROM_WriteBuffer(EE_timeaddr, &zero, 24);
		  		  osDelay(10);
		  			  I2C_EEPROM_WriteBuffer(EE_timeaddr+24, &zero, 24);
		  		  osDelay(10);
		  		  HAL_IWDG_Refresh(&hiwdg);
		  			  I2C_EEPROM_WriteBuffer(EE_timeflageaddr, &zero, 24);
		  		  osDelay(10);
		  			  I2C_EEPROM_WriteBuffer(EE_modeflageaddr, &zero, 10);
		  		  osDelay(10);
		  			  I2C_EEPROM_WriteBuffer(EE_setipflageaddr, &zero, 10);
		  		  taskENABLE_INTERRUPTS(); 
		  		  osDelay(10);
		  		  printf("restoreok\n");
		  		  printf("reboot\n");
		  		  __set_FAULTMASK(1);
		  		  HAL_NVIC_SystemReset();
		  
		  			
		      }
	  

		  	  osDelay(100);
		  //	  vTaskDelete(NULL);
	  }
	  osDelay(10);
  }
	 
  /* USER CODE END socket1 */
}

/* USER CODE BEGIN Application */
eMBErrorCode
eMBRegInputCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs)
{
	eMBErrorCode    eStatus = MB_ENOERR;
	int             iRegIndex;

	if ((usAddress >= REG_INPUT_START)
	    && (usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS))
	{
		iRegIndex = (int)(usAddress - usRegInputStart);
		while (usNRegs > 0)
		{
			*pucRegBuffer++ = (unsigned char)(usRegInputBuf[iRegIndex] >> 8);
			*pucRegBuffer++ = (unsigned char)(usRegInputBuf[iRegIndex] & 0xFF);
			iRegIndex++;
			usNRegs--;
		}
	}
	else
	{
		eStatus = MB_ENOREG;
	}
	return eStatus;
}

eMBErrorCode
eMBRegHoldingCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode)
{
	eMBErrorCode    eStatus = MB_ENOERR;
	int             iRegIndex;

	if ((usAddress >= REG_HOLDING_START) &&
	    (usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS))
	{
		iRegIndex = (int)(usAddress - usRegHoldingStart);
		switch (eMode)
		{
			/* Pass current register values to the protocol stack. */
		case MB_REG_READ:
			while (usNRegs > 0)
			{
				*pucRegBuffer++ = (UCHAR)(usRegHoldingBuf[iRegIndex] >> 8);
				*pucRegBuffer++ = (UCHAR)(usRegHoldingBuf[iRegIndex] & 0xFF);
				iRegIndex++;
				usNRegs--;
			}
			break;

			/* Update current register values with new values from the
			 * protocol stack. */
		case MB_REG_WRITE:
			if (usNRegs != 0)
			{
				
				while (usNRegs > 0)
				{
					usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
					usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
					iRegIndex++;
					usNRegs--;
				}
				xSemaphoreGive(MBholdingMutexHandle);
				
			}
		}
	}
	else
	{
		eStatus = MB_ENOREG;
	}
	return eStatus;
}

eMBErrorCode
eMBRegCoilsCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode)
{
	return MB_ENOREG;
}

eMBErrorCode
eMBRegDiscreteCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete)
{
	return MB_ENOREG;
}

uint8_t User_notification(struct netif *netif) 
{
	if (netif_is_up(netif))
	{
#ifdef USE_DHCP
		/* Update DHCP state machine */
		DHCP_state = DHCP_START;
#else
//		uint8_t iptxt[20];
//		sprintf((char *)iptxt, "%s", ip4addr_ntoa((const ip4_addr_t *)&netif->ip_addr));
//		printf("Static IP address: %s\n", iptxt);
		return 0;
#endif /* USE_DHCP */
	}
	else
	{  
#ifdef USE_DHCP
		/* Update DHCP state machine */
		DHCP_state = DHCP_LINK_DOWN;
#endif  /* USE_DHCP */
		printf("The network cable is not connected \n");
		return 1;
	} 
}

void 
rede_adc()
{
	int32_t adctemp;
//	int32_t adctemp;
//	InitADline();
	
	 filter(&adctemp);
		  usRegHoldingBuf[11] = adctemp <<16;
		  usRegHoldingBuf[11] |= adctemp;
		  printf("adcvule:%d\n",adctemp);
	 
}

int NetworkConnect(Network* n, char* addr, char* port)
{	
	int type = SOCK_STREAM;
	struct sockaddr_in address;
	int rc = -1;
	sa_family_t family = AF_INET;
	struct addrinfo *result = NULL;
	struct addrinfo hints = { 0, AF_UNSPEC, SOCK_STREAM, IPPROTO_TCP, 0, NULL, NULL, NULL };
	static struct timeval tv;
	int eer;
	int nNetTimeout = 1000; //1秒
	TickType_t xTicksToWait = 100 / portTICK_PERIOD_MS;

	n->my_socket = -1;
	if (addr[0] == '[')
		++addr;

	if ((rc = getaddrinfo(addr, port, &hints, &result)) == 0)
	{
		struct addrinfo* res = result;
		/* prefer ip4 addresses */
		while (res)
		{
			if (res->ai_family == AF_INET)
			{
				result = res;
				break;
			}
			res = res->ai_next;
		}
		if (result->ai_family == AF_INET)
		{
			address.sin_port = ((struct sockaddr_in*)(result->ai_addr))->sin_port;     // htons(port);
			address.sin_family = family = AF_INET;
			address.sin_addr = ((struct sockaddr_in*)(result->ai_addr))->sin_addr;
		}
		else
			rc = -1;
		freeaddrinfo(result);
	}
	if (rc == 0)
	{
		n->my_socket =	socket(family, type, 0);
		if (n->my_socket != -1)
		{	
//			eer = setsockopt(n->my_socket，SOL_S0CKET, SO_RCVTIMEO，(char *)&nNetTimeout, sizeof(nNetTimeout));
//			printf("ferr:%d", eer);
			if (family == AF_INET)
				rc = connect(n->my_socket, (struct sockaddr*)&address, sizeof(address));
		}
	}

	return rc;
}

int FreeRTOS_read(Network* n, unsigned char* buffer, int len, int timeout_ms)
{
	uint32_t tickstart = 0U;
	tickstart = HAL_GetTick();
	uint32_t  phyreg = 0U;
	int eer;
/* Check for the Timeout */
		
	struct timeval xTicksToWait;
	xTicksToWait.tv_sec = timeout_ms / portTICK_PERIOD_MS; /* convert milliseconds to ticks */

	TimeOut_t xTimeOut;
	int recvLen = 0;
	do
	{
		int rc = 0;
		eer = setsockopt(n->my_socket, 0, SO_RCVTIMEO, &xTicksToWait, sizeof(xTicksToWait));
		printf("err:%d", eer);
		rc = recv(n->my_socket, buffer + recvLen, len - recvLen, 0);
		if (rc > 0)
			recvLen += rc;
		else if (rc < 0)
		{
			recvLen = rc;
			break;
		}
		if ((HAL_GetTick() - tickstart) > timeout_ms)
		{
			/* In case of write timeout */
			break;
		}
		osDelay(1);
	} while (recvLen < len  == pdFALSE);

	return recvLen;
}


int FreeRTOS_write(Network* n, unsigned char* buffer, int len, int timeout_ms)
{
	TickType_t xTicksToWait = timeout_ms / portTICK_PERIOD_MS; /* convert milliseconds to ticks */
	TimeOut_t xTimeOut;
	int sentLen = 0;

	vTaskSetTimeOutState(&xTimeOut); /* Record the time at which this function was entered. */
	do
	{
		int rc = 0;

		setsockopt(n->my_socket, 0, SO_RCVTIMEO, &xTicksToWait, sizeof(xTicksToWait));
		rc = send(n->my_socket, buffer + sentLen, len - sentLen, 0);
		if (rc > 0)
			sentLen += rc;
		else if (rc < 0)
		{
			sentLen = rc;
			break;
		}
	} while (sentLen < len && xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) == pdFALSE);

	return sentLen;
}


void FreeRTOS_disconnect(Network* n)
{
	closesocket(n->my_socket);
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
