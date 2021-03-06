/**
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
#include "lwip/apps/mqtt.h"
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
#include "mymqtt.h"

#include "MQTTClient.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId mainMB_TASKHandle;
osThreadId monitorTaskHandle;
osThreadId uMBpoll_taskHandle;
osThreadId periodTaskHandle;
osThreadId mqtt_TaskHandle;
osThreadId socketTaskHandle;
osMessageQId recvQueueHandle;
osMessageQId MBQueueHandle;
osMessageQId timeQueueHandle;
osMutexId MBholdingMutexHandle;
osSemaphoreId TIMEBinarySemHandle;

/* USER CODE BEGIN Variables */

//#define mainMB_TASK_PRIORITY    ( tskIDLE_PRIORITY + 3 )
#define PROG "FreeModbus"
#define REG_INPUT_START 1000
#define REG_INPUT_NREGS 4
#define REG_HOLDING_START 1
#define REG_HOLDING_NREGS 130

//#define MQTT_TASK 1
/* ----------------------- Static variables ---------------------------------*/

//struct netif gnetif; /* network interface structure */

static USHORT usRegInputStart = REG_INPUT_START;
static USHORT usRegInputBuf[REG_INPUT_NREGS];
static USHORT usRegHoldingStart = REG_HOLDING_START;
static USHORT usRegHoldingBuf[REG_HOLDING_NREGS];
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void vMBServerTask(void const * argument);
void monitor(void const * argument);
void uMBpoll(void const * argument);
void period(void const * argument);
void mqtt_clien(void const * argument);
void socket_sever(void const * argument);
void restore(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
extern STDATETIME stDateTime;
uint8_t User_notification(struct netif *netif);
//int NetworkConnect(Network *n, char *addr, char *port);
//int FreeRTOS_read(Network *, unsigned char *, int, int);
//int FreeRTOS_write(Network *, unsigned char *, int, int);
//void FreeRTOS_disconnect(Network *);
int server_read(int newconn, unsigned char *buffer, int len, int timeout_ms);
void rede_adc();
void tcp_server(int conn);
void messageArrived(MessageData* data);
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

  /* Create the semaphores(s) */
  /* definition and creation of TIMEBinarySem */
  osSemaphoreDef(TIMEBinarySem);
  TIMEBinarySemHandle = osSemaphoreCreate(osSemaphore(TIMEBinarySem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityHigh, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of mainMB_TASK */
//  osThreadDef(mainMB_TASK, vMBServerTask, osPriorityAboveNormal, 0, 512);
//  mainMB_TASKHandle = osThreadCreate(osThread(mainMB_TASK), NULL);

  /* definition and creation of monitorTask */
  osThreadDef(monitorTask, monitor, osPriorityBelowNormal, 0, 128);
  monitorTaskHandle = osThreadCreate(osThread(monitorTask), NULL);

  /* definition and creation of uMBpoll_task */
//  osThreadDef(uMBpoll_task, uMBpoll, osPriorityNormal, 0, 512);
//  uMBpoll_taskHandle = osThreadCreate(osThread(uMBpoll_task), NULL);

  /* definition and creation of periodTask */
  osThreadDef(periodTask, period, osPriorityNormal, 0, 256);
  periodTaskHandle = osThreadCreate(osThread(periodTask), NULL);

  /* definition and creation of mqtt_Task */
  osThreadDef(mqtt_Task, mqtt_clien, osPriorityAboveNormal, 0, 640);
  mqtt_TaskHandle = osThreadCreate(osThread(mqtt_Task), NULL);

  /* definition and creation of socketTask */
  osThreadDef(socketTask, socket_sever, osPriorityAboveNormal, 0, 512);
  socketTaskHandle = osThreadCreate(osThread(socketTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of recvQueue */
/* what about the sizeof here??? cd native code */
//  osMessageQDef(recvQueue, 5, uint16_t);
//  recvQueueHandle = osMessageCreate(osMessageQ(recvQueue), NULL);

  /* definition and creation of MBQueue */
/* what about the sizeof here??? cd native code */
  osMessageQDef(MBQueue, 16, uint16_t);
  MBQueueHandle = osMessageCreate(osMessageQ(MBQueue), NULL);

  /* definition and creation of timeQueue */
/* what about the sizeof here??? cd native code */
  osMessageQDef(timeQueue, 1, uint16_t);
  timeQueueHandle = osMessageCreate(osMessageQ(timeQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	osMessageQDef(recvQueue, 5, sizeof(struct Msg));
	recvQueueHandle = osMessageCreate(osMessageQ(recvQueue), NULL);
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */

//	printf("systeminit...");
	BSP_LED_On(LED_RED);
	EEinit();
	tcpip_init(NULL, NULL);
	HAL_IWDG_Refresh(&hiwdg);
	MX_LWIP_Init();
	HAL_IWDG_Refresh(&hiwdg);
	uint16_t Timeout = 4000;
	uint32_t tickstart = 0U;
	tickstart = HAL_GetTick();
	uint32_t phyreg = 0U;
	

	if (User_notification(&gnetif))
	{
		do
		{
			HAL_ETH_ReadPHYRegister(&heth, PHY_BSR, &phyreg);
			HAL_Delay(100);
			BSP_LED_Toggle(LED_RED);
			//	HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
			/* Check for the Timeout */
			if ((HAL_GetTick() - tickstart) > Timeout)
			{
				HAL_IWDG_Refresh(&hiwdg);
				/* In case of write timeout */
			}
		} while (((phyreg & PHY_LINKED_STATUS) != PHY_LINKED_STATUS));
		HAL_IWDG_Refresh(&hiwdg);
		MX_LWIP_Init();
	}
	//	taskDISABLE_INTERRUPTS();
	//
	//	I2C_EEPROM_WriteBuffer(EE_timeaddr, a, 5);
	//
	//	I2C_EEPROM_ReadBuffer(EE_timeaddr, eeprombuff, 48);
	//	taskENABLE_INTERRUPTS();
	BSP_LED_Off(LED_RED);
	bsp_init();

	/* definition and creation of socketTask */
	//	  osThreadDef(socketTask, socketsever, osPriorityAboveNormal, 0, 512);
	//	  socketTaskHandle = osThreadCreate(osThread(socketTask), NULL);
	/* Infinite loop */
	for (;;)
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

	eMBErrorCode xStatus;

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
	//	MX_IWDG_Init();
	//
	static portTickType xLastWakeTime;
	const portTickType xFrequency = pdMS_TO_TICKS(4000);

	xLastWakeTime = xTaskGetTickCount();
	/* Infinite loop */
	for (;;)
	{
		//	  HAL_GPIO_TogglePin(WDI_GPIO_Port, WDI_Pin);
		//  HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
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
			if (dac > 3055)
				dac = 3055;
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
	__IO uint32_t usTick;
	STDATETIME time;
	uint8_t time_buf[50];
	uint8_t flage[26];
	uint16_t dac;
	uint8_t modeflage;
	BaseType_t xResult;
	uint8_t restoretime;
	//	const TickType_t xMaxBlockTime = pdMS_TO_TICKS(200); /* 鐠佸墽鐤嗙粵澶婄窡閺冨爼�?? wait_time */

	I2C_EEPROM_ReadBuffer(EE_timeaddr, time_buf, 48);
	//
	I2C_EEPROM_ReadBuffer(EE_timeflageaddr, flage, 24);
	//	for (u_int8_t i = 0; i < 24; i++)
	//	{
	//		printf("dac%2d:%d  ", i, flage[i]);
	//	}
	I2C_EEPROM_ReadBuffer(EE_modeflageaddr, &modeflage, 1);
	//	printf("mode:%d\n", modeflage);
	restoretime = 0;
	for (;;)
	{
		// 信号量获�? 设置阻塞时间 10 ticks
		xResult = xSemaphoreTake(TIMEBinarySemHandle, (TickType_t)portMAX_DELAY);
		if (xResult == pdTRUE)
		{
			
			//osDelay(10);
			if (HAL_GPIO_ReadPin(restore_GPIO_Port, restore_Pin)==0)
			{
				restoretime++;
				if (restoretime>3)
				{
					restoretime = 0;
					restore();
				}
			}
			else
			{
				restoretime = 0;
			}
			
			HAL_GPIO_TogglePin(WDI_GPIO_Port, WDI_Pin);
			BSP_LED_Toggle(LED_GREEN);
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
						dac = dac * 3.055f;
						if (dac > 3055)
							dac = 3055;
						spi1_dac_write_chb(dac);
						spi1_dac_write_cha(dac);
					}
				}
			}

			// DbgLog("ATgprs:%s", gprs_msg->usData);

			//  memset(&rs485_MSG, 0, sizeof(rs485_MSG));
		}
		else
		{
			/* 瓒呮�? */
			//  ErrLog("瓒呮�?");
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
			//  rede_adc();

		osDelay(1);
	}
  /* USER CODE END period */
}

/* mqtt_clien function */
void mqtt_clien(void const * argument)
{
	
  /* USER CODE BEGIN mqtt_clien */
/* connect to m2m.eclipse.org, subscribe to a topic, send and receive messages regularly every 1 sec */
	MQTTClient client;
	Network network;
	unsigned char sendbuf[80], readbuf[80];
	int rc = 0, 
		count = 0;
	MQTTPacket_connectData connectData = MQTTPacket_connectData_initializer;

	argument = 0;
	NetworkInit(&network);
	MQTTClientInit(&client, &network, 30000, sendbuf, sizeof(sendbuf), readbuf, sizeof(readbuf));

//	char* address = "45.40.243.84";
//	char* address = "176.122.166.83";
//	char* address = "66.154.108.162";
	char* address = "192.168.1.113";
//	char* port = "1883";
	if ((rc = NetworkConnect(&network, address, 1883)) != 0)
		printf("Return code from network connect is %d\n", rc);

#if defined(MQTT_TASK)
	if ((rc = MQTTStartTask(&client)) != pdPASS)
		printf("Return code from start tasks is %d\n", rc);
#endif

	connectData.MQTTVersion = 3;
	connectData.clientID.cstring = "mqtt1test";
	connectData.keepAliveInterval = 50;           //seconds
	connectData.cleansession = 1;
//	connectData.username.cstring = "gaohongwei/iot";
	//	connectData.password.cstring = "f+Q9Lp++T5nysNVcLVfOWpIIDVz8MaVm5dyJA8jEXdU=";

	if ((rc = MQTTConnect(&client, &connectData)) != 0)
		printf("Return code from MQTT connect is %d\n", rc);
	else
		printf("MQTT Connected\n");

	if ((rc = MQTTSubscribe(&client, "test", 2, messageArrived)) != 0)
		printf("Return code from MQTT subscribe is %d\n", rc);
  /* Infinite loop */
  for(;;)
  {
	  
	  MQTTMessage message;
	  char payload[30];

	  message.qos = 1;
	  message.retained = 0;
	  message.payload = payload;
	  sprintf(payload, "message number %d", count);
	  message.payloadlen = strlen(payload);

	  //		if ((rc = MQTTPublish(&client, "test1", &message)) != 0)
	  //			printf("Return code from MQTT publish is %d\n", rc);

	
	  		if((rc = MQTTYield(&client, 1000)) != 0)
	  			printf("Return code from yield is %d\n", rc);

	  osDelay(10);
    osDelay(1000);
  }
  /* USER CODE END mqtt_clien */
}

/* socket_sever function */
void socket_sever(void const * argument)
{
  /* USER CODE BEGIN socket_sever */
	uint32_t phyreg = 0U;
	__IO uint32_t uwCRCValue = 0;
	uint8_t buflen = 32;
	int8_t ret;
	uint8_t  error;
	uint16_t crc;
	unsigned char recv_buffer[buflen];
	int sock, newconn, size;
	struct sockaddr_in address, remotehost;
	int8_t Timeout = 10;
	int32_t tickstart = 0U;
	struct timeval interval;
	interval.tv_sec = 2;
	interval.tv_usec = 0;
	
	/* create a TCP socket */
	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) 
	{
		printf("can not create socket\r\n");
		osDelay(1);
		//	OSTimeDlyHMSM(0, 0, 1, 0);
//			continue;
	}

	address.sin_family = AF_INET;
	address.sin_port = htons(8088);      // mosbus tcp port
	address.sin_addr.s_addr = INADDR_ANY;

	if (bind(sock, (struct sockaddr *)&address, sizeof(address)) < 0)
	{
		printf("can not bind socket\r\n");
		close(sock);
		osDelay(1);
		//OSTimeDlyHMSM(0, 0, 2, 0);
//		continue;
	}

	/* listen for incoming connections (TCP listen backlog = 1) */
	listen(sock, 1);

	size = sizeof(remotehost);
	/* Infinite loop */
	for (;;)
	{
		newconn = accept(sock, (struct sockaddr *)&remotehost, (socklen_t *)&size);
		if (newconn >= 0)
		{
			printf("connect socket\r\n");
			tcp_server(newconn);
		}
		else
		{
			//close(sock);
			close(newconn);
		}
	}
  /* USER CODE END socket_sever */
}

/* USER CODE BEGIN Application */
void messageArrived(MessageData* data)
{
	printf("Message arrived on topic %.*s: %.*s\n",
		data->topicName->lenstring.len,
		data->topicName->lenstring.data,
		data->message->payloadlen,
		data->message->payload);
}
eMBErrorCode
eMBRegInputCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs)
{
	eMBErrorCode eStatus = MB_ENOERR;
	int iRegIndex;

	if ((usAddress >= REG_INPUT_START) && (usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS))
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
eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode)
{
	eMBErrorCode eStatus = MB_ENOERR;
	int iRegIndex;

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
eMBRegCoilsCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode)
{
	return MB_ENOREG;
}

eMBErrorCode
eMBRegDiscreteCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNDiscrete)
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
#endif /* USE_DHCP */
		printf("The network cable is not connected \n");
		return 1;
	}
}

void rede_adc()
{
	int32_t adctemp;
	//	int32_t adctemp;
	//	InitADline();

	filter(&adctemp);
//	usRegHoldingBuf[11] = adctemp << 16;
//	usRegHoldingBuf[11] |= adctemp;
	printf("adcvule:%d\n", adctemp);
}
void tcp_server(int conn) 
{
	int buflen = 128;
	int ret;
	unsigned char recv_buffer[128];
	uint8_t  error, lenth;
	uint16_t crc;
	uint8_t Timeout = 240;
	int32_t tickstart = 0U;
	printf("start modbus tcp\r\n");
//		ret = recv(conn, recv_buffer, buflen,0); 
//	while (ret > 0) 
//	{
//		ret = read(conn, recv_buffer, buflen);
//		write(conn, recv_buffer, ret);
//		memset(recv_buffer, 0, sizeof(recv_buffer));
//	}
	
	//ret = server_read(conn, recv_buffer, buflen, 200);
	tickstart = user_GetTick();
	while ((ret = server_read(conn, recv_buffer, buflen, 200)) != 0) 
	{
		if (ret>0)
		{
			tickstart = user_GetTick();
			
			crc = (recv_buffer[(ret - 1)] << 8) | (recv_buffer[(ret - 2)]);

			if (crc != usMBCRC16(recv_buffer, (ret - 2)))
			{
//				close(conn);
				break;
			}
			decoding(recv_buffer, &error,&lenth);
			ret = ret + lenth;
			if (error == 1)
			{
//				close(conn);
				break;
			}
			crc = usMBCRC16(recv_buffer, (ret - 2));
			recv_buffer[(ret - 1)] = crc >> 8;
			recv_buffer[(ret - 2)] = crc & 0xff;
			taskENTER_CRITICAL();
			write(conn, recv_buffer, ret);
			taskEXIT_CRITICAL();
			memset(recv_buffer, 0, sizeof(recv_buffer));
		}
	//	printf("s:%d", ret);
		if ((user_GetTick() - tickstart) > Timeout)
		{
			printf("close:%d\n", conn);
			break;
			/* In case of write timeout */	
		}
	}
	//shutdown(conn, SHUT_RD);
	close(conn);
	printf("close modbus tcp\r\n");
}
int server_read(int newconn, unsigned char *buffer, int len, int timeout_ms)
{
	uint8_t myerrno;
	struct timeval interval = { timeout_ms / portTICK_PERIOD_MS / 1000, (timeout_ms % (portTICK_PERIOD_MS * 1000)) * 1000 };
	//	if (interval.tv_sec < 0 || (interval.tv_sec == 0 && interval.tv_usec <= 0))
	//	{
	//		interval.tv_sec = 0;
	//		interval.tv_usec = 1000;
	//	}

	setsockopt(newconn, SOL_SOCKET, SO_RCVTIMEO, (char *)&interval, sizeof(struct timeval));
	//	setsockopt(n->my_socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&xTicksToWait, sizeof( xTicksToWait));
	//	printf("err:%d", eer);
	//	int bytes = 0;
	//	while (bytes < len)
	//	{
	int rc = recv(newconn, buffer, len, 0);
	myerrno = errno;
	if (rc == -1)
	{
	//	printf("servererrno:%d\n", myerrno);
		if (myerrno == EAGAIN || myerrno == EINTR) return -2;
		//	else if (errno == EINTR)return -3 ;
		else if(myerrno > 0)  return 0;    //0闁哄鏅濋崑鐐垫暜閹绢喖妫樻い鎾跺仧�???
		else return - 1;

		//if (errno != ENOTCONN && errno != ECONNRESET)
		//{
		//}
	}
	return rc;
}

void restore(void)
{
	uint8_t zero = 0;
	printf("\nstart restore\n");
	HAL_IWDG_Refresh(&hiwdg);
	HAL_GPIO_TogglePin(WDI_GPIO_Port, WDI_Pin);
	BSP_LED_On(LED_RED);
	BSP_LED_On(LED_GREEN);
	osDelay(500);
	HAL_GPIO_TogglePin(WDI_GPIO_Port, WDI_Pin);
	taskDISABLE_INTERRUPTS();


	I2C_EEPROM_WriteBuffer(EE_ipaddr, &zero, 12);
	osDelay(10);
	I2C_EEPROM_WriteBuffer(EE_timeaddr, &zero, 24);
	osDelay(10);
	I2C_EEPROM_WriteBuffer(EE_timeaddr + 24, &zero, 24);
	osDelay(10);
	HAL_GPIO_TogglePin(WDI_GPIO_Port, WDI_Pin);
	//HAL_IWDG_Refresh(&hiwdg);*/
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


//int keepalive(Client* c)
//{
//	int rc = FAILURE;
//	static Timer sTmrBack;
//
//	if (c->keepAliveInterval == 0)
//	{
//		rc = OKDONE;
//		goto exit;
//	}
//
//	if (c->ping_outstanding == 1 && expired(&sTmrBack))
//		return -1;		// No PINGRESP received after ping
//
//	rc = OKDONE;
//	if (expired(&c->ping_timer))
//	{
//		if (!c->ping_outstanding)
//		{
//			Timer timer;
//			InitTimer(&timer, 10);
//			countdown_ms(&timer, 1000);
//			int len = MQTTSerialize_pingreq(c->buf, c->buf_size);
//			if (len > 0)
//			{
//				rc = sendPacket(c, len, &timer);
//				if (rc == OKDONE)
//				{
//					c->ping_outstanding = 1;
//					countdown_ms(&sTmrBack, 5000);
//				}
//			}
//		}
//	}
//
////exit:
////	return rc;
//}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
