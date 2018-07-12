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
osThreadId socket_cTaskHandle;
osThreadId socketTaskHandle;
osThreadId recvTaskHandle;
osMessageQId recvQueueHandle;
osMessageQId MBQueueHandle;
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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityHigh, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
//
//  /* definition and creation of mainMB_TASK */
//  osThreadDef(mainMB_TASK, vMBServerTask, osPriorityAboveNormal, 0, 512);
//  mainMB_TASKHandle = osThreadCreate(osThread(mainMB_TASK), NULL);
//
//  /* definition and creation of monitorTask */
  osThreadDef(monitorTask, monitor, osPriorityBelowNormal, 0, 128);
  monitorTaskHandle = osThreadCreate(osThread(monitorTask), NULL);
//
//  /* definition and creation of uMBpoll_task */
//  osThreadDef(uMBpoll_task, uMBpoll, osPriorityNormal, 0, 512);
//  uMBpoll_taskHandle = osThreadCreate(osThread(uMBpoll_task), NULL);
//
//  /* definition and creation of periodTask */
  osThreadDef(periodTask, period, osPriorityNormal, 0, 256);
  periodTaskHandle = osThreadCreate(osThread(periodTask), NULL);
//
//  /* definition and creation of socket_cTask */
  osThreadDef(socket_cTask, socket_clien, osPriorityAboveNormal, 0, 640);
  socket_cTaskHandle = osThreadCreate(osThread(socket_cTask), NULL);
//
//  /* definition and creation of socketTask */
  osThreadDef(socketTask, socketsever, osPriorityAboveNormal, 0, 512);
  socketTaskHandle = osThreadCreate(osThread(socketTask), NULL);

  /* definition and creation of recvTask */
  osThreadDef(recvTask, socket1, osPriorityNormal, 0, 128);
  recvTaskHandle = osThreadCreate(osThread(recvTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of recvQueue */
/* what about the sizeof here??? cd native code */
	osMessageQDef(recvQueue, 5, sizeof(struct Msg));
  recvQueueHandle = osMessageCreate(osMessageQ(recvQueue), NULL);

  /* definition and creation of MBQueue */
/* what about the sizeof here??? cd native code */
  osMessageQDef(MBQueue, 16, uint16_t);
  MBQueueHandle = osMessageCreate(osMessageQ(MBQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
//  MX_LWIP_Init();

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
	const portTickType xFrequency = pdMS_TO_TICKS(5000);  
   
	
	xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
//	  HAL_GPIO_TogglePin(WDI_GPIO_Port, WDI_Pin);
	  HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
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
	uint32_t  phyreg = 0U;
	u_int8_t buflen = 32;
	u_int8_t  error;
	u_int16_t crc;
	u_int8_t connectflage = 0, errorcant = 0;
	int ret;
	unsigned char recv_buffer[buflen];
	uint8_t buff[] = { 0x01, 0x02, 0x03 };
	uint8_t rc = 0;
	Network network;	
	/* -----------------------  ---------------------------------*/
	BaseType_t xResult;
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS(200); /*  wait_time */
	RS485_MSG_T *tcp_c_msg; 
	memset(tcp_c_msg, 0, sizeof(*tcp_c_msg)); 
	/* -----------------------  ---------------------------------*/
	char* address = "192.168.1.92";
	char* port = "1992";
	rc = NetworkConnect(&network, address, port);
	if (rc != 0)	
		{
			connectflage = 1;
			close(network.my_socket);
			printf("Return code from network connect is %d\n", rc);
		}
	printf("socket:%d\n", network.my_socket);
	/* -----------------------  ---------------------------------*/
 
  /* Infinite loop */
  for(;;)
  {
	  
	 
//	
/* -----------------------  ---------------------------------*/
	  if (connectflage==2)
	  {
		  __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE); 
		  __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
		  BSP_LED_On(LED_RED);
		  HAL_ETH_ReadPHYRegister(&heth, PHY_BSR, &phyreg);
		  if ((phyreg & PHY_LINKED_STATUS) == PHY_LINKED_STATUS)
		  {
			  osDelay(500);  
			  rc = NetworkConnect(&network, address, port);
			  if (rc == 0)
			  {
				  connectflage = 0;
				  errorcant = 0;
				  BSP_LED_Off(LED_RED);
			  }
		  }
	  }
/* -----------------------  ---------------------------------*/	 
	  if (connectflage==1)
	  {
		  __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE); 
		  __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
		  BSP_LED_On(LED_RED);
		  osDelay(500);  
		  rc = NetworkConnect(&network, address, port);
		  if (rc == 0)
		  {
			  connectflage = 0;
			  errorcant = 0;
			  BSP_LED_Off(LED_RED);
		  }
		  else
		  {
			  osDelay(10); 
			  close(network.my_socket);
			  printf("Return code from network connect is %d\n", rc);
		  }
		  errorcant++;
	  }
	  if (errorcant > 5)
	  {
		  errorcant = 0;
	  }
 /* -----------------------  ---------------------------------*/
	  
	  else if (connectflage == 0)
	  {
		  
 /* -----------------------  ---------------------------------*/
		  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); 
		  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
		  
		  xResult = xQueueReceive( recvQueueHandle,
			  		  /* 消息队列句柄 */
			  		    (void *)&tcp_c_msg,
			  		  /* 存储接收到的数据变量 ucQueueMsgValue 中 */ 
			  		    (TickType_t)xMaxBlockTime); /* 设置阻塞时间 */
			  	  if (xResult == pdPASS)
			  	  {
			  		  BSP_LED_Toggle(LED_RED);
				  	  
//			  		  HAL_GPIO_WritePin(_485DIR_GPIO_Port, _485DIR_Pin, GPIO_PIN_SET);
//			  		  osDelay(10);
					write(network.my_socket, rs485_MSG.Data, rs485_MSG.lengh );
//			  		  HAL_UART_Transmit(&huart1, tcp_c_msg->Data, tcp_c_msg->lengh,100);
//			  		  HAL_GPIO_WritePin(_485DIR_GPIO_Port, _485DIR_Pin, GPIO_PIN_RESET);
			  		  /* 成功接收，并通过串口将数据打印出来 */ 
			  		  // DbgLog("ATgprs:%s", gprs_msg->usData);
				  	  
			  		   memset(&rs485_MSG, 0, sizeof(rs485_MSG));  
			  	  }
			  	  else 
			  	  {
			  		  /* 超时 */
				  	//  memset(&rs485_MSG, 0, sizeof(rs485_MSG));  
			  		  //  ErrLog("超时");
			  	  } 
		  
 /* -----------------------  ---------------------------------*/
		  
		  
/* -----------------------  ---------------------------------*/
		  ret = FreeRTOS_read(&network, recv_buffer, buflen, 500);
		  
		  if (ret > 0)
		  {
			  //		  crc = (recv_buffer[(ret - 1)] << 8) | (recv_buffer[(ret - 2)]);
			  //
			  //		  if (crc != usMBCRC16(recv_buffer, (ret - 2)))
			  //		  {
			  //			  FreeRTOS_disconnect(&network);
			  //			  break;
			  //		  }
			  //		  decoding(recv_buffer, &error);
			  //		  if (error == 1)
			  //		  {
			  //			  FreeRTOS_disconnect(&network);
			  //			  break;
			  //		  }
			  //		  crc = usMBCRC16(recv_buffer, (ret - 2));
			  //		  recv_buffer[(ret - 1)] = crc >> 8;
			  //		  recv_buffer[(ret - 2)] = crc & 0xff;
			 // FreeRTOS_write(&network, recv_buffer, ret, 100);
			   write(network.my_socket, recv_buffer, buflen);
			  memset(recv_buffer, 0, buflen);
		  }
/* -----------------------  ---------------------------------*/
		  else if (ret == 0)
		  {
			  connectflage = 1;	  
			  close(network.my_socket);
		  }
		  else
		  { 
			  HAL_ETH_ReadPHYRegister(&heth, PHY_BSR, &phyreg);
			  if ( (phyreg & PHY_LINKED_STATUS) != PHY_LINKED_STATUS)
				{
					connectflage = 2;	  
					close(network.my_socket);
				}
		  }
/* -----------------------  ---------------------------------*/
		    printf("ret:%d\n", ret);
		  //  BSP_LED_Toggle(LED_RED);  
		      write(network.my_socket, buff, 3);
	  }
	  osDelay(100); 
//	  printf("cant:%d", errorcant);  
//	  printf("cantf:%d\n", connectflage);  
	   
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
//	BaseType_t xResult;
//	const TickType_t xMaxBlockTime = pdMS_TO_TICKS(200); /* 璁剧疆绛夊緟鏃堕棿 wait_time */
//	RS485_MSG_T *tcp_c_msg; 
//	memset(tcp_c_msg, 0, sizeof(*tcp_c_msg)); 
//	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); 
//	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  /* Infinite loop */
  for(;;)
  {
//	  xResult = xQueueReceive( recvQueueHandle,
//		  /* 消息队列句柄 */
//		    (void *)&tcp_c_msg,
//		  /* 存储接收到的数据变量 ucQueueMsgValue 中 */ 
//		    (TickType_t)xMaxBlockTime); /* 设置阻塞时间 */
//	  if (xResult == pdPASS)
//	  {
//		  BSP_LED_Toggle(LED_RED);
//		  HAL_GPIO_WritePin(_485DIR_GPIO_Port, _485DIR_Pin, GPIO_PIN_SET);
//		  osDelay(10);
//
//		  HAL_UART_Transmit(&huart1, tcp_c_msg->Data, tcp_c_msg->lengh,100);
//		  HAL_GPIO_WritePin(_485DIR_GPIO_Port, _485DIR_Pin, GPIO_PIN_RESET);
//		  /* 成功接收，并通过串口将数据打印出来 */ 
//		  // DbgLog("ATgprs:%s", gprs_msg->usData);
//
//		   memset(&rs485_MSG, 0, sizeof(rs485_MSG));  
//	  }
//	  else 
//	  {
//		  /* 超时 */
//		  //  ErrLog("超时");
//	  } 
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
	struct sockaddr_in address;
	int rc = -1;
	sa_family_t family = AF_INET;
	struct addrinfo *result = NULL;
	struct addrinfo hints = { 0, AF_UNSPEC, SOCK_STREAM, IPPROTO_TCP, 0, NULL, NULL, NULL };
//	static struct timeval tv;

	
	if ((rc = getaddrinfo(addr, port, &hints, &result)) == 0)
	{
		struct addrinfo* res = result;
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
			address.sin_family  = AF_INET;
			address.sin_addr = ((struct sockaddr_in*)(result->ai_addr))->sin_addr;
		}
		else
			rc = -1;
		freeaddrinfo(result);
	}
	if (rc == 0)
	{
		n->my_socket =	socket(AF_INET, SOCK_STREAM, 0);
		if (n->my_socket != -1)
		{	
//			eer = setsockopt(n->my_socket闂備焦瀵х粙鎴︻敆閻滀竞_S0CKET, SO_RCVTIMEO闂?????(char *)&nNetTimeout, sizeof(nNetTimeout));
//			printf("ferr:%d", eer);
			if (family == AF_INET)
			rc = connect(n->my_socket, (struct sockaddr*)&address, sizeof(address));
		}
	}
//	if (rc == 0)
//	{
////		rc = getsockname(n->my_socket, (struct sockaddr *)&address, &len);
//		//if (error  >= 0) printf("Server %s connected, local port %d\n", srv, ntohs(servaddr.sin_port));
////		return n->my_socket;
//	}

	return rc;
}

int FreeRTOS_read(Network* n, unsigned char* buffer, int len, int timeout_ms)
{
	struct timeval interval = { timeout_ms / portTICK_PERIOD_MS/1000, (timeout_ms % (portTICK_PERIOD_MS*1000)) * 1000 };
//	if (interval.tv_sec < 0 || (interval.tv_sec == 0 && interval.tv_usec <= 0))
//	{
//		interval.tv_sec = 0;
//		interval.tv_usec = 1000;
//	}

	setsockopt(n->my_socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&interval, sizeof(struct timeval));
//	setsockopt(n->my_socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&xTicksToWait, sizeof( xTicksToWait));
//	printf("err:%d", eer);
//	int bytes = 0;
//	while (bytes < len)
//	{
		int rc = recv(n->my_socket, buffer, len, 0);
		if (rc == -1)
		{ 
			printf("errno:%d", errno);
			if (errno == EAGAIN) return -2;
			else if (errno == EINTR)return -3 ;
			else if (errno == ECONNRESET)return 0 ; //0閺夆晝鍋炵敮鎾棘椤撶偟纾?
			else return -1;
			
			
			//if (errno != ENOTCONN && errno != ECONNRESET)
			//{
			//}
		}
//		else if (rc == 0) return -1;
		//else if (rc==0) break;
//		else bytes += rc;
//	}
	return rc;
}


int FreeRTOS_write(Network* n, unsigned char* buffer, int len, int timeout_ms)
{
	struct timeval tv;

	tv.tv_sec = 0; /* 30 Secs Timeout */
	tv.tv_usec = timeout_ms * 1000;   // Not init'ing this can cause strange errors

	setsockopt(n->my_socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(struct timeval));
	int	rc = write(n->my_socket, buffer, len);
	return rc;
}


void FreeRTOS_disconnect(Network* n)
{
	close(n->my_socket);
//	closesocket(n->my_socket);
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
