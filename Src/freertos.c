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

/* ------------------------ FreeModbus includes --------------------------- */
#include "mb.h"
#include "iwdg.h"
/* ------------------------ Project includes ------------------------------ */
#include "tim.h"

#include  "common.h"
//#include "stm32f4xx_hal.h"
#include "Relay.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId mainMB_TASKHandle;
osThreadId ledTaskHandle;
osThreadId uMBpoll_taskHandle;
osMutexId MBholdingMutexHandle;

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void vMBServerTask(void const * argument);
void led(void const * argument);
void uMBpoll(void const * argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
/* ------------------------ Defines --------------------------------------- */

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

  /* definition and creation of mainMB_TASK */
  osThreadDef(mainMB_TASK, vMBServerTask, osPriorityAboveNormal, 0, 512);
  mainMB_TASKHandle = osThreadCreate(osThread(mainMB_TASK), NULL);

  /* definition and creation of ledTask */
  osThreadDef(ledTask, led, osPriorityBelowNormal, 0, 128);
  ledTaskHandle = osThreadCreate(osThread(ledTask), NULL);

  /* definition and creation of uMBpoll_task */
  osThreadDef(uMBpoll_task, uMBpoll, osPriorityNormal, 0, 512);
  uMBpoll_taskHandle = osThreadCreate(osThread(uMBpoll_task), NULL);

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
//  MX_LWIP_Init();

  /* USER CODE BEGIN StartDefaultTask */
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
	uint16_t pulse = 0;
	uint16_t step;
	for (;;)
	{	
		if (pulse == 0) 
		{
			step = 1;
			HAL_GPIO_WritePin(REALY4_GPIO_Port, REALY4_Pin, GPIO_PIN_RESET);
			osDelay(1000);
			// HAL_GPIO_TogglePin(REALY4_GPIO_Port, REALY4_Pin);
			HAL_GPIO_WritePin(REALY4_GPIO_Port, REALY4_Pin, GPIO_PIN_SET);
			osDelay(2500);	
		}
		
		if (pulse == 1000) step = -1;
	
		pulse += step;
		
		TIM1->CCR1 = pulse;
		osDelay(3);
	}
//	struct netconn *conn, *newconn;
//	err_t err, accept_err;
//	struct netbuf *buf;
//	char  tcpbuf;
//	uint8_t *data;
//	uint16_t Pulse;
//	u16_t len;
//      
//	LWIP_UNUSED_ARG(argument);
//
//	/* Create a new connection identifier. */
//	conn = netconn_new(NETCONN_TCP);
//  
//	if (conn != NULL)
//	{  
//		/* Bind connection to well known port number 7. */
//		err = netconn_bind(conn, NULL,500);
//    
//		if (err == ERR_OK)
//		{
//			/* Tell connection to go into listening mode. */
//			netconn_listen(conn);
//			for (;;)
//			{
//				/* */
//				accept_err = netconn_accept(conn, &newconn);
//    
//				/* Process the new connection. */
//				if (accept_err == ERR_OK) 
//				{
//
//					while (netconn_recv(newconn,&buf) == ERR_OK) 
//					{
//						
//						do 
//						{
//						//	taskDISABLE_INTERRUPTS(); 	
//							netbuf_data(buf, (void * *)&data, &len);
//							
//							if (data[0] == 0 && data[1] == 0x06)
//							{
//								if (data[2] == 0 && data[3] == 0x01) 
//								{
//									if (data[5] == 0) 	HAL_GPIO_WritePin(REALY4_GPIO_Port, REALY4_Pin, GPIO_PIN_RESET);
//									else	HAL_GPIO_WritePin(REALY4_GPIO_Port, REALY4_Pin, GPIO_PIN_SET);
//								}	
//								else if (data[2] == 0 && data[3] == 0x0a) 
//								{
//									Pulse = ((uint16_t)data[4] << 8) | ((uint16_t)(data[5])); 
//									if (Pulse > 1000) Pulse = 1000;
//									
//									TIM1->CCR1 = Pulse;
//				 				}	
//							}
//							netconn_write(newconn, data, len, NETCONN_COPY);
//						//	taskENABLE_INTERRUPTS();
//          
//						} while (netbuf_next(buf) >= 0);
//          
//						netbuf_delete(buf);
//					}
//        
//					/* Close connection and discard connection identifier. */
//					netconn_close(newconn);
//					netconn_delete(newconn);
//				}
//				osDelay(1);
//			}
//		}
//		else
//		{
//			netconn_delete(newconn);
//		}
//	}
  /* USER CODE END vMBServerTask */
}

/* led function */
void led(void const * argument)
{
  /* USER CODE BEGIN led */
	uint16_t pwm_value;
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(REALY7_GPIO_Port, REALY7_Pin);
	  HAL_IWDG_Refresh(&hiwdg);
	  osDelay(1000);
  }
  /* USER CODE END led */
}

/* uMBpoll function */
void uMBpoll(void const * argument)
{
  /* USER CODE BEGIN uMBpoll */
//	uint8_t Relay;
//	uint8_t DI;
//	uint16_t pulse;
	/* Infinite loop */
	for (;;)
	{
		osDelay(1);
	}
  /* USER CODE END uMBpoll */
}

/* USER CODE BEGIN Application */

//void user_pwm_setvalue(uint16_t value)
//{
//	TIM_OC_InitTypeDef sConfigOC;
// 
//	sConfigOC.OCMode = TIM_OCMODE_PWM1;
//	sConfigOC.Pulse = value;
//	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//	HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);   
//}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
