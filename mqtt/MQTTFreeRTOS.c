/*******************************************************************************
 * Copyright (c) 2014, 2015 IBM Corp.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v10.html
 * and the Eclipse Distribution License is available at
 *   http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    Allan Stockdill-Mander - initial API and implementation and/or initial documentation
 *    Ian Craggs - convert to FreeRTOS
 *******************************************************************************/

#include "MQTTFreeRTOS.h"
#include <string.h>
//typedef struct Network Network;

int ThreadStart(Thread* thread, void (*fn)(void*), void* arg)
{
	int rc = 0;
	uint16_t usTaskStackSize = (configMINIMAL_STACK_SIZE * 5);
	UBaseType_t uxTaskPriority = uxTaskPriorityGet(NULL); /* set the priority as the same as the calling task*/

	rc = xTaskCreate(fn,	/* The function that implements the task. */
		"MQTTTask",			/* Just a text name for the task to aid debugging. */
		usTaskStackSize,	/* The stack size is defined in FreeRTOSIPConfig.h. */
		arg,				/* The task parameter, not used in this case. */
		uxTaskPriority,		/* The priority assigned to the task is defined in FreeRTOSConfig.h. */
		&thread->task);		/* The task handle is not used. */

	return rc;
}


void MutexInit(Mutex* mutex)
{
	mutex->sem = xSemaphoreCreateMutex();
}

int MutexLock(Mutex* mutex)
{
	return xSemaphoreTake(mutex->sem, portMAX_DELAY);
}

int MutexUnlock(Mutex* mutex)
{
	return xSemaphoreGive(mutex->sem);
}


void TimerCountdownMS(Timer* timer, unsigned int timeout_ms)
{
	timer->init_tick = HAL_GetTick();
	timer->timeout_ms = timeout_ms;
}


void TimerCountdown(Timer* timer, unsigned int timeout) 
{
	TimerCountdownMS(timer, timeout * 1000);
}


int TimerLeftMS(Timer* timer) 
{
	int ret = 0;
	uint32_t cur_tick = HAL_GetTick();   // The HAL tick period is 1 millisecond.
	if(cur_tick < timer->init_tick)
	{
		 // Timer wrap-around detected
	  // printf("Timer: wrap-around detected from %d to %d\n", timer->init_tick, cur_tick);
	  timer->timeout_ms -= 0xFFFFFFFF - timer->init_tick;
		timer->init_tick = 0;
	}
	ret = timer->timeout_ms - (cur_tick - timer->init_tick);

	return (ret >= 0) ? ret : 0;
}


char TimerIsExpired(Timer* timer)
{
	return (TimerLeftMS(timer) > 0) ? 0 : 1;
}


void TimerInit(Timer* timer)
{
	timer->init_tick = 0;
	timer->timeout_ms = 0;
}


int FreeRTOS_read(Network* n, unsigned char* buffer, int len, int timeout_ms)
{
	struct timeval interval = { timeout_ms / 1000, (timeout_ms % 1000) * 1000 };
	if (interval.tv_sec < 0 || (interval.tv_sec == 0 && interval.tv_usec <= 0))
	{
		interval.tv_sec = 0;
		interval.tv_usec = 1000;
	}

	setsockopt(n->my_socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&interval, sizeof(struct timeval));

	int bytes = 0;
	while (bytes < len)
	{
		int rc = recv(n->my_socket, &buffer[bytes], (size_t)(len - bytes), 0);
		if (rc == -1)
		{
			if (errno == EAGAIN) return -2;
			else if (errno == EINTR) continue;
			else return -1;
			//if (errno != ENOTCONN && errno != ECONNRESET)
			//{
			//}
		}
		else if (rc == 0) return -1;
		//else if (rc==0) break;
		else bytes += rc;
	}
	return bytes;
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
//	FreeRTOS_closesocket(n->my_socket);
}


void NetworkInit(Network* n)
{
	n->my_socket = 0;
	n->mqttread = FreeRTOS_read;
	n->mqttwrite = FreeRTOS_write;
	n->disconnect = FreeRTOS_disconnect;
}


int NetworkConnect(Network* n, char* addr, int port)
{
	int sockfd, error;
	struct sockaddr_in servaddr;
	socklen_t len;
	struct hostent *host;
    
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) return sockfd;

	host = gethostbyname(addr);
	if (NULL == host || host->h_addrtype != AF_INET) 
	{
		close(sockfd);
		return -2;
	}

	memset(&servaddr, 0, sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_port = htons(port);
	memcpy(&servaddr.sin_addr, host->h_addr, sizeof(struct in_addr));
	//inet_aton(srv, &(servaddr.sin_addr));
//inet_pton(AF_INET, srv, &servaddr.sin_addr);

	// TODO: Use SetSockOpt to
error = connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr));

	if (error == 0)
	{
		error = getsockname(sockfd, (struct sockaddr *)&servaddr, &len);
		//if (error  >= 0) printf("Server %s connected, local port %d\n", srv, ntohs(servaddr.sin_port));
		return sockfd;
	}
	else
	{
		//printf("Error connecting %d\n", error);
close(sockfd);
		return error;
	}
//	int type = SOCK_STREAM;
//	struct sockaddr_in address;
//	int rc = -1;
//	sa_family_t family = AF_INET;
//	struct addrinfo *result = NULL;
//	struct addrinfo hints = { 0, AF_UNSPEC, SOCK_STREAM, IPPROTO_TCP, 0, NULL, NULL, NULL };
//	static struct timeval tv;
//
//	n->my_socket = -1;
//	if (addr[0] == '[')
//		++addr;
//
//	if ((rc = getaddrinfo(addr, port, &hints, &result)) == 0)
//	{
//		struct addrinfo* res = result;
//		/* prefer ip4 addresses */
//		while (res)
//		{
//			if (res->ai_family == AF_INET)
//			{
//				result = res;
//				break;
//			}
//			res = res->ai_next;
//		}
//		if (result->ai_family == AF_INET)
//		{
//			address.sin_port = ((struct sockaddr_in*)(result->ai_addr))->sin_port;      // htons(port);
//			address.sin_family = family = AF_INET;
//			address.sin_addr = ((struct sockaddr_in*)(result->ai_addr))->sin_addr;
//		}
//		else
//			rc = -1;
//		freeaddrinfo(result);
//	}
//	if (rc == 0)
//	{
//		n->my_socket =	socket(family, type, 0);
//		if (n->my_socket != -1)
//		{
//			if (family == AF_INET)
//				rc = connect(n->my_socket, (struct sockaddr*)&address, sizeof(address));
//		}
//	}
//#ifdef USE_LCD 
//	uint8_t iptxt[20];
//	sprintf((char *)iptxt, "%s", ip4addr_ntoa((const ip4_addr_t *)&address.sin_addr));
//	printf("Static IP address: %s\n", iptxt);
//#endif
//	return rc;
//	struct sockaddr_in sAddr;
//	int retVal = -1;
//	struct addrinfo* res = NULL;//
//	struct addrinfo *result = NULL;
//	struct addrinfo hints = { 0, AF_UNSPEC, SOCK_STREAM, IPPROTO_TCP, 0, NULL, NULL, NULL };
//	if ((getaddrinfo(addr, NULL, &hints, &result)) == 0)
//	{
//		
//		res	= result;
//		/* prefer ip4 addresses */
//		while (res)
//		{
//			if (res->ai_family == AF_INET)
//			{
//				result = res;
//				break;
//			}
//			res = res->ai_next;
//		}
//		if (result->ai_family == AF_INET)
//		{
//			sAddr.sin_port = ((struct sockaddr_in*)(result->ai_addr))->sin_port; 
////			sAddr.sin_port = htons(port);
//			sAddr.sin_addr = ((struct sockaddr_in*)(result->ai_addr))->sin_addr;
//		}
//		else 	goto exit;
//		
//		freeaddrinfo(result);
//	
//		//sAddr.sin_port = ((struct sockaddr_in*)(result->ai_addr))->sin_port;       // htons(port);
//			
//	
//
//		if((n->my_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0)
//			goto exit;
//
//		if ((retVal = connect(n->my_socket, (struct sockaddr*) &sAddr, sizeof(sAddr))) < 0)
//		{
//			closesocket(n->my_socket);
//			goto exit;
//		}
//	}
//	else
//	{
//		goto exit;
//	}
//
//exit:
//	return retVal;
}


#if 0
int NetworkConnectTLS(Network *n, char* addr, int port, SlSockSecureFiles_t* certificates, unsigned char sec_method, unsigned int cipher, char server_verify)
{
	SlSockAddrIn_t sAddr;
	int addrSize;
	int retVal;
	unsigned long ipAddress;

	retVal = sl_NetAppDnsGetHostByName(addr, strlen(addr), &ipAddress, AF_INET);
	if (retVal < 0) {
		return -1;
	}

	sAddr.sin_family = AF_INET;
	sAddr.sin_port = sl_Htons((unsigned short)port);
	sAddr.sin_addr.s_addr = sl_Htonl(ipAddress);

	addrSize = sizeof(SlSockAddrIn_t);

	n->my_socket = sl_Socket(SL_AF_INET, SL_SOCK_STREAM, SL_SEC_SOCKET);
	if (n->my_socket < 0) {
		return -1;
	}

	SlSockSecureMethod method;
	method.secureMethod = sec_method;
	retVal = sl_SetSockOpt(n->my_socket, SL_SOL_SOCKET, SL_SO_SECMETHOD, &method, sizeof(method));
	if (retVal < 0) {
		return retVal;
	}

	SlSockSecureMask mask;
	mask.secureMask = cipher;
	retVal = sl_SetSockOpt(n->my_socket, SL_SOL_SOCKET, SL_SO_SECURE_MASK, &mask, sizeof(mask));
	if (retVal < 0) {
		return retVal;
	}

	if (certificates != NULL) {
		retVal = sl_SetSockOpt(n->my_socket, SL_SOL_SOCKET, SL_SO_SECURE_FILES, certificates->secureFiles, sizeof(SlSockSecureFiles_t));
		if (retVal < 0)
		{
			return retVal;
		}
	}

	retVal = sl_Connect(n->my_socket, (SlSockAddr_t *)&sAddr, addrSize);
	if (retVal < 0) {
		if (server_verify || retVal != -453) {
			sl_Close(n->my_socket);
			return retVal;
		}
	}

	SysTickIntRegister(SysTickIntHandler);
	SysTickPeriodSet(80000);
	SysTickEnable();

	return retVal;
}
#endif
