#ifndef __COMMMOCN_H
#define __COMMMOCN_H
#include "stm32f4xx_hal.h"
#ifdef __cplusplus
 extern "C" {
#endif

#define	PORT			8088
	 
	 
typedef enum 
{
    DO1=1,
 
	DO2,
	
	DO3,
	
	DO4,
	
	DO5,
	
	DO6,
	
	DO7,
	
    DO8
	
} DO;
		 
extern uint8_t IP_ADDRESS[4];
extern uint8_t NETMASK_ADDRESS[4];
extern uint8_t GATEWAY_ADDRESS[4];
extern uint8_t	 mode_flage ;

#endif