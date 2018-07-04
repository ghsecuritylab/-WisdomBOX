/**
  ******************************************************************************
  * @file           : cat1023.h
  * @brief          : 
  ******************************************************************************
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

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __CAT1023_H__
#define __CAT1023_H__

#ifdef __cplusplus
 extern "C" {
#endif	
	 
	 /* Includes ------------------------------------------------------------------*/
#include "common.h"
#include "stm32f4xx_hal.h"

/* Private define ------------------------------------------------------------*/
#define        CAT1023_ADDR_READ    0xA1

#define        CAT1023_ADDR_WRITE   0xA0

/* ########################## Assert Selection ############################## */
	 uint8_t I2C_EEPROM_WriteBuffer( uint16_t Reg, uint8_t *pBuffer, uint16_t Length);
	 
	 
		 
	uint8_t I2C_EEPROM_ReadBuffer( uint16_t Reg, uint8_t *pBuffer, uint16_t Length);	
	 
	 
		 
#endif	
		 
		 
		 