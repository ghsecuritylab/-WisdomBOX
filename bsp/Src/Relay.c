
/**
  ******************************************************************************
  * @file           : raly.h
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
/* Includes ------------------------------------------------------------------*/
#include "Relay.h"

void relaycontrol(DO realynumber,u_int8_t DI)	
{
	switch (realynumber)
	{
	case DO1 :
		if (DI == 0) 
			HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_RESET);
		else
			HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_SET);
		break;
	case DO2 :
		if (DI == 0) 
			HAL_GPIO_WritePin(RELAY2_GPIO_Port,RELAY2_Pin ,GPIO_PIN_RESET);
		else  
			HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin,GPIO_PIN_SET);
		break;
	case DO3 :
		if (DI == 0) 
			HAL_GPIO_WritePin(RELAY3_GPIO_Port, RELAY3_Pin, GPIO_PIN_RESET);
		else
			HAL_GPIO_WritePin(RELAY3_GPIO_Port, RELAY3_Pin, GPIO_PIN_SET);
		break;
	case DO4 :
		if (DI == 0) 
			HAL_GPIO_WritePin(RELAY4_GPIO_Port, RELAY4_Pin, GPIO_PIN_RESET);
		else
			HAL_GPIO_WritePin(RELAY4_GPIO_Port, RELAY4_Pin, GPIO_PIN_SET);
		break;
	case DO5 :
		if (DI == 0) 
			HAL_GPIO_WritePin(RELAY5_GPIO_Port, RELAY5_Pin,GPIO_PIN_RESET);
		else
			HAL_GPIO_WritePin(RELAY5_GPIO_Port, RELAY5_Pin,GPIO_PIN_SET);
		break;
	case DO6 :
		if (DI == 0) 
			HAL_GPIO_WritePin(RELAY6_GPIO_Port, RELAY6_Pin,GPIO_PIN_RESET);
		else
			HAL_GPIO_WritePin(RELAY6_GPIO_Port, RELAY6_Pin,GPIO_PIN_SET);
		break;
	case DO7 :
		if (DI == 0) 
			HAL_GPIO_WritePin(RELAY7_GPIO_Port, RELAY7_Pin, GPIO_PIN_RESET);
		else
			HAL_GPIO_WritePin(RELAY7_GPIO_Port, RELAY7_Pin, GPIO_PIN_SET);
		break;
	case DO8 :
		if (DI == 0) 
			HAL_GPIO_WritePin(RELAY8_GPIO_Port, RELAY8_Pin, GPIO_PIN_RESET);
		else
			HAL_GPIO_WritePin(RELAY8_GPIO_Port, RELAY8_Pin, GPIO_PIN_SET);
		break;
		
	default:
		break;
	}
}
	
