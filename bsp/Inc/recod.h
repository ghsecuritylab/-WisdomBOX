/**
  ******************************************************************************
  * @file           : recod.h
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

#ifndef __RECOD_H__
#define __RECOD_H__


/* Includes ------------------------------------------------------------------*/
#include "common.h"
#include "stm32f4xx_hal.h"

/* Private define ------------------------------------------------------------*/

//控制
#define  setrelay1 0x01
#define  setrelay2 0x02
#define  setdac1   0x0a
#define  setdac2   0x0b
//电流
#define  converI   0x09
#define  readI     0x26
//时间
#define set               0x3a 
#define	set_Second        0x34  //秒寄存器

#define	set_Minute        0x33  //分寄存器

#define	set_Hour          0x32  //时寄存器
#define	set_Week          0x39  //星期寄存器
#define	set_Day           0x38  //日寄存器
#define	set_Month         0x37  //月寄存器
#define	set_Year          0x36  //年寄存器
//模式切换
#define  set_mode         0x3c


//ip地址
#define  setip1   0x3e
#define  setip2   0x3f

#define  setip3   0x42
#define  setip4   0x43
#define  setip5   0x44
#define  setip6   0x46


//定时

//重启
#define  reboot  

/* ########################## Assert Selection ############################## */

#ifdef __cplusplus
 extern "C" {
#endif	 
	 
void decoding(uint8_t * data, uint8_t * error);
	 

	 
	 
	 
#endif