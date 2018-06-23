#ifndef __ADS1230_H
#define __ADS1230_H
#include "gpio.h"
/**********************************************************************
                      ADS1230
*   接口定义：DOUT<--->P2.0; SCLK<--->P2.1; PDWN<--->P2.2   *
***********************************************************************/
//#define ADS_IE                  P2IE
//#define ADS_IFG                 P2IFG
//#define ADS_REN                 P1REN
//#define ADS_DIR                 P1DIR
//#define ADS_OUT                 P1OUT
//#define ADS_IN                  P1IN
//#define ADS_DATA_BIT            BIT0  //ADS_DOUT
//#define ADS_CLK_BIT             BIT5  //ADS_CLK
//#define ADS_PD_BIT              BIT6  //ADS_PDWN
//#define ADS_SPEED_BIT           BIT7  //ADS_speed
////    P3DIR |= BIT7; //speed
////    P3OUT|=BIT7;
//#define ADS_DATA_DIR_OUT()       ADS_DIR |= ADS_DATA_BIT;
//#define ADS_DATA_DIR_IN()      ADS_DIR &= ~ADS_DATA_BIT;
//// {ADS_REN |= ADS_DATA_BIT;}
//#define ADS_CLK_DIR_OUT()       ADS_DIR |= ADS_CLK_BIT;
//#define ADS_PD_DIR_OUT()        ADS_DIR |= ADS_PD_BIT;
//
//#define AD_IS_ON                (ADS_OUT & ADS_PD_BIT)                  
//#define AD_ON()                 ADS_OUT |=  ADS_PD_BIT;                  
//#define AD_OFF()                ADS_OUT &= ~ADS_PD_BIT;
//
//#define ADS_DATA_H()            ADS_OUT |= ADS_DATA_BIT;

#define ADS_CLK_H()             HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);  
#define ADS_CLK_L()             HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);  
//
//#define ADS_SPEED_H()             ADS_OUT |= ADS_SPEED_BIT;
//#define ADS_SPEED_L()             ADS_OUT &= ~ADS_SPEED_BIT;


#define ADS_DATA_HI              HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)
//#define ADS_DATA_NOT_READY      (ADS_IN & ADS_DATA_BIT)//AD_CLK输出位1

uint32_t DWT_Delay_Init(void);

/**
 * @brief  This function provides a delay (in microseconds)
 * @param  microseconds: delay in microseconds
 */
__STATIC_INLINE void DWT_Delay_us(volatile uint32_t microseconds)
{
	uint32_t clk_cycle_start = DWT->CYCCNT;

	/* Go to number of cycles for system */
	microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000);
    
	/* Delay till end */
	while ((clk_cycle_start-DWT->CYCCNT ) < microseconds) ;
}


int32_t  ReadAD(void);     //读AD，从数据线上读取AD输出的数据,可在查询或中断中调用
void  OffsetAD(void);  //失调校准，一般用来在启动AD后，进行一次AD校准
void  InitADline(void);//初始化AD控制线，在启动AD前调用
#endif
/************************************END***************************************/