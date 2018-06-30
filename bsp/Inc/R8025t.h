#ifndef _RX8025_H_
#define	_RX8025_H_

#include "stm32f4xx_hal.h"
typedef struct
{
	uint8_t second;
	uint8_t minute;
	uint8_t hour;  
	uint8_t week;
	uint8_t date;   
	uint8_t month;
	uint8_t year;
	uint8_t reserve;
}STDATETIME;

typedef struct                   
{ 
	uint16_t I2C8025F : 1;          //8563时钟自检故障

}SPECIALFLAG;

typedef enum { false = 0, true = !false } bool;

#define I2C_RX8025SA_ADDR	0x64


// 时间寄存器定义
#define	RTC8025_Second        0  //秒寄存器
#define	RTC8025_Minute        1  //分寄存器
#define	RTC8025_Hour          2  //时寄存器
#define	RTC8025_Week          3  //星期寄存器
#define	RTC8025_Day           4  //日寄存器
#define	RTC8025_Month         5  //月寄存器
#define	RTC8025_Year          6  //年寄存器

// 控制寄存器定义(时钟芯片型号不相同，相应的配置也是不相同的)
#define	RTC8025T_Control1     (0x0D)  //控制1 寄存器 （R8025T） 
#define	RTC8025_Control1      (0x0E)  //控制1 寄存器  (R8025AC)

#define	RTC8025_PON           (0x10)  // RTC电源失效标志位
#define	RTC8025_XST           (0x20)  // RTC内部晶振失效标志位

// 工作模式定义
#define	RTC8025_Standard_Read (0x00)  //标准读模式
#define	RTC8025_Simple_Read   (0x04)  //简单读模式

void Get8025(uint8_t addr, uint8_t *data, uint8_t counter);
void Set8025(uint8_t addr, uint8_t *data, uint8_t counter);
void Init8025(void);
void RtcSetDateTime(STDATETIME *pTime);
void RtcSetLocalTime(void);
void UpdateDateTime(void);


#endif

