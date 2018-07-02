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

// 设备读写地址

#define        RX8025_ADDR_READ                0x65

#define        RX8025_ADDR_WRITE                0x64
//#define I2C_RX8025SA_ADDR	0x64

#define REGADDR_SEC             0x00
#define REGADDR_MIN             0x01
#define REGADDR_HOUR            0x02
#define REGADDR_WEEK            0x03
#define REGADDR_DAY             0x04
#define REGADDR_MONTH           0x05
#define REGADDR_YEAR            0x06
#define REGADDR_MIN_ALARM       0x07
#define REGADDR_HOUR_ALARM      0x08
#define REGADDR_WEEK_ALARM      0x09
#define REGADDR_DAY_ALARM       0x0A
#define REGADDR_TIM_CNT0        0x0B
#define REGADDR_TIM_CNT1        0x0C
#define REGADDR_EXTEN           0x0D
#define EXTEN_TEST            (1<<7)
#define EXTEN_WADA            (1<<6)
#define EXTEN_USEL            (1<<5)
#define EXTEN_TE              (1<<4)
#define EXTEN_FSEL1           (1<<3)
#define EXTEN_FSEL0           (1<<2)
#define EXTEN_TSEL1           (1<<1)
#define EXTEN_TSEL0           (1<<0)
#define REGADDR_FLAG            0x0E
#define FLAG_UF               (1<<5)
#define FLAG_TF               (1<<4)
#define FLAG_AF               (1<<3)
#define FLAG_VLF              (1<<1)
#define FLAG_VDET             (1<<0)
#define REGADDR_CONTROL         0x0F
#define CONTR_CSEL1           (1<<7)
#define CONTR_CSEL0           (1<<6)
#define CONTR_UIE             (1<<5)
#define CONTR_TIE             (1<<4)
#define CONTR_AIE             (1<<3)
#define CONTR_RESET           (1<<0)

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

#define	RTC8025_PON           (0x02)  // RTC电源失效标志位
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

