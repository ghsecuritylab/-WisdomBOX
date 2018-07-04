
#include "R8025t.h"
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include <string.h>
#include <stdio.h>
#include <memory.h>



STDATETIME      stDateTime;
SPECIALFLAG     specialFlag;


//DEVSTATE        devState;



//BCD码转二位十进制
uint8_t BCD2DEC(uint8_t temp)
{
  temp = (temp >> 4) * 10 + (temp & 0x0f);
  return temp;
}

//二位十进制转BCD码
uint8_t DEC2BCD(uint8_t temp)
{
  temp = (temp / 10) * 16 + (temp % 10);
  return temp;
}

void Get8025(uint8_t addr, uint8_t *data, uint8_t counter)//I2C_RX8025SA_ADDR
{ 
	HAL_I2C_Mem_Read(&hi2c3, RX8025_ADDR_READ, addr, I2C_MEMADD_SIZE_8BIT, data, counter, 100);
} 

void Set8025(uint8_t addr, uint8_t *data, uint8_t counter)
{ 
	HAL_I2C_Mem_Write(&hi2c3, RX8025_ADDR_WRITE, addr, I2C_MEMADD_SIZE_8BIT, data, counter, 100);
}
uint8_t       temp[3], pubRam = 0xff; 
uint8_t da[5];
void Init8025(void)
{   
	/* 电源复位检测功能 */
	Get8025(REGADDR_EXTEN, temp, 3);
	printf("old:%d\n\r", temp[0]);
	temp[1] &= RTC8025_PON;
	if (temp[1] == RTC8025_PON) //电源复位
	{
		da[0] = 0x00;
		da[1] = 0x00;          // 24小时模式设置,1Hz  频率输出
		da[2] = 0x00;
		da[3] = 0x00;
		da[4] = 0x60;  //0x68 (‭01101000‬)开启报警和跟新中断 0x60(‭01100000‬)之开启更新中断
		Set8025(REGADDR_TIM_CNT0, & da[0], 1);
		Set8025(REGADDR_TIM_CNT1, & da[1], 1);
		Set8025(REGADDR_EXTEN, & da[2], 1);
		Set8025(REGADDR_FLAG, & da[3], 1);
		Set8025(REGADDR_CONTROL, & da[4], 1);
		//    memset(pubRam,0XFF,1);
		    Get8025(REGADDR_CONTROL, &pubRam, 1);
    
		if (pubRam != da[4])
		{
			specialFlag.I2C8025F = 1;
		}
		else
		{
			specialFlag.I2C8025F = 0;
		}
		RtcSetLocalTime();
	}
 
	
	
//	Set8025(RTC8025T_Control1, 0, 1);        //清除标志位，为下次做准备
//	printf("new:%d\n", temp);
//
}  

void TimerDataHandle(uint8_t* pDate)
{
    stDateTime.second = BCD2DEC(pDate[0]);   
    stDateTime.minute = BCD2DEC(pDate[1]);
    
    if(pDate[2]==0x24)
        pDate[2] = 0;
    stDateTime.hour = BCD2DEC(pDate[2]);
    
    if(pDate[3] == 0x01)
        stDateTime.week = 0;
    else if(pDate[3] == 0x02)
        stDateTime.week = 1;
    else if(pDate[3] == 0x04)
        stDateTime.week = 2;
    else if(pDate[3] == 0x08)
        stDateTime.week = 3;
    else if(pDate[3] == 0x10)
        stDateTime.week = 4;
    else if(pDate[3] == 0x20)
        stDateTime.week = 5;
    else if(pDate[3] == 0x40)
        stDateTime.week = 6;
    
    stDateTime.day  = BCD2DEC(pDate[4]);
    stDateTime.month = BCD2DEC(pDate[5]);
    stDateTime.year  = BCD2DEC(pDate[6]);
}

uint8_t RtcSetoneTime(STDATETIME *pTime, uint8_t *flage)	
{
	uint8_t Timebuf[7];
	if (flage[0]==1)
	{
		if (Timebuf[0] > 59) return 0;
		Timebuf[0] = DEC2BCD(pTime->second);
		Set8025(0, &Timebuf[0], 1);     //Timebuf中数据为BCD码
	}
	if (flage[1] == 1)
	{
		if (Timebuf[1] > 59)
			return 0;
		Timebuf[1] = DEC2BCD(pTime->minute);
		Set8025(1, &Timebuf[1], 1);      //Timebuf中数据为BCD码
	}
	if (flage[2] == 1)
	{
		if (Timebuf[2] > 23)
			return 0;

		Timebuf[2] = DEC2BCD(pTime->hour);
		Set8025(2, &Timebuf[2], 1);      //Timebuf中数据为BCD码
	}
	if (flage[3] == 1)
	{
		if (Timebuf[3] > 6)
			return 0;

		Timebuf[3] = (0x01) << (pTime->week); 
		Set8025(3, &Timebuf[3], 1);      //Timebuf中数据为BCD码
	}
	if (flage[4] == 1)
	{
		printf("%d\n", Timebuf[4]);
		if ((Timebuf[4] < 1) || (Timebuf[4] > 31))
			return 0;

		Timebuf[4] = DEC2BCD(pTime->day);
		Set8025(4, &Timebuf[4], 1);      //Timebuf中数据为BCD码
	}
	if (flage[5] == 1)
	{
		if (Timebuf[5] > 11)
			return 0;
		Timebuf[5] = DEC2BCD(pTime->month);
		Set8025(5, &Timebuf[5], 1);      //Timebuf中数据为BCD码
	}
	if (flage[6] == 1)
	{
		Timebuf[6] = DEC2BCD(pTime->year);
		Set8025(6, &Timebuf[6], 1);      //Timebuf中数据为BCD码
	}

	return 1;
//	TimerDataHandle(Timebuf);
	
}

void RtcSetDateTime(STDATETIME *pTime)
{
	uint8_t Timebuf[7];
   
   Timebuf[0] = DEC2BCD(pTime->second);
   Timebuf[1] = DEC2BCD(pTime->minute);
   Timebuf[2] = DEC2BCD(pTime->hour);
   Timebuf[3] = (0x01)<<(pTime->week);  
   Timebuf[4] = DEC2BCD(pTime->day);
   Timebuf[5] = DEC2BCD(pTime->month);
   Timebuf[6] = DEC2BCD(pTime->year);
   
   Set8025(0,Timebuf,7);   //Timebuf中数据为BCD码
   TimerDataHandle(Timebuf);
}
  
void RtcSetLocalTime()
{  
  struct    tm *now_ptm;
  time_t     timep;
  STDATETIME set_time;                      //年月日时分秒都是BCD码
    
//  timep = time(NULL);                       //获取当前RTC时间戳
//  timep += 8 * 3600;                        //RTC时间戳转化成北京时间的时间戳  
//  now_ptm = gmtime(&timep);                 //指针指向结构体中所存为十进制
  set_time.second  = 0;//now_ptm->tm_sec;       //取值区间为[0,59]
  set_time.minute  = 0;//now_ptm->tm_min;       //取值区间为[0,59]
  set_time.hour    = 17;//now_ptm->tm_hour;      //取值区间为[0,23]
  set_time.week    = 1;//now_ptm->tm_wday;      //取值区间为[0,6]，0为星期天
  set_time.day    = 2;//now_ptm->tm_mday;      //取值区间为[1,31]
  set_time.month   = 7;// now_ptm->tm_mon + 1;   //取值区间为[0,11] ，0为1月
  set_time.year    = 18;// now_ptm->tm_year - 100;//tm的年从1900开始计算
  set_time.reserve = 0;  
  
  RtcSetDateTime(&set_time);
}
/****************************************************************
// Summary: 	判断时间是否有效
// Parameter: 	[in/u8*]pucTime 时间结构体
//
// return:		成功与否 
****************************************************************/
uint8_t CheckTime(uint8_t *pucTime)
{
	if (pucTime[0] > 59)
		return 0;
	if ((pucTime[1] < 1) || (pucTime[1] > 12))
		return 0;
	if ((pucTime[2] < 1) || (pucTime[2] > 31))
		return 0;
	if (pucTime[3] > 23)
		return 0;
	if (pucTime[4] > 59)
		return 0;
	if (pucTime[5] > 59)
		return 0;

	return 1;
}
void UpdateDateTime()
{
	uint8_t Timebuf[7];
    Get8025(RTC8025_Second, Timebuf, 7);   //Timebuf中数据为BCD码
    TimerDataHandle(Timebuf);
}


