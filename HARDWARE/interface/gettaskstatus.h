#ifndef __GETTASKSTATUS_H__
#define __GETTASKSTATUS_H__
#include "stm32f4xx.h"


extern volatile unsigned long long FreeRTOSRunTimeTicks;
void ConfigureTimeForRunTimeStats(void);
void TIM5_Init(u16 arr, u16 psc);

#endif
