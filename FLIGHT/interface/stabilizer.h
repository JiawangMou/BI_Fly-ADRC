#ifndef __STABALIZER_H
#define __STABALIZER_H
#include <stdbool.h>
#include <stdint.h>
#include "stabilizer_types.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 四轴自稳控制代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/


#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (MAIN_LOOP_RATE / RATE_HZ)) == 0)

#define MAIN_LOOP_RATE 			RATE_1000_HZ
#define MAIN_LOOP_DT			(u32)(1000/MAIN_LOOP_RATE)	/*单位ms*/
#define MAIN_LOOP_DTS    		(1.0/MAIN_LOOP_RATE)	    /*单位s*/

#define POSITION_ESTIMAT_RATE	RATE_250_HZ	//位置预估速率
#define ATTITUDE_ESTIMAT_RATE	RATE_250_HZ	//姿态解算速率
#define ATTITUDE_ESTIMAT_DT		(1.0/ATTITUDE_ESTIMAT_RATE)


#define POSITION_ESTIMAT_DT		(1.0/POSITION_ESTIMAT_RATE)

// #define RATE_PID_RATE			RATE_500_HZ //角速度环（内环）PID速率
// #define RATE_PID_DT				(1.0/RATE_PID_RATE)

// #define ANGEL_PID_RATE			RATE_250_HZ //角度环（外环）PID速率
// #define ANGEL_PID_DT			(1.0/ANGEL_PID_RATE)


// #define VEL_PID_RATE		    RATE_250_HZ //位置环（外环）PID速率
// #define VEL_PID_DT			    (1.0/VEL_PID_RATE)

// #define POS_PID_RATE		    RATE_100_HZ //位置环（外环）PID速率
// #define POS_PID_DT			    (1.0/POS_PID_RATE)




// #define MBD_RATE                RATE_250_HZ
// #define MBD_DT                  (1.0/MBD_RATE)

extern Debug_stabi_tick_t stabi_tick;




#ifdef TEST
#define WAIT_RATE 
#endif

void stabilizerInit(void);
void stabilizerTask(void* param);
bool stabilizerTest(void);

void getAttitudeData(attitude_t* get);
float getBaroData(void);

void getSensorData(sensorData_t* get);	
void getStateData(Axis3f* acc, Axis3f* vel, Axis3f* pos);
void setFastAdjustPosParam(u16 velTimes, u16 absTimes, float height);/*设置快速调整位置参数*/
control_t getControlData(void);
mode_e getZmode(void);
int8_t calculateThrottlePercentAbs(void);
state_t getState(void);
setpoint_t getSetpoint(void);


#endif /* __STABALIZER_H */
