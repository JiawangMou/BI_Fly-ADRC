#ifndef __STABALIZER_H
#define __STABALIZER_H
#include <stdbool.h>
#include <stdint.h>
#include "stabilizer_types.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * �������ȿ��ƴ���	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/


#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (MAIN_LOOP_RATE / RATE_HZ)) == 0)

#define MAIN_LOOP_RATE 			RATE_1000_HZ
#define MAIN_LOOP_DT			(u32)(1000/MAIN_LOOP_RATE)	/*��λms*/
#define MAIN_LOOP_DTS    		(1.0/MAIN_LOOP_RATE)	    /*��λs*/

#define POSITION_ESTIMAT_RATE	RATE_250_HZ	//λ��Ԥ������
#define ATTITUDE_ESTIMAT_RATE	RATE_250_HZ	//��̬��������
#define ATTITUDE_ESTIMAT_DT		(1.0/ATTITUDE_ESTIMAT_RATE)


#define POSITION_ESTIMAT_DT		(1.0/POSITION_ESTIMAT_RATE)

// #define RATE_PID_RATE			RATE_500_HZ //���ٶȻ����ڻ���PID����
// #define RATE_PID_DT				(1.0/RATE_PID_RATE)

// #define ANGEL_PID_RATE			RATE_250_HZ //�ǶȻ����⻷��PID����
// #define ANGEL_PID_DT			(1.0/ANGEL_PID_RATE)


// #define VEL_PID_RATE		    RATE_250_HZ //λ�û����⻷��PID����
// #define VEL_PID_DT			    (1.0/VEL_PID_RATE)

// #define POS_PID_RATE		    RATE_100_HZ //λ�û����⻷��PID����
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
void setFastAdjustPosParam(u16 velTimes, u16 absTimes, float height);/*���ÿ��ٵ���λ�ò���*/
control_t getControlData(void);
mode_e getZmode(void);
int8_t calculateThrottlePercentAbs(void);
state_t getState(void);
setpoint_t getSetpoint(void);


#endif /* __STABALIZER_H */
