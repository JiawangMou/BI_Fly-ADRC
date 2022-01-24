#ifndef __POWER_CONTROL_H
#define __POWER_CONTROL_H
#include "stabilizer_types.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 功率输出控制代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

typedef struct 
{
	u32 motorPWM_l;
	u32 motorPWM_r;
	u32 servoPWM_l;
	u32 servoPWM_r;
	u32 servoPWM_m;
}motorPWM_t;
typedef struct 
{
	u32 	PWM;
	float	angle;
	float	actual_angle;
}Servostatus_t;

typedef struct 
{
	u32 	PWM;
	float	f_Hz;
	float	actual_f_Hz;
}Motorstatus_t;
typedef struct 
{
	Servostatus_t servo_l;
	Servostatus_t servo_r;
#ifdef DOUBLE_WING			
	Servostatus_t servo_m;	
#endif	

	Motorstatus_t motor_l;
	Motorstatus_t motor_r;
}actuatorStatus_t;



void powerControlInit(void);
bool powerControlTest(void);
void motorControl(control_t *control);

void getMotorPWM(actuatorStatus_t* get);
void setMotorPWM(bool enable,motorPWM_t set);
//return 为舵机的位置 单位为us
u16 Servo_Int16ToPWM(u8 id, float value);
float ServoPWM2angle(u32 PWM,u8 id);
#endif 
