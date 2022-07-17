#ifndef __POWER_CONTROL_H
#define __POWER_CONTROL_H
#include "stabilizer_types.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * ����������ƴ���	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) �������������ӿƼ����޹�˾ 2014-2024
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
}Servostatus_t;

typedef struct 
{
	u32 	PWM;
	float	f_Hz;
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
//return Ϊ�����λ�� ��λΪus
u16 Servo_Int16ToPWM(u8 id, float value);
float ServoPWM2angle(u32 PWM,u8 id);

#endif 
