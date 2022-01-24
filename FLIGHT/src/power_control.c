#include "power_control.h"
#include "motors.h"
#include "config_param.h"
#include "model.h"
#include "math.h"
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

static bool motorSetEnable = false;
static actuatorStatus_t actuator;
static motorPWM_t motorPWMSet = {0, 0, 0, 0, 0};

void powerControlInit(void)
{
	motorsInit();
}

bool powerControlTest(void)
{
	bool pass = true;

	pass &= motorsTest();

	return pass;
}

u16 limitThrust(int value)
{
	if (value > UINT16_MAX)
	{
		value = UINT16_MAX;
	}
	else if (value < 0)
	{
		value = 0;
	}

	return (u16)value;
}

/*************************************************
Function: Servo_Int16ToPWM
Description: 将舵机相对中立位置位移的范围从-32767~32767 映射到 -SERVO_HalfRANGE ~ SERVO_HalfRANGE 范围内（SERVO_HalfRANGE 为舵机行程的一半）
			并且加上中立位置，得到最终的输出：900~2100的占空比
Input:  id: 指明是设置哪个舵机
        value: 舵机变化量对应的值（-32767~32767）；
Output: 范围在900~2100的占空比
Return: 范围在900~2100的占空比
Others: 无
********************************************/
u16 Servo_Int16ToPWM(u8 id, float value)
{
	float ratio = 0;
	float PWM_Value = 0;
	if (value > INT16_MAX)
	{
		value = INT16_MAX;
	}
	else if (value < -INT16_MAX)
	{
		value = -INT16_MAX;
	}

	ratio = value / INT16_MAX;
	PWM_Value = ratio * SERVO_HalfRANGE;
	PWM_Value += getservoinitpos_configParam(id);
	return PWM_Value;
}


void motorControl(control_t *control) /*功率输出控制*/
{
#ifdef FOUR_WING
//	s16 r = control->roll / 2.0f;
//	s16 p = control->pitch / 2.0f;
	s16 r = control->roll;
	s16 p = control->pitch;
	//控制分配	改！
	actuator.motor_l.PWM = limitThrust(control->thrust  + r / 2);
	actuator.motor_r.PWM = limitThrust(control->thrust  - r / 2);
	actuator.servo_l.PWM = Servo_Int16ToPWM(PWM_LEFT, p - control->yaw * 1.5f );
	actuator.servo_r.PWM = Servo_Int16ToPWM(PWM_MIDDLE, -p - control->yaw * 1.5f );

	if (motorSetEnable)
	{
		actuator.motor_l.PWM = motorPWMSet.motorPWM_l;
		actuator.motor_r.PWM = motorPWMSet.motorPWM_r;
		actuator.servo_l.PWM = motorPWMSet.servoPWM_l;
		actuator.servo_r.PWM = motorPWMSet.servoPWM_r;
		#ifdef DOUBLE_WING 
		actuator.servo_m.PWM = motorPWMSet.servoPWM_m;
		#endif
	}
	motorsSetRatio(PWMF1, actuator.motor_l.PWM); /*控制电机输出百分比*/
	motorsSetRatio(PWMF2, actuator.motor_r.PWM);
	servoSetPWM(PWM_LEFT,  actuator.servo_l.PWM); /*舵机输出占空比设置*/
	servoSetPWM(PWM_MIDDLE,actuator.servo_r.PWM);
	//servo Tf apply
	control->actual_servoPWM = TfApply(&servotf,0.5f*(actuator.servo_l.PWM + actuator.servo_r.PWM));
	control->actual_servoangle = ServoPWM2Servoangle(control->actual_servoPWM);
	//motor Tf apply
	control->actual_motorPWM = TfApply(&motortf,0.5f*(actuator.motor_l.PWM + actuator.motor_r.PWM));

#elif defined DOUBLE_WING
	s16 r = control->roll;
	s16 p = control->pitch;
	//控制分配
	motorPWM.f1 = limitThrust(control->thrust );

	motorPWM.s_left = Servo_Int16ToPWM(PWM_LEFT, -p - r);
	motorPWM.s_middle = Servo_Int16ToPWM(PWM_MIDDLE, control->yaw );
	motorPWM.s_right = Servo_Int16ToPWM(PWM_RIGHT, p - r);

	if (motorSetEnable)
	{
		motorPWM = motorPWMSet;
	}
	motorsSetRatio(PWMF1, sqrt(motorPWM.f1) * 256); /*控制电机输出百分比*/

	servoSetPWM(PWM_LEFT, motorPWM.s_left); /*舵机输出占空比设置*/
	servoSetPWM(PWM_MIDDLE, motorPWM.s_middle);
	servoSetPWM(PWM_RIGHT, motorPWM.s_right);
#endif
}

void getMotorPWM(actuatorStatus_t *get)
{
	(*get).motor_l.PWM = motorPWMSet.motorPWM_l;
	(*get).motor_r.PWM = motorPWMSet.motorPWM_r;
	(*get).servo_l.PWM = motorPWMSet.servoPWM_l;
	(*get).servo_r.PWM = motorPWMSet.servoPWM_r;
#ifdef DOUBLE_WING
    *get.servo_m.PWM = motorPWMSet.servoPWM_m;
#endif
}

void setMotorPWM(bool enable,motorPWM_t set)
{
	motorSetEnable = enable;
	motorPWMSet.motorPWM_l = set.motorPWM_l;
	motorPWMSet.motorPWM_r = set.motorPWM_r;
	motorPWMSet.servoPWM_l = set.servoPWM_l;
	motorPWMSet.servoPWM_r = set.servoPWM_r;
#ifdef DOUBLE_WING
    motorPWMSet.servoPWM_m = set.servoPWM_m;
#endif
}

float ServoPWM2angle(u32 PWM,u8 id)
{
	return PWM2ANGLE( PWM - getservoinitpos_configParam(id) );
}
