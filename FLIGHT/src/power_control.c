#include "power_control.h"
#include "motors.h"
#include "config_param.h"
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
static motorPWM_t motorPWM;
static motorPWM_t motorPWMSet = {0, 0, 0, 0, 0, 0};

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
	motorPWM.f2 = limitThrust(control->thrust + r);
	motorPWM.f1 = limitThrust(control->thrust - r);
	motorPWM.s_left = Servo_Int16ToPWM(PWM_LEFT, -p + control->yaw * 1.5f );
	motorPWM.s_middle = Servo_Int16ToPWM(PWM_MIDDLE, p + control->yaw * 1.5f );


	if (motorSetEnable)
	{
		motorPWM = motorPWMSet;
	}
	motorsSetRatio(PWMF1, sqrt(motorPWM.f1) * 256); /*控制电机输出百分比*/
	motorsSetRatio(PWMF2, sqrt(motorPWM.f2) * 256);
	servoSetPWM(PWM_LEFT, motorPWM.s_left); /*舵机输出占空比设置*/
	servoSetPWM(PWM_MIDDLE, motorPWM.s_middle);

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

void getMotorPWM(motorPWM_t *get)
{
	*get = motorPWM;
}

void setMotorPWM(bool enable, u16 f1_set, u16 f2_set, u16 s1_set, u16 s2_set, u16 s3_set, u16 r1_set)
{
	motorSetEnable = enable;
	motorPWMSet.f1 = f1_set;
	motorPWMSet.f2 = f2_set;
	motorPWMSet.s_left = s1_set;
	motorPWMSet.s_right = s2_set;
	motorPWMSet.s_middle = s3_set;
	motorPWMSet.r1 = r1_set;
}
