#include <stdbool.h>
#include "pid.h"
#include "sensors.h"
#include "attitude_pid.h"
#include "attitude_adrc.h"
#include "adrc.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * ��̬PID���ƴ���	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
 *
 * �޸�˵��:
 * �汾V1.3 �����ǶȻ��ͽ��ٶȻ�����ʱ����������bug��
********************************************************************************/

/*�ǶȻ������޷�*/
#define PID_ANGLE_ROLL_INTEGRATION_LIMIT 30.0
#define PID_ANGLE_PITCH_INTEGRATION_LIMIT 30.0
#define PID_ANGLE_YAW_INTEGRATION_LIMIT 180.0

/*���ٶȻ������޷�*/
#define PID_RATE_ROLL_INTEGRATION_LIMIT 500.0
#define PID_RATE_PITCH_INTEGRATION_LIMIT 500.0
#define PID_RATE_YAW_INTEGRATION_LIMIT 50.0

/*Dterm filter cutoff frequency*/
#define ROLL_ANGLE_PID_DTERM_CUTOFF_FREQ 100.0
#define PITCH_ANGLE_PID_DTERM_CUTOFF_FREQ 0.0
#define YAW_ANGLE_PID_DTERM_CUTOFF_FREQ 100.0

#define ROLL_RATE_PID_DTERM_CUTOFF_FREQ 90.0
#define PITCH_RATE_PID_DTERM_CUTOFF_FREQ 0.0
#define YAW_RATE_PID_DTERM_CUTOFF_FREQ 90.0

PidObject pidAngleRoll;
PidObject pidAnglePitch;
PidObject pidAngleYaw;
PidObject pidRateRoll;
PidObject pidRatePitch;
PidObject pidRateYaw;

extern adrcObject_t ADRCAnglePitch;
extern adrcObject_t ADRCAngleRoll;
extern adrcObject_t ADRCRatePitch;
extern adrcObject_t ADRCRateRoll;


static inline int16_t pidOutLimit(float in)
{
	if (in > INT16_MAX)
		return INT16_MAX;
	else if (in < -INT16_MAX)
		return -INT16_MAX;
	else
		return (int16_t)in;
}

void attitudeControlInit(float ratePidDt, float anglePidDt,float maindt)
{
	pidInit(&pidAngleRoll, 0, configParam.pidAngle.roll, anglePidDt,ROLL_ANGLE_PID_DTERM_CUTOFF_FREQ);		/*roll  �Ƕ�PID��ʼ��*/
	pidInit(&pidAnglePitch, 0, configParam.pidAngle.pitch, anglePidDt,PITCH_ANGLE_PID_DTERM_CUTOFF_FREQ);		/*pitch �Ƕ�PID��ʼ��*/
	pidInit(&pidAngleYaw, 0, configParam.pidAngle.yaw, anglePidDt,YAW_ANGLE_PID_DTERM_CUTOFF_FREQ);			/*yaw   �Ƕ�PID��ʼ��*/
	pidSetIntegralLimit(&pidAngleRoll, PID_ANGLE_ROLL_INTEGRATION_LIMIT);	/*roll  �ǶȻ����޷�����*/
	pidSetIntegralLimit(&pidAnglePitch, PID_ANGLE_PITCH_INTEGRATION_LIMIT); /*pitch �ǶȻ����޷�����*/
	pidSetIntegralLimit(&pidAngleYaw, PID_ANGLE_YAW_INTEGRATION_LIMIT);		/*yaw   �ǶȻ����޷�����*/

	pidInit(&pidRateRoll, 0, configParam.pidRate.roll, ratePidDt,ROLL_RATE_PID_DTERM_CUTOFF_FREQ);		  /*roll  ���ٶ�PID��ʼ��*/
	pidInit(&pidRatePitch, 0, configParam.pidRate.pitch, ratePidDt,PITCH_RATE_PID_DTERM_CUTOFF_FREQ);	  /*pitch ���ٶ�PID��ʼ��*/
	pidInit(&pidRateYaw, 0, configParam.pidRate.yaw, ratePidDt,YAW_RATE_PID_DTERM_CUTOFF_FREQ);		  /*yaw   ���ٶ�PID��ʼ��*/
	pidSetIntegralLimit(&pidRateRoll, PID_RATE_ROLL_INTEGRATION_LIMIT);	  /*roll  ���ٶȻ����޷�����*/
	pidSetIntegralLimit(&pidRatePitch, PID_RATE_PITCH_INTEGRATION_LIMIT); /*pitch ���ٶȻ����޷�����*/
	pidSetIntegralLimit(&pidRateYaw, PID_RATE_YAW_INTEGRATION_LIMIT);	  /*yaw   ���ٶȻ����޷�����*/
	#ifdef ADRC_CONTROL
	// adrc_init(&ADRCRatePitch,&configParam.adrcRate.pitch, maindt,maindt,ratePidDt);
	adrc_init(&ADRCRateRoll, &configParam.adrcRate.roll,  maindt,maindt,ratePidDt);
	#endif
}

bool attitudeControlTest()
{
	return true;
}

void attitudeRatePID(Axis3f *actualRate, attitude_t *desiredRate, control_t *output) /* ���ٶȻ�PID */
{
	#ifdef PID_CONTROL
	output->roll = pidOutLimit(pidUpdate(&pidRateRoll, desiredRate->roll - actualRate->x));
	output->pitch = pidOutLimit(pidUpdate(&pidRatePitch, desiredRate->pitch - actualRate->y));
	output->yaw = pidOutLimit(pidUpdate(&pidRateYaw, desiredRate->yaw - actualRate->z));
	#endif

	#ifdef ADRC_CONTROL
	output->pitch = pidOutLimit(pidUpdate(&pidRatePitch, desiredRate->pitch - actualRate->y));
	ADRC_RateControl(&ADRCRateRoll,desiredRate->roll,actualRate->x);
	output->roll = pidOutLimit(ADRCRateRoll.u);
	output->yaw = pidOutLimit(pidUpdate(&pidRateYaw, desiredRate->yaw - actualRate->z));
    #endif
}

void attitudeAnglePID(attitude_t *actualAngle, attitude_t *desiredAngle, attitude_t *outDesiredRate) /* �ǶȻ�PID */
{
	outDesiredRate->roll = pidUpdate(&pidAngleRoll, desiredAngle->roll - actualAngle->roll);
	outDesiredRate->pitch = pidUpdate(&pidAnglePitch, desiredAngle->pitch - actualAngle->pitch);

	float yawError = desiredAngle->yaw - actualAngle->yaw;
	if (yawError > 180.0f)
		yawError -= 360.0f;
	else if (yawError < -180.0)
		yawError += 360.0f;
	outDesiredRate->yaw = pidUpdate(&pidAngleYaw, yawError);
}

void attitudeControllerResetRollAttitudePID(void)
{
	pidReset(&pidAngleRoll);
}

void attitudeControllerResetPitchAttitudePID(void)
{
	pidReset(&pidAnglePitch);
}

void attitudeResetAllPID(void) /*��λPID*/
{
	pidReset(&pidAngleRoll);
	pidReset(&pidAnglePitch);
	pidReset(&pidAngleYaw);
	pidReset(&pidRateRoll);
	pidReset(&pidRatePitch);
	pidReset(&pidRateYaw);
}

void attitudePIDwriteToConfigParam(void)
{
	configParam.pidAngle.roll.kp = pidAngleRoll.kp;
	configParam.pidAngle.roll.ki = pidAngleRoll.ki;
	configParam.pidAngle.roll.kd = pidAngleRoll.kd;
	configParam.pidAngle.roll.outputLimit = pidAngleRoll.outputLimit;

	configParam.pidAngle.pitch.kp = pidAnglePitch.kp;
	configParam.pidAngle.pitch.ki = pidAnglePitch.ki;
	configParam.pidAngle.pitch.kd = pidAnglePitch.kd;
	configParam.pidAngle.pitch.outputLimit = pidAnglePitch.outputLimit;

	configParam.pidAngle.yaw.kp = pidAngleYaw.kp;
	configParam.pidAngle.yaw.ki = pidAngleYaw.ki;
	configParam.pidAngle.yaw.kd = pidAngleYaw.kd;
	configParam.pidAngle.yaw.outputLimit = pidAngleYaw.outputLimit;

	configParam.pidRate.roll.kp = pidRateRoll.kp;
	configParam.pidRate.roll.ki = pidRateRoll.ki;
	configParam.pidRate.roll.kd = pidRateRoll.kd;
	configParam.pidRate.roll.outputLimit = pidRateRoll.outputLimit;

	configParam.pidRate.pitch.kp = pidRatePitch.kp;
	configParam.pidRate.pitch.ki = pidRatePitch.ki;
	configParam.pidRate.pitch.kd = pidRatePitch.kd;
	configParam.pidRate.pitch.outputLimit = pidRatePitch.outputLimit;

	configParam.pidRate.yaw.kp = pidRateYaw.kp;
	configParam.pidRate.yaw.ki = pidRateYaw.ki;
	configParam.pidRate.yaw.kd = pidRateYaw.kd;
	configParam.pidRate.yaw.outputLimit = pidRateYaw.outputLimit;
}
void attitudeResetAllPID_TEST(void) /*ֻ��λ���ֵ���*/
{
	pidReset_test(&pidAngleRoll);
	pidReset_test(&pidAnglePitch);
	pidReset_test(&pidAngleYaw);
	pidReset_test(&pidRateRoll);
	pidReset_test(&pidRatePitch);
	pidReset_test(&pidRateYaw);
}
