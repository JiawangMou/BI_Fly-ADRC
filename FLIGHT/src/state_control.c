#include "state_control.h"
#include "attitude_pid.h"
#include "config.h"
#include "config_param.h"
#include "maths.h"
#include "position_pid.h"
#include "stabilizer.h"
#include <math.h>
#include "ADRC.h"
/********************************************************************************
 * ������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * ������̬���ƴ���
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) �������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
 ********************************************************************************/

static float      actualThrust;
static attitude_t attitudeDesired;
static attitude_t rateDesired;

extern adrcObject_t ADRCAnglePitch;
extern adrcObject_t ADRCAngleRoll;
extern adrcObject_t ADRCRatePitch;
extern adrcObject_t ADRCRateRoll;

void stateControlInit(void)
{
	attitudeControlInit(RATE_PID_DT, ANGEL_PID_DT, MAIN_LOOP_DTS); /*��ʼ����̬PID*/	
	positionControlInit(VELOCITY_PID_DT, POSITION_PID_DT); /*��ʼ��λ��PID*/
}

bool stateControlTest(void)
{
    bool pass = true;
    pass &= attitudeControlTest();
    return pass;
}

void stateControl(control_t* control, sensorData_t* sensors, state_t* state, setpoint_t* setpoint, const u32 tick)
{
    static u16 cnt = 0;

    if (RATE_DO_EXECUTE(POSITION_PID_RATE, tick)) {
        if (setpoint->mode.x != modeDisable || setpoint->mode.y != modeDisable || setpoint->mode.z != modeDisable) {
            positionController(&actualThrust, &attitudeDesired, setpoint, state, POSITION_PID_DT);
        }
    }

    //�ǶȻ����⻷��
    if (RATE_DO_EXECUTE(ANGEL_PID_RATE, tick)) {
        if (setpoint->mode.z == modeDisable) {
            actualThrust = setpoint->thrust;
        }
        if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {
            attitudeDesired.roll  = setpoint->attitude.roll;
            attitudeDesired.pitch = setpoint->attitude.pitch;
        }

        if (control->flipDir == CENTER) {
            attitudeDesired.yaw += setpoint->attitude.yaw / ANGEL_PID_RATE; /*����YAW ����ģʽ*/
            if (attitudeDesired.yaw > 180.0f)
                attitudeDesired.yaw -= 360.0f;
            if (attitudeDesired.yaw < -180.0f)
                attitudeDesired.yaw += 360.0f;
        }

        attitudeDesired.roll += configParam.trimR; //����΢��ֵ
        attitudeDesired.pitch += configParam.trimP;

        attitudeAnglePID(&state->attitude, &attitudeDesired, &rateDesired);
    }
    /*ADRC-ESO*/
    // adrc_leso(&ADRCRatePitch.leso, sensors->gyro.y,ADRCRatePitch.u);
    adrc_leso(&ADRCRateRoll.leso, sensors->gyro.x,ADRCRateRoll.u);	
	/*ADRC-TD*/
    // adrc_td(&ADRCRatePitch.td, rateDesired.pitch - ADRCRatePitch.leso.z1);
    adrc_td(&ADRCRateRoll.td,  rateDesired.roll - ADRCRateRoll.leso.z1);


    //���ٶȻ����ڻ���
    if (RATE_DO_EXECUTE(RATE_PID_RATE, tick)) {
        if (setpoint->mode.roll == modeVelocity) {
            rateDesired.roll = setpoint->attitudeRate.roll;
            attitudeControllerResetRollAttitudePID();
        }
        if (setpoint->mode.pitch == modeVelocity) {
            rateDesired.pitch = setpoint->attitudeRate.pitch;
            attitudeControllerResetPitchAttitudePID();
        }
        extern u8 fstate;
        if (control->flipDir != CENTER && fstate == 4) /*�շ�����ֻʹ���ڻ�PID*/
        {
            rateDesired.pitch = setpoint->attitude.pitch;
            rateDesired.roll  = setpoint->attitude.roll;
            
        }
        attitudeRatePID(&sensors->gyro, &rateDesired, control);

// #ifdef BI_Fly_2
        // control->yaw = setpoint->attitude.yaw * 100;
// #endif
    }
    control->thrust = constrainf(actualThrust, 0.0f, 55000.0f);
    // control->thrust = actualThrust;

    if (control->thrust < 5.f) {
        control->roll = 0;
        // control->pitch = 0;
        // control->yaw = 0;

        attitudeResetAllPID_TEST();
        // attitudeResetAllPID();	/*��λ��̬PID*/
        // /*����ȡ����λ��ԭ���ǣ��÷���������Ķ���ʱ�򣬻��ܿ�������ķ�Ӧ���Ӷ�ȷ��PID�������Ƿ������������ǽ����Ƿ�������*/
        positionResetAllPID();                     /*��λλ��PID*/
        // adrc_reset(&ADRCRatePitch);
		// adrc_reset(&ADRCRateRoll);
        attitudeDesired.yaw = state->attitude.yaw; /*��λ���������yawֵ*/

        if (cnt++ > 1500) {
            cnt = 0;
            configParamGiveSemaphore();
        }
    } else {
        cnt = 0;
    }
}

void getrateDesired(attitude_t *get)
{
	get->pitch = rateDesired.pitch;
	get->roll = rateDesired.roll;	
	get->yaw = rateDesired.yaw;
}
void getattitudeDesired(attitude_t *get)
{
	*get = attitudeDesired;
}