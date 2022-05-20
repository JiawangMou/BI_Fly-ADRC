#include "pid.h"


// static lpf2pData pidRatePitchDTermFilter;
// static lpf2pData pidRateRollDTermFilter;

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * PID��������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
 *
 * �޸�˵��:
 * �汾V1.3 ����PID�ṹ������һ��(out)��
********************************************************************************/

void pidInit(PidObject* pid, const float desired, const pidInit_t pidParam, const float dt,const float cuttoff_freq)
{
	pid->error     = 0;
	pid->prevError = 0;
	pid->integ     = 0;
	pid->deriv     = 0;
	pid->desired = desired;
	pid->kp = pidParam.kp;
	pid->ki = pidParam.ki;
	pid->kd = pidParam.kd;
	pid->iLimit = DEFAULT_PID_INTEGRATION_LIMIT;
	pid->outputLimit = pidParam.outputLimit;
	pid->dt = dt;
	if(cuttoff_freq == 0)
		pid->dtermFilter = 0;
	else
		lpf2pInit(pid->dtermFilter, 1.0f/dt, cuttoff_freq);

}

float pidUpdate(PidObject* pid, const float error)
{
	float output;
	float integ_dt = 0;

	pid->error = error;   
	integ_dt = pid->error * pid->dt;
	pid->integ += integ_dt;
	
	//�����޷�
	if (pid->integ > pid->iLimit)
	{
		pid->integ = pid->iLimit;
	}
	else if (pid->integ < -pid->iLimit)
	{
		pid->integ = -pid->iLimit;
	}

	pid->outP = pid->kp * pid->error;
	pid->outI = pid->ki * pid->integ;
	if(pid->kd > 0){
		pid->deriv = (pid->error - pid->prevError) / pid->dt;
		pid->outD = pid->kd * pid->deriv;

		if(pid->dtermFilter) pid->outD = lpf2pApply(pid->dtermFilter, pid->outD);
	}else{
		pid->outD = 0;
	}

	output = pid->outP + pid->outI + pid->outD;
	
	//����޷�
	if (pid->outputLimit != 0)
	{
		if (output > pid->outputLimit)
		{
			output = pid->outputLimit;
			if(integ_dt > 0)
				pid->integ -= integ_dt;
		}

		else if (output < -pid->outputLimit)
		{
			output = -pid->outputLimit;
			if(integ_dt < 0)
				pid->integ -= integ_dt;		
		}
	}
	
	pid->prevError = pid->error;

	pid->out = output;
	return output;
}

void pidSetIntegralLimit(PidObject* pid, const float limit) 
{
    pid->iLimit = limit;
}

void pidSetOutputLimit(PidObject* pid, const float limit) 
{
	pid->outputLimit = limit;
}

void pidSetError(PidObject* pid, const float error)
{
	pid->error = error;
}

void pidSetDesired(PidObject* pid, const float desired)
{
	pid->desired = desired;
}

float pidGetDesired(PidObject* pid)
{
	return pid->desired;
}

void pidSetKp(PidObject* pid, const float kp)
{
	pid->kp = kp;
}

void pidSetKi(PidObject* pid, const float ki)
{
	pid->ki = ki;
}

void pidSetKd(PidObject* pid, const float kd)
{
	pid->kd = kd;
}

void pidSetDt(PidObject* pid, const float dt) 
{
    pid->dt = dt;
}

void pidReset(PidObject* pid)
{
    pid->error     = 0;
    pid->prevError = 0;
    pid->integ     = 0;
    pid->deriv     = 0;
    pid->out       = 0;
    pid->outP      = 0;
    pid->outI      = 0;
    pid->outD      = 0;
}
void pidReset_test(PidObject* pid)
{
	pid->integ     = 0;
}
