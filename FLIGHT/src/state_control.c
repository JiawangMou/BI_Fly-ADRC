#include "state_control.h"
#include "attitude_pid.h"
#include "config.h"
#include "config_param.h"
#include "maths.h"
#include "position_pid.h"
#include "stabilizer.h"
#include <math.h>
#include "ADRC.h"
#include "attitude_adrc.h"
#include "stm32f4xx_gpio.h"

/********************************************************************************
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 四轴姿态控制代码
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
 ********************************************************************************/

static float      actualThrust;
static attitude_t attitudeDesired;
static attitude_t rateDesired;

extern adrcObject_t ADRCAnglePitch;
extern adrcObject_t ADRCAngleRoll;
extern adrcObject_t ADRCRatePitch;
extern adrcObject_t ADRCRateRoll;

static uint8_t thrust_count = 0;
static uint16_t thrust_init = 0;
static uint16_t thrust_init_cmd = 10000;

static uint8_t phase = 0;

enum PHASE{start = 0, waiting = 1, accelerating = 2, scanning = 3, holdon = 4};


// // remoter setpoint(roll,pitch) filter
// static lpf2pData setpointFilter[2];

void stateControlInit(void)
{
	attitudeControlInit(RATE_PID_DT, ANGEL_PID_DT, MAIN_LOOP_DTS); /*初始化姿态PID*/	
	positionControlInit(VELOCITY_PID_DT, POSITION_PID_DT); /*初始化位置PID*/

    //     // Filter the setpoint
    // lpf2pInit(&setpointFilter[0], ANGEL_PID_RATE, 20);
    // lpf2pInit(&setpointFilter[1], ANGEL_PID_RATE, 20);
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
#ifndef TEST
    if (RATE_DO_EXECUTE(POSITION_PID_RATE, tick)) {
        if (setpoint->mode.x != modeDisable || setpoint->mode.y != modeDisable || setpoint->mode.z != modeDisable) {
            positionController(&actualThrust, &attitudeDesired, setpoint, state, POSITION_PID_DT);
        }
    }

    //角度环（外环）
    if (RATE_DO_EXECUTE(ANGEL_PID_RATE, tick)) {
        if (setpoint->mode.z == modeDisable) {
            actualThrust = setpoint->thrust;
        }
        if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {
            attitudeDesired.roll  = setpoint->attitude.roll;
            attitudeDesired.pitch = -setpoint->attitude.pitch;
        }

        if (control->flipDir == CENTER) {
            attitudeDesired.yaw -= setpoint->attitude.yaw / ANGEL_PID_RATE; /*期望YAW 速率模式*/
            if (attitudeDesired.yaw > 180.0f)
                attitudeDesired.yaw -= 360.0f;
            if (attitudeDesired.yaw < -180.0f)
                attitudeDesired.yaw += 360.0f;
        }

        attitudeDesired.roll += configParam.trimR; //叠加微调值
        attitudeDesired.pitch += configParam.trimP;

        // attitudeDesired.roll  = lpf2pApply(&setpointFilter[0], attitudeDesired.roll);
        // attitudeDesired.pitch = lpf2pApply(&setpointFilter[1], attitudeDesired.pitch);

        attitudeAnglePID(&state->attitude, &attitudeDesired, &rateDesired);
    }
    /*ADRC-ESO*/
    // adrc_leso(&ADRCRatePitch.leso, sensors->gyro.y,ADRCRatePitch.u);
    adrc_leso(&ADRCRateRoll.leso, sensors->gyro.x,ADRCRateRoll.u);	
	/*ADRC-TD*/
    // adrc_td(&ADRCRatePitch.td, rateDesired.pitch - ADRCRatePitch.leso.z1);
    adrc_td(&ADRCRateRoll.td,  rateDesired.roll - ADRCRateRoll.leso.z1);


    //角速度环（内环）
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
        if (control->flipDir != CENTER && fstate == 4) /*空翻过程只使用内环PID*/
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
        // attitudeResetAllPID();	/*复位姿态PID*/
        // /*这里取消复位的原因是，让飞行器翅膀不拍动的时候，还能看到舵机的反应，从而确认PID计算结果是否正常，或者是接线是否有问题*/
        positionResetAllPID();                     /*复位位置PID*/
        // adrc_reset(&ADRCRatePitch);
		adrc_reset(&ADRCRateRoll);
        attitudeDesired.yaw = state->attitude.yaw; /*复位计算的期望yaw值*/

        if (cnt++ > 1500) {
            cnt = 0;
            configParamGiveSemaphore();
        }
    } else {
        cnt = 0;
    }
#endif

#ifdef TEST
        static u32 tick_init = 0;
        static int16_t control_roll = 0;
        static u16 control_thrust = 0;
        actualThrust = setpoint->thrust;


        if (actualThrust > 5.f){
            switch(phase)
            {
                case start:
                {
                    GPIO_SetBits(GPIOB, GPIO_Pin_14);
                    phase++;
                    tick_init = tick;
                };break;
                case waiting:
                {
                    if(((tick-tick_init) % 1000) == 0 ) {
                        thrust_init = thrust_init_cmd + thrust_count * 2000;
                        control_thrust = thrust_init / 10;
                        control_roll = -30000/ 10;
                        phase++;
                        tick_init = tick;
                    }
                };break;
                case accelerating:
                {
                    if(((tick-tick_init) % 10000) == 0 ){
                        control_thrust = thrust_init;
                        control_roll = -30000;
                        phase++;
                        tick_init = tick;
                    }else{
                        if(((tick-tick_init) % 1000) == 0 )
                        {
                            control_roll += - 30000/ 10;  
                            control_thrust += thrust_init / 10;
                        }
                    }
                };break;
                case scanning:
                {
                    if(((tick-tick_init) % 155000) == 0 )  //31* 5s
                    {
                        control_thrust = 0;
                        control_roll = 0;
                        GPIO_ResetBits(GPIOB, GPIO_Pin_14);
                        phase++;
                        tick_init = tick;
                    }else{
                        if(((tick-tick_init) % 5000) == 0 )
                        {
                            control_roll += 2000;  
                        }
                    }
                };break;
                case holdon:
                {
                    if(((tick-tick_init) % 10000) == 0 )//10s
                    {
                        phase = 0; 
                        thrust_count++;
                        if(thrust_count >25)
                        {
                            thrust_count=0;
                            control_thrust = 0;
                            control_roll = 0;
                            phase = 5;
                        }
                    } 
                };break;
                default:
                {
                    phase = 5;
                    control_thrust = 0;
                    control_roll = 0;
                };
            }
            cnt = 0;
        }
        else{
            if (cnt++ > 1500) {
                cnt = 0;
                configParamGiveSemaphore();
            }
            GPIO_ResetBits(GPIOB, GPIO_Pin_14);
            control_thrust = 0;
            control_roll = 0;
            tick_init = tick;
        }

        control->thrust = constrainf(control_thrust, 0.0f, 60000.0f);
        control->pitch = 0;
        control->yaw = 0;
        control->roll = control_roll;

        // if( (control->thrust > 5.f) && (end_flag == 0 ))
        // {
        //     if(thrustchange_flag == 0)      //之前油门的状态为0
        //     {
        //         thrustchange_flag = 1;
        //         count = 0;
        //         GPIO_SetBits(GPIOA, GPIO_Pin_9);
        //     }
		// 	if (cnt++ > 1500) 
		// 	{
		// 		cnt = 0;
		// 		configParamGiveSemaphore();
		// 	}
        // }else 
		// {
		// 	cnt = 0;
		// }
				
		// if((count >= 1000) && (thrustchange_flag == 1) && (Start_flag == 0 )) //等待NI设备bias时间到
        // {
        //     Start_flag = 1;   //转换start_flag的状态
        //     count = 0;
        //     Roll_count = 0;
		// 	thrust_count++;
        //     thrust_init = thrust_init_cmd + thrust_count * 1000;
		// 	if(thrust_init > 60000)
		// 	{
		// 		end_flag = 1;
		// 		thrustchange_flag = 0;
		// 		Start_flag = 0;
		// 	}
        // }

        // if((Start_flag == 1) && (thrustchange_flag == 1))  
        // {
        //     roll_init = -30000 + Roll_count * 2000;
        //     if(Roll_count > 30)
        //     {
		// 		GPIO_ResetBits(GPIOA, GPIO_Pin_9);
		// 		count = 0;
		// 		Start_flag = 2;
        //     }
        // }
        // else
        // {
        //     roll_init = 0;
		// 	thrust_init = 0;
        //     Roll_count = 0;
        // }
				
		// if((Start_flag == 2) && (thrustchange_flag == 1) && (count >= 4000))
		// {
		// 	Start_flag = 0;
		// 	thrustchange_flag = 0;
		// }
        // control->thrust = thrust_init;
        // control->roll = roll_init;
        // count++; 
        // if(count == 5000)
        // {
        //     Roll_count++;
        //     count = 0;
        // }

#endif



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

#ifdef TEST
void setThrust_cmd(uint16_t Thrust_cmd)
{
    thrust_init_cmd = Thrust_cmd;
}

#endif
