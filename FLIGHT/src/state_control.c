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
#include "position_adrc.h"
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

static attitude_t attitudeDesired;
static float      actualThrust;
static attitude_t rateDesired;

extern adrcObject_t ADRCAnglePitch;
extern adrcObject_t ADRCAngleRoll;
extern adrcObject_t ADRCRatePitch;
extern adrcObject_t ADRCRateRoll;


#ifdef TEST

#define THRUST_NUM 10
#define THRUST_STARTPOINT 10000
#define THRUST_ENDPOINT 50000
#define THRUST_DELTA ((THRUST_ENDPOINT - THRUST_STARTPOINT) / THRUST_NUM )

#define THRUST_WAIT_ACCEL_TIME 10000 // scan 模型下的加速时间 单位：ms
#define THRUST_SCAN_TIME 5000 // scan 模型下的扫描时间 单位：ms
#define THRUST_SCAN_ACCEL_TIME 2000 // scan 模型下的加速时间 单位：ms
#define THRUST_ACCEL_DIV_MS 10 // scan模式下，加速1s的细分，10表示10ms改变一次速度
#define THRUST_WAIT_ACCEL_COUNT (THRUST_WAIT_ACCEL_TIME / THRUST_ACCEL_DIV_MS)
#define THRUST_SCAN_ACCEL_COUNT (THRUST_SCAN_ACCEL_TIME / THRUST_ACCEL_DIV_MS)



static uint8_t thrust_count = 0;
static uint16_t thrust_init_cmd = 10000;

static uint8_t phase = 0;

enum PHASE{start = 0, waiting = 1, accelerating = 2, scanning = 3, holdon = 4};
#endif




// // remoter setpoint(roll,pitch) filter
// static lpf2pData setpointFilter[2];

void stateControlInit(void)
{
	attitudeControlInit(RATE_PID_DT, ANGEL_PID_DT, MAIN_LOOP_DTS); /*初始化姿态PID*/	
	positionControlInit(VEL_PID_DT, POS_PID_DT); /*初始化位置PID*/

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

    if (RATE_DO_EXECUTE(POS_PID_RATE, tick)) {
        if (setpoint->mode.x != modeDisable || setpoint->mode.y != modeDisable || setpoint->mode.z != modeDisable) {
            #ifdef USE_MBD
            positionController(setpoint, state);
            #else 
            positionController(&actualThrust, &attitudeDesired, setpoint, state, POSITION_PID_DT);
            #endif
        }
    }

    if (RATE_DO_EXECUTE(VEL_PID_RATE, tick)) {
        if (setpoint->mode.x != modeDisable || setpoint->mode.y != modeDisable || setpoint->mode.z != modeDisable) {
            velocityController(&actualThrust,control, &attitudeDesired, setpoint, state, sensors);
        }
    }

    if (RATE_DO_EXECUTE(VEL_ESO_RATE, tick)) {
            velZ_ESO_estimate(control->thrust,state->velocity.z);
    }

    //角度环（外环）
    if (RATE_DO_EXECUTE(ANGEL_PID_RATE, tick)) {
        if (setpoint->mode.z == modeDisable) {
            actualThrust = setpoint->thrust;
        }
        if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {
            attitudeDesired.roll  = setpoint->attitude.roll;
            attitudeDesired.pitch = setpoint->attitude.pitch;
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
#ifdef ADRC_CONTROL
    /*ADRC-ESO*/
    // adrc_leso(&ADRCRatePitch.leso, sensors->gyro.y,ADRCRatePitch.u);
    adrc_leso(&ADRCRateRoll.leso, sensors->gyro.x,ADRCRateRoll.u);	
	/*ADRC-TD*/
    // adrc_td(&ADRCRatePitch.td, rateDesired.pitch - ADRCRatePitch.leso.z1);
    adrc_td(&ADRCRateRoll.td,  rateDesired.roll - ADRCRateRoll.leso.z1);
#endif


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
    }
    control->thrust = constrainf(actualThrust, 0.0f, (float)FULLTHROTTLE);
    // control->thrust = actualThrust;

    if (control->thrust < 5.f) {
#ifdef FOUR_WING
        control->roll = 0;
#endif
        // control->pitch = 0;
        // control->yaw = 0;

        attitudeResetAllPID_TEST();
        // attitudeResetAllPID();	/*复位姿态PID*/
        // /*这里取消复位的原因是，让飞行器翅膀不拍动的时候，还能看到舵机的反应，从而确认PID计算结果是否正常，或者是接线是否有问题*/
        positionResetAllPID();                     /*复位位置PID*/
        // adrc_reset(&ADRCRatePitch);
#ifdef ADRC_CONTROL
		adrc_reset(&ADRCRateRoll);
#endif
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
        static u16 control_thrust_target = 0;
        static u16 control_thrust_current = 0;
        static u16  control_thrust_delta = 0;
        static bool start_flag = 0;
        static bool start_scan_flag = 0;
        static u8 scan_cnt = 0;
        static bool key_flag = 0;


        actualThrust = setpoint->thrust;
        if (actualThrust > 60000.f){
            key_flag = 1; 
        }

        if((key_flag == 1) && (actualThrust < 60000.f))
        {
            start_flag = !start_flag;
            key_flag = 0; 
        }
            
        
        if (start_flag){
            switch(phase)
            {
                case start:
                {
                    GPIO_SetBits(GPIOC, GPIO_Pin_4);
                    phase = waiting;
                    tick_init = tick;
                };break;
                case waiting:
                {
                    if (((tick - tick_init) % 1000) == 0) {
                        control_thrust_target = thrust_init_cmd + thrust_count * THRUST_DELTA;
                        control_thrust_delta = control_thrust_target / THRUST_WAIT_ACCEL_COUNT; // 每次刷新增量
                        phase                = accelerating;
                        tick_init            = tick;
                    }
                };break;
                case accelerating:
                {
                    if(start_scan_flag ==1){//scan模型下的加速度过程
                        if(((tick-tick_init) % THRUST_SCAN_ACCEL_TIME ) == 0){
                            control_thrust_current = control_thrust_target;
                            phase = scanning;
                            tick_init = tick;
                        }else{
                            if(((tick-tick_init) % THRUST_ACCEL_DIV_MS) == 0 )
                            {
                                control_thrust_current += control_thrust_delta; 
                            }
                        }
                    }else{//waiting 模式下的加速过程
                        if(((tick-tick_init) % THRUST_WAIT_ACCEL_TIME) == 0){
                            control_thrust_current = control_thrust_target;
                            phase = scanning;
                            tick_init = tick;
                            start_scan_flag = 1;       //开始扫描
                            scan_cnt = 0;
                        }else{
                            if(((tick-tick_init) % 10) == 0){
                                control_thrust_current += control_thrust_delta;
                            }
                        }
                    }
                };break;
                case scanning:
                {
                    if(((tick-tick_init) % THRUST_SCAN_TIME) == 0 )
                    {
                        scan_cnt++;
                        if(scan_cnt < (THRUST_NUM + 1)){
                            control_thrust_target = THRUST_DELTA + control_thrust_current;
                            control_thrust_delta = THRUST_DELTA / THRUST_SCAN_ACCEL_COUNT;
                            phase = accelerating;
                            tick_init = tick;
                        }else{
                            control_thrust_target = 0;
                            control_thrust_current = 0;
                            GPIO_ResetBits(GPIOC, GPIO_Pin_4);
                            phase = holdon;
                            tick_init = tick;
                        }
                    }
                };break;
                case holdon:
                {
                    control_thrust_target = 0;
                    control_thrust_current = 0;
                };break;               
                default:
                {
                    control_thrust_target = 0;
                    control_thrust_current = 0;
                };
            }
            cnt = 0;
        }
        else{
            if (cnt++ > 1500) {
                cnt = 0;
                configParamGiveSemaphore();
            }
            GPIO_ResetBits(GPIOC, GPIO_Pin_4);
            control_thrust_target = 0;
            control_thrust_current = 0;
            phase = start;
            start_scan_flag = 0;
            tick_init = tick;
        }

        control->thrust = constrainf(control_thrust_current, 0.0f, 50000.0f);
        control->roll = 0;
        control->pitch = 0;
        control->yaw = 0;

#endif



}

void getRateDesired(attitude_t *get){
    *get = rateDesired;
}

void getAngleDesired(attitude_t *get){
    *get = attitudeDesired;
}

#ifdef TEST
void setThrust_cmd(uint16_t Thrust_cmd)
{
    thrust_init_cmd = Thrust_cmd;
}

#endif
