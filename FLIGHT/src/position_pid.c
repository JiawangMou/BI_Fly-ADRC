#include "position_pid.h"
#include "commander.h"
#include "config_param.h"
#include "maths.h"
#include "pid.h"
#include "remoter_ctrl.h"
#include <math.h>

/********************************************************************************
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 位置PID控制代码
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
 *
 * 修改说明:
 * 版本V1.3 水平定点PID输出较大，所以在位置环输出设置0.1的系数，
        速率环输出设置0.15系数，从而增加PID的可调性。
********************************************************************************/

#define THRUST_BASE (35000) /*基础油门值*/

#define PIDVX_OUTPUT_LIMIT 120.0f  // ROLL限幅	(单位°带0.15的系数)
#define PIDVY_OUTPUT_LIMIT 120.0f  // PITCH限幅	(单位°带0.15的系数)


#define PIDX_OUTPUT_LIMIT 1200.0f // X轴速度限幅(单位cm/s 带0.1的系数)
#define PIDY_OUTPUT_LIMIT 1200.0f // Y轴速度限幅(单位cm/s 带0.1的系数)

#ifdef USE_MBD
#define PIDZ_OUTPUT_LIMIT  (40000.0f) //Z轴速度限幅(单位cm/s)
#define PIDVZ_OUTPUT_LIMIT (40000.0f) /*PID VZ限幅*/
#else
#define PIDZ_OUTPUT_LIMIT  100.0f //Z轴速度限幅(单位cm/s)
#define PIDVZ_OUTPUT_LIMIT (65500) /*PID VZ限幅*/
#endif
//临时版本，定高变为单环PID,输出限幅直接到油门输出
// #define PIDZ_OUTPUT_LIMIT 65500.0f // Z轴速度限幅(油门量)

/*Dterm filter cutoff frequency*/
#define POS_X_PID_DTERM_CUTOFF_FREQ 100.0
#define POS_Y_PID_DTERM_CUTOFF_FREQ 60.0
#define POS_Z_PID_DTERM_CUTOFF_FREQ 100.0

#define VEL_X_PID_DTERM_CUTOFF_FREQ 90.0
#define VEL_Y_PID_DTERM_CUTOFF_FREQ 40.0
#define VEL_Z_PID_DTERM_CUTOFF_FREQ 90.0

static float thrustLpf        = THRUST_BASE; /*油门低通*/
static float thrustHover      = 0.f;
static bool  enterVelModeFlag = false;

PidObject pidVX;
PidObject pidVY;
PidObject pidVZ;

PidObject pidX;
PidObject pidY;
PidObject pidZ;

void positionControlInit(float velocityPidDt, float posPidDt) 
{
    pidInit(&pidVX, 0, configParam.pidPos.vx, velocityPidDt,VEL_X_PID_DTERM_CUTOFF_FREQ);     /*vx PID初始化*/
    pidInit(&pidVY, 0, configParam.pidPos.vy, velocityPidDt,VEL_Y_PID_DTERM_CUTOFF_FREQ);     /*vy PID初始化*/
    pidInit(&pidVZ, 0, configParam.pidPos.vz, velocityPidDt,VEL_Z_PID_DTERM_CUTOFF_FREQ);     /*vz PID初始化*/
    pidSetOutputLimit(&pidVX, configParam.pidPos.vx.outputLimit); /* 输出限幅 */
    pidSetOutputLimit(&pidVY, configParam.pidPos.vy.outputLimit); /* 输出限幅 */
    pidSetOutputLimit(&pidVZ, configParam.pidPos.vz.outputLimit); /* 输出限幅 */

    pidInit(&pidX, 0, configParam.pidPos.x, posPidDt,POS_X_PID_DTERM_CUTOFF_FREQ);          /*x PID初始化*/
    pidInit(&pidY, 0, configParam.pidPos.y, posPidDt,POS_Y_PID_DTERM_CUTOFF_FREQ);          /*y PID初始化*/
    pidInit(&pidZ, 0, configParam.pidPos.z, posPidDt,POS_Z_PID_DTERM_CUTOFF_FREQ);          /*z PID初始化*/
    pidSetOutputLimit(&pidX, configParam.pidPos.x.outputLimit); /* 输出限幅 */
    pidSetOutputLimit(&pidY, configParam.pidPos.y.outputLimit); /* 输出限幅 */
    pidSetOutputLimit(&pidZ, configParam.pidPos.z.outputLimit); /* 输出限幅 */
}



#ifdef USE_MBD
static void velocityController(float* thrust, control_t *control,attitude_t* attitude, setpoint_t* setpoint, const state_t* state)
{
    static u16 altholdCount = 0;

    // Roll and Pitch 
    // TODO:XY 与 Z轴的位置控制控制器结构不一样，还没有统一
    attitude->pitch = 0.15f * pidUpdate(&pidVX, setpoint->velocity.x - state->velocity.x);
    attitude->roll  = 0.15f * pidUpdate(&pidVY, setpoint->velocity.y - state->velocity.y);

    // Thrust
    //TEST:定高油门修改，进入速率模式之后，在定高油门上叠加加速油门
    // float thrustRaw = 0.f;
    // if (setpoint->mode.z == modeVelocity) {
    //     thrustRaw        = pidUpdate(&pidVZ, setpoint->velocity.z) + thrustHover;
    //     enterVelModeFlag = true;
    // } else {
    //     thrustRaw   = pidUpdate(&pidZ, setpoint->position.z - state->position.z);
    //     thrustHover = thrustRaw;
    // }

	// Thrust
	control->thrust_part.vel = constrainf(pidUpdate(&pidVZ, setpoint->velocity.z - state->velocity.z),-PIDVZ_OUTPUT_LIMIT,PIDVZ_OUTPUT_LIMIT);
    float thrustRaw = control->thrust_part.vel + control->thrust_part.pos + control->thrust_part.MBD;
    *thrust = constrainf(thrustRaw, 1000, 65500); /*油门限幅*/

    // //防止PID计算油门降得太快，让飞行器停机，影响定高效果，所以对低于基础油门的油门进行小变化范围处理，无论如何使油门不低于35000
    // if (*thrust < THRUST_BASE)
    //     *thrust = *thrust / 8 + 35000;

    thrustLpf += (*thrust - thrustLpf) * 0.003f;

    if (getCommanderKeyFlight()) /*定高飞行状态*/
    {
        // //TEST: 推出速率模式的时候更新基础油门
        // if (enterVelModeFlag && (setpoint->mode.z != modeVelocity)) {
        //     enterVelModeFlag       = false;
        //     configParam.thrustBase = thrustLpf;
        // }
        if (fabs(state->acc.z) < 35.f) {
            altholdCount++;
            if (altholdCount > 1000) {
                altholdCount = 0;
                if (fabs(configParam.thrustBase - thrustLpf) > 1000.f) /*更新基础油门值*/
                    configParam.thrustBase = thrustLpf;
            }
        } else {
            altholdCount = 0;
        }
    } else if (getCommanderKeyland() == false) /*降落完成，油门清零*/
    {
        *thrust = 0;
    }
}
void positionController(float* thrust, control_t *control,attitude_t* attitude, setpoint_t* setpoint, const state_t* state, float dt)
{
    if (setpoint->mode.x == modeAbs || setpoint->mode.y == modeAbs) {
        setpoint->velocity.x = 0.1f * pidUpdate(&pidX, setpoint->position.x - state->position.x);
        setpoint->velocity.y = 0.1f * pidUpdate(&pidY, setpoint->position.y - state->position.y);
    }

    if (setpoint->mode.z == modeAbs) {
        control->thrust_part.pos = constrainf(pidUpdate(&pidZ, setpoint->position.z - state->position.z),-PIDZ_OUTPUT_LIMIT,PIDZ_OUTPUT_LIMIT);
    }
    velocityController(thrust,control, attitude, setpoint, state);
}
#else
static void velocityController(float* thrust, attitude_t* attitude, setpoint_t* setpoint, const state_t* state)
{
    static u16 altholdCount = 0;

    // Roll and Pitch
    attitude->pitch = 0.15f * pidUpdate(&pidVX, setpoint->velocity.x - state->velocity.x);
    attitude->roll  = 0.15f * pidUpdate(&pidVY, setpoint->velocity.y - state->velocity.y);

    // Thrust
    //TEST:定高油门修改，进入速率模式之后，在定高油门上叠加加速油门
    // float thrustRaw = 0.f;
    // if (setpoint->mode.z == modeVelocity) {
    //     thrustRaw        = pidUpdate(&pidVZ, setpoint->velocity.z) + thrustHover;
    //     enterVelModeFlag = true;
    // } else {
    //     thrustRaw   = pidUpdate(&pidZ, setpoint->position.z - state->position.z);
    //     thrustHover = thrustRaw;
    // }

	// Thrust
	float thrustRaw = pidUpdate(&pidVZ, setpoint->velocity.z - state->velocity.z);

    *thrust = constrainf(thrustRaw + THRUST_BASE, 1000, 65500); /*油门限幅*/

    // //防止PID计算油门降得太快，让飞行器停机，影响定高效果，所以对低于基础油门的油门进行小变化范围处理，无论如何使油门不低于35000
    // if (*thrust < THRUST_BASE)
    //     *thrust = *thrust / 8 + 35000;

    thrustLpf += (*thrust - thrustLpf) * 0.003f;

    if (getCommanderKeyFlight()) /*定高飞行状态*/
    {
        // //TEST: 推出速率模式的时候更新基础油门
        // if (enterVelModeFlag && (setpoint->mode.z != modeVelocity)) {
        //     enterVelModeFlag       = false;
        //     configParam.thrustBase = thrustLpf;
        // }
        if (fabs(state->acc.z) < 35.f) {
            altholdCount++;
            if (altholdCount > 1000) {
                altholdCount = 0;
                if (fabs(configParam.thrustBase - thrustLpf) > 1000.f) /*更新基础油门值*/
                    configParam.thrustBase = thrustLpf;
            }
        } else {
            altholdCount = 0;
        }
    } else if (getCommanderKeyland() == false) /*降落完成，油门清零*/
    {
        *thrust = 0;
    }
}
void positionController(float* thrust, attitude_t* attitude, setpoint_t* setpoint, const state_t* state, float dt)
{
    if (setpoint->mode.x == modeAbs || setpoint->mode.y == modeAbs) {
        setpoint->velocity.x = 0.1f * pidUpdate(&pidX, setpoint->position.x - state->position.x);
        setpoint->velocity.y = 0.1f * pidUpdate(&pidY, setpoint->position.y - state->position.y);
    }

    if (setpoint->mode.z == modeAbs) {
        setpoint->velocity.z = constrainf(0.1f * pidUpdate(&pidZ, setpoint->position.z - state->position.z), -PIDZ_OUTPUT_LIMIT, PIDZ_OUTPUT_LIMIT);
    }

    velocityController(thrust, attitude, setpoint, state);
}
#endif


/*获取定高油门值*/
float getAltholdThrust(void) { return thrustLpf; }

void positionResetAllPID(void)
{
    pidReset(&pidVX);
    pidReset(&pidVY);
    pidReset(&pidVZ);

    pidReset(&pidX);
    pidReset(&pidY);
    pidReset(&pidZ);
}

void positionPIDwriteToConfigParam(void)
{
    configParam.pidPos.vx.kp          = pidVX.kp;
    configParam.pidPos.vx.ki          = pidVX.ki;
    configParam.pidPos.vx.kd          = pidVX.kd;
    configParam.pidPos.vx.outputLimit = pidVX.outputLimit;

    configParam.pidPos.vy.kp          = pidVY.kp;
    configParam.pidPos.vy.ki          = pidVY.ki;
    configParam.pidPos.vy.kd          = pidVY.kd;
    configParam.pidPos.vy.outputLimit = pidVY.outputLimit;

    configParam.pidPos.vz.kp          = pidVZ.kp;
    configParam.pidPos.vz.ki          = pidVZ.ki;
    configParam.pidPos.vz.kd          = pidVZ.kd;
    configParam.pidPos.vz.outputLimit = pidVZ.outputLimit;

    configParam.pidPos.x.kp          = pidX.kp;
    configParam.pidPos.x.ki          = pidX.ki;
    configParam.pidPos.x.kd          = pidX.kd;
    configParam.pidPos.x.outputLimit = pidX.outputLimit;

    configParam.pidPos.y.kp          = pidY.kp;
    configParam.pidPos.y.ki          = pidY.ki;
    configParam.pidPos.y.kd          = pidY.kd;
    configParam.pidPos.y.outputLimit = pidY.outputLimit;

    configParam.pidPos.z.kp          = pidZ.kp;
    configParam.pidPos.z.ki          = pidZ.ki;
    configParam.pidPos.z.kd          = pidZ.kd;
    configParam.pidPos.z.outputLimit = pidZ.outputLimit;
}
