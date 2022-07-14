#include "atkp.h"
// #include "attitude_pid.h"
#include "commander.h"
#include "config_param.h"
#include "flip.h"
#include "motors.h"
#include "optical_flow.h"
// #include "pid.h"
#include "pm.h"
// #include "position_pid.h"
#include "power_control.h"
#include "radiolink.h"
#include "remoter_ctrl.h"
#include "sensfusion6.h"
#include "sensors.h"
#include "stabilizer.h"
#include "state_estimator.h"
#include "uart_3.h"
#include "usblink.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "ADRC.h"
#include "axis.h"
#include "state_control.h"
#include "sensors.h"
#include "BASC_controller.h"
#include "config_param.h"
#include "attitude_adrc.h"
#include "dyn_notch_filter.h"
#include "position_adrc.h"
#include "config.h"


/*FeeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "sensors.h"
#include "task.h"

/********************************************************************************
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 无线通信驱动代码
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
 * 说明：此文件程序基于于匿名科创地面站V4.34通信协议下位机,示例代码修改。
 *
 * 修改说明:
 * 版本V1.3 增加用户数据(USERDATA)上传功能，方便用户上传需要调试的数据到上位机，
 * 上位机实时打印波形，方便用户调试。
 ********************************************************************************/

//数据拆分宏定义
#define BYTE0(dwTemp) (*((u8*)(&dwTemp)))
#define BYTE1(dwTemp) (*((u8*)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((u8*)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((u8*)(&dwTemp) + 3))

//数据返回周期时间（单位ms）
#define PERIOD_STATUS 30
#define PERIOD_SENSOR 10
#define PERIOD_RCDATA 40
#define PERIOD_POWER 100
#define PERIOD_MOTOR 40
#define PERIOD_SENSOR2 40
#define PERIOD_SPEED 50
#define PERIOD_USERDATA 10
#define PERIOD_PIDOUT 20

#define ATKP_RX_QUEUE_SIZE 10 /*ATKP包接收队列消息个数*/

typedef struct {
    u16 roll;
    u16 pitch;
    u16 yaw;
    u16 thrust;
} joystickFlyui16_t;

bool                     isConnect = false;
bool                     isInit    = false;
bool                     flyable   = false;
static joystickFlyui16_t rcdata;
static xQueueHandle      rxQueue;

//extern PidObject pidAngleRoll;
//extern PidObject pidAnglePitch;
//extern PidObject pidAngleYaw;
//extern PidObject pidRateRoll;
//extern PidObject pidRatePitch;
//extern PidObject pidRateYaw;

//extern adrcObject_t ADRCRatePitch;
//extern adrcObject_t ADRCRateRoll;
//extern adrcObject_t ADRCRateYaw;

//extern adrcObject_t ADRCAngleRoll;
//extern adrcObject_t ADRCAnglePitch;
//extern adrcObject_t ADRCAngleYaw;

//extern nlsefObject_t velZ_nlsef;
//extern tdObject_t posZ_TD;

static void atkpSendPacket(atkp_t* p)
{
    radiolinkSendPacket(p);

    if (getusbConnectState()) {
        usblinkSendPacket(p);
    }
}

/***************************发送至匿名上位机指令******************************/

// // TEST:sendTestData
// static void sendTestData(
//     float roll, float pitch, float yaw, s32 alt, Axis3f acc, Acc_Send accRawData, float ZPredict, uint32_t timestamp)
// {

//     u8     _cnt = 0;
//     atkp_t p;
//     vs16   _temp;
//     vs32   _temp2 = alt;

//     p.msgID = 0x88;

//     _temp          = (int)(roll * 100);
//     p.data[_cnt++] = BYTE1(_temp);
//     p.data[_cnt++] = BYTE0(_temp);
//     _temp          = (int)(pitch * 100);
//     p.data[_cnt++] = BYTE1(_temp);
//     p.data[_cnt++] = BYTE0(_temp);
//     _temp          = (int)(yaw * 100);
//     p.data[_cnt++] = BYTE1(_temp);
//     p.data[_cnt++] = BYTE0(_temp);

//     _temp          = (int)(alt * 10);
//     p.data[_cnt++] = BYTE1(_temp);
//     p.data[_cnt++] = BYTE0(_temp);

//     _temp          = (int)(acc.x * 10);
//     p.data[_cnt++] = BYTE1(_temp);
//     p.data[_cnt++] = BYTE0(_temp);
//     _temp          = (int)(acc.y * 10);
//     p.data[_cnt++] = BYTE1(_temp);
//     p.data[_cnt++] = BYTE0(_temp);
//     _temp          = (int)(acc.z * 10);
//     p.data[_cnt++] = BYTE1(_temp);
//     p.data[_cnt++] = BYTE0(_temp);

//     _temp          = (int)(accRawData.acc_beforefusion.x * 1000);
//     p.data[_cnt++] = BYTE1(_temp);
//     p.data[_cnt++] = BYTE0(_temp);
//     _temp          = (int)(accRawData.acc_beforefusion.y * 1000);
//     p.data[_cnt++] = BYTE1(_temp);
//     p.data[_cnt++] = BYTE0(_temp);
//     _temp          = (int)(accRawData.acc_beforefusion.z * 1000);
//     p.data[_cnt++] = BYTE1(_temp);
//     p.data[_cnt++] = BYTE0(_temp);

//     _temp          = (int)(ZPredict * 10);
//     p.data[_cnt++] = BYTE1(_temp);
//     p.data[_cnt++] = BYTE0(_temp);

//     p.data[_cnt++] = BYTE3(timestamp);
//     p.data[_cnt++] = BYTE2(timestamp);
//     p.data[_cnt++] = BYTE1(timestamp);
//     p.data[_cnt++] = BYTE0(timestamp);
//     p.dataLen      = _cnt;
//     atkpSendPacket(&p);
// }

static void sendStatus(float roll, float pitch, float yaw, s32 alt, u8 fly_model, u8 armed, uint32_t timestamp)
{
    u8     _cnt = 0;
    atkp_t p;
    vs16   _temp;
    vs32   _temp2 = alt;

    p.msgID = UP_STATUS;

    _temp          = (int)(roll * 100);
    p.data[_cnt++] = BYTE1(_temp);
    p.data[_cnt++] = BYTE0(_temp);
    _temp          = (int)(pitch * 100);
    p.data[_cnt++] = BYTE1(_temp);
    p.data[_cnt++] = BYTE0(_temp);
    _temp          = (int)(yaw * 100);
    p.data[_cnt++] = BYTE1(_temp);
    p.data[_cnt++] = BYTE0(_temp);

    p.data[_cnt++] = BYTE3(_temp2);
    p.data[_cnt++] = BYTE2(_temp2);
    p.data[_cnt++] = BYTE1(_temp2);
    p.data[_cnt++] = BYTE0(_temp2);

    p.data[_cnt++] = fly_model;
    p.data[_cnt++] = armed;

    p.data[_cnt++] = BYTE3(timestamp);
    p.data[_cnt++] = BYTE2(timestamp);
    p.data[_cnt++] = BYTE1(timestamp);
    p.data[_cnt++] = BYTE0(timestamp);
    p.dataLen      = _cnt;
    atkpSendPacket(&p);
}

static void sendSenser(float a_x, float a_y, float a_z, s16 g_x, s16 g_y, s16 g_z, s16 m_x, s16 m_y, s16 m_z, u8 accuse)
{
    u8     _cnt = 0;
    atkp_t p;
    vs16   _temp;

    p.msgID = UP_SENSER;

    _temp          = (s16)(a_x );
    p.data[_cnt++] = BYTE1(_temp);
    p.data[_cnt++] = BYTE0(_temp);
    _temp          = (s16)(a_y );
    p.data[_cnt++] = BYTE1(_temp);
    p.data[_cnt++] = BYTE0(_temp);
    _temp          = (s16)(a_z );
    p.data[_cnt++] = BYTE1(_temp);
    p.data[_cnt++] = BYTE0(_temp);

    _temp          = g_x;
    p.data[_cnt++] = BYTE1(_temp);
    p.data[_cnt++] = BYTE0(_temp);
    _temp          = g_y;
    p.data[_cnt++] = BYTE1(_temp);
    p.data[_cnt++] = BYTE0(_temp);
    _temp          = g_z;
    p.data[_cnt++] = BYTE1(_temp);
    p.data[_cnt++] = BYTE0(_temp);

    _temp          = m_x;
    p.data[_cnt++] = BYTE1(_temp);
    p.data[_cnt++] = BYTE0(_temp);
    _temp          = m_y;
    p.data[_cnt++] = BYTE1(_temp);
    p.data[_cnt++] = BYTE0(_temp);
    _temp          = m_z;
    p.data[_cnt++] = BYTE1(_temp);
    p.data[_cnt++] = BYTE0(_temp);

    p.data[_cnt++] = accuse;

    p.dataLen = _cnt;
    atkpSendPacket(&p);
}
static void sendRCData(
    u16 thrust, u16 yaw, u16 roll, u16 pitch, u16 aux1, u16 aux2, u16 aux3, u16 aux4, u16 aux5, u16 aux6)
{
    u8     _cnt = 0;
    atkp_t p;

    p.msgID        = UP_RCDATA;
    p.data[_cnt++] = BYTE1(thrust);
    p.data[_cnt++] = BYTE0(thrust);
    p.data[_cnt++] = BYTE1(yaw);
    p.data[_cnt++] = BYTE0(yaw);
    p.data[_cnt++] = BYTE1(roll);
    p.data[_cnt++] = BYTE0(roll);
    p.data[_cnt++] = BYTE1(pitch);
    p.data[_cnt++] = BYTE0(pitch);
    p.data[_cnt++] = BYTE1(aux1);
    p.data[_cnt++] = BYTE0(aux1);
    p.data[_cnt++] = BYTE1(aux2);
    p.data[_cnt++] = BYTE0(aux2);
    p.data[_cnt++] = BYTE1(aux3);
    p.data[_cnt++] = BYTE0(aux3);
    p.data[_cnt++] = BYTE1(aux4);
    p.data[_cnt++] = BYTE0(aux4);
    p.data[_cnt++] = BYTE1(aux5);
    p.data[_cnt++] = BYTE0(aux5);
    p.data[_cnt++] = BYTE1(aux6);
    p.data[_cnt++] = BYTE0(aux6);

    p.dataLen = _cnt;
    atkpSendPacket(&p);
}

static void sendPower(u16 votage, u16 current)
{
    u8     _cnt = 0;
    atkp_t p;

    p.msgID = UP_POWER;

    p.data[_cnt++] = BYTE1(votage);
    p.data[_cnt++] = BYTE0(votage);
    p.data[_cnt++] = BYTE1(current);
    p.data[_cnt++] = BYTE0(current);

    p.dataLen = _cnt;
    atkpSendPacket(&p);
}

static void sendMotorPWM(u16 m_1, u16 m_2, u16 m_3, u16 m_4, u16 m_5, u16 m_6, u16 m_7, u16 m_8)
{
    u8     _cnt = 0;
    atkp_t p;

    p.msgID = UP_MOTOR;

    p.data[_cnt++] = BYTE1(m_1);
    p.data[_cnt++] = BYTE0(m_1);
    p.data[_cnt++] = BYTE1(m_2);
    p.data[_cnt++] = BYTE0(m_2);
    p.data[_cnt++] = BYTE1(m_3);
    p.data[_cnt++] = BYTE0(m_3);
    p.data[_cnt++] = BYTE1(m_4);
    p.data[_cnt++] = BYTE0(m_4);
    p.data[_cnt++] = BYTE1(m_5);
    p.data[_cnt++] = BYTE0(m_5);
    p.data[_cnt++] = BYTE1(m_6);
    p.data[_cnt++] = BYTE0(m_6);
    p.data[_cnt++] = BYTE1(m_7);
    p.data[_cnt++] = BYTE0(m_7);
    p.data[_cnt++] = BYTE1(m_8);
    p.data[_cnt++] = BYTE0(m_8);

    p.dataLen = _cnt;
    atkpSendPacket(&p);
}
//static void sendPIDOUT(u8 id, float pid1, float pid2, float pid3, float pid4, float pid5, float pid6)
//{
//    u8     _cnt = 0;
//    atkp_t p;
//    p.msgID = UP_PIDOUT;

//    p.data[_cnt++] = id;
//    p.data[_cnt++] = BYTE0(pid1);
//    p.data[_cnt++] = BYTE1(pid1);
//    p.data[_cnt++] = BYTE2(pid1);
//    p.data[_cnt++] = BYTE3(pid1);

//    p.data[_cnt++] = BYTE0(pid2);
//    p.data[_cnt++] = BYTE1(pid2);
//    p.data[_cnt++] = BYTE2(pid2);
//    p.data[_cnt++] = BYTE3(pid2);

//    p.data[_cnt++] = BYTE0(pid3);
//    p.data[_cnt++] = BYTE1(pid3);
//    p.data[_cnt++] = BYTE2(pid3);
//    p.data[_cnt++] = BYTE3(pid3);

//    p.data[_cnt++] = BYTE0(pid4);
//    p.data[_cnt++] = BYTE1(pid4);
//    p.data[_cnt++] = BYTE2(pid4);
//    p.data[_cnt++] = BYTE3(pid4);

//    p.data[_cnt++] = BYTE0(pid5);
//    p.data[_cnt++] = BYTE1(pid5);
//    p.data[_cnt++] = BYTE2(pid5);
//    p.data[_cnt++] = BYTE3(pid5);

//    p.data[_cnt++] = BYTE0(pid6);
//    p.data[_cnt++] = BYTE1(pid6);
//    p.data[_cnt++] = BYTE2(pid6);
//    p.data[_cnt++] = BYTE3(pid6);

//    p.dataLen = _cnt;
//    atkpSendPacket(&p);
//}

// static void sendSenser2(s32 bar_alt, u16 csb_alt)
// {
//     u8     _cnt = 0;
//     atkp_t p;

//     p.msgID = UP_SENSER2;

//     p.data[_cnt++] = BYTE3(bar_alt);
//     p.data[_cnt++] = BYTE2(bar_alt);
//     p.data[_cnt++] = BYTE1(bar_alt);
//     p.data[_cnt++] = BYTE0(bar_alt);

//     p.data[_cnt++] = BYTE1(csb_alt);
//     p.data[_cnt++] = BYTE0(csb_alt);

//     p.dataLen = _cnt;
//     atkpSendPacket(&p);
// }

//sendPid 输入的数据最好不要超过3276.8,因为输入数据是先乘以10然后转换成int16_t类型，只有2个字节，16位，如果超过输入数据超过3276.8会造成输入溢出
static void sendPid(u8 group, float p1_p, float p1_i, float p1_d, float p2_p, float p2_i, float p2_d, float p3_p,
    float p3_i, float p3_d)
{
    u8     _cnt = 0;
    atkp_t p;
    vs16   _temp;

    p.msgID = 0x10 + group - 1;

    _temp          = p1_p * 10;
    p.data[_cnt++] = BYTE1(_temp);
    p.data[_cnt++] = BYTE0(_temp);
    _temp          = p1_i * 10;
    p.data[_cnt++] = BYTE1(_temp);
    p.data[_cnt++] = BYTE0(_temp);
    _temp          = p1_d * 100;
    p.data[_cnt++] = BYTE1(_temp);
    p.data[_cnt++] = BYTE0(_temp);
    _temp          = p2_p * 10;
    p.data[_cnt++] = BYTE1(_temp);
    p.data[_cnt++] = BYTE0(_temp);
    _temp          = p2_i * 10;
    p.data[_cnt++] = BYTE1(_temp);
    p.data[_cnt++] = BYTE0(_temp);
    _temp          = p2_d * 100;
    p.data[_cnt++] = BYTE1(_temp);
    p.data[_cnt++] = BYTE0(_temp);
    _temp          = p3_p * 10;
    p.data[_cnt++] = BYTE1(_temp);
    p.data[_cnt++] = BYTE0(_temp);
    _temp          = p3_i * 10;
    p.data[_cnt++] = BYTE1(_temp);
    p.data[_cnt++] = BYTE0(_temp);
    _temp          = p3_d * 100;
    p.data[_cnt++] = BYTE1(_temp);
    p.data[_cnt++] = BYTE0(_temp);

    p.dataLen = _cnt;
    atkpSendPacket(&p);
}



static void sendCheck(u8 head, u8 check_sum)
{
    atkp_t p;

    p.msgID   = UP_CHECK;
    p.dataLen = 2;
    p.data[0] = head;
    p.data[1] = check_sum;
    atkpSendPacket(&p);
}

void sendaccBiasprocess_ACK(u8 step_num,float accbias_x,float accbias_y,float accbias_z)
{
    atkp_t p;
    u8 _cnt = 0;
    p.msgID   = UP_ACK_ACC_CALIB;

    p.data[_cnt++] = step_num;
    p.data[_cnt++] = BYTE0(accbias_x);
    p.data[_cnt++] = BYTE1(accbias_x);
    p.data[_cnt++] = BYTE2(accbias_x);
    p.data[_cnt++] = BYTE3(accbias_x);

    p.data[_cnt++] = BYTE0(accbias_y);
    p.data[_cnt++] = BYTE1(accbias_y);
    p.data[_cnt++] = BYTE2(accbias_y);
    p.data[_cnt++] = BYTE3(accbias_y);

    p.data[_cnt++] = BYTE0(accbias_z);
    p.data[_cnt++] = BYTE1(accbias_z);
    p.data[_cnt++] = BYTE2(accbias_z);
    p.data[_cnt++] = BYTE3(accbias_z);

    p.dataLen = _cnt;
    atkpSendPacket(&p);
}
static void sendUserData(u8 group, s16 a_x, s16 a_y, s16 a_z, s16 v_x, s16 v_y, s16 v_z, s16 p_x, s16 p_y, s16 p_z)
{
    u8     _cnt = 0;
    atkp_t p;
    vs16   _temp;

    p.msgID = UP_USER_DATA1 + group - 1;

    _temp          = a_x;
    p.data[_cnt++] = BYTE1(_temp);
    p.data[_cnt++] = BYTE0(_temp);
    _temp          = a_y;
    p.data[_cnt++] = BYTE1(_temp);
    p.data[_cnt++] = BYTE0(_temp);
    _temp          = a_z;
    p.data[_cnt++] = BYTE1(_temp);
    p.data[_cnt++] = BYTE0(_temp);

    _temp          = v_x;
    p.data[_cnt++] = BYTE1(_temp);
    p.data[_cnt++] = BYTE0(_temp);
    _temp          = v_y;
    p.data[_cnt++] = BYTE1(_temp);
    p.data[_cnt++] = BYTE0(_temp);
    _temp          = v_z;
    p.data[_cnt++] = BYTE1(_temp);
    p.data[_cnt++] = BYTE0(_temp);

    _temp          = p_x;
    p.data[_cnt++] = BYTE1(_temp);
    p.data[_cnt++] = BYTE0(_temp);
    _temp          = p_y;
    p.data[_cnt++] = BYTE1(_temp);
    p.data[_cnt++] = BYTE0(_temp);
    _temp          = p_z;
    p.data[_cnt++] = BYTE1(_temp);
    p.data[_cnt++] = BYTE0(_temp);

    p.dataLen = _cnt;
    atkpSendPacket(&p);
}
/****************************************************************************/
/*数据周期性发送给上位机，每1ms调用一次*/
static void atkpSendPeriod(void)
{
    static u16 count_ms = 1;
    // TEST: 300HZ 发送数据

    // if (!(count_ms % 3)) {
    //     attitude_t attitude;
    //     Axis3f     acc, vel, pos;
    //     Acc_Send   accRawData;

    //     getAttitudeData(&attitude);
    //     getStateData(&acc, &vel, &pos);
    //     getAcc_SendData(&accRawData);

    //     float zPredict = getPosZPredictData();

    //     u32 timestamp = getSysTickCnt();
    //     sendTestData(attitude.roll, attitude.pitch, attitude.yaw, pos.z, acc, accRawData, zPredict, timestamp);
    // }
    if (!(count_ms % PERIOD_STATUS)) {
        attitude_t attitude;
        Axis3f     acc, vel, pos;
        getAttitudeData(&attitude);
        getStateData(&acc, &vel, &pos);
        sendStatus(attitude.roll, attitude.pitch, attitude.yaw, pos.z, 0, flyable, attitude.timestamp);
    }
    if (!(count_ms % PERIOD_SENSOR)) {
        Axis3i16 acc;
        Axis3i16 gyro;
        Axis3i16 mag;
        Acc_Send acc_send;
        getSensorRawData(&acc, &gyro, &mag);
        getAcc_SendData(&acc_send);
        sendSenser(acc.x, acc.y, acc.z,gyro.x,gyro.y, gyro.z, mag.x, mag.y, mag.z, acc_send.useAcc);
    }
    if (!(count_ms % PERIOD_USERDATA)) /*用户数据*/
    {
        sensorData_t sensordata;
        attitude_t   attitude;
        control_t    control;
        getAttitudeData(&attitude);
        setpoint_t setpoint = getSetpoint();
        control             = getControlData();
        u32 timestamp       = getSysTickCnt();
        getSensorData(&sensordata);
        sendUserData(1, 100.0f * BASCAtti.delta1[0], 100.0f * BASCAtti.delta2[0], 0.01f * BASCAtti.Tao0_hat[0],
            100.0f * BASCAtti.x2d[0], 10.0f * setpoint.attitudedesired.roll, 0.01f * BASCAtti.Torque[0], 10.0f * Roll_td.x1,
            10.0f * attitude.roll, 100.0f * sensordata.gyro_R.x);
        sendUserData(2, 100.0f * BASCAtti.delta1[2], 100.0f * BASCAtti.delta2[2], 0.01f * BASCAtti.Tao0_hat[2],
            100.0f * BASCAtti.x2d[2], 10.0f * setpoint.attitudedesired.yaw, 0.01f * BASCAtti.Torque[2], 10.0f * Yaw_td.x1,
            10.0f * attitude.yaw, 100.0f * sensordata.gyro_R.z);

        //#ifdef ADRC_CONTROL
        //		sensorData_t sensordata;
        ////		float thrustBase = 0.1f * configParam.thrustBase;
        //		attitude_t rateDesired;
        //		attitude_t attitude;
        //		attitude_t attitudeDesired;
        //        control_t control;
        //		getAttitudeData(&attitude);
        //		getAngleDesired(&attitudeDesired);
        //        getSensorData(&sensordata);
        //		getRateDesired( &rateDesired );
        //        control  = getControlData();
        //        u32 timestamp = getSysTickCnt();
        ////        sendUserData(1, attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
        ///ADRCAngleRoll.td.x1, ADRCAnglePitch.td.x1, ADRCAngleYaw.td.x1,  ADRCRateRoll.td.x1,
        ///rateDesired.roll,rateDesired.pitch ); /		sendUserData(2, rateDesired.yaw,
        ///control.ADRC_u0[0],control.ADRC_u0[1], control.ADRC_u0[2], control.ADRC_u0[3], control.ADRC_u[0],
        ///control.ADRC_u[1],control.ADRC_u[2], control.ADRC_u[3]);
        //        // sendUserData(2, opFlow.velLpf[X],opFlow.velLpf[Y],opFlow.posSum[X],opFlow.posSum[Y],
        //		//
        //0,getFusedHeight(),vl53lxx.distance,100.f*vl53lxx.quality,thrustBase); #endif

        //#ifdef PID_CONTROL
        //        // float  thrustBase = 0.1f * configParam.thrustBase;
        //        // sensorData_t sensor;
        //        // control_t control;
        //        // getSensorData(&sensor);
        //        // getRateDesired(&rateDesired);
        //        // getAngleDesired(&angleDesired);
        //        // getAttitudeData(&attitude);
        ////         Axis3i16 acc;
        ////         Axis3i16 gyro;
        ////         Axis3i16 mag;
        ////        Acc_Send acc_send;
        ////         getSensorRawData(&acc, &gyro, &mag);
        //        // control  = getControlData();
        //        // sendUserData(1, angleDesired.roll, attitude.roll, rateDesired.roll, sensor.gyro.x,
        //        pidAngleRoll.outP, pidAngleRoll.outI, pidRateRoll.outP,
        //        //     pidRateRoll.outI, pidRateRoll.outD);
        //        // sendUserData(2, angleDesired.pitch, attitude.pitch, angleDesired.yaw, attitude.yaw, sensor.gyro.y,
        //        //     sensor.gyro.z, control.pitch, control.yaw, control.roll);

        //        //定高测试用通讯协议
        //        // sendUserData(1, 10*state.position.z,10* setpoint.position.z,
        //        10*state.velocity.z,10*setpoint.velocity.z,10*setpoint.acc.z,10*state.acc.z,control.thrust/10,10*laser_height,(s16)(timestamp
        //        & 0x00ffff));
        //        // sendUserData(2, control.thrust_part.pos /10 ,control.thrust_part.vel /10 ,control.thrust_part.MBD
        //        /10,rateDesired.roll
        //        ,rateDesired.pitch,rateDesired.yaw,attitudeDesired.roll,attitudeDesired.pitch,attitudeDesired.yaw);
        //        //定高加速度计滤波测试用协议
        //        // sendUserData(1,
        //        10*state.position.z,10*state.velocity.z,10*state.acc.z,10*setpoint.position.z,10*setpoint.velocity.z,10*velZ_TD.x1,10*velZ_TD.x2,10*posZ_TD.x1,10*posZ_TD.x2);
        //        // sendUserData(2, velZ_nlsef.e1_out/10,velZ_nlsef.e2_out/10,velZ_nlsef.u0/10,control.thrust_part.MBD
        //        /10,control.thrust/10,10*laser_height,motorPWM.f1/10,motorPWM.f2/10,(s16)(timestamp & 0x00ffff));
        //        // sendUserData(1,
        //        10*laser_height,100*sensordata.acc.x,100*sensordata.acc.y,100*sensordata.acc.z,100*state.velocity.x,100*state.velocity.y,100*state.velocity.z,100*state.position.z,(s16)(timestamp
        //        & 0x00ffff));
        //        // sendUserData(2,
        //        q0*1000,q1*1000,q2*1000,q3*1000,100*state.attitude.pitch,100*state.attitude.roll,100*state.attitude.yaw,motorPWM.f1/10,motorPWM.f2/10);
        //        //测试stabilizer中的函数运行调度是否正常
        //        // sendUserData(1,
        //        stabi_tick.sensorsAcquire_tick,stabi_tick.imuUpdate_tick,stabi_tick.positionEstimate_tick,stabi_tick.commanderGetSetpoint_tick,stabi_tick.getOpFlowData_tick,stabi_tick.flyerFlipCheck_tick,stabi_tick.stateControl_tick,stabi_tick.motorControl_tick,(s16)(timestamp
        //        & 0x00ffff));
        //        // 光流数据测试
        //        // sendUserData(1, 10*attitudeDesired.roll,10*state.attitude.roll,10*attitudeDesired.pitch,
        //        10*state.attitude.pitch,opFlow.velLpf[X],opFlow.velLpf[Y], opFlow.posSum[X],
        //        opFlow.posSum[Y],sensordata.zrange.distance);
        //        // sendUserData(2, sensordata.zrange.distance_uncomp,opFlow.pixDelta[X],opFlow.pixDelta[Y],
        //        opFlow.deltaVel[X],
        //        opFlow.deltaVel[Y],opFlow.pixdeltaveluncopm[X],opFlow.pixdeltaveluncopm[Y],opFlow.deltaVelComp[X],opFlow.deltaVelComp[Y]);
        //        // //带模型控制器和ESO测试
        //        sendUserData(1,
        //        setpoint.position.z,posZ_TD.x1,state.position.z,posZ_TD.x2,state.velocity.z,setpoint.velocity.z,velZ_LESO.z1,velZ_LESO.disturb,control.u);
        //        sendUserData(2,
        //        velZ_TD.x1,velZ_TD.x2,velZ_nlsef.e1_out,control.a,control.b,velZ_nlsef.u0,state.velocity.x,opFlow.velLpf[X],opFlow.velLpf[Y]);

        //#else
        //        sendUserData(1, 10*state.position.z, 10*setpoint.position.z,
        //        10*state.velocity.z,10*setpoint.velocity.z,10*laser_height,10*state.acc.z,control.thrust,0,(s16)(timestamp
        //        & 0x00ffff)); sendUserData(2, acc.x
        //        ,acc.y,acc.z,gyro_UnLPF.x,gyro_UnLPF.y,gyro_UnLPF.z,10*sensordata.acc.z,(s16)(opFlow.timestamp &
        //        0x00ffff),0);

        //        // sendUserData(1, gyro.x, gyro_LPF.x,
        //        sensordata.gyro.x,attitudeDesired.roll,attitude.roll,rateDesired.roll,control.roll,0,(s16)(timestamp &
        //        0x00ffff));
        //        // sendUserData(2, gyro.y, gyro_LPF.y,
        //        sensordata.gyro.y,attitudeDesired.pitch,attitude.pitch,rateDesired.pitch,control.pitch,0,0);
        //#endif
        //		// sendUserData(2, attitudeDesired.roll, attitude.roll, attitudeDesired.pitch,
        //attitude.pitch,rateDesired.pitch, sensordata.gyro.y, attitude.yaw,rateDesired.yaw, sensordata.gyro.z);
        //        // sendUserData(2, opFlow.velLpf[X],opFlow.velLpf[Y],opFlow.posSum[X],opFlow.posSum[Y],
        //		//
        //0,getFusedHeight(),vl53lxx.distance,100.f*vl53lxx.quality,thrustBase); #endif

    }
    // if (!(count_ms % PERIOD_RCDATA)) {
    //     sendRCData(rcdata.thrust, rcdata.yaw, rcdata.roll, rcdata.pitch, 0, 0, 0, 0, 0, 0);
    // }
    // if (!(count_ms % PERIOD_POWER)) {
    //     float bat = getBatteryVoltage();
    //     sendPower(bat * 100, 500);
    // }
    // if (!(count_ms % PERIOD_MOTOR)) {
    //     u16        f1, f2, s1, s2, s3, r1;
    //     motorPWM_t motorPWM;
    //     getMotorPWM(&motorPWM);
    //     f1 = (float)motorPWM.f1 / 65535 * 1000;
    //     f2 = (float)motorPWM.f2 / 65535 * 1000;
    //     s1 = (float)motorPWM.s_left;
    //     s2 = (float)motorPWM.s_right;
    //     s3 = (float)motorPWM.s_middle;
    //     r1 = (float)motorPWM.r1 / 65535 * 1000;

    //     sendMotorPWM(f1, f2, s1, s2, s3, r1, 0, 0);
    // }
    // if (!(count_ms % PERIOD_SENSOR2))
    // {
    //     int baro = getBaroData() * 100.f;
    //     sendSenser2(baro, 0);
    // }
    
    //用自己的地面站查看的数据，现在用匿名地面站，所以注释掉
    // if (!(count_ms % PERIOD_PIDOUT)) {
    //     Axis3f acc, vel, pos;
    //     mode_e z_mode;
    //     u8 commander;
    //     sendPIDOUT(0, pidAngleRoll.outP, pidAngleRoll.outI, pidAngleRoll.outD, pidAnglePitch.outP,
    //     pidAnglePitch.outI,pidAnglePitch.outD);
    //     sendPIDOUT(1, pidAngleYaw.outP, pidAngleYaw.outI, pidAngleYaw.outD, pidRateRoll.outP, pidRateRoll.outI,
    //         pidRateRoll.outD);
    //     sendPIDOUT(2, pidRatePitch.outP, pidRatePitch.outI, pidRatePitch.outD, pidRateYaw.outP, pidRateYaw.outI,
    //         pidRateYaw.outD);
    //     getStateData(&acc, &vel, &pos);
    //     z_mode = getZmode();
    //     commander = getCommanderBits();
    //     u32 modeCommander =((((u32)z_mode&0xff) << 8) | ((u32)commander&0xff));
    //     sendPIDOUT(3, pidZ.out, pidVZ.out, getControlData().thrust, vel.z, *(float*)(&modeCommander), 0);
    // }
    if (++count_ms >= 65535)
        count_ms = 1;
}

static u8 atkpCheckSum(atkp_t* packet)
{
    u8 sum;
    sum = DOWN_BYTE1;
    sum += DOWN_BYTE2;
    sum += packet->msgID;
    sum += packet->dataLen;
    for (int i = 0; i < packet->dataLen; i++) {
        sum += packet->data[i];
    }
    return sum;
}

static void atkpReceiveAnl(atkp_t* anlPacket)
{
    if (anlPacket->msgID == DOWN_COMMAND) {
        switch (anlPacket->data[0]) {
        case D_COMMAND_ACC_CALIB:
            break;

        case D_COMMAND_GYRO_CALIB:
            break;

        case D_COMMAND_MAG_CALIB:
            break;

        case D_COMMAND_BARO_CALIB:
            break;
        case D_COMMAND_ACC_CALIB_EXIT:
        {
            reset_accbiasRunning();
            resetaccBias_accScale();
        }break;
        case D_COMMAND_ACC_CALIB_STEP1:
            setprocessAccBias_stepnum(0);
            break;
        case D_COMMAND_ACC_CALIB_STEP2:
            setprocessAccBias_stepnum(1);
            break;
        case D_COMMAND_ACC_CALIB_STEP3:
            setprocessAccBias_stepnum(2);
            break;
        case D_COMMAND_ACC_CALIB_STEP4:
            setprocessAccBias_stepnum(3);
            break;
        case D_COMMAND_ACC_CALIB_STEP5:
            setprocessAccBias_stepnum(4);
            break;
        case D_COMMAND_ACC_CALIB_STEP6:
            setprocessAccBias_stepnum(5);
            break;
        case D_COMMAND_ACC_CALIB_WRITE_FLASH:
        {
            accbias_writeFlash();
            configParamGiveSemaphore(); //将修改的configparamdefault写入flash
        }break;  
        case D_COMMAND_FLIGHT_LOCK:
            flyable = false;
            break;
        case D_COMMAND_FLIGHT_ULOCK:
            flyable = true;
            break;
        }
		u8 cksum = atkpCheckSum(anlPacket);
        sendCheck(anlPacket->msgID, cksum);   
    } else if (anlPacket->msgID == DOWN_ACK) {
        if (anlPacket->data[0] == D_ACK_READ_PID) /*读取PID参数*/
        {
            sendPid(1, 0.01f * BASCAtti.A3[0], BASCAtti.A1[0], 0.001f * BASCAtti.A2[0], 0.01f * BASCAtti.A3[4],
                BASCAtti.A1[4], 0.001f * BASCAtti.A2[4], 0.01f * BASCAtti.A3[8], BASCAtti.A1[8],
                0.001f * BASCAtti.A2[8]);
            sendPid(2, Roll_td.r, Pitch_td.r, Yaw_td.r, BASCAtti.J_gamma[0], BASCAtti.J_gamma[4], BASCAtti.J_gamma[8],
                0.01f * BASCAtti.Tao0_gamma[0], BASCAtti.Tao0_gamma[4], 0.001f * BASCAtti.Tao0_gamma[8]);
            sendPid(3, BASCPos.A1, BASCPos.A2, BASCPos.A3, posZ_TD.r, BASCPos.M_gamma, 0, 0, 0, 0);
            sendPid(4, 0, 0, 0, getservoinitpos_configParam(PWM_LEFT), getservoinitpos_configParam(PWM_RIGHT),
                getservoinitpos_configParam(PWM_MIDDLE) / 10, 0, 0, 0);
            sendPid(5, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        }
        if (anlPacket->data[0] == D_ACK_RESET_PARAM) /*恢复默认参数*/
        {
            resetConfigParam_BASCcontriller();
            stateControlInit();
            sendPid(1, 0.01f * BASCAtti.A3[0], BASCAtti.A1[0], 0.001f * BASCAtti.A2[0], 0.01f * BASCAtti.A3[4],
                BASCAtti.A1[4], 0.001f * BASCAtti.A2[4], 0.01f * BASCAtti.A3[8], BASCAtti.A1[8],
                0.001f * BASCAtti.A2[8]);
            sendPid(2, Roll_td.r, Pitch_td.r, Yaw_td.r, BASCAtti.J_gamma[0], BASCAtti.J_gamma[4], BASCAtti.J_gamma[8],
                0.01f * BASCAtti.Tao0_gamma[0], BASCAtti.Tao0_gamma[4], 0.001f * BASCAtti.Tao0_gamma[8]);
            sendPid(3, BASCPos.A1, BASCPos.A2, BASCPos.A3, posZ_TD.r, BASCPos.M_gamma, 0, 0, 0, 0);
            sendPid(4, 0, 0, 0, getservoinitpos_configParam(PWM_LEFT), getservoinitpos_configParam(PWM_RIGHT),
                getservoinitpos_configParam(PWM_MIDDLE) / 10, 0, 0, 0);
            sendPid(5, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        }
    } else if (anlPacket->msgID == DOWN_RCDATA) {
        rcdata = *((joystickFlyui16_t*)anlPacket->data);
    } else if (anlPacket->msgID == DOWN_POWER) /*nrf51822*/
    {
        pmSyslinkUpdate(anlPacket);
    } else if (anlPacket->msgID == DOWN_REMOTER) /*遥控器*/
    {
        remoterCtrlProcess(anlPacket);
    } else if (anlPacket->msgID == DOWN_PID1) {
        BASCAtti.A3[0] = 10.0f * ((s16)(*(anlPacket->data + 0) << 8) | *(anlPacket->data + 1));
        BASCAtti.A1[0] =  0.1f * ((s16)(*(anlPacket->data + 2) << 8) | *(anlPacket->data + 3));
        BASCAtti.A2[0] = 10.0f * ((s16)(*(anlPacket->data + 4) << 8) | *(anlPacket->data + 5));
        BASCAtti.A3[4] = 10.0f * ((s16)(*(anlPacket->data + 6) << 8) | *(anlPacket->data + 7));
        BASCAtti.A1[4] =  0.1f * ((s16)(*(anlPacket->data + 8) << 8) | *(anlPacket->data + 9));
        BASCAtti.A2[4] = 10.0f * ((s16)(*(anlPacket->data + 10) << 8) | *(anlPacket->data + 11));
        BASCAtti.A3[8] = 10.0f * ((s16)(*(anlPacket->data + 12) << 8) | *(anlPacket->data + 13));
        BASCAtti.A1[8] =  0.1f * ((s16)(*(anlPacket->data + 14) << 8) | *(anlPacket->data + 15));
        BASCAtti.A2[8] = 10.0f * ((s16)(*(anlPacket->data + 16) << 8) | *(anlPacket->data + 17));

        // attitudePIDwriteToConfigParam();
        // configParamGiveSemaphore(); //将修改的configparamdefault写入flash
        u8 cksum = atkpCheckSum(anlPacket);
        sendCheck(anlPacket->msgID, cksum);
    } else if (anlPacket->msgID == DOWN_PID2) {
        Roll_td.r              = 0.1 * ((s16)(*(anlPacket->data + 0) << 8) | *(anlPacket->data + 1));
        Pitch_td.r             = 0.1 * ((s16)(*(anlPacket->data + 2) << 8) | *(anlPacket->data + 3));
        Yaw_td.r               = 0.01 * ((s16)(*(anlPacket->data + 4) << 8) | *(anlPacket->data + 5));
        BASCAtti.J_gamma[0]    = 0.1 * ((s16)(*(anlPacket->data + 6) << 8) | *(anlPacket->data + 7));
        BASCAtti.J_gamma[4]    = 0.1 * ((s16)(*(anlPacket->data + 8) << 8) | *(anlPacket->data + 9));
        BASCAtti.J_gamma[8]    = 0.01 * ((s16)(*(anlPacket->data + 10) << 8) | *(anlPacket->data + 11));
        BASCAtti.Tao0_gamma[0] = 10.0f * ((s16)(*(anlPacket->data + 12) << 8) | *(anlPacket->data + 13));
        BASCAtti.Tao0_gamma[4] = 0.1 * ((s16)(*(anlPacket->data + 14) << 8) | *(anlPacket->data + 15));
        BASCAtti.Tao0_gamma[8] = 10.0f * ((s16)(*(anlPacket->data + 16) << 8) | *(anlPacket->data + 17));

        u8 cksum = atkpCheckSum(anlPacket);
        sendCheck(anlPacket->msgID, cksum);
    } else if (anlPacket->msgID == DOWN_PID3) {
        BASCPos.A1 = 0.1 * ((s16)(*(anlPacket->data + 0) << 8) | *(anlPacket->data + 1));
        BASCPos.A2 = 0.1 * ((s16)(*(anlPacket->data + 2) << 8) | *(anlPacket->data + 3));
        BASCPos.A3 = 0.01 * ((s16)(*(anlPacket->data + 4) << 8) | *(anlPacket->data + 5));

        posZ_TD.r = 0.1 * ((s16)(*(anlPacket->data + 6) << 8) | *(anlPacket->data + 7));
        BASCPos.M_gamma = 0.1 * ((s16)(*(anlPacket->data + 8) << 8) | *(anlPacket->data + 9));
        // pidZ.kd = 0.01 * ((s16)(*(anlPacket->data + 10) << 8) | *(anlPacket->data + 11));

        // pidVX.kp = 0.1 * ((s16)(*(anlPacket->data + 12) << 8) | *(anlPacket->data + 13));
        // pidVX.ki = 0.1 * ((s16)(*(anlPacket->data + 14) << 8) | *(anlPacket->data + 15));
        // pidVX.kd = 0.01 * ((s16)(*(anlPacket->data + 16) << 8) | *(anlPacket->data + 17));
        u8 cksum = atkpCheckSum(anlPacket);
        sendCheck(anlPacket->msgID, cksum);
    } else if (anlPacket->msgID == DOWN_PID4) {
    //    pidX.kp = 0.1 * ((s16)(*(anlPacket->data + 0) << 8) | *(anlPacket->data + 1));
    //    pidX.ki = 0.1 * ((s16)(*(anlPacket->data + 2) << 8) | *(anlPacket->data + 3));
    //    pidX.kd = 0.01 * ((s16)(*(anlPacket->data + 4) << 8) | *(anlPacket->data + 5));
       // pidY              = pidX; //位置保持PID，X\Y方向是一样的
       u16 s_left_set    = 0.1 * ((s16)(*(anlPacket->data + 6) << 8) | *(anlPacket->data + 7));
       u16 s_right_set   = 0.1 * ((s16)(*(anlPacket->data + 8) << 8) | *(anlPacket->data + 9));
       u16 s_middle_set  = 0.1 * ((s16)(*(anlPacket->data + 10) << 8) | *(anlPacket->data + 11));

    //    ADRCAngleRoll.td.r  = 10 * ((s16)(*(anlPacket->data + 12) << 8) | *(anlPacket->data + 13));
    //    ADRCAnglePitch.td.r = 10 * ((s16)(*(anlPacket->data + 14) << 8) | *(anlPacket->data + 15));
    //    ADRCAngleYaw.td.r   = ((s16)(*(anlPacket->data + 16) << 8) | *(anlPacket->data + 17));

    //    posZ_nlsef.I_limit = 0.1 * ((s16)(*(anlPacket->data + 12) << 8) | *(anlPacket->data + 13));
    //    velZ_LESO.w0       = 0.1 * ((s16)(*(anlPacket->data + 14) << 8) | *(anlPacket->data + 15));
    //    posZ_nlsef.zeta    = 0.01 * ((s16)(*(anlPacket->data + 16) << 8) | *(anlPacket->data + 17));

       changeServoinitpos_configParam(s_left_set, s_right_set, s_middle_set);
       servoSetPWM(PWM_LEFT, s_left_set);
       servoSetPWM(PWM_RIGHT, s_right_set);
       servoSetPWM(PWM_MIDDLE, s_middle_set);

        u8 cksum = atkpCheckSum(anlPacket);
        sendCheck(anlPacket->msgID, cksum);
    } else if (anlPacket->msgID == DOWN_PID5) {

       u8 cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);

    } else if (anlPacket->msgID == DOWN_PID6) {

        // attitudePIDwriteToConfigParam();
		// positionPIDwriteToConfigParam();
        attitudeADRCwriteToConfigParam();
        BASCwriteToConfigParam();
		u8 cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);
    } 
}

void atkpTxTask(void* param)
{
    u32 lastWakeTime = getSysTickCnt();
    sendMsgACK();
    while (1) {
        atkpSendPeriod();
        // vTaskDelay(10);
        vTaskDelayUntil(&lastWakeTime, 1);
    }
}

void atkpRxAnlTask(void* param)
{
    atkp_t p;
    while (1) {
        xQueueReceive(rxQueue, &p, portMAX_DELAY);
        atkpReceiveAnl(&p);
    }
}

void atkpInit(void)
{
    if (isInit)
        return;
    rxQueue = xQueueCreate(ATKP_RX_QUEUE_SIZE, sizeof(atkp_t));
    ASSERT(rxQueue);
    isInit = true;
}

bool atkpReceivePacketBlocking(atkp_t* p)
{
    ASSERT(p);
    ASSERT(p->dataLen <= ATKP_MAX_DATA_SIZE);
    return xQueueSend(rxQueue, p, portMAX_DELAY);
}
