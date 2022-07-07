#include "state_control.h"
// #include "attitude_pid.h"
#include "config.h"
#include "config_param.h"
#include "maths.h"
// #include "position_pid.h"
#include "stabilizer.h"
#include "ADRC.h"
#include "attitude_adrc.h"
#include "position_adrc.h"
#include "stm32f4xx_gpio.h"
#include "model.h"
#include "axis.h"
#include "BASC_controller.h"


static attitude_t attitudeDesired;
//static attitude_t atti_TD;
static float      actualThrust;
static attitude_t rateDesired;


static float thrustLpf        = 35000; /*基础油门*/



#ifdef TEST

#define THRUST_NUM 10
#define THRUST_STARTPOINT 10000
#define THRUST_ENDPOINT 50000
#define THRUST_DELTA ((THRUST_ENDPOINT - THRUST_STARTPOINT) / THRUST_NUM )

#define THRUST_WAIT_ACCEL_TIME 10000 // scan ģ���µļ���ʱ�� ��λ��ms
#define THRUST_SCAN_TIME 5000 // scan ģ���µ�ɨ��ʱ�� ��λ��ms
#define THRUST_SCAN_ACCEL_TIME 2000 // scan ģ���µļ���ʱ�� ��λ��ms
#define THRUST_ACCEL_DIV_MS 10 // scanģʽ�£�����1s��ϸ�֣�10��ʾ10ms�ı�һ���ٶ�
#define THRUST_WAIT_ACCEL_COUNT (THRUST_WAIT_ACCEL_TIME / THRUST_ACCEL_DIV_MS)
#define THRUST_SCAN_ACCEL_COUNT (THRUST_SCAN_ACCEL_TIME / THRUST_ACCEL_DIV_MS)



static uint8_t thrust_count = 0;
static uint16_t thrust_init_cmd = 10000;

static uint8_t phase = 0;

enum PHASE{start = 0, waiting = 1, accelerating = 2, scanning = 3, holdon = 4};
#endif



float getAltholdThrust(void) { return thrustLpf; }
// // remoter setpoint(roll,pitch) filter
// static lpf2pData setpointFilter[2];

void stateControlInit(void)
{
	// attitudeControlInit(RATE_PID_DT, ANGEL_PID_DT, MAIN_LOOP_DTS); /*��ʼ����̬PID*/	
	// positionControlInit(VEL_PID_DT, POS_PID_DT); /*��ʼ��λ��PID*/
    // attitudeADRCinit();
    // positionADRCinit();
    //     // Filter the setpoint
    // lpf2pInit(&setpointFilter[0], ANGEL_PID_RATE, 20);
    // lpf2pInit(&setpointFilter[1], ANGEL_PID_RATE, 20);
}

bool stateControlTest(void)
{
    bool pass = true;
    // pass &= attitudeControlTest();
    return pass;
}

void stateControl(control_t* control, sensorData_t* sensors, state_t* state, setpoint_t* setpoint, const u32 tick)
{
    static u16 cnt = 0;
#ifndef TEST

#ifdef ADRC_CONTROL
    if (RATE_DO_EXECUTE(POSZ_TD_RATE, tick)) {
        if (setpoint->mode.z != modeDisable) 
            posZ_transient_process_update(setpoint);
    }
    if (RATE_DO_EXECUTE(ATTITUDE_TD_RATE, tick)) { 
        attitudeTD(setpoint);
    }
#endif

//     if (RATE_DO_EXECUTE(POS_PID_RATE, tick)) {
//         if (setpoint->mode.x != modeDisable || setpoint->mode.y != modeDisable || setpoint->mode.z != modeDisable) {
//             #ifdef USE_MBD
//             positionController(setpoint, state);
//             #else 
//             positionController(&actualThrust, &attitudeDesired, setpoint, state, POS_PID_DT);
//             #endif
//         }
//     }

//     if (RATE_DO_EXECUTE(VELZ_LOOP_RATE, tick)) {
//         if (setpoint->mode.x != modeDisable || setpoint->mode.y != modeDisable || setpoint->mode.z != modeDisable) {
//             velocityController(&actualThrust,control, &attitudeDesired,setpoint, state, sensors);
//         }
//     }

//     //角度环
//     if (RATE_DO_EXECUTE(ANGEL_PID_RATE, tick)) {
//         if (setpoint->mode.z == modeDisable) {
//             actualThrust = Thrustcommand2ADRC_u0(setpoint->thrust);
//             actualThrust = constrainf(actualThrust, 0.0f, (float)FULLTHROTTLE);
//             if(actualThrust < G)
//                 control->ADRC_u0[3] = 0;
//             else
//                 control->ADRC_u0[3] = actualThrust - G; 
//         }
//         if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {
//             attitudeDesired.roll  = setpoint->attitude.roll;
//             attitudeDesired.pitch = setpoint->attitude.pitch;
//         }

//         if (control->flipDir == CENTER) {
//             attitudeDesired.yaw -= setpoint->attitude.yaw / ANGEL_PID_RATE; /*����YAW ����ģʽ*/
//             if (attitudeDesired.yaw > 180.0f)
//                 attitudeDesired.yaw -= 360.0f;
//             if (attitudeDesired.yaw < -180.0f)
//                 attitudeDesired.yaw += 360.0f;
//         }

//         attitudeDesired.roll += configParam.trimR; //����΢��ֵ
//         attitudeDesired.pitch += configParam.trimP;


// #ifdef ADRC_CONTROL
//         if (RATE_DO_EXECUTE(ANGLE_TD_RATE, tick)) { 
//             attitudeTD(ROLL, attitudeDesired.roll, &atti_TD.roll);
//             attitudeTD(PITCH, attitudeDesired.pitch, &atti_TD.pitch);
//             attitudeTD(YAW, attitudeDesired.yaw, &atti_TD.yaw);
//         }
// #endif
//         // attitudeDesired.roll  = lpf2pApply(&setpointFilter[0], attitudeDesired.roll);
//         // attitudeDesired.pitch = lpf2pApply(&setpointFilter[1], attitudeDesired.pitch);

//         attitudeAnglePID(&state->attitude, &atti_TD, &rateDesired);
//     }

    if (RATE_DO_EXECUTE(RATE_BASC_RATE, tick)) {
        Torque_Cal(&sensors->gyro, &state->attitude);
        
    }
//     //角速度环
//     if (RATE_DO_EXECUTE(RATE_PID_RATE, tick)) {
//         if (setpoint->mode.roll == modeVelocity) {
//             rateDesired.roll = setpoint->attitudeRate.roll;
//             attitudeControllerResetRollAttitudePID();
//         }
//         if (setpoint->mode.pitch == modeVelocity) {
//             rateDesired.pitch = setpoint->attitudeRate.pitch;
//             attitudeControllerResetPitchAttitudePID();
//         }
//         extern u8 fstate;
//         if (control->flipDir != CENTER && fstate == 4) /*�շ�����ֻʹ���ڻ�PID*/
//         {
//             rateDesired.pitch = setpoint->attitude.pitch;
//             rateDesired.roll  = setpoint->attitude.roll;
//         }
//         #ifdef PID_CONTROL
//         attitudeRatePID(&sensors->gyro, &rateDesired, control);
//         #elif defined ADRC_CONTROL
//         attitudeRateADRC(&sensors->gyro,  &rateDesired, control->ADRC_u0);
//         U_cal(&sensors->gyro,&state->attitude, control->ADRC_u0, control->ADRC_u);
//         control_allocation(control);
//         if(setpoint->mode.z == modeDisable) {
//             if(actualThrust < G){
//                 diff_Thrust = control->actuator[T_l] - control->actuator[T_r];
//                 control->actuator[T_l] = actualThrust * MASS / 100 + diff_Thrust;
//                 control->actuator[T_r] = actualThrust * MASS / 100 - diff_Thrust;
//                 control->actuator[T_l] = constrainf(control->actuator[T_l], 0.0f, 200.0f);
//                 control->actuator[T_r] = constrainf(control->actuator[T_r], 0.0f, 200.0f); //200 对应于200mN
//             }
//         }
//         #endif
//     }
    
    // control->thrust = actualThrust;

    if (actualThrust < 5.f) {
#ifdef FOUR_WING
        // control->roll = 0;
#endif
        // control->pitch = 0;
        // control->yaw = 0;

        // attitudeResetAllPID_TEST();
        // attitudeResetAllPID();	/*��λ��̬PID*/
        // /*����ȡ����λ��ԭ���ǣ��÷���������Ķ���ʱ�򣬻��ܿ�������ķ�Ӧ���Ӷ�ȷ��PID�������Ƿ������������ǽ����Ƿ�������*/
        // positionResetAllPID();                     /*��λλ��PID*/
        // velZ_LESO.z2 = 0;
        control->actuator[T_l] = 0;
        control->actuator[T_r] = 0;

        // adrc_reset(&ADRCRatePitch);
#ifdef ADRC_CONTROL
		// adrc_reset(&ADRCRateRoll);
#endif
        attitudeDesired.yaw = state->attitude.yaw; /*��λ���������yawֵ*/

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
                        control_thrust_delta = control_thrust_target / THRUST_WAIT_ACCEL_COUNT; // ÿ��ˢ������
                        phase                = accelerating;
                        tick_init            = tick;
                    }
                };break;
                case accelerating:
                {
                    if(start_scan_flag ==1){//scanģ���µļ��ٶȹ���
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
                    }else{//waiting ģʽ�µļ��ٹ���
                        if(((tick-tick_init) % THRUST_WAIT_ACCEL_TIME) == 0){
                            control_thrust_current = control_thrust_target;
                            phase = scanning;
                            tick_init = tick;
                            start_scan_flag = 1;       //��ʼɨ��
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
