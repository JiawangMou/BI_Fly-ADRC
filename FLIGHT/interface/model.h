/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: model.h
 *
 * Code generated for Simulink model 'model'.
 *
 * Model version                  : 1.9
 * Simulink Coder version         : 9.5 (R2021a) 14-Nov-2020
 * C/C++ source code generated on : Tue Nov  9 11:58:00 2021
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. RAM efficiency
 *    2. ROM efficiency
 *    3. Traceability
 * Validation result: Not run
 */

#ifndef RTW_HEADER_model_h_
#define RTW_HEADER_model_h_
#include <math.h>
#include <string.h>
#ifndef model_COMMON_INCLUDES_
#define model_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "ADRC.h"
#include "stabilizer_types.h"
#include "sensors_types.h"
#endif                                 /* model_COMMON_INCLUDES_ */
#include "arm_math.h"
#include "power_control.h"

/* Model Code Variants */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#define MOTOR_PWM2F_A 0.0003685f   //PWM to  flappingHZ  cofficient a; f = motor_a * u    unit : Hz
#define MOTOR_F2T_A 0.3543f    //flappingHZ to  Thrust  cofficient a; Thrust = wing_a*f^2    unit : mN

#define SERVO_PWM2ANGLE_A 0.06   //PWM to servo angle  cofficient a; servo angle(unit: °) = servo_a * PWM + servo_b 
#define SERVO_PWM2ANGLE_B -90    

#define MASS 29.0f
#define G 980.0f

#define Jxx 364  //x轴转动惯量（单位： g.cm^2）
#define Jyy 294  //y轴转动惯量（单位： g.cm^2）
#define Jzz 343  //z轴转动惯量（单位： g.cm^2）

#define WINGCR 0.3543f


#define MOTORCR  0.000368f  

/* Forward declaration for rtModel */
typedef struct tag_RTM_model_T RT_MODEL_model_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T DiscreteTransferFcn_states[2];/* '<S1>/Discrete Transfer Fcn' */
  real_T DiscreteTransferFcn1_states[2];/* '<S1>/Discrete Transfer Fcn1' */
  real_T UD_DSTATE;                    /* '<S18>/UD' */
  real_T UD_DSTATE_f;                  /* '<S17>/UD' */
  real_T UD_DSTATE_c;                  /* '<S4>/UD' */
} DW_model_T;

/* Invariant block signals (default storage) */
typedef struct {
  const real_T mg_E[3];             /* '<S1>/Product' */
  const real_T Subtract[3];            /* '<S6>/Subtract' */
  const real_T Subtract1[3];           /* '<S6>/Subtract1' */
} ConstB_model_T;

typedef struct {
    float flappingHz;
    float servoangle;
    
    float thrust;

} ModelStatus_t;

typedef struct {
    u8 order;
    float32_t *nump;
    float32_t *decp;
    float32_t *x;
    float32_t *y;
} Tf_t;


/* Control state after transfer function*/
typedef struct
{
	float32_t Tao_Fz[4];    //Tao_Fz[0]~Tao_Fz[2]： Tao, unit: g*cm^2/s^2;   Tao_Fz[3] ：Fz,  unit: g*cm/s^2; 
	float U[4];				//U[0]~U[3]: T_lcos(beta_l), T_lsin(beta_l),T_rcos(beta_r),T_rsin(beta_r)  unit: mN; 
	float actuator[4];  	//T_l  T_r  beta_l  beta_r	 unit: mN, rad 
  arm_matrix_instance_f32 mat_Tao_Fz_41;
  arm_matrix_instance_f32 mat_U_41;
} control_Tf_t;


extern control_Tf_t control_Tf;

/* Constant parameters (default storage) */

/* Model entry point functions */
void model_initialize(void);
// u16 MBD_update(float setpoint_Z, velocity_t state_velE,Axis3f gyro);
// void model_terminate(void);
void model_reset(void);

// extern Tf_t motortf;
// extern Tf_t servotf;

float ServoPWM2Servoangle(u32 servoPWM);
// float flappingHZ2ThrustZ_E(const float f_hz,float servoangle,float pitchangle);
float TfApply(Tf_t *tf,const float input, const float lowLimit, const float highLimit);
void actuator_TfApply(float *actuator);

// float Fdz_coffe_cal(const attitude_t *atti,velocity_t vel,float servoangle);
// float Ffz_coffe_cal(const attitude_t *atti,float servoangle);

arm_status U_cal(control_t *control, attitude_t *angle);
// void D_coffe_cal(float32_t *D, Axis3f *anglerate);
void C_coffe_cal(float32_t *C, Axis3f *angle);
void inv_C_coffe_cal(float32_t *inv_C, attitude_t *angle_R);
void Thrust2motorPWM(Motorstatus_t *motorstatus,float32_t u );
void ServoAngle2ServoPWM(Servostatus_t *servo_l,Servostatus_t *servo_r, float32_t *u );
void control_allocation(control_t *control);
void control_allocation_inv(Axis3f *angle);
void actuator2PWM(control_t *control, actuatorStatus_t *actuatorStatus);

#endif                                 /* RTW_HEADER_model_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
