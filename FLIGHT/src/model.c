/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: model.c
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

#include "model.h"
#include "sys.h"
#include "config_param.h"
#include "power_control.h"
#include "sensfusion6.h"
#include "arm_math.h"
#include "maths.h"
#include "motors.h"




/* Invariant block signals (default storage) */
const ConstB_model_T model_ConstB = {
  { 0.0, 0.0, -27000.0 },              /* '<S1>/Product' */

  { 0.0, -7.289, -1.5 },               /* '<S6>/Subtract' */

  { 0.0, 7.289, -1.5 }                 /* '<S6>/Subtract1' */
};

/* Constant parameters (default storage) */
const ConstP_model_T model_ConstP = {
  /* Expression: param.ModelParam_Rb_CoR
   * Referenced by: '<S1>/param.ModelParam_Rb_CoR'
   */
  { 0.0, 0.0, 6.0 }
};

/* Block states (default storage) */
DW_model_T model_DW;

/* External inputs (root inport signals with default storage) */
ExtU_model_T model_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_model_T model_Y;

/* Real-time model */
static RT_MODEL_model_T model_M_;
RT_MODEL_model_T *const model_M = &model_M_;

/* Exported data definition */

/* Definition for custom storage class: Global */
real_T DCMbe[9];                       /* '<Root>/DCMbe' */
real_T velE[3];                        /* '<Root>/velE' */

arm_matrix_instance_f32  DCMbe_arm;
arm_matrix_instance_f32  DCMeb_arm;
float DCMeb[9];

#define  MOTORTF_ORDER 2  // 这里的阶数是系统实际阶数+1
 
float32_t motortf_x[MOTORTF_ORDER] = {0};
float32_t motortf_y[MOTORTF_ORDER-1] = {0};
float32_t motortf_num[MOTORTF_ORDER] = {0,0.0601}; //Fs = 1000 这里的系数与采样率有关，一定注意！！
float32_t motortf_dec[MOTORTF_ORDER-1] = {-0.9399};//Fs = 1000 这里的系数与采样率有关，一定注意！！

#define  SERVOTF_ORDER 3  // 这里的阶数是系统实际阶数+1
float servotf_x[SERVOTF_ORDER] = {0};
float servotf_y[SERVOTF_ORDER-1] = {0};
float servotf_num[SERVOTF_ORDER] = {0,0.0186,0.0186};//Fs = 250 这里的系数与采样率有关，一定注意！！
float servotf_dec[SERVOTF_ORDER-1] = {-1.6455,0.6828};//Fs = 250 这里的系数与采样率有关，一定注意！！
// static arm_matrix_instance_f32 pSrc_servotf_x; 
// static arm_matrix_instance_f32 pSrc_servotf_y; 
// static arm_matrix_instance_f32 pSrc_servotf_num; 
// static arm_matrix_instance_f32 pSrc_servotf_dec; 
Tf_t motortf;
Tf_t servotf;

// /* Model update function */
// u16 MBD_update(float aZ_E_desired, velocity_t state_velE,Axis3f gyro)
// {
//   real_T rtb_Transpose[9];
//   real_T rtb_Transpose1[9];
//   real_T rtb_Transpose_0[9];
//   real_T rtb_Cop_L_b[3];
//   real_T rtb_Cop_L_b_i[3];
//   real_T rtb_Cop_R_b[3];
//   real_T rtb_Cop_R_b_g[3];
//   real_T rtb_Sum_p[3];
//   real_T rtb_Thrust_coeff_B_vector[3];
//   real_T rtb_vel_B_vector[3];
//   real_T tmp[3];
//   real_T a;
//   real_T denAccum;
//   real_T denAccum_0;
//   real_T rtb_Cop_L_b_j;
//   real_T rtb_Gain1_idx_0;
//   real_T rtb_Gain1_idx_1;
//   real_T rtb_Sum_d_idx_2;
//   real_T rtb_Transpose1_1;
//   real_T rtb_Transpose1_f;
//   real_T rtb_Transpose_2;
//   real_T rtb_Transpose_o;
//   real_T rtb_vel_B_vector_tmp;
//   real_T rtb_vel_B_vector_tmp_0;
//   int32_T i;
//   actuatorStatus_t motorPWM;
// //get DCMbe
//   getDCMeb(DCMeb);
//   arm_status result = arm_mat_inverse_f32 (&DCMeb_arm,&DCMbe_arm);
// //get VelE 单位：state_velE cm -> m velE
//     velE[0] = state_velE.x;
//     velE[1] = state_velE.y;   
//     velE[2] = state_velE.z;
// //get servo command
//     getMotorPWM(&motorPWM);
//     model_U.angle_command[0] = constrainf( ServoPWM2angle(motorPWM.s_middle,PWM_MIDDLE),-50.0f,50.0f);
//     model_U.angle_command[1] = constrainf( ServoPWM2angle(motorPWM.s_left,  PWM_LEFT  ),-50.0f,50.0f);
// //get Wb
//   for(int i=0;i< 3;i++)
//     model_U.Wb[i]= gyro.axis[i]*DEG2RAD;
// //get aZ_E_desired
//   model_U.aZ_E_desired = aZ_E_desired;

//   /* DiscreteTransferFcn: '<S1>/Discrete Transfer Fcn' incorporates:
//    *  Inport: '<Root>/angle_command'
//    */
//   denAccum = (model_U.angle_command[0] - 1.6451487309864259f *
//               model_DW.DiscreteTransferFcn_states[0]) - 0.682538626130998f *
//     model_DW.DiscreteTransferFcn_states[1];
//   rtb_Gain1_idx_0 = (0.009347473786143f * denAccum + 0.018694947572286f *
//                      model_DW.DiscreteTransferFcn_states[0]) + 0.009347473786143f
//     * model_DW.DiscreteTransferFcn_states[1];

//   /* DiscreteTransferFcn: '<S1>/Discrete Transfer Fcn1' incorporates:
//    *  Inport: '<Root>/angle_command'
//    */
//   denAccum_0 = (model_U.angle_command[1] - 1.6451487309864259f *
//                 model_DW.DiscreteTransferFcn1_states[0]) - 0.682538626130998f *
//     model_DW.DiscreteTransferFcn1_states[1];
//   rtb_Gain1_idx_1 = (0.009347473786143f * denAccum_0 + 0.018694947572286f *
//                      model_DW.DiscreteTransferFcn1_states[0]) +
//     0.009347473786143f * model_DW.DiscreteTransferFcn1_states[1];

//   /* Saturate: '<S1>/Signal_Saturation_3' incorporates:
//    *  DiscreteTransferFcn: '<S1>/Discrete Transfer Fcn'
//    */
//   if (rtb_Gain1_idx_0 > 50.0f) {
//     rtb_Gain1_idx_0 = 50.0f;
//   } else if (rtb_Gain1_idx_0 < -50.0f) {
//     rtb_Gain1_idx_0 = -50.0f;
//   }

//   /* Gain: '<S2>/Gain1' */
//   rtb_Gain1_idx_0 *= 0.017453292519943295f;

//   /* Saturate: '<S1>/Signal_Saturation_3' incorporates:
//    *  DiscreteTransferFcn: '<S1>/Discrete Transfer Fcn1'
//    */
//   if (rtb_Gain1_idx_1 > 50.0f) {
//     rtb_Gain1_idx_1 = 50.0f;
//   } else if (rtb_Gain1_idx_1 < -50.0f) {
//     rtb_Gain1_idx_1 = -50.0f;
//   }

//   /* Gain: '<S2>/Gain1' */
//   rtb_Gain1_idx_1 *= 0.017453292519943295f;

//   /* Trigonometry: '<S20>/sincos' incorporates:
//    *  SignalConversion generated from: '<S20>/sincos'
//    */
//   rtb_Cop_R_b_g[1] = arm_cos_f32(rtb_Gain1_idx_1);
//   a = arm_sin_f32(rtb_Gain1_idx_1);

//   /* Fcn: '<S20>/Fcn11' */
//   rtb_Transpose[0] = rtb_Cop_R_b_g[1];

//   /* Fcn: '<S20>/Fcn21' */
//   rtb_Transpose[1] = 0.0f;

//   /* Fcn: '<S20>/Fcn31' */
//   rtb_Transpose[2] = a;

//   /* Fcn: '<S20>/Fcn12' */
//   rtb_Transpose[3] = 0.0f;

//   /* Fcn: '<S20>/Fcn22' */
//   rtb_Transpose[4] = 1.0f;

//   /* Fcn: '<S20>/Fcn32' */
//   rtb_Transpose[5] = 0.0f;

//   /* Fcn: '<S20>/Fcn13' */
//   rtb_Transpose[6] = -a;

//   /* Fcn: '<S20>/Fcn23' */
//   rtb_Transpose[7] = 0.0f;

//   /* Fcn: '<S20>/Fcn33' */
//   rtb_Transpose[8] = rtb_Cop_R_b_g[1];

//   /* Math: '<S8>/Transpose' */
//   for (i = 0; i < 3; i++) {
//     rtb_Transpose_0[3 * i] = rtb_Transpose[i];
//     rtb_Transpose_0[3 * i + 1] = rtb_Transpose[i + 3];
//     rtb_Transpose_0[3 * i + 2] = rtb_Transpose[i + 6];
//   }

//   memcpy(&rtb_Transpose[0], &rtb_Transpose_0[0], 9U * sizeof(real_T));

//   /* End of Math: '<S8>/Transpose' */

//   /* Trigonometry: '<S19>/sincos' incorporates:
//    *  Gain: '<S8>/Gain1'
//    */
//   rtb_Cop_R_b[1] = arm_cos_f32(-rtb_Gain1_idx_0);
//   a = arm_sin_f32(-rtb_Gain1_idx_0);

//   /* Fcn: '<S19>/Fcn11' */
//   rtb_Transpose1[0] = rtb_Cop_R_b[1];

//   /* Fcn: '<S19>/Fcn21' */
//   rtb_Transpose1[1] = 0.0f;

//   /* Fcn: '<S19>/Fcn31' */
//   rtb_Transpose1[2] = a;

//   /* Fcn: '<S19>/Fcn12' */
//   rtb_Transpose1[3] = 0.0f;

//   /* Fcn: '<S19>/Fcn22' */
//   rtb_Transpose1[4] = 1.0f;

//   /* Fcn: '<S19>/Fcn32' */
//   rtb_Transpose1[5] = 0.0f;

//   /* Fcn: '<S19>/Fcn13' */
//   rtb_Transpose1[6] = -a;

//   /* Fcn: '<S19>/Fcn23' */
//   rtb_Transpose1[7] = 0.0f;

//   /* Fcn: '<S19>/Fcn33' */
//   rtb_Transpose1[8] = rtb_Cop_R_b[1];

//   /* Math: '<S8>/Transpose1' */
//   for (i = 0; i < 3; i++) {
//     rtb_Transpose_0[3 * i] = rtb_Transpose1[i];
//     rtb_Transpose_0[3 * i + 1] = rtb_Transpose1[i + 3];
//     rtb_Transpose_0[3 * i + 2] = rtb_Transpose1[i + 6];
//   }

//   memcpy(&rtb_Transpose1[0], &rtb_Transpose_0[0], 9U * sizeof(real_T));

//   /* End of Math: '<S8>/Transpose1' */

//   /* SampleTimeMath: '<S18>/TSamp'
//    *
//    * About '<S18>/TSamp':
//    *  y = u * K where K = 1 / ( w * Ts )
//    */
//   rtb_Gain1_idx_1 *= 250.0f;

//   /* SampleTimeMath: '<S17>/TSamp' incorporates:
//    *  Gain: '<S8>/Gain1'
//    *
//    * About '<S17>/TSamp':
//    *  y = u * K where K = 1 / ( w * Ts )
//    */
//   rtb_Gain1_idx_0 = -rtb_Gain1_idx_0 * 250.0f;

//   /* Product: '<S1>/Product5' incorporates:
//    *  Constant: '<S1>/mass'
//    *  DotProduct: '<S1>/Dot Product'
//    *  Inport: '<Root>/aZ_E_desired'
//    */
//   /* :  a = K(3,1) * K1; */
//   /* :  b =  - Cdz * (V_L(3,1)+Vz_b) - Cdz * (V_R(3,1)+Vz_b); */
//   /* :  c = -ma(3,1) + K(3,1) * K2; */
//   /* f = ma(3,1) - K(3,1) *(K1*f^2 + K2) + Cdz * (V_L(3,1)+Vz_b) *f + Cdz * (V_R(3,1)+Vz_b)*f; */
//   /* :  result = (-b + sqrt(b^2 -4*a*c))/(2*a); */
//   rtb_Sum_d_idx_2 = model_U.aZ_E_desired * 27.0f;
//   for (i = 0; i < 3; i++) {
//     /* Product: '<S1>/Product3' incorporates:
//      *  Math: '<S8>/Transpose'
//      */
//     rtb_Transpose_o = rtb_Transpose[i];

//     /* Product: '<S1>/Product2' incorporates:
//      *  Math: '<S8>/Transpose1'
//      */
//     rtb_Transpose1_f = rtb_Transpose1[i];
//     rtb_Transpose1_1 = 0.0f;

//     /* Product: '<S1>/Product3' */
//     rtb_Transpose_2 = 0.0f;

//     /* Product: '<S6>/Product1' incorporates:
//      *  Sum: '<S6>/Subtract'
//      */
//     a = rtb_Transpose1_f * model_ConstB.Subtract[0];

//     /* Product: '<S6>/Product2' incorporates:
//      *  Sum: '<S6>/Subtract1'
//      */
//     rtb_Cop_L_b_j = rtb_Transpose_o * model_ConstB.Subtract1[0];

//     /* Product: '<S1>/Product3' incorporates:
//      *  Math: '<S8>/Transpose'
//      */
//     rtb_Transpose_o = rtb_Transpose[i + 3];

//     /* Product: '<S1>/Product2' incorporates:
//      *  Math: '<S8>/Transpose1'
//      */
//     rtb_Transpose1_f = rtb_Transpose1[i + 3];
//     rtb_Transpose1_1 += 0.0f;

//     /* Product: '<S1>/Product3' */
//     rtb_Transpose_2 += 0.0f;

//     /* Product: '<S6>/Product1' incorporates:
//      *  Sum: '<S6>/Subtract'
//      */
//     a += rtb_Transpose1_f * model_ConstB.Subtract[1];

//     /* Product: '<S6>/Product2' incorporates:
//      *  Sum: '<S6>/Subtract1'
//      */
//     rtb_Cop_L_b_j += rtb_Transpose_o * model_ConstB.Subtract1[1];

//     /* Product: '<S1>/Product6' incorporates:
//      *  Inport: '<Root>/DCMbe'
//      *  Product: '<S1>/Product4'
//      *  Product: '<S1>/Product5'
//      *  Sum: '<S1>/Add1'
//      */
//     rtb_vel_B_vector_tmp = DCMbe[i + 3];

//     /* Product: '<S1>/Product3' incorporates:
//      *  Math: '<S8>/Transpose'
//      */
//     rtb_Transpose_o = rtb_Transpose[i + 6];

//     /* Product: '<S1>/Product2' incorporates:
//      *  Math: '<S8>/Transpose1'
//      */
//     rtb_Transpose1_f = rtb_Transpose1[i + 6];

//     /* Product: '<S6>/Product1' incorporates:
//      *  Sum: '<S6>/Subtract'
//      */
//     a += rtb_Transpose1_f * model_ConstB.Subtract[2];

//     /* Product: '<S6>/Product2' incorporates:
//      *  Sum: '<S6>/Subtract1'
//      */
//     rtb_Cop_L_b_j += rtb_Transpose_o * model_ConstB.Subtract1[2];

//     /* Product: '<S1>/Product6' incorporates:
//      *  Inport: '<Root>/DCMbe'
//      *  Product: '<S1>/Product4'
//      *  Product: '<S1>/Product5'
//      *  Sum: '<S1>/Add1'
//      */
//     rtb_vel_B_vector_tmp_0 = DCMbe[i + 6];

//     /* Sum: '<S1>/Add8' incorporates:
//      *  Product: '<S1>/Product2'
//      *  Product: '<S1>/Product3'
//      */
//     rtb_Thrust_coeff_B_vector[i] = (rtb_Transpose1_1 + rtb_Transpose1_f) +
//       (rtb_Transpose_2 + rtb_Transpose_o);

//     /* Sum: '<S6>/Add' incorporates:
//      *  Constant: '<S1>/param.ModelParam_Rb_CoR'
//      */
//     rtb_Cop_R_b[i] = a + model_ConstP.paramModelParam_Rb_CoR_Value[i];

//     /* Sum: '<S6>/Add1' incorporates:
//      *  Constant: '<S1>/param.ModelParam_Rb_CoR'
//      */
//     rtb_Cop_L_b_i[i] = rtb_Cop_L_b_j +
//       model_ConstP.paramModelParam_Rb_CoR_Value[i];

//     /* Gain: '<S3>/Gain1' incorporates:
//      *  Inport: '<Root>/Wb'
//      *  Sum: '<S23>/Sum'
//      */
//     rtb_Sum_p[i] = 0.017453292519943295f * model_U.Wb[i];

//     /* Sum: '<S1>/Add1' incorporates:
//      *  Inport: '<Root>/DCMbe'
//      *  Product: '<S1>/Product'
//      *  Product: '<S1>/Product4'
//      *  Product: '<S1>/Product5'
//      */
//     tmp[i] = (rtb_vel_B_vector_tmp_0 * rtb_Sum_d_idx_2) -
//       (rtb_vel_B_vector_tmp_0 * model_ConstB.mg_E[2] + (rtb_vel_B_vector_tmp *
//         model_ConstB.mg_E[1] + DCMbe[i] * model_ConstB.mg_E[0]));

//     /* Product: '<S6>/Product1' */
//     rtb_Cop_R_b_g[i] = a;

//     /* Product: '<S6>/Product2' */
//     rtb_Cop_L_b[i] = rtb_Cop_L_b_j;

//     /* Product: '<S1>/Product6' incorporates:
//      *  Inport: '<Root>/DCMbe'
//      *  Inport: '<Root>/velE'
//      */
//     rtb_vel_B_vector[i] = rtb_vel_B_vector_tmp_0 * velE[2] +
//       (rtb_vel_B_vector_tmp * velE[1] + DCMbe[i] * velE[0]);
//   }

//   /* Sum: '<S24>/Sum' incorporates:
//    *  Product: '<S27>/i x j'
//    *  Product: '<S28>/j x i'
//    */
//   rtb_Sum_d_idx_2 = rtb_Sum_p[0] * rtb_Cop_L_b_i[1] - rtb_Cop_L_b_i[0] * rtb_Sum_p[1];

//   /* Sum: '<S23>/Sum' incorporates:
//    *  Product: '<S25>/i x j'
//    *  Product: '<S26>/j x i'
//    */
//   rtb_Transpose_o = rtb_Sum_p[0] * rtb_Cop_R_b[1];
//   rtb_Transpose1_f = rtb_Cop_R_b[0] * rtb_Sum_p[1];

//   /* MATLAB Function: '<S1>/solve for thrust' incorporates:
//    *  Constant: '<S1>/Constant2'
//    *  Constant: '<S1>/Constant3'
//    *  Constant: '<S1>/param.ModelParam_Cdz'
//    *  Product: '<S13>/i x j'
//    *  Product: '<S14>/j x i'
//    *  Product: '<S15>/i x j'
//    *  Product: '<S16>/j x i'
//    *  Sum: '<S10>/Add4'
//    *  Sum: '<S10>/Add5'
//    *  Sum: '<S11>/Sum'
//    *  Sum: '<S12>/Sum'
//    *  Sum: '<S17>/Diff'
//    *  Sum: '<S18>/Diff'
//    *  Sum: '<S1>/Add8'
//    *  Sum: '<S23>/Sum'
//    *  UnitDelay: '<S17>/UD'
//    *  UnitDelay: '<S18>/UD'
//    *
//    * Block description for '<S17>/Diff':
//    *
//    *  Add in CPU
//    *
//    * Block description for '<S18>/Diff':
//    *
//    *  Add in CPU
//    *
//    * Block description for '<S17>/UD':
//    *
//    *  Store in Global RAM
//    *
//    * Block description for '<S18>/UD':
//    *
//    *  Store in Global RAM
//    */
//   a = rtb_Thrust_coeff_B_vector[2] * 35.43f;
//   rtb_Sum_d_idx_2 = ((( - (rtb_Gain1_idx_1 -
//     model_DW.UD_DSTATE) * rtb_Cop_L_b[0]) + (rtb_vel_B_vector[2] +
//     rtb_Sum_d_idx_2)) + rtb_vel_B_vector[2]) * -0.00198f - ((((- (rtb_Gain1_idx_0 - model_DW.UD_DSTATE_f) * rtb_Cop_R_b_g[0]) + 
//     rtb_vel_B_vector[2]) + (rtb_Transpose_o - rtb_Transpose1_f)) +
//     rtb_vel_B_vector[2]) * 0.00198f;
//   rtb_Sum_d_idx_2 = (sqrt(rtb_Sum_d_idx_2 * rtb_Sum_d_idx_2 -
//     (rtb_Thrust_coeff_B_vector[2] * -202.7f + -tmp[2]) * (4.0f * a)) +
//                      -rtb_Sum_d_idx_2) / (2.0f * a);

//   /* SampleTimeMath: '<S4>/TSamp' incorporates:
//    *  MATLAB Function: '<S1>/solve for thrust'
//    *
//    * About '<S4>/TSamp':
//    *  y = u * K where K = 1 / ( w * Ts )
//    */
//   /* :  Fd_L = -Cdz * (V_L(3,1)+Vz_b) *result; */
//   /* :  Fd_R = -Cdz * (V_R(3,1)+Vz_b)*result; */
//   /* :  F = K(3,1) *(K1*result^2 + K2); */
//   /* :  y = result; */
//   a = rtb_Sum_d_idx_2 * 250.0f;

//   /* Sum: '<S1>/Add4' incorporates:
//    *  Gain: '<S1>/Gain'
//    *  MATLAB Function: '<S1>/solve for thrust'
//    *  Sum: '<S4>/Diff'
//    *  UnitDelay: '<S4>/UD'
//    *
//    * Block description for '<S4>/Diff':
//    *
//    *  Add in CPU
//    *
//    * Block description for '<S4>/UD':
//    *
//    *  Store in Global RAM
//    */
//   rtb_Sum_d_idx_2 += (a - model_DW.UD_DSTATE_c) * 0.016129032258064516f;

//   /* Saturate: '<S1>/Saturation5' */
//   if (rtb_Sum_d_idx_2 > 25.580016f) {
//     rtb_Sum_d_idx_2 = 25.580016f;
//   } else if (rtb_Sum_d_idx_2 < 0.0f) {
//     rtb_Sum_d_idx_2 = 0.0f;
//   }

//   /* End of Saturate: '<S1>/Saturation5' */

//   /* MATLAB Function: '<S1>/flappingHz2PWM' */
//   /* :  if f < 1.5 */
//   if (rtb_Sum_d_idx_2 < 1.5f) {
//     /* Outport: '<Root>/PWM_compensation' */
//     /* :  PWM = 0; */
//     model_Y.PWM_compensation = 0.0f;
//   } else {
//     /* Outport: '<Root>/PWM_compensation' incorporates:
//      *  Constant: '<S1>/ModelParam_motorCb'
//      *  Constant: '<S1>/ModelParam_motorCr'
//      */
//     /* :  else */
//     /* :  PWM = (f - Cb) / Cr; */
//     model_Y.PWM_compensation = (rtb_Sum_d_idx_2 - 1.43f) / 0.0003685f;
//   }

//   /* End of MATLAB Function: '<S1>/flappingHz2PWM' */

//   /* Update for DiscreteTransferFcn: '<S1>/Discrete Transfer Fcn' */
//   /* :  y = PWM; */
//   model_DW.DiscreteTransferFcn_states[1] = model_DW.DiscreteTransferFcn_states[0];
//   model_DW.DiscreteTransferFcn_states[0] = denAccum;

//   /* Update for DiscreteTransferFcn: '<S1>/Discrete Transfer Fcn1' */
//   model_DW.DiscreteTransferFcn1_states[1] =
//     model_DW.DiscreteTransferFcn1_states[0];
//   model_DW.DiscreteTransferFcn1_states[0] = denAccum_0;

//   /* Update for UnitDelay: '<S18>/UD'
//    *
//    * Block description for '<S18>/UD':
//    *
//    *  Store in Global RAM
//    */
//   model_DW.UD_DSTATE = rtb_Gain1_idx_1;

//   /* Update for UnitDelay: '<S17>/UD'
//    *
//    * Block description for '<S17>/UD':
//    *
//    *  Store in Global RAM
//    */
//   model_DW.UD_DSTATE_f = rtb_Gain1_idx_0;

//   /* Update for UnitDelay: '<S4>/UD'
//    *
//    * Block description for '<S4>/UD':
//    *
//    *  Store in Global RAM
//    */
//   model_DW.UD_DSTATE_c = a;
//   return model_Y.PWM_compensation;
// }

/* Model initialize function */
void model_initialize(void)
{
  //init DCMbe_arm  DCMeb_arm
  DCMbe_arm.numCols = 3;
  DCMbe_arm.numRows = 3;
  DCMbe_arm.pData = DCMbe;

  DCMeb_arm.numCols = 3;
  DCMeb_arm.numRows = 3;
  DCMeb_arm.pData = DCMeb;
  /* Registration code */

  /* external inputs */
  DCMbe[0] = 1.0;
  DCMbe[4] = 1.0;
  DCMbe[8] = 1.0;

  motortf.order = MOTORTF_ORDER;
  motortf.decp  = motortf_dec;
  motortf.nump  = motortf_num;
  motortf.x     = motortf_x;
  motortf.y     = motortf_y;

  servotf.order = SERVOTF_ORDER;
  servotf.decp  = servotf_dec;
  servotf.nump  = servotf_num;
  servotf.x     = servotf_x;
  servotf.y     = servotf_y;

  // arm_mat_init_f32(&pSrc_motortf_x, MOTORTF_ORDER,1, motortf_x);
  // arm_mat_init_f32(&pSrc_motortf_y, MOTORTF_ORDER,1, motortf_y);
  // arm_mat_init_f32(&pSrc_motortf_num, MOTORTF_ORDER,1, motortf_x);
  // arm_mat_init_f32(&pSrc_motortf_dec, MOTORTF_ORDER,1, motortf_y);

  // arm_mat_init_f32(&pSrc_servotf_x, SERVOTF_ORDER,1, servotf_x);
  // arm_mat_init_f32(&pSrc_servotf_y, SERVOTF_ORDER,1, servotf_y);
  // arm_mat_init_f32(&pSrc_servotf_num, SERVOTF_ORDER,1, servotf_num);
  // arm_mat_init_f32(&pSrc_servotf_dec, SERVOTF_ORDER,1, servotf_dec);
}

/* Model terminate function */
void model_terminate(void)
{
  /* (no terminate code required) */
}

void model_reset(void)
{
  model_Y.PWM_compensation = 0;
}

/*

* Function: motorPWM2FlappingHZ

* Description:  根据占空比计算扑翼频率

* Input: 两个电机的motorPWM

* Output: 两个电机的平均扑翼频率

* Return: 平均扑翼频率 HZ

* Others: f = motor_a * u   % unit : Hz

*/
float motorPWM2FlappingHZ(Motorstatus_t *motorstatus_l,Motorstatus_t *motorstatus_r)
{
    u32 motor_PWM = motorstatus_l->PWM + motorstatus_r->PWM;
    float f_hz = motor_PWM * MOTOR_PWM2F_A;
    motorstatus_l->f_Hz = motorstatus_l->PWM * MOTOR_PWM2F_A;
    motorstatus_r->f_Hz = motorstatus_r->PWM * MOTOR_PWM2F_A;
    return f_hz;
}
/*

* Function: flappingHZ2Thrust

* Description:  根据扑翼频率计算升力大小

* Input:  1.平均扑翼频率 
          2. 舵机偏转角 
          3. pitch角

* Output: 平均升力

* Return: 平均升力 

* Others: T = wing_a*f^2;% unit : mN

*/
float flappingHZ2ThrustZ_E(const float f_hz,float servoangle,float pitchangle)
{
    return MOTOR_F2T_A * f_hz * f_hz * arm_cos_f32(servoangle*DEG2RAD)*arm_cos_f32(pitchangle*DEG2RAD);
}
/*

* Function: ServoPWM2Servoangle

* Description:  根据舵机的PWM计算舵机转动角度

* Input: 平均扑翼频率

* Output: 舵机转动角度

* Return: 舵机转动角度

* Others: T = wing_a*f^2;% unit : mN

*/
float ServoPWM2Servoangle(u32 servoPWM)
{
    if(servoPWM < 900)
        servoPWM = 900;
    else if(servoPWM > 2100)
        servoPWM = 2100;
    return servoPWM * SERVO_PWM2ANGLE_A + SERVO_PWM2ANGLE_B;
}

/*

* Function: TfApply

* Description: 计算变量经过传递函数后的结果

* Input: 1. 传递函数结构体tf

* Output: 最新的输出y

* Return: // 函数返回值的说明

* Others: 在这个函数里面更新了tf中的状态量

*/
float TfApply(Tf_t *tf,const float input)
{
    float32_t result_num = 0;
    float32_t result_dec = 0;
    float32_t y;
    for (u8 i = (tf->order-1); i > 0; i--)
        *(tf->x + i) = *(tf->x + i - 1);
    *(tf->x) = input;
    arm_dot_prod_f32(tf->x, tf->nump, tf->order, &result_num);
    arm_dot_prod_f32(tf->y, tf->decp, (tf->order-1), &result_dec);
    y = result_num - result_dec;
    for (u8 i = (tf->order-2); i > 0; i--)
        *(tf->y + i) = *(tf->y + i - 1);
    *(tf->y) = y;
    return y;
} 
/*

* Function: Fdz_coffe_cal

* Description: Fdz_E 是关于电机输入PWM（u）的函数，此函数的目的是求u前面的系数

* Input: 1. 飞行器的姿态 用到theta(Pitch)  
         2. 飞行器测量速度，XY轴速度数据由光流测得，Z轴由激光数据测得；
         3. 舵机角度

* Output: 返回 Fdz_E = b*u  中的 系数b

* Return: // 函数返回值的说明

* Others: 

*/
float Fdz_coffe_cal(const attitude_t *atti,velocity_t vel,float servoangle)
{
//当不假定Roll角为0时
// -u*(22.11*(cos(beta)*sin(theta) + cos(phi)*sin(beta)*cos(theta))*(0.0489*V_laserZ*cos(beta)*sin(theta) - 0.0489*V_flowX*cos(beta)*cos(theta) - 0.0489*V_flowY*sin(beta)*sin(phi) + 0.0489*V_laserZ*cos(phi)*sin(beta)*cos(theta) + 0.0489*V_flowX*cos(phi)*sin(beta)*sin(theta)) 
//...- 22.11*(sin(beta)*sin(theta) - 1.0*cos(beta)*cos(phi)*cos(theta))*(0.0198*V_flowX*sin(beta)*cos(theta) - 0.0198*V_flowY*cos(beta)*sin(phi) - 0.0198*V_laserZ*sin(beta)*sin(theta) + 0.0198*V_laserZ*cos(beta)*cos(phi)*cos(theta) + 0.0198*V_flowX*cos(beta)*cos(phi)*sin(theta)) + 1.081179*cos(theta)*sin(phi)*(V_flowY*cos(phi) + V_laserZ*cos(theta)*sin(phi) + V_flowX*sin(phi)*sin(theta)))

    // float psi   = atti->yaw * DEG2RAD;
    // float theta = atti->pitch * DEG2RAD;
    // float phi   = atti->roll * DEG2RAD;
    // float beta  = servoangle * DEG2RAD;
    // float Cb = arm_cos_f32(beta);
    // float Sphi = arm_sin_f32(phi);
    // float Spsi = arm_sin_f32(psi);
    // float Cphi = arm_cos_f32(phi);
    // float Cpsi = arm_cos_f32(psi);
    // float Ct = arm_cos_f32(theta);
    // float Sb = arm_sin_f32(beta);
    // float St = arm_sin_f32(theta);
    // float CbSt = Cb*St;
    // float SbSt = Sb*St;
    // float SbCt = Sb*Ct;
    // float CbCt = Cb*Ct;
    // float CtSt = Ct*St;
    // float CbCtCphi = Cphi*CbCt;
    // float CtSphi = Ct*Sphi;
    // float CtStCphi = Cphi*CtSt;

    // return -22.11f*(CbSt + SbCt * Cphi)*(0.0489f*vel.z*CbSt- 0.0489f*vel.x*CbCt - 0.0489f*vel.y*Sb*Sphi + 0.0489f*vel.z*Cphi*SbCt + 0.0489f*vel.x*Cphi*SbSt) -22.11f*(SbSt - CbCtCphi)*(0.0198f*vel.x*SbCt - 0.0198f*vel.y*Cb*Sphi - 0.0198f*vel.z*SbSt + 0.0198f*vel.z*CbCtCphi + 0.0198f*vel.x*CtStCphi) + 1.081179f*CtSphi*(vel.y*Cphi + vel.z*CtSphi + vel.x*Sphi*St);

//当假定Roll角为0时
//Fd_Z = u*(0.3217005*V_laserZ*cos(2.0*beta + 2.0*theta) - 0.7594785*V_laserZ + 0.3217005*V_flowX*sin(2.0*beta + 2.0*theta))
    float psi   = atti->yaw * DEG2RAD;
    float theta = atti->pitch * DEG2RAD;
    float phi   = atti->roll * DEG2RAD;
    float beta  = servoangle * DEG2RAD;

    return 0.3217f*vel.z*arm_cos_f32(2*beta + 2*theta) - 0.7595f*vel.z + 0.3217f*vel.x*arm_sin_f32(2*beta + 2*theta);
}

/*

* Function: Ffz_coffe_cal

* Description: Ffz_E 是关于电机输入PWM（u）的函数，此函数的目的是求u前面的系数

* Input: 1. 飞行器的姿态 用到theta(Pitch)  
         2. 飞行器测量速度，XY轴速度数据由光流测得，Z轴由激光数据测得；
         3. 舵机角度

* Output: 返回 Ffz_E = a*u^2  中的 系数a

* Return: // 函数返回值的说明

* Others: 

*/
float Ffz_coffe_cal(const attitude_t *atti,float servoangle)
{
    float theta = atti->pitch * DEG2RAD;
    float phi = atti->roll * DEG2RAD;
    float beta = servoangle * DEG2RAD;
    float Cb = arm_cos_f32(beta);
    float Cp = arm_cos_f32(phi);
    float Ct = arm_cos_f32(theta);
    float Sb = arm_sin_f32(beta);
    float St = arm_sin_f32(theta);
    return 440.0f*(Cb*Cp*Ct - Sb*St); 
}
/*
* Function: U_cal

* Description: Ffz_E 是关于电机输入PWM（u）的函数，此函数的目的是求u前面的系数

* Input: 1. 上面函数求得的参数a  
         2. 上面函数求得的参数b 
         3. ESO估计的扰动
         4. 极点配置得到的u0

* Output: 返回 U 的值

* Return: // 函数返回值的说明

* Others: 控制律设计
*/
float U_cal(const float a,const float b,const float disturb,const float u0)
{
    float result = 0;
    float temp = 0;
    if (a != 0.0f) {
        // temp = (u0  + 25.0f * b * b / (MASS * a) + G - disturb) * MASS /(100.0f * a);
        temp = (u0  + 25.0f * b * b / (MASS * a) + G - 0.5f * disturb) * MASS /(100.0f * a);
        if (temp <= 0)
            result = 0;
        else
            arm_sqrt_f32(temp, &result);
        return result - b / (2.0f * a);
    } else {
        if (b != 0.0f)
            return (u0 - 0.5f * disturb + G) * MASS / (100.0f *b);
        else
            return 0;
    }
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

