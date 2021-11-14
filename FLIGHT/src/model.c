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
  { 0.0, 0.0, -0.2646 },               /* '<S1>/Product' */

  { 0.0, -0.07289, -0.015 },           /* '<S6>/Subtract' */

  { 0.0, 0.07289, -0.015 }             /* '<S6>/Subtract1' */
};

/* Constant parameters (default storage) */
const ConstP_model_T model_ConstP = {
  /* Expression: param.ModelParam_Rb_CoR
   * Referenced by: '<S1>/param.ModelParam_Rb_CoR'
   */
  { 0.0, 0.0, 0.06 }
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

tdObject_t Z_TD;
arm_matrix_instance_f32  DCMbe_arm;
arm_matrix_instance_f32  DCMeb_arm;
float DCMeb[9];

/* Model update function */
u16 MBD_update(float aZ_E_desired, velocity_t state_velE,Axis3f gyro)
{
  real_T rtb_Transpose[9];
  real_T rtb_Transpose1[9];
  real_T rtb_Transpose_0[9];
  real_T rtb_Cop_L_b[3];
  real_T rtb_Cop_L_b_i[3];
  real_T rtb_Cop_R_b[3];
  real_T rtb_Cop_R_b_g[3];
  real_T rtb_Sum_p[3];
  real_T rtb_Thrust_coeff_B_vector[3];
  real_T rtb_vel_B_vector[3];
  real_T tmp[3];
  real_T a;
  real_T denAccum;
  real_T denAccum_0;
  real_T rtb_Cop_L_b_j;
  real_T rtb_Gain1_idx_0;
  real_T rtb_Gain1_idx_1;
  real_T rtb_Sum_d_idx_2;
  real_T rtb_Transpose1_1;
  real_T rtb_Transpose1_f;
  real_T rtb_Transpose_2;
  real_T rtb_Transpose_o;
  real_T rtb_vel_B_vector_tmp;
  real_T rtb_vel_B_vector_tmp_0;
  int32_T i;
  motorPWM_t motorPWM;
//get DCMbe
  getDCMeb(DCMeb);
  arm_status result = arm_mat_inverse_f32 (&DCMeb_arm,&DCMbe_arm);
//get VelE 单位：state_velE cm -> m velE
    velE[0] = state_velE.x / 100.0f;
    velE[1] = state_velE.y / 100.0f;   
    velE[2] = state_velE.z / 100.0f;
//get servo command
    getMotorPWM(&motorPWM);
    model_U.angle_command[0] = constrainf( ServoPWM2angle(motorPWM.s_middle,PWM_MIDDLE),-50.0f,50.0f);
    model_U.angle_command[1] = constrainf( ServoPWM2angle(motorPWM.s_left,  PWM_LEFT  ),-50.0f,50.0f);
//get Wb
  for(int i=0;i< 3;i++)
    model_U.Wb[i]= gyro.axis[i]*DEG2RAD;
//get aZ_E_desired
  model_U.aZ_E_desired = aZ_E_desired;

   /* DiscreteTransferFcn: '<S1>/Discrete Transfer Fcn' incorporates:
   *  Inport: '<Root>/angle_command'
   */
  denAccum = (model_U.angle_command[0] - 1.6451487309864259f *
              model_DW.DiscreteTransferFcn_states[0]) - 0.682538626130998f *
    model_DW.DiscreteTransferFcn_states[1];
  rtb_Gain1_idx_0 = (0.009347473786143f * denAccum + 0.018694947572286f *
                     model_DW.DiscreteTransferFcn_states[0]) + 0.009347473786143f
    * model_DW.DiscreteTransferFcn_states[1];

  /* DiscreteTransferFcn: '<S1>/Discrete Transfer Fcn1' incorporates:
   *  Inport: '<Root>/angle_command'
   */
  denAccum_0 = (model_U.angle_command[1] - 1.6451487309864259f *
                model_DW.DiscreteTransferFcn1_states[0]) - 0.682538626130998f *
    model_DW.DiscreteTransferFcn1_states[1];
  rtb_Gain1_idx_1 = (0.009347473786143f * denAccum_0 + 0.018694947572286f *
                     model_DW.DiscreteTransferFcn1_states[0]) +
    0.009347473786143f * model_DW.DiscreteTransferFcn1_states[1];

  /* Saturate: '<S1>/Signal_Saturation_3' incorporates:
   *  DiscreteTransferFcn: '<S1>/Discrete Transfer Fcn'
   */
  if (rtb_Gain1_idx_0 > 50.0f) {
    rtb_Gain1_idx_0 = 50.0f;
  } else if (rtb_Gain1_idx_0 < -50.0f) {
    rtb_Gain1_idx_0 = -50.0f;
  }

  /* Gain: '<S2>/Gain1' */
  rtb_Gain1_idx_0 *= 0.017453292519943295f;

  /* Saturate: '<S1>/Signal_Saturation_3' incorporates:
   *  DiscreteTransferFcn: '<S1>/Discrete Transfer Fcn1'
   */
  if (rtb_Gain1_idx_1 > 50.0f) {
    rtb_Gain1_idx_1 = 50.0f;
  } else if (rtb_Gain1_idx_1 < -50.0f) {
    rtb_Gain1_idx_1 = -50.0f;
  }

  /* Gain: '<S2>/Gain1' */
  rtb_Gain1_idx_1 *= 0.017453292519943295f;

  /* Trigonometry: '<S20>/sincos' incorporates:
   *  SignalConversion generated from: '<S20>/sincos'
   */
  rtb_Cop_R_b_g[1] = cos(rtb_Gain1_idx_1);
  a = sin(rtb_Gain1_idx_1);

  /* Fcn: '<S20>/Fcn11' */
  rtb_Transpose[0] = rtb_Cop_R_b_g[1];

  /* Fcn: '<S20>/Fcn21' */
  rtb_Transpose[1] = 0.0f;

  /* Fcn: '<S20>/Fcn31' */
  rtb_Transpose[2] = a;

  /* Fcn: '<S20>/Fcn12' */
  rtb_Transpose[3] = 0.0f ;

  /* Fcn: '<S20>/Fcn22' */
  rtb_Transpose[4] = 1.0f;

  /* Fcn: '<S20>/Fcn32' */
  rtb_Transpose[5] = 0.0f;

  /* Fcn: '<S20>/Fcn13' */
  rtb_Transpose[6] = -a;

  /* Fcn: '<S20>/Fcn23' */
  rtb_Transpose[7] = 0.0f;

  /* Fcn: '<S20>/Fcn33' */
  rtb_Transpose[8] = rtb_Cop_R_b_g[1];

  /* Math: '<S8>/Transpose' */
  for (i = 0; i < 3; i++) {
    rtb_Transpose_0[3 * i] = rtb_Transpose[i];
    rtb_Transpose_0[3 * i + 1] = rtb_Transpose[i + 3];
    rtb_Transpose_0[3 * i + 2] = rtb_Transpose[i + 6];
  }

  memcpy(&rtb_Transpose[0], &rtb_Transpose_0[0], 9U * sizeof(real_T));

  /* End of Math: '<S8>/Transpose' */

  /* Trigonometry: '<S19>/sincos' incorporates:
   *  Gain: '<S8>/Gain1'
   */
  rtb_Cop_R_b[1] = cos(-rtb_Gain1_idx_0);
  a = sin(-rtb_Gain1_idx_0);

  /* Fcn: '<S19>/Fcn11' */
  rtb_Transpose1[0] = rtb_Cop_R_b[1];

  /* Fcn: '<S19>/Fcn21' */
  rtb_Transpose1[1] = 0.0f;

  /* Fcn: '<S19>/Fcn31' */
  rtb_Transpose1[2] = a;

  /* Fcn: '<S19>/Fcn12' */
  rtb_Transpose1[3] = 0.0f ;

  /* Fcn: '<S19>/Fcn22' */
  rtb_Transpose1[4] = 1.0f;

  /* Fcn: '<S19>/Fcn32' */
  rtb_Transpose1[5] = 0.0f;

  /* Fcn: '<S19>/Fcn13' */
  rtb_Transpose1[6] = -a;

  /* Fcn: '<S19>/Fcn23' */
  rtb_Transpose1[7] = 0.0f;

  /* Fcn: '<S19>/Fcn33' */
  rtb_Transpose1[8] = rtb_Cop_R_b[1];

  /* Math: '<S8>/Transpose1' */
  for (i = 0; i < 3; i++) {
    rtb_Transpose_0[3 * i] = rtb_Transpose1[i];
    rtb_Transpose_0[3 * i + 1] = rtb_Transpose1[i + 3];
    rtb_Transpose_0[3 * i + 2] = rtb_Transpose1[i + 6];
  }

  memcpy(&rtb_Transpose1[0], &rtb_Transpose_0[0], 9U * sizeof(real_T));

  /* End of Math: '<S8>/Transpose1' */

  /* SampleTimeMath: '<S18>/TSamp'
   *
   * About '<S18>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  rtb_Gain1_idx_1 *= 250.0f;

  /* SampleTimeMath: '<S17>/TSamp' incorporates:
   *  Gain: '<S8>/Gain1'
   *
   * About '<S17>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  rtb_Gain1_idx_0 = -rtb_Gain1_idx_0 * 250.0f;

  /* Product: '<S1>/Product5' incorporates:
   *  Constant: '<S1>/mass'
   *  DotProduct: '<S1>/Dot Product'
   *  Inport: '<Root>/aZ_E_desired'
   */
  rtb_Sum_d_idx_2 = model_U.aZ_E_desired * 0.027f;
  for (i = 0; i < 3; i++) {
    /* Product: '<S1>/Product3' incorporates:
     *  Math: '<S8>/Transpose'
     */
    rtb_Transpose_o = rtb_Transpose[i];

    /* Product: '<S1>/Product2' incorporates:
     *  Math: '<S8>/Transpose1'
     */
    rtb_Transpose1_f = rtb_Transpose1[i];
    rtb_Transpose1_1 = rtb_Transpose1_f * 0.0f;

    /* Product: '<S1>/Product3' */
    rtb_Transpose_2 = rtb_Transpose_o * 0.0f;

    /* Product: '<S6>/Product1' incorporates:
     *  Sum: '<S6>/Subtract'
     */
    a = rtb_Transpose1_f * model_ConstB.Subtract[0];

    /* Product: '<S6>/Product2' incorporates:
     *  Sum: '<S6>/Subtract1'
     */
    rtb_Cop_L_b_j = rtb_Transpose_o * model_ConstB.Subtract1[0];

    /* Product: '<S1>/Product3' incorporates:
     *  Math: '<S8>/Transpose'
     */
    rtb_Transpose_o = rtb_Transpose[i + 3];

    /* Product: '<S1>/Product2' incorporates:
     *  Math: '<S8>/Transpose1'
     */
    rtb_Transpose1_f = rtb_Transpose1[i + 3];
    rtb_Transpose1_1 += rtb_Transpose1_f * 0.0f;

    /* Product: '<S1>/Product3' */
    rtb_Transpose_2 += rtb_Transpose_o * 0.0f;

    /* Product: '<S6>/Product1' incorporates:
     *  Sum: '<S6>/Subtract'
     */
    a += rtb_Transpose1_f * model_ConstB.Subtract[1];

    /* Product: '<S6>/Product2' incorporates:
     *  Sum: '<S6>/Subtract1'
     */
    rtb_Cop_L_b_j += rtb_Transpose_o * model_ConstB.Subtract1[1];

    /* Product: '<S1>/Product6' incorporates:
     *  Inport: '<Root>/DCMbe'
     *  Product: '<S1>/Product4'
     *  Product: '<S1>/Product5'
     *  Sum: '<S1>/Add1'
     */
    rtb_vel_B_vector_tmp = DCMbe[i + 3];

    /* Product: '<S1>/Product3' incorporates:
     *  Math: '<S8>/Transpose'
     */
    rtb_Transpose_o = rtb_Transpose[i + 6];

    /* Product: '<S1>/Product2' incorporates:
     *  Math: '<S8>/Transpose1'
     */
    rtb_Transpose1_f = rtb_Transpose1[i + 6];

    /* Product: '<S6>/Product1' incorporates:
     *  Sum: '<S6>/Subtract'
     */
    a += rtb_Transpose1_f * model_ConstB.Subtract[2];

    /* Product: '<S6>/Product2' incorporates:
     *  Sum: '<S6>/Subtract1'
     */
    rtb_Cop_L_b_j += rtb_Transpose_o * model_ConstB.Subtract1[2];

    /* Product: '<S1>/Product6' incorporates:
     *  Inport: '<Root>/DCMbe'
     *  Product: '<S1>/Product4'
     *  Product: '<S1>/Product5'
     *  Sum: '<S1>/Add1'
     */
    rtb_vel_B_vector_tmp_0 = DCMbe[i + 6];

    /* Sum: '<S1>/Add8' incorporates:
     *  Product: '<S1>/Product2'
     *  Product: '<S1>/Product3'
     */
    rtb_Thrust_coeff_B_vector[i] = (rtb_Transpose1_1 + rtb_Transpose1_f) +
      (rtb_Transpose_2 + rtb_Transpose_o);

    /* Sum: '<S6>/Add' incorporates:
     *  Constant: '<S1>/param.ModelParam_Rb_CoR'
     */
    rtb_Cop_R_b[i] = a + model_ConstP.paramModelParam_Rb_CoR_Value[i];

    /* Sum: '<S6>/Add1' incorporates:
     *  Constant: '<S1>/param.ModelParam_Rb_CoR'
     */
    rtb_Cop_L_b_i[i] = rtb_Cop_L_b_j +
      model_ConstP.paramModelParam_Rb_CoR_Value[i];

    /* Gain: '<S3>/Gain1' incorporates:
     *  Inport: '<Root>/Wb'
     *  Sum: '<S23>/Sum'
     */
    rtb_Sum_p[i] = 0.017453292519943295f * model_U.Wb[i];

    /* Sum: '<S1>/Add1' incorporates:
     *  Inport: '<Root>/DCMbe'
     *  Product: '<S1>/Product'
     *  Product: '<S1>/Product4'
     *  Product: '<S1>/Product5'
     */
    tmp[i] = ((rtb_vel_B_vector_tmp * 0.0f + DCMbe[i] * 0.0f) +
              rtb_vel_B_vector_tmp_0 * rtb_Sum_d_idx_2) -
      (rtb_vel_B_vector_tmp_0 * model_ConstB.Product[2] + (rtb_vel_B_vector_tmp *
        model_ConstB.Product[1] + DCMbe[i] * model_ConstB.Product[0]));

    /* Product: '<S6>/Product1' */
    rtb_Cop_R_b_g[i] = a;

    /* Product: '<S6>/Product2' */
    rtb_Cop_L_b[i] = rtb_Cop_L_b_j;

    /* Product: '<S1>/Product6' incorporates:
     *  Inport: '<Root>/DCMbe'
     *  Inport: '<Root>/velE'
     */
    rtb_vel_B_vector[i] = rtb_vel_B_vector_tmp_0 * velE[2] +
      (rtb_vel_B_vector_tmp * velE[1] + DCMbe[i] * velE[0]);
  }

  /* Sum: '<S24>/Sum' incorporates:
   *  Product: '<S27>/i x j'
   *  Product: '<S28>/j x i'
   */
  rtb_Sum_d_idx_2 = rtb_Sum_p[0] * rtb_Cop_L_b_i[1] - rtb_Cop_L_b_i[0] *
    rtb_Sum_p[1];

  /* Sum: '<S23>/Sum' incorporates:
   *  Product: '<S25>/i x j'
   *  Product: '<S26>/j x i'
   */
  rtb_Transpose_o = rtb_Sum_p[0] * rtb_Cop_R_b[1];
  rtb_Transpose1_f = rtb_Cop_R_b[0] * rtb_Sum_p[1];

  /* MATLAB Function: '<S1>/solve for thrust' incorporates:
   *  Constant: '<S1>/Constant2'
   *  Constant: '<S1>/Constant3'
   *  Constant: '<S1>/param.ModelParam_Cdz'
   *  Product: '<S13>/i x j'
   *  Product: '<S14>/j x i'
   *  Product: '<S15>/i x j'
   *  Product: '<S16>/j x i'
   *  Sum: '<S10>/Add4'
   *  Sum: '<S10>/Add5'
   *  Sum: '<S11>/Sum'
   *  Sum: '<S12>/Sum'
   *  Sum: '<S17>/Diff'
   *  Sum: '<S18>/Diff'
   *  Sum: '<S1>/Add8'
   *  Sum: '<S23>/Sum'
   *  UnitDelay: '<S17>/UD'
   *  UnitDelay: '<S18>/UD'
   *
   * Block description for '<S17>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S18>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S17>/UD':
   *
   *  Store in Global RAM
   *
   * Block description for '<S18>/UD':
   *
   *  Store in Global RAM
   */
  a = rtb_Thrust_coeff_B_vector[2] * 0.0003543f;
  rtb_Sum_d_idx_2 = (((0.0f * rtb_Cop_L_b[1] - (rtb_Gain1_idx_1 -
    model_DW.UD_DSTATE) * rtb_Cop_L_b[0]) + (rtb_vel_B_vector[2] +
    rtb_Sum_d_idx_2)) + rtb_vel_B_vector[2]) * -0.00198f - ((((0.0f *
    rtb_Cop_R_b_g[1] - (rtb_Gain1_idx_0 - model_DW.UD_DSTATE_f) * rtb_Cop_R_b_g
    [0]) + rtb_vel_B_vector[2]) + (rtb_Transpose_o - rtb_Transpose1_f)) +
    rtb_vel_B_vector[2]) * 0.00198f;
  rtb_Sum_d_idx_2 = (sqrt(rtb_Sum_d_idx_2 * rtb_Sum_d_idx_2 -
    (rtb_Thrust_coeff_B_vector[2] * -0.002027f + -tmp[2]) * (4.0f * a)) +
                     -rtb_Sum_d_idx_2) / (2.0f * a);

  /* SampleTimeMath: '<S4>/TSamp' incorporates:
   *  MATLAB Function: '<S1>/solve for thrust'
   *
   * About '<S4>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  a = rtb_Sum_d_idx_2 * 250.0f;

  /* Sum: '<S1>/Add4' incorporates:
   *  Gain: '<S1>/Gain'
   *  MATLAB Function: '<S1>/solve for thrust'
   *  Sum: '<S4>/Diff'
   *  UnitDelay: '<S4>/UD'
   *
   * Block description for '<S4>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S4>/UD':
   *
   *  Store in Global RAM
   */
  rtb_Sum_d_idx_2 += (a - model_DW.UD_DSTATE_c) * 0.016129032258064516f;

  /* Saturate: '<S1>/Saturation5' */
  if (rtb_Sum_d_idx_2 > 25.580016f) {
    rtb_Sum_d_idx_2 = 25.580016f;
  } else if (rtb_Sum_d_idx_2 < 0.0f) {
    rtb_Sum_d_idx_2 = 0.0f;
  }

  /* End of Saturate: '<S1>/Saturation5' */

  /* MATLAB Function: '<S1>/flappingHz2PWM' */
  if (rtb_Sum_d_idx_2 < 1.5f) {
    /* Outport: '<Root>/PWM_compensation' */
    model_Y.PWM_compensation = 0.0f;
  } else {
    /* Outport: '<Root>/PWM_compensation' incorporates:
     *  Constant: '<S1>/ModelParam_motorCb'
     *  Constant: '<S1>/ModelParam_motorCr'
     */
    model_Y.PWM_compensation = (rtb_Sum_d_idx_2 - 1.43f) / 0.0003685f;
  }

  /* End of MATLAB Function: '<S1>/flappingHz2PWM' */

  /* Update for DiscreteTransferFcn: '<S1>/Discrete Transfer Fcn' */
  model_DW.DiscreteTransferFcn_states[1] = model_DW.DiscreteTransferFcn_states[0];
  model_DW.DiscreteTransferFcn_states[0] = denAccum;

  /* Update for DiscreteTransferFcn: '<S1>/Discrete Transfer Fcn1' */
  model_DW.DiscreteTransferFcn1_states[1] =
    model_DW.DiscreteTransferFcn1_states[0];
  model_DW.DiscreteTransferFcn1_states[0] = denAccum_0;

  /* Update for UnitDelay: '<S18>/UD'
   *
   * Block description for '<S18>/UD':
   *
   *  Store in Global RAM
   */
  model_DW.UD_DSTATE = rtb_Gain1_idx_1;

  /* Update for UnitDelay: '<S17>/UD'
   *
   * Block description for '<S17>/UD':
   *
   *  Store in Global RAM
   */
  model_DW.UD_DSTATE_f = rtb_Gain1_idx_0;

  /* Update for UnitDelay: '<S4>/UD'
   *
   * Block description for '<S4>/UD':
   *
   *  Store in Global RAM
   */
  model_DW.UD_DSTATE_c = a;
  return model_Y.PWM_compensation;
}

/* Model initialize function */
void model_initialize(float dt)
{

  td_init(&Z_TD,&configParam.Z_TDconfig, dt);
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
}

/* Model terminate function */
void model_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

