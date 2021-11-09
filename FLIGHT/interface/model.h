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

/* Model Code Variants */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

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
  const real_T Product[3];             /* '<S1>/Product' */
  const real_T Subtract[3];            /* '<S6>/Subtract' */
  const real_T Subtract1[3];           /* '<S6>/Subtract1' */
} ConstB_model_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: param.ModelParam_Rb_CoR
   * Referenced by: '<S1>/param.ModelParam_Rb_CoR'
   */
  real_T paramModelParam_Rb_CoR_Value[3];
} ConstP_model_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T angle_command[2];             /* '<Root>/angle_command' */
  real_T Wb[3];                        /* '<Root>/Wb' */
  real_T aZ_E_desired;                 /* '<Root>/aZ_E_desired' */
} ExtU_model_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  u16 PWM_compensation;             /* '<Root>/PWM_compensation' */
} ExtY_model_T;

/* Real-time Model Data Structure */
struct tag_RTM_model_T {
  const char_T * volatile errorStatus;
};

/* Block states (default storage) */
extern DW_model_T model_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_model_T model_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_model_T model_Y;
extern const ConstB_model_T model_ConstB;/* constant block i/o */

/* Constant parameters (default storage) */
extern const ConstP_model_T model_ConstP;

/* Model entry point functions */
extern void model_initialize(float dt);
extern u16 MBD_update(float setpoint_Z, velocity_t state_velE,Axis3f gyro);
extern void model_terminate(void);

/* Real-time Model object */
extern RT_MODEL_model_T *const model_M;

/* Exported data declaration */

/* Declaration for custom storage class: Global */
extern real_T DCMbe[9];                /* '<Root>/DCMbe' */
extern real_T velE[3];                 /* '<Root>/velE' */

extern tdObject_t Z_TD;
/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S4>/Data Type Duplicate' : Unused code path elimination
 * Block '<S1>/Scope' : Unused code path elimination
 * Block '<S17>/Data Type Duplicate' : Unused code path elimination
 * Block '<S18>/Data Type Duplicate' : Unused code path elimination
 * Block '<S10>/Gain1' : Unused code path elimination
 * Block '<S10>/Gain2' : Unused code path elimination
 * Block '<S10>/Gain3' : Unused code path elimination
 * Block '<S10>/Scope1' : Unused code path elimination
 * Block '<S10>/Scope2' : Unused code path elimination
 * Block '<S10>/Scope4' : Unused code path elimination
 * Block '<S1>/Reshape1' : Reshape block reduction
 * Block '<S1>/Reshape2' : Reshape block reduction
 * Block '<S6>/Reshape1' : Reshape block reduction
 * Block '<S6>/Reshape2' : Reshape block reduction
 * Block '<S6>/Reshape3' : Reshape block reduction
 * Block '<S21>/Reshape (9) to [3x3] column-major' : Reshape block reduction
 * Block '<S22>/Reshape (9) to [3x3] column-major' : Reshape block reduction
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'model'
 * '<S1>'   : 'model/MBD_control'
 * '<S2>'   : 'model/MBD_control/Degrees to Radians'
 * '<S3>'   : 'model/MBD_control/Degrees to Radians1'
 * '<S4>'   : 'model/MBD_control/Discrete Derivative'
 * '<S5>'   : 'model/MBD_control/Vcop introduced by servo'
 * '<S6>'   : 'model/MBD_control/coordinate of Cop'
 * '<S7>'   : 'model/MBD_control/flappingHz2PWM'
 * '<S8>'   : 'model/MBD_control/servoangle To DCM & W_servo'
 * '<S9>'   : 'model/MBD_control/solve for thrust'
 * '<S10>'  : 'model/MBD_control/velocity of Cop'
 * '<S11>'  : 'model/MBD_control/Vcop introduced by servo/3x3 Cross Product'
 * '<S12>'  : 'model/MBD_control/Vcop introduced by servo/3x3 Cross Product1'
 * '<S13>'  : 'model/MBD_control/Vcop introduced by servo/3x3 Cross Product/Subsystem'
 * '<S14>'  : 'model/MBD_control/Vcop introduced by servo/3x3 Cross Product/Subsystem1'
 * '<S15>'  : 'model/MBD_control/Vcop introduced by servo/3x3 Cross Product1/Subsystem'
 * '<S16>'  : 'model/MBD_control/Vcop introduced by servo/3x3 Cross Product1/Subsystem1'
 * '<S17>'  : 'model/MBD_control/servoangle To DCM & W_servo/Discrete Derivative'
 * '<S18>'  : 'model/MBD_control/servoangle To DCM & W_servo/Discrete Derivative1'
 * '<S19>'  : 'model/MBD_control/servoangle To DCM & W_servo/Rotation Angles to Direction Cosine Matrix'
 * '<S20>'  : 'model/MBD_control/servoangle To DCM & W_servo/Rotation Angles to Direction Cosine Matrix1'
 * '<S21>'  : 'model/MBD_control/servoangle To DCM & W_servo/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S22>'  : 'model/MBD_control/servoangle To DCM & W_servo/Rotation Angles to Direction Cosine Matrix1/Create 3x3 Matrix'
 * '<S23>'  : 'model/MBD_control/velocity of Cop/3x3 Cross Product'
 * '<S24>'  : 'model/MBD_control/velocity of Cop/3x3 Cross Product1'
 * '<S25>'  : 'model/MBD_control/velocity of Cop/3x3 Cross Product/Subsystem'
 * '<S26>'  : 'model/MBD_control/velocity of Cop/3x3 Cross Product/Subsystem1'
 * '<S27>'  : 'model/MBD_control/velocity of Cop/3x3 Cross Product1/Subsystem'
 * '<S28>'  : 'model/MBD_control/velocity of Cop/3x3 Cross Product1/Subsystem1'
 */
#endif                                 /* RTW_HEADER_model_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
