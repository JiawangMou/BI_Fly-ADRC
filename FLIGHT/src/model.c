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


arm_matrix_instance_f32 mat_model_C_44;
arm_matrix_instance_f32 mat_model_C_inv_44;


float32_t model_C[16] = {7.289f,0.000001f,-7.289f,0.000001f,0.000001f,6.0f,0.000001f,6.0f,0.000001f,-7.289f,0.000001f,7.289f,0,0,0,0};
// float32_t model_C[16] = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
float32_t model_C_inv[16] = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};




#define  MOTORTF_ORDER 2  // 这里的阶数是系统实际阶数+1
 
float32_t left_motortf_x[MOTORTF_ORDER] = {0};
float32_t left_motortf_y[MOTORTF_ORDER-1] = {0};
float32_t left_motortf_num[MOTORTF_ORDER] = {0,0.0601}; //Fs = 1000 这里的系数与采样率有关，一定注意！！
float32_t left_motortf_dec[MOTORTF_ORDER-1] = {-0.9399};//Fs = 1000 这里的系数与采样率有关，一定注意！！

#define  SERVOTF_ORDER 3  // 这里的阶数是系统实际阶数+1
float left_servotf_x[SERVOTF_ORDER] = {0};
float left_servotf_y[SERVOTF_ORDER-1] = {0};
float left_servotf_num[SERVOTF_ORDER] = {0,0.0186,0.0186};//Fs = 250 这里的系数与采样率有关，一定注意！！
float left_servotf_dec[SERVOTF_ORDER-1] = {-1.6455,0.6828};//Fs = 250 这里的系数与采样率有关，一定注意！！


float32_t right_motortf_x[MOTORTF_ORDER] = {0};
float32_t right_motortf_y[MOTORTF_ORDER-1] = {0};
float32_t right_motortf_num[MOTORTF_ORDER] = {0,0.0601}; //Fs = 1000 这里的系数与采样率有关，一定注意！！
float32_t right_motortf_dec[MOTORTF_ORDER-1] = {-0.9399};//Fs = 1000 这里的系数与采样率有关，一定注意！！

#define  SERVOTF_ORDER 3  // 这里的阶数是系统实际阶数+1
float right_servotf_x[SERVOTF_ORDER] = {0};
float right_servotf_y[SERVOTF_ORDER-1] = {0};
float right_servotf_num[SERVOTF_ORDER] = {0,0.0186,0.0186};//Fs = 250 这里的系数与采样率有关，一定注意！！
float right_servotf_dec[SERVOTF_ORDER-1] = {-1.6455,0.6828};//Fs = 250 这里的系数与采样率有关，一定注意！！
// // static arm_matrix_instance_f32 pSrc_servotf_x; 
// // static arm_matrix_instance_f32 pSrc_servotf_y; 
// // static arm_matrix_instance_f32 pSrc_servotf_num; 
// // static arm_matrix_instance_f32 pSrc_servotf_dec; 
Tf_t right_motortf;
Tf_t right_servotf;
Tf_t left_motortf;
Tf_t left_servotf;

control_Tf_t control_Tf;

/* Model initialize function */
void model_initialize(void)
{
//   //init DCMbe_arm  DCMeb_arm
//   DCMbe_arm.numCols = 3;
//   DCMbe_arm.numRows = 3;
//   DCMbe_arm.pData = DCMbe;

//   DCMeb_arm.numCols = 3;
//   DCMeb_arm.numRows = 3;
//   DCMeb_arm.pData = DCMeb;
//   /* Registration code */

//   /* external inputs */
//   DCMbe[0] = 1.0;
//   DCMbe[4] = 1.0;
//   DCMbe[8] = 1.0;

    right_motortf.order = MOTORTF_ORDER;
    right_motortf.decp  = right_motortf_dec;
    right_motortf.nump  = right_motortf_num;
    right_motortf.x     = right_motortf_x;
    right_motortf.y     = right_motortf_y;
    
    right_servotf.order = SERVOTF_ORDER;
    right_servotf.decp  = right_servotf_dec;
    right_servotf.nump  = right_servotf_num;
    right_servotf.x     = right_servotf_x;
    
    left_motortf.order = MOTORTF_ORDER;
    left_motortf.decp  = left_motortf_dec;
    left_motortf.nump  = left_motortf_num;
    left_motortf.x     = left_motortf_x;
    left_motortf.y     = left_motortf_y;

    left_servotf.order = SERVOTF_ORDER;
    left_servotf.decp  = left_servotf_dec;
    left_servotf.nump  = left_servotf_num;
    left_servotf.x     = left_servotf_x;
    left_servotf.y     = left_servotf_y;

    arm_mat_init_f32(&mat_model_C_44, 4, 4, model_C);
    arm_mat_init_f32(&mat_model_C_inv_44, 4, 4, model_C_inv);

    arm_mat_init_f32(&control_Tf.mat_Tao_Fz_41, 4, 1, control_Tf.Tao_Fz);
    arm_mat_init_f32(&control_Tf.mat_U_41, 4, 1, control_Tf.U);

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
//   model_Y.PWM_compensation = 0;
}

/*

* Function: motorPWM2FlappingHZ

* Description:  根据占空比计算扑翼频率

* Input: 两个电机的motorPWM

* Output: 两个电机的平均扑翼频率

* Return: 平均扑翼频率 HZ

* Others: f = motor_a * u   % unit : Hz

*/
// float motorPWM2FlappingHZ(Motorstatus_t *motorstatus_l,Motorstatus_t *motorstatus_r)
// {
//     u32 motor_PWM = motorstatus_l->PWM + motorstatus_r->PWM;
//     float f_hz = motor_PWM * MOTOR_PWM2F_A;
//     motorstatus_l->f_Hz = motorstatus_l->PWM * MOTOR_PWM2F_A;
//     motorstatus_r->f_Hz = motorstatus_r->PWM * MOTOR_PWM2F_A;
//     return f_hz;
// }
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
// float flappingHZ2ThrustZ_E(const float f_hz,float servoangle,float pitchangle)
// {
//     return MOTOR_F2T_A * f_hz * f_hz * arm_cos_f32(servoangle*DEG2RAD)*arm_cos_f32(pitchangle*DEG2RAD);
// }
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
float TfApply(Tf_t *tf,const float input, const float lowLimit, const float highLimit)
{
    float32_t result_num = 0;
    float32_t result_dec = 0;
    float32_t y;
    for (u8 i = (tf->order-1); i > 0; i--)
        *(tf->x + i) = *(tf->x + i - 1);
    *(tf->x) = input;
    arm_dot_prod_f32(tf->x, tf->nump, tf->order, &result_num);
    arm_dot_prod_f32(tf->y, tf->decp, (tf->order-1), &result_dec);
    y = constrainf(result_num - result_dec, lowLimit, highLimit);;
    for (u8 i = (tf->order-2); i > 0; i--)
        *(tf->y + i) = *(tf->y + i - 1);
    *(tf->y) = y;
    return y;
} 

void actuator_TfApply(control_t *control)
{
    control_Tf.actuator[T_l]    = TfApply(&left_motortf, control->actuator[T_l], 0.0f, 200.0f); // 200 对应于200mN
    control_Tf.actuator[T_r]    = TfApply(&right_motortf, control->actuator[T_r], 0.0f, 200.0f);
    control_Tf.actuator[beta_l] = TfApply(&left_servotf, control->actuator[beta_l], -0.9f, 0.9f); // 0.8726对应50°
    control_Tf.actuator[beta_r] = TfApply(&right_servotf, control->actuator[beta_r], -0.9f, 0.9f);
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
    return 420.0f*(Cb*Cp*Ct - Sb*St); 
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
// float U_cal(const float a,const float b,const float disturb,const float u0)
// {
//     float result = 0;
//     float temp = 0;
//     if (a != 0.0f) {
//         // temp = (u0  + 25.0f * b * b / (MASS * a) + G - disturb) * MASS /(100.0f * a);
//         temp = (u0  + 25.0f * b * b / (MASS * a) + G - 0.5f * disturb) * MASS /(100.0f * a);
//         if (temp <= 0)
//             result = 0;
//         else
//             arm_sqrt_f32(temp, &result);
//         return result - b / (2.0f * a);
//     } else {
//         if (b != 0.0f)
//             return (u0 - 0.5f * disturb + G) * MASS / (100.0f *b);
//         else
//             return 0;
//     }
// }

/*
U = C_inv * Tao_Fz
*/
arm_status U_cal(control_t *control, attitude_t *angle)
{
    float U_temp[4] = {0};
    arm_status status;
    arm_matrix_instance_f32 mat_U_temp_41;
    arm_mat_init_f32(&mat_U_temp_41, 4,1, U_temp);

    // C_coffe_cal(model_C, angle);

    // status = arm_mat_inverse_f32(&mat_model_C_44, &mat_model_C_inv_44);
//     if(status == ARM_MATH_SUCCESS ){
// //TODO: the version of DSP library is low, the function of arm_mat_vec_mult_f32 is not defined.
    inv_C_coffe_cal(model_C_inv,angle);
    arm_mat_mult_f32(&mat_model_C_inv_44, &control->mat_Tao_Fz_41, &mat_U_temp_41);
    arm_scale_f32 (U_temp, 0.01f, control->U, 4);  //unit change from g*cm/s^2 to mN
        // arm_mat_vec_mult_f32( &mat_model_C_inv_44, temp, u);
    // }	
    return status;
}

// void D_coffe_cal(float32_t *D, Axis3f *wb)
// {
//     *D = (Jzz * wb->y * wb->z - Jyy * wb->y * wb->z)/Jxx;
//     *(D+1) = (-Jzz * wb->x * wb->z + Jxx * wb->x * wb->z)/Jyy;
//     *(D+2) = ( Jyy * wb->x * wb->y - Jxx * wb->x * wb->y)/Jzz;
//     *(D+3) = -G;
// }

void C_coffe_cal(float32_t *C, Axis3f *angle_R)
{
    float Cr = arm_cos_f32(angle_R->x);
    float Cp = arm_cos_f32(angle_R->y);
    float Sp = arm_sin_f32(angle_R->y);

    *C        = 7.289f;
    *(C + 1)  = 0.000001f;
    *(C + 2)  = -7.289f;
    *(C + 3)  = 0.000001f;
    *(C + 4)  = 0.000001f;
    *(C + 5)  = 6.0f;
    *(C + 6)  = 0.000001f;
    *(C + 7)  = 6.0f;
    *(C + 8)  = 0.000001f;
    *(C + 9)  = -7.289f;
    *(C + 10) = 0.000001f;
    *(C + 11) = 7.289f;
    *(C + 12) = Cr * Cp ;
    *(C + 13) = -Sp;
    *(C + 14) = Cr * Cp;
    *(C + 15) = -Sp;
}

void inv_C_coffe_cal(float32_t *inv_C, attitude_t *angle_R)
{
    float Cr = arm_cos_f32(angle_R->roll);
    float Cp = arm_cos_f32(angle_R->pitch);
    float Sp = arm_sin_f32(angle_R->pitch);
    float CrCp = Cr * Cp;

    float temp_1 = 0.08333f * Sp / CrCp;
    float temp_2 = 0.5f * CrCp;

    *inv_C        = 0.0686f;
    *(inv_C + 1)  = temp_1;
    *(inv_C + 2)  = 0.0f;
    *(inv_C + 3)  = temp_2;
    *(inv_C + 4)  = 0.0f;
    *(inv_C + 5)  = 0.08333f;
    *(inv_C + 6)  = -0.0686f;
    *(inv_C + 7)  = 0.0f;
    *(inv_C + 8)  = -0.0686f;
    *(inv_C + 9)  = temp_1;
    *(inv_C + 10) = 0.0f;
    *(inv_C + 11) = temp_2;
    *(inv_C + 12) = 0.0f ;
    *(inv_C + 13) = 0.08333f;
    *(inv_C + 14) = 0.0686f;
    *(inv_C + 15) = 0.0f;
}

void control_allocation(control_t *control)
{
    arm_sqrt_f32(((control->U[0] * control->U[0]) + (control->U[1] * control->U[1])), &(control->actuator[T_l]));
    arm_sqrt_f32(((control->U[2] * control->U[2]) + (control->U[3] * control->U[3])), &(control->actuator[T_r]));
    control->actuator[beta_l] = atan2_approx(control->U[1], control->U[0]);
    control->actuator[beta_r] = atan2_approx(control->U[3], control->U[2]);

    control->actuator[T_l] = constrainf(control->actuator[T_l], 0.0f, 200.0f);
    control->actuator[T_r] = constrainf(control->actuator[T_r], 0.0f, 200.0f); //200 对应于200mN
    control->actuator[beta_l] = constrainf(control->actuator[beta_l], -0.9f, 0.9f);  //0.8726对应50°
    control->actuator[beta_r] = constrainf(control->actuator[beta_r], -0.9f, 0.9f);  //0.8726对应50°
}

void control_allocation_inv(Axis3f *angle)
{
    control_Tf.U[0] = control_Tf.actuator[T_l] * arm_cos_f32(control_Tf.actuator[beta_l]);
    control_Tf.U[1] = control_Tf.actuator[T_l] * arm_sin_f32(control_Tf.actuator[beta_l]);
    control_Tf.U[2] = control_Tf.actuator[T_r] * arm_cos_f32(control_Tf.actuator[beta_r]);
    control_Tf.U[3] = control_Tf.actuator[T_r] * arm_sin_f32(control_Tf.actuator[beta_r]);

    C_coffe_cal(model_C, angle);
    arm_mat_mult_f32(&mat_model_C_44, &control_Tf.mat_U_41 , &control_Tf.mat_Tao_Fz_41);
    arm_scale_f32 (control_Tf.Tao_Fz, 100.0f, control_Tf.Tao_Fz, 4);  //unit change from mN to g*cm/s^2
}

void actuator2PWM(control_t *control, actuatorStatus_t *actuatorStatus)
{
    Thrust2motorPWM(&actuatorStatus->motor_l,control->actuator[T_l]);
    Thrust2motorPWM(&actuatorStatus->motor_r,control->actuator[T_r]);
    ServoAngle2ServoPWM(&actuatorStatus->servo_l, &actuatorStatus->servo_r, control->actuator);
}

/*

* Function: Thrust2motorPWM

* Description:  根据占空比计算扑翼频率

* Input: 升力大小 mN

* Return: 平均扑翼频率 HZ

* Others: T = ModelParam_wingCr * f^2 + ModelParam_wingCb 单位：mN(10^2 g*cm/s^2)
          param.ModelParam_wingCr = 0.3543;
          param.ModelParam_wingCb = -2.027;
          f = ModelParam_motorCr * PWM + ModelParam_motorCb;
          ModelParam_motorCr = 0.0003685;    %控制器输入PWM-扑翼频率曲线斜率
          ModelParam_motorCb = 1.43;    %控制器输入PWM-扑翼频率曲线常数项

*/
void Thrust2motorPWM(Motorstatus_t *motorstatus,float32_t u )
{
    arm_sqrt_f32(u / WINGCR, &(motorstatus->f_Hz));
    motorstatus->PWM = motorstatus->f_Hz  / MOTORCR;
}
/*

* Function: ServoAngle2ServoPWM

* Description:  根据舵机角度计算舵机控制

* Input:  1. 两个舵机角度 

* Output: 舵机控制量

* Others: T = wing_a*f^2;% unit : mN

*/
void ServoAngle2ServoPWM(Servostatus_t *servo_l,Servostatus_t *servo_r, float32_t *u )
{
    float servo_l_initpos = getservoinitpos_configParam(PWM_LEFT);
    float servo_r_initpos = getservoinitpos_configParam(PWM_RIGHT);
    servo_l->angle = *(u+2) * RAD2DEG;
    servo_r->angle = -(*(u+3) * RAD2DEG);

    // servo_l->PWM = ((servo_l->angle + 125) *12 - servo_l_initpos) *2 * 32767 / 1200;
    // servo_l->PWM = ((servo_l->angle + 125) *12 - servo_l_initpos) * 54.6166f;
    // servo_r->PWM = ((servo_r->angle + 125) *12 - servo_r_initpos) * 54.6166f;

    servo_l->PWM = servo_l->angle * 12 + servo_l_initpos;
    servo_r->PWM = servo_r->angle * 12 + servo_r_initpos;
}
/*
 * File trailer for generated code.
 * [EOF]
 */

