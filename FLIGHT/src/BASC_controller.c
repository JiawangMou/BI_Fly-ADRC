#include "config_param.h"
#include "sensors_types.h"
#include "attitude_adrc.h"
#include "position_adrc.h"
#include "maths.h"
#include "arm_math.h"
#include "ADRC.h"
#include "model.h"
#include "BASC_controller.h"

// Mass * (G + 200 ) = 29 * 1180 = 34220
#define FZMAX 34220


arm_matrix_instance_f32 mat_A1;
arm_matrix_instance_f32 mat_A2;
arm_matrix_instance_f32 mat_A3;
arm_matrix_instance_f32 mat_J_hat;
arm_matrix_instance_f32 mat_Tao0_hat;
arm_matrix_instance_f32 mat_J_gamma_33;
arm_matrix_instance_f32 mat_Tao0_gamma;

arm_matrix_instance_f32 mat_angle;
arm_matrix_instance_f32 mat_desiredangle;
arm_matrix_instance_f32 mat_x2;

arm_matrix_instance_f32 mat_delta1;
arm_matrix_instance_f32 mat_delta2;
arm_matrix_instance_f32 mat_x2d; 
arm_matrix_instance_f32 mat_delta1_dot;

arm_matrix_instance_f32 mat_A1_delta1;
arm_matrix_instance_f32 mat_A1_delta1dot;
arm_matrix_instance_f32 mat_J_hat_x2d_dot;

arm_matrix_instance_f32 mat_A3_delta1;
arm_matrix_instance_f32 mat_A2_delta2;
arm_matrix_instance_f32 mat_A1_delta1_dot;

arm_matrix_instance_f32 mat_x2d_dot;

arm_matrix_instance_f32 mat_Y_J_T_33;
arm_matrix_instance_f32 mat_Y_tao0_T;
float temp_31[3];
arm_matrix_instance_f32 mat_temp_31;
float J_dot[3];
arm_matrix_instance_f32 mat_J_dot_31;
float J_hat[3];
arm_matrix_instance_f32 mat_J_hat_31;


float Tao0_dot[3];
arm_matrix_instance_f32 mat_Tao0_dot_31;



float td_x1[3]= {0};
float td_x2[3]= {0};
float td_x3[3]= {0};

float wb_rad[3] = {0};
float angle_rad[3] = {0};
float desiredangle_rad[3] = {0};

arm_matrix_instance_f32 mat_td_x1;
arm_matrix_instance_f32 mat_td_x2;
arm_matrix_instance_f32 mat_td_x3;

BASC_Attitude_Object BASCAtti;
BASC_Pos_Object BASCPos;

float A1_delta1[3] = {0};
// float A3_delta1[3] = {0};
// float A2_delta2[3] = {0};
float delta1_dot[3] = {0};
float A1_delta1_dot[3] = {0};

float J_hat_x2d_dot[3] = {0};
float x2_J_hat_x2[3] = {0};


void BASCAttitudeInit(void)
{
    float temp[9] = {-1,0,0,0,-1,0,0,0,-1};
    memcpy(BASCAtti.Y_tao0_transpose, temp, sizeof(BASCAtti.Y_tao0_transpose));
    BASCAtti.J_hat[0] = J_hat[0] = 364;
    BASCAtti.J_hat[4] = J_hat[1] = 294;
    BASCAtti.J_hat[8] = J_hat[2] = 343;

    BASCAtti.A1[0] = configParam.BASCAtti_param.A1[0];
    BASCAtti.A1[4] = configParam.BASCAtti_param.A1[1];  
    BASCAtti.A1[8] = configParam.BASCAtti_param.A1[2];

    BASCAtti.A2[0] = configParam.BASCAtti_param.A2[0];
    BASCAtti.A2[4] = configParam.BASCAtti_param.A2[1];  
    BASCAtti.A2[8] = configParam.BASCAtti_param.A2[2];

    BASCAtti.A3[0] = configParam.BASCAtti_param.A3[0];
    BASCAtti.A3[4] = configParam.BASCAtti_param.A3[1];  
    BASCAtti.A3[8] = configParam.BASCAtti_param.A3[2];  

    BASCAtti.J_gamma[0] = configParam.BASCAtti_param.J_gamma[0];
    BASCAtti.J_gamma[4] = configParam.BASCAtti_param.J_gamma[1];  
    BASCAtti.J_gamma[8] = configParam.BASCAtti_param.J_gamma[2];

    BASCAtti.Tao0_gamma[0] = configParam.BASCAtti_param.Tao0_gamma[0];
    BASCAtti.Tao0_gamma[4] = configParam.BASCAtti_param.Tao0_gamma[1];  
    BASCAtti.Tao0_gamma[8] = configParam.BASCAtti_param.Tao0_gamma[2];      

    arm_mat_init_f32(&mat_J_hat, 3,3, BASCAtti.J_hat);
    arm_mat_init_f32(&mat_A1, 3,3, BASCAtti.A1);
    arm_mat_init_f32(&mat_A2, 3,3, BASCAtti.A2);
    arm_mat_init_f32(&mat_A3, 3,3, BASCAtti.A3);
    arm_mat_init_f32(&mat_Tao0_hat, 3,3, BASCAtti.Tao0_hat);
    arm_mat_init_f32(&mat_J_gamma_33, 3,3, BASCAtti.J_gamma);
    arm_mat_init_f32(&mat_Tao0_gamma, 3,3, BASCAtti.Tao0_gamma);
    
    arm_mat_init_f32(&mat_delta1, 3,1, BASCAtti.delta1);
    arm_mat_init_f32(&mat_delta2, 3,1, BASCAtti.delta2);
    arm_mat_init_f32(&mat_x2d, 3,1, BASCAtti.x2d);

    arm_mat_init_f32(&mat_delta1_dot, 3,1, delta1_dot);
    arm_mat_init_f32(&mat_J_hat_x2d_dot, 3,1, J_hat_x2d_dot);

    arm_mat_init_f32(&mat_A1_delta1, 3,1, A1_delta1);
    arm_mat_init_f32(&mat_A3_delta1, 3,1, BASCAtti.A3_delta1);
    arm_mat_init_f32(&mat_A2_delta2, 3,1, BASCAtti.A2_delta2);
    arm_mat_init_f32(&mat_A1_delta1_dot, 3,1, A1_delta1_dot);

    arm_mat_init_f32(&mat_td_x1, 3,1, td_x1);
    arm_mat_init_f32(&mat_td_x2, 3,1, td_x2);
    arm_mat_init_f32(&mat_td_x3, 3,1, td_x3);

    arm_mat_init_f32(&mat_angle, 3,1, angle_rad);
    arm_mat_init_f32(&mat_x2, 3,1, wb_rad);
    arm_mat_init_f32(&mat_desiredangle, 3,1, desiredangle_rad);

    arm_mat_init_f32(&mat_Y_J_T_33, 3,3, BASCAtti.Y_J_transpose);
    arm_mat_init_f32(&mat_Y_tao0_T, 3,3, BASCAtti.Y_tao0_transpose);

    arm_mat_init_f32(&mat_temp_31, 3,1, temp_31);
    arm_mat_init_f32(&mat_J_dot_31, 3,1, J_dot);
    arm_mat_init_f32(&mat_J_hat_31, 3,1, J_hat);
    arm_mat_init_f32(&mat_Tao0_dot_31, 3,1, Tao0_dot);
    BASCAtti.AL_Dt = RATE_ADAPITVE_DT;
}

void BASCPositionInit(void)
{
    BASCPos.A1 = configParam.BASCPos_param.A1;
    BASCPos.A2 = configParam.BASCPos_param.A2;
    BASCPos.A3 = configParam.BASCPos_param.A3;

    BASCPos.M_gamma = 1.0f / configParam.BASCPos_param.M_gamma;
    BASCPos.m_hat = MASS;
}


void Torque_Cal(control_t* control,Axis3f *Wb, attitude_t *actualAngle)
{
    memcpy(wb_rad, Wb->axis, sizeof(float)*3);
    memcpy(angle_rad, actualAngle->axis, sizeof(float)*3);

    td_x1[0] = Roll_td.x[0];
    td_x2[0] = Roll_td.x[1];
    td_x3[0] = Roll_td.x[2];

    td_x1[1] = Pitch_td.x[0];
    td_x2[1] = Pitch_td.x[1];
    td_x3[1] = Pitch_td.x[2];

    td_x1[2] = Yaw_td.x[0];
    td_x2[2] = Yaw_td.x[1];
    td_x3[2] = Yaw_td.x[2];

    arm_scale_f32(td_x1, DEG2RAD, td_x1, 3); 
    arm_scale_f32(td_x2, DEG2RAD, td_x2, 3); 
    arm_scale_f32(td_x3, DEG2RAD, td_x3, 3); 

    //delta1
    arm_mat_sub_f32(&mat_td_x1, &mat_angle, &mat_delta1);
    //A1_delta1
    arm_mat_mult_f32(&mat_A1, &mat_delta1, &mat_A1_delta1);
    //A3_delta1
    arm_mat_mult_f32(&mat_A3, &mat_delta1, &mat_A3_delta1);
    //x2d
    arm_mat_add_f32(&mat_A1_delta1, &mat_td_x2, &mat_x2d);
    //delta2
    arm_mat_sub_f32(&mat_x2d, &mat_x2, &mat_delta2);
    //A2_delta2
    arm_mat_mult_f32(&mat_A2, &mat_delta2, &mat_A2_delta2);
    //delta1_dot
    arm_mat_sub_f32(&mat_td_x2, &mat_x2, &mat_delta1_dot);
    //A1_delta1_dot
    arm_mat_mult_f32(&mat_A1, &mat_delta1_dot, &mat_A1_delta1_dot);
    //x2d_dot
    arm_mat_add_f32(&mat_A1_delta1_dot, &mat_td_x3, &mat_x2d_dot);
    //J_hat_x2d_dot
    arm_mat_mult_f32(&mat_J_hat, &mat_x2d_dot, &mat_J_hat_x2d_dot);
    //x2_J_hat_x2
    x2_J_hat_x2[0] = BASCAtti.J_hat[8]*wb_rad[1]*wb_rad[2] - BASCAtti.J_hat[4]*wb_rad[1]*wb_rad[2];
    x2_J_hat_x2[1] = BASCAtti.J_hat[0]*wb_rad[0]*wb_rad[2] - BASCAtti.J_hat[8]*wb_rad[0]*wb_rad[2];
    x2_J_hat_x2[2] = BASCAtti.J_hat[4]*wb_rad[0]*wb_rad[1] - BASCAtti.J_hat[0]*wb_rad[0]*wb_rad[1];

    for(int i=0; i<3; i++){
        BASCAtti.Torque[i] =  BASCAtti.A3_delta1[i] + BASCAtti.A2_delta2[i] + J_hat_x2d_dot[i] + x2_J_hat_x2[i] - BASCAtti.Tao0_hat[i];
        control->Tao_Fz[i] = BASCAtti.Torque[i];
    }
        
}

void Fz_Cal(float *thrust,const float posZ, const float velZ)
{
    float delta1_dot = 0;
    BASCPos.delta1 = posZ_TD.x1 - posZ;
    BASCPos.x2d = BASCPos.A2 * BASCPos.delta1 + posZ_TD.x2;
    BASCPos.delta2 = BASCPos.x2d - velZ;
    delta1_dot = posZ_TD.x2 - velZ;
    BASCPos.x2d_dot = BASCPos.A2 * delta1_dot + posZ_TD.fh;
    BASCPos.A3_delta2 = BASCPos.A3 * BASCPos.delta2;
    BASCPos.A1_delta1 = BASCPos.A1 * BASCPos.delta1;
    
    BASCPos.Fz = constrainf(BASCPos.A1_delta1 + BASCPos.m_hat * BASCPos.x2d_dot + BASCPos.m_hat * G + BASCPos.A3_delta2, 0.0f,FZMAX);
    *thrust = BASCPos.Fz;
}

//将遥控指令转换成Fz g*cm/s^2  Fzmax  = 34220 g*cm/s^2 
float Thrustcommand2Fz(float command)
{
    float Fz = command * FZMAX / 65000;
    BASCPos.Fz = (Fz >= FZMAX ? FZMAX : Fz);

    return BASCPos.Fz;
}

void resetBASCAttitudecontroller(void)
{
    BASCAtti.A1[0] = configParam.BASCAtti_param.A1[0];
    BASCAtti.A1[4] = configParam.BASCAtti_param.A1[1];  
    BASCAtti.A1[8] = configParam.BASCAtti_param.A1[2];

    BASCAtti.A2[0] = configParam.BASCAtti_param.A2[0];
    BASCAtti.A2[4] = configParam.BASCAtti_param.A2[1];  
    BASCAtti.A2[8] = configParam.BASCAtti_param.A2[2];

    BASCAtti.A3[0] = configParam.BASCAtti_param.A3[0];
    BASCAtti.A3[4] = configParam.BASCAtti_param.A3[1];  
    BASCAtti.A3[8] = configParam.BASCAtti_param.A3[2];  

    BASCAtti.J_gamma[0] = configParam.BASCAtti_param.J_gamma[0];
    BASCAtti.J_gamma[4] = configParam.BASCAtti_param.J_gamma[1];  
    BASCAtti.J_gamma[8] = configParam.BASCAtti_param.J_gamma[2];

    BASCAtti.Tao0_gamma[0] = configParam.BASCAtti_param.Tao0_gamma[0];
    BASCAtti.Tao0_gamma[4] = configParam.BASCAtti_param.Tao0_gamma[1];  
    BASCAtti.Tao0_gamma[8] = configParam.BASCAtti_param.Tao0_gamma[2];      
}

void resetBASCPositioncontroller(void)
{
    BASCPos.A1 = configParam.BASCPos_param.A1;
    BASCPos.A2 = configParam.BASCPos_param.A2;
    BASCPos.A3 = configParam.BASCPos_param.A3;

    BASCPos.M_gamma = configParam.BASCPos_param.M_gamma;
}

void Attitude_Adaptive_law(Axis3f *Wb)
{
    memcpy(wb_rad, Wb->axis, sizeof(float)*3);

    BASCAtti.Y_J_transpose[0]  = BASCAtti.x2d_dot[0];
    BASCAtti.Y_J_transpose[1]  = Wb->axis[0] * Wb->axis[2];
    BASCAtti.Y_J_transpose[2]  = -Wb->axis[0] * Wb->axis[1];
    BASCAtti.Y_J_transpose[3]  = -Wb->axis[1] * Wb->axis[2];
    BASCAtti.Y_J_transpose[4]  = BASCAtti.x2d_dot[1];
    BASCAtti.Y_J_transpose[5]  = Wb->axis[0] * Wb->axis[1];
    BASCAtti.Y_J_transpose[6]  = Wb->axis[1] * Wb->axis[2];
    BASCAtti.Y_J_transpose[7]  = -Wb->axis[0] * Wb->axis[2];
    BASCAtti.Y_J_transpose[8]  = BASCAtti.x2d_dot[2];
    //Y_T*delta2
    arm_mat_mult_f32(&mat_Y_J_T_33, &mat_delta2, &mat_temp_31);
    arm_mat_mult_f32(&mat_J_gamma_33, &mat_temp_31, &mat_J_dot_31);

    arm_scale_f32(J_dot, RATE_ADAPITVE_DT, temp_31, 3);

    arm_add_f32(J_hat, temp_31, J_hat, 3);

    //Y_T*delta2
    arm_mat_mult_f32(&mat_Y_tao0_T, &mat_delta2, &mat_temp_31);
    arm_mat_mult_f32(&mat_Tao0_gamma, &mat_temp_31, &mat_Tao0_dot_31);
    //scale dt
    arm_scale_f32(Tao0_dot, RATE_ADAPITVE_DT, temp_31, 3);

    arm_add_f32(BASCAtti.Tao0_hat, temp_31, BASCAtti.Tao0_hat, 3);
    
    for(int i = 0; i < 3; i++){
        BASCAtti.Tao0_hat[i] = constrainf(BASCAtti.Tao0_hat[i], TAO0_HAT_MIN, TAO0_HAT_MAX);
        J_hat[i] = constrainf(J_hat[i], J_HAT_MIN, J_HAT_MAX);
        BASCAtti.J_hat[i*4] = J_hat[i];
    }
    
}


void Position_Adaptive_law(void)
{
    BASCPos.Y_m_transpose = BASCPos.x2d_dot + G;
    BASCPos.m_hat = BASCPos.m_hat + BASCPos.M_gamma * BASCPos.Y_m_transpose * BASCPos.delta2 * RATE_ADAPITVE_DT;

    BASCPos.m_hat = constrainf(BASCPos.m_hat, M_HAT_MIN, M_HAT_MAX);
}

void BASCwriteToConfigParam(void)
{
    //BASCAtti parameters
    configParam.BASCAtti_param.A1[0] = BASCAtti.A1[0];
    configParam.BASCAtti_param.A1[1] = BASCAtti.A1[4];  
    configParam.BASCAtti_param.A1[2] = BASCAtti.A1[8];
    configParam.BASCAtti_param.A2[0] = BASCAtti.A2[0];
    configParam.BASCAtti_param.A2[1] = BASCAtti.A2[4];  
    configParam.BASCAtti_param.A2[2] = BASCAtti.A2[8];
    configParam.BASCAtti_param.A3[0] = BASCAtti.A3[0];
    configParam.BASCAtti_param.A3[1] = BASCAtti.A3[4];  
    configParam.BASCAtti_param.A3[2] = BASCAtti.A3[8];  

    configParam.BASCAtti_param.J_gamma[0] = BASCAtti.J_gamma[0];
    configParam.BASCAtti_param.J_gamma[1] = BASCAtti.J_gamma[4];  
    configParam.BASCAtti_param.J_gamma[2] = BASCAtti.J_gamma[8];

    configParam.BASCAtti_param.Tao0_gamma[0] = BASCAtti.Tao0_gamma[0];
    configParam.BASCAtti_param.Tao0_gamma[1] = BASCAtti.Tao0_gamma[4];  
    configParam.BASCAtti_param.Tao0_gamma[2] = BASCAtti.Tao0_gamma[8];
    
    //BASCPos parameters
    configParam.BASCPos_param.A1 = BASCPos.A1;
    configParam.BASCPos_param.A2 = BASCPos.A2;
    configParam.BASCPos_param.A3 = BASCPos.A3;  

    configParam.BASCPos_param.M_gamma= BASCPos.M_gamma;
}



