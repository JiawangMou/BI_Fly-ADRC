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
arm_matrix_instance_f32 mat_tao0_hat;
arm_matrix_instance_f32 mat_J_gamma;
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
float A3_delta1[3] = {0};
float A2_delta2[3] = {0};
float delta1_dot[3] = {0};
float A1_delta1_dot[3] = {0};

float J_hat_x2d_dot[3] = {0};
float x2_J_hat_x2[3] = {0};


void BASCAttitudeInit(void)
{
    BASCAtti.J_hat[0] = 364;
    BASCAtti.J_hat[4] = 294;
    BASCAtti.J_hat[8] = 343;

    BASCAtti.A1[0] = configParam.BASCAtti.A1[0];
    BASCAtti.A1[3] = configParam.BASCAtti.A1[1];  
    BASCAtti.A1[6] = configParam.BASCAtti.A1[2];

    BASCAtti.A2[0] = configParam.BASCAtti.A2[0];
    BASCAtti.A2[3] = configParam.BASCAtti.A2[1];  
    BASCAtti.A2[6] = configParam.BASCAtti.A2[2];

    BASCAtti.A3[0] = configParam.BASCAtti.A3[0];
    BASCAtti.A3[3] = configParam.BASCAtti.A3[1];  
    BASCAtti.A3[6] = configParam.BASCAtti.A3[2];  

    BASCAtti.J_gamma[0] = configParam.BASCAtti.J_gamma[0];
    BASCAtti.J_gamma[3] = configParam.BASCAtti.J_gamma[1];  
    BASCAtti.J_gamma[6] = configParam.BASCAtti.J_gamma[2];

    BASCAtti.Tao0_gamma[0] = configParam.BASCAtti.Tao0_gamma[0];
    BASCAtti.Tao0_gamma[3] = configParam.BASCAtti.Tao0_gamma[1];  
    BASCAtti.Tao0_gamma[6] = configParam.BASCAtti.Tao0_gamma[2];      

    arm_mat_init_f32(&mat_J_hat, 3,3, BASCAtti.J_hat);
    arm_mat_init_f32(&mat_A1, 3,3, BASCAtti.A1);
    arm_mat_init_f32(&mat_A2, 3,3, BASCAtti.A2);
    arm_mat_init_f32(&mat_A3, 3,3, BASCAtti.A3);
    arm_mat_init_f32(&mat_tao0_hat, 3,3, BASCAtti.tao0_hat);
    arm_mat_init_f32(&mat_J_gamma, 3,3, BASCAtti.J_gamma);
    arm_mat_init_f32(&mat_Tao0_gamma, 3,3, BASCAtti.Tao0_gamma);
    
    arm_mat_init_f32(&mat_delta1, 3,1, BASCAtti.delta1);
    arm_mat_init_f32(&mat_delta2, 3,1, BASCAtti.delta2);
    arm_mat_init_f32(&mat_x2d, 3,1, BASCAtti.x2d);

    arm_mat_init_f32(&mat_delta1_dot, 3,1, delta1_dot);
    arm_mat_init_f32(&mat_J_hat_x2d_dot, 3,1, J_hat_x2d_dot);

    arm_mat_init_f32(&mat_A1_delta1, 3,1, A1_delta1);
    arm_mat_init_f32(&mat_A3_delta1, 3,1, A3_delta1);
    arm_mat_init_f32(&mat_A2_delta2, 3,1, A2_delta2);
    arm_mat_init_f32(&mat_A1_delta1_dot, 3,1, A1_delta1_dot);

    arm_mat_init_f32(&mat_td_x1, 3,1, td_x1);
    arm_mat_init_f32(&mat_td_x2, 3,1, td_x2);
    arm_mat_init_f32(&mat_td_x3, 3,1, td_x3);

    arm_mat_init_f32(&mat_angle, 3,1, angle_rad);
    arm_mat_init_f32(&mat_x2, 3,1, wb_rad);
    arm_mat_init_f32(&mat_desiredangle, 3,1, desiredangle_rad);

}

void BASCPositionInit(void)
{
    BASCPos.A1 = configParam.BASCPos.A1;
    BASCPos.A2 = configParam.BASCPos.A2;
    BASCPos.A2 = configParam.BASCPos.A2;

    BASCPos.M_gamma = configParam.BASCPos.M_gamma;
    BASCPos.m_hat = MASS;
}


void Torque_Cal(control_t* control,Axis3f *Wb, attitude_t *actualAngle)
{
    arm_scale_f32(Wb->axis, RAD, wb_rad, 3); 
    arm_scale_f32(actualAngle->axis, RAD, angle_rad, 3); 
    arm_scale_f32(td_x1, RAD, desiredangle_rad, 3); 

    td_x1[0] = Roll_td.x[0];
    td_x2[0] = Roll_td.x[1];
    td_x3[0] = Roll_td.x[2];

    td_x1[1] = Pitch_td.x[0];
    td_x2[1] = Pitch_td.x[1];
    td_x3[1] = Pitch_td.x[2];

    td_x1[2] = Yaw_td.x[0];
    td_x2[2] = Yaw_td.x[1];
    td_x3[2] = Yaw_td.x[2];
    //delta1
    arm_mat_sub_f32(&mat_desiredangle, &mat_angle, &mat_delta1);
    //A1_delta1
    arm_mat_mult_f32(&mat_A1, &mat_delta1, &mat_A1_delta1);
    //A3_delta1
    arm_mat_mult_f32(&mat_A3, &mat_delta1, &mat_A3_delta1);
    //delta2
    arm_mat_sub_f32(&mat_x2d, &mat_x2, &mat_delta2);
    //A2_delta2
    arm_mat_mult_f32(&mat_A2, &mat_delta2, &mat_A2_delta2);
    //x2d
    arm_mat_add_f32(&mat_A1_delta1, &mat_td_x2, &mat_x2d);
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
        BASCAtti.Torque[i] =  A3_delta1[i] + A2_delta2[i] + J_hat_x2d_dot[i] + x2_J_hat_x2[i] - BASCAtti.Tao0[i];
        control->Tao_Fz[i] = BASCAtti.Torque[i];
    }
        
}

void Fz_Cal(control_t* control,const float posZ, const float velZ)
{
    float delta1_dot = 0;
    BASCPos.delta1 = posZ_TD.x1 - posZ;
    BASCPos.x2d = BASCPos.A2 * BASCPos.delta1 + posZ_TD.x2;
    BASCPos.delta2 = BASCPos.x2d - velZ;
    delta1_dot = posZ_TD.x2 - velZ;
    BASCPos.x2d_dot = BASCPos.A2 * delta1_dot + posZ_TD.fh;

    BASCPos.Y_transpose = BASCPos.x2d_dot + G;
    BASCPos.Fz = BASCPos.A1 * BASCPos.delta1 + BASCPos.m_hat * BASCPos.x2d_dot + BASCPos.m_hat * G + BASCPos.A3 * BASCPos.delta2;
    control->Tao_Fz[3] = BASCPos.Fz;
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
    BASCAtti.A1[0] = configParam.BASCAtti.A1[0];
    BASCAtti.A1[3] = configParam.BASCAtti.A1[1];  
    BASCAtti.A1[6] = configParam.BASCAtti.A1[2];

    BASCAtti.A2[0] = configParam.BASCAtti.A2[0];
    BASCAtti.A2[3] = configParam.BASCAtti.A2[1];  
    BASCAtti.A2[6] = configParam.BASCAtti.A2[2];

    BASCAtti.A3[0] = configParam.BASCAtti.A3[0];
    BASCAtti.A3[3] = configParam.BASCAtti.A3[1];  
    BASCAtti.A3[6] = configParam.BASCAtti.A3[2];  

    BASCAtti.J_gamma[0] = configParam.BASCAtti.J_gamma[0];
    BASCAtti.J_gamma[3] = configParam.BASCAtti.J_gamma[1];  
    BASCAtti.J_gamma[6] = configParam.BASCAtti.J_gamma[2];

    BASCAtti.Tao0_gamma[0] = configParam.BASCAtti.Tao0_gamma[0];
    BASCAtti.Tao0_gamma[3] = configParam.BASCAtti.Tao0_gamma[1];  
    BASCAtti.Tao0_gamma[6] = configParam.BASCAtti.Tao0_gamma[2];      
}

void resetBASCPositioncontroller(void)
{
    BASCPos.A1 = configParam.BASCPos.A1;
    BASCPos.A2 = configParam.BASCPos.A2;
    BASCPos.A2 = configParam.BASCPos.A2;

    BASCPos.M_gamma = configParam.BASCPos.M_gamma;
}

void Attitude_Adaptive_law(Axis3f *anglerate)
{
    Axis3f Wb = *anglerate
    BASCAtti.Y_transpose[0] = BASCAtti.x2d_dot[0];
    BASCAtti.Y_transpose[0] = BASCAtti.x2d_dot[0];
}