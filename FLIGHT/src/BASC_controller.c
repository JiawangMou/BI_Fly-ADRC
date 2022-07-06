#include "BASC_controller.h"
#include "config_param.h"
#include "arm_math.h"
#include "ADRC.h"

arm_matrix_instance_f32 mat_A1;
arm_matrix_instance_f32 mat_A2;
arm_matrix_instance_f32 mat_A3;
arm_matrix_instance_f32 mat_J_hat;
arm_matrix_instance_f32 mat_tao0_hat;
arm_matrix_instance_f32 mat_J_gamma;
arm_matrix_instance_f32 mat_Tao0_gamma;

arm_matrix_instance_f32 mat_angle;
arm_matrix_instance_f32 mat_desiredangle;
arm_matrix_instance_f32 mat_anglerate;

arm_matrix_instance_f32 mat_delta1;
arm_matrix_instance_f32 mat_delta2;
arm_matrix_instance_f32 mat_x2d;

arm_matrix_instance_f32 mat_A1_delta1;
arm_matrix_instance_f32 mat_A1_delta1dot;

arm_matrix_instance_f32 mat_td_x;


BASC_Attitude_Object BASCAtti;
BASC_Pos_Object BASCPos;

float A1_delta1[3] = {0};

void BASCAttitudeInit()
{
    BASCAtti.J_hat = {364,0,0,0,294,0,0,0,343};
    BASCAtti.tao0_hat = {0,0,0};
    BASCAtti.A1 = {0};
    BASCAtti.A2 = {0};
    BASCAtti.A3 = {0};
    BASCAtti.J_gamma = {0};
    BASCAtti.Tao0_gamma = {0};

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
    arm_mat_init_f32(&mat_x2d, 3,1, BASCAtti.delta2);

    arm_mat_init_f32(&mat_A1_delta1, 3,1, A1_delta1);
}

void td_mat()

void Torque_Cal(Axis3f *Wb, attitude_t *actualAngle, attitude_t *desiredAngle, tdObject_t *td)
{
    arm_mat_init_f32(&mat_angle, 3,1, actualAngle->axis);
    arm_mat_init_f32(&mat_anglerate, 3,1, Wb->axis);
    arm_mat_init_f32(&mat_desiredangle, 3,1, desiredAngle->axis);

    arm_mat_init_f32(&mat_td_x, 3,1, td->x);

    arm_mat_sub_f32(&mat_desiredangle, &mat_angle, &mat_delta1);
    arm_mat_sub_f32(&mat_A1, &mat_delta1, &mat_A1_delta1);

    arm_mat_add_f32(&mat_A1, &mat_delta1, &mat_A1_delta1);

}
