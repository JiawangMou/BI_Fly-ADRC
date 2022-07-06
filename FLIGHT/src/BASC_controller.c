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
arm_matrix_instance_f32 mat_x2;

arm_matrix_instance_f32 mat_delta1;
arm_matrix_instance_f32 mat_delta2;
arm_matrix_instance_f32 mat_x2d;

arm_matrix_instance_f32 mat_A1_delta1;
arm_matrix_instance_f32 mat_A1_delta1dot;

float td_x1[3]= {0};
float td_x2[3]= {0};
float td_x3[3]= {0};


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
float x2_J_hat_x2[3] = {0};s

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

}

void Torque_Cal(Axis3f *Wb, attitude_t *actualAngle, attitude_t *desiredAngle, tdObject_t *Roll_td, tdObject_t *Pitch_td, tdObject_t *Yaw_td)
{
    arm_mat_init_f32(&mat_angle, 3,1, actualAngle->axis);
    arm_mat_init_f32(&mat_x2, 3,1, Wb->axis);
    arm_mat_init_f32(&mat_desiredangle, 3,1, desiredAngle->axis);

    td_x1[0] = Roll_td->x[0];
    td_x2[0] = Roll_td->x[1];
    td_x3[0] = Roll_td->x[2];

    td_x1[1] = Pitch_td->x[0];
    td_x2[1] = Pitch_td->x[1];
    td_x3[1] = Pitch_td->x[2];

    td_x1[2] = Yaw_td->x[0];
    td_x2[2] = Yaw_td->x[1];
    td_x3[2] = Yaw_td->x[2];
    //delta1
    arm_mat_sub_f32(&mat_desiredangle, &mat_angle, &mat_delta1);
    //A1_delta1
    arm_mat_mult_f32(&mat_A1, &mat_delta1, &mat_A1_delta1);
    //A3_delta1
    arm_mat_mult_f32(&mat_A3, &mat_delta1, &mat_A3_delta1);
    //A2_delta2
    arm_mat_mult_f32(&mat_A2, &mat_delta2, &mat_A2_delta2);
    //x2d
    arm_mat_add_f32(&mat_A1_delta1, &mat_td_x2, &mat_x2d);
    //delta2
    arm_mat_sub_f32(&mat_x2d, &mat_x2, &mat_delta2);
    //delta_dot
    arm_mat_sub_f32(&mat_td_x2, &mat_x2, &mat_delta1_dot);
    //A1_delta1_dot
    arm_mat_mult_f32(&mat_A1, &mat_delta1_dot, &mat_A1_delta1_dot);
    //x2d_dot
    arm_mat_add_f32(&mat_A1_delta1_dot, &mat_td_x3, &mat_x2d_dot);
    //J_hat_x2d_dot
    arm_mat_mult_f32(&mat_J_hat, &mat_x2d_dot, &mat_J_hat_x2d_dot);
    //x2_J_hat_x2
    x2_J_hat_x2(0) = BASCAtti.J_hat(8)*Wb->y*Wb->z - BASCAtti.J_hat(4)*Wb->y*Wb->z;
    x2_J_hat_x2(1) = BASCAtti.J_hat(0)*Wb->x*Wb->z - BASCAtti.J_hat(8)*Wb->x*Wb->z;
    x2_J_hat_x2(2) = BASCAtti.J_hat(4)*Wb->x*Wb->y - BASCAtti.J_hat(0)*Wb->x*Wb->y;

    for(int i=0; i<3; i++)
        BASCAtti.Torque(i) =  A3_delta1(i) + A2_delta2(i) + J_hat_x2d_dot(i) + x2_J_hat_x2(i) - BASCAtti.Tao0(i);

}
