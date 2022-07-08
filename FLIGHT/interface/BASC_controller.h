#ifndef _BASC_CONTROLLER_H_
#define _BASC_CONTROLLER_H_

#include <stdint.h>
#include "sensors_types.h"
#include "stabilizer_types.h"

typedef struct
{
    float delta1[3];
    float delta2[3];
    float A1[9];
    float A2[9];
    float A3[9];
    float x2d[3];
    float x2d_dot[3];
    float disturb[3];
    float tao0_hat[3];
    float J_hat[9];
    float Y_transpose[18];
    float J_gamma[9];
    float Tao0_gamma[9];
    float Tao0[3];
    float Torque[3];
} BASC_Attitude_Object;

typedef struct
{
    float delta1;
    float delta2;
    float A1;
    float A2;
    float A3;
    float x2d;
    float x2d_dot;
    float disturb;
    float m_hat;
    float Y_transpose;
    float M_gamma;
    float Fz;
} BASC_Pos_Object;

extern BASC_Attitude_Object BASCAtti;
extern BASC_Pos_Object BASCPos;

void BASCAttitudeInit(void);

void Torque_Cal(control_t *control, Axis3f *Wb, attitude_t *actualAngle);
void Fz_Cal(control_t *control, const float posZ, const float velZ);
void Thrustcommand2Fz(control_t* control,float command);
// BASC_Attitude_Object getBASCAtti_controller(void);
// BASC_Pos_Object getBASCPos_controller(void);

#endif
