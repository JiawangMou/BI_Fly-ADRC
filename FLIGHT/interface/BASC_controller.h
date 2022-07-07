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
    float tao0_hat;
    float J_hat;
    float Y_transpose;
    float M_gamma;
} BASC_Pos_Object;

void BASCAttitudeInit(void);

void Torque_Cal(Axis3f *Wb, attitude_t *actualAngle);
void Fz_Cal(BASC_Pos_Object *BASC, float *Wb, float *Vb);



#endif
