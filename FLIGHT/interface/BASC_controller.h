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
    float J_hat[9];
    float Y_J_transpose[9];
    float Y_tao0_transpose[9];
    float J_gamma[9];
    float Tao0_gamma[9];
    float Tao0_hat[3];
    float Torque[3];
    float AL_Dt;  //Adaptive Law dt
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
    float Y_m_transpose;
    float M_gamma;
    float Fz;
    float AL_Dt;  //Adaptive Law dt
} BASC_Pos_Object;

extern BASC_Attitude_Object BASCAtti;
extern BASC_Pos_Object BASCPos;

#define RATE_BASC_RATE			RATE_500_HZ //角速度环（内环）PID速率
#define RATE_BASC_DT			(1.0/RATE_PID_RATE)

#define RATE_ADAPITVE_RATE      RATE_1000_HZ
#define RATE_ADAPITVE_DT	    0.001f

#define M_HAT_MAX  40.0f
#define M_HAT_MIN  25.0f

#define J_HAT_MAX  500.0f
#define J_HAT_MIN  200.0f

#define TAO0_HAT_MAX  20000.0f
#define TAO0_HAT_MIN  -20000.0f

void BASCAttitudeInit(void);
void BASCPositionInit(void);

void Torque_Cal(control_t *control, Axis3f *Wb, attitude_t *actualAngle);
void Fz_Cal(control_t *control, const float posZ, const float velZ);
float Thrustcommand2Fz(float command);
void resetBASCAttitudecontroller(void);
void resetBASCPositioncontroller(void);
void Attitude_Adaptive_law(Axis3f *Wb);
void Position_Adaptive_law(void);
void BASCwriteToConfigParam(void);

// BASC_Attitude_Object getBASCAtti_controller(void);
// BASC_Pos_Object getBASCPos_controller(void);

#endif
