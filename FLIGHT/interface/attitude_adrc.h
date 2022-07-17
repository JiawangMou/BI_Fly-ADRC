#ifndef _ATTITUDE_ADRC_H_
#define _ATTITUDE_ADRC_H_

#include "ADRC.h"
#include "stabilizer_types.h"
#include "model.h"

#define ANGLE_TD_RATE             RATE_1000_HZ
#define ANGLE_TD_DT               (1.0f/ANGLE_TD_RATE)


#define AXES_ESO_RATE  RATE_500_HZ
#define AXES_ESO_DT    (1.0f/AXES_ESO_RATE)

extern tdObject_t Roll_td;
extern tdObject_t Pitch_td;
extern tdObject_t Yaw_td;

// #define RATE_LOOP_RATE RATE_500_HZ
// #define RATE_LOOP_DT   (1.0f/RATE_LOOP_RATE)

// void adrc_init(adrcObject_t *adrcobject,adrcInit_t *param, float tdDt,float lesoDt,float nlsefDt);
// void adrc_reset(adrcObject_t *adrcobject);
// void ADRC_RateControl(adrcObject_t *adrcObject,float expect_ADRC,float feedback_ADRC);
// void ADRC_AngleControl(adrcObject_t *adrcObject,float expect_ADRC,float feedback);
void attitudeADRCwriteToConfigParam(void);
// void attitudeRateADRC(Axis3f *actualRate, attitude_t *desiredRate, float32_t *ADRC_u0);
void attitudeADRCinit(void);
void attitudeTD(setpoint_t *setpoint);
void Axes_Attitude_ESO(control_Tf_t *control_Tf, Axis3f *wb);
void Axes_Attitude_ESO_init(float tdDt);
void Axes_Attitude_ESO_beta_update(void);

#endif
