#ifndef _ATTITUDE_ADRC_H_
#define _ATTITUDE_ADRC_H_

#include "ADRC.h"

void adrc_init(adrcObject_t *adrcobject,adrcInit_t *param, float tdDt,float lesoDt,float nlsefDt);
void adrc_reset(adrcObject_t *adrcobject);
void ADRC_RateControl(adrcObject_t *adrcObject,float expect_ADRC,float feedback_ADRC);
void ADRC_AngleControl(adrcObject_t *adrcObject,float expect_ADRC,float feedback);

#endif
