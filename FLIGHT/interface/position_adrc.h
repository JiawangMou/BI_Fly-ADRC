#ifndef _POSITION_ADRC_H_
#define _POSITION_ADRC_H_

#include "ADRC.h"
#include "stabilizer_types.h"

#define POSZ_TD_RATE             RATE_500_HZ
#define POSZ_TD_DT               (1.0/POSZ_TD_RATE)

#define POSZ_LESO_RATE           RATE_1000_HZ
#define POSZ_LESO_DT             (1.0/POSZ_LESO_RATE)

extern tdObject_t posZ_TD;
// nlsefObject_t posZ_nlsef;
extern lesoObject_3rd_t posZ_LESO;

void  pos_adrc_init(adrcInit_t *param);
void  pos_adrc_reset(adrcObject_t* adrcobject);
float adrc_VelControl(PidObject* pid,setpoint_t *setpoint);
float adrc_PosControl(PidObject* pid,setpoint_t *setpoint);
void  posZ_adrc_writeToConfigParam(void);
void  posZ_td_states_set(const float x1,const float x2);
void  posZ_transient_process_update(setpoint_t *setpoint);
void  posZ_state_estimate(sensorData_t* sensorData, state_t* state, float u);
#endif
