#ifndef _POSITION_ADRC_H_
#define _POSITION_ADRC_H_

#include "ADRC.h"
#include "stabilizer_types.h"

#define POSZ_TD_RATE             RATE_500_HZ
#define POSZ_TD_DT               (1.0/POSZ_TD_RATE)

#define POSZ_NLSEF_RATE          RATE_500_HZ
#define POSZ_NLSEF_DT            (1.0/POSZ_TD_RATE)

#define VELZ_NLSEF_RATE          RATE_500_HZ
#define VELZ_NLSEF_DT            (1.0/POSZ_TD_RATE)

#define POSZ_LESO_RATE           RATE_1000_HZ
#define POSZ_LESO_DT             (1.0/POSZ_LESO_RATE)

#define VELZ_LESO_RATE          RATE_500_HZ
#define VELZ_LESO_DT            (1.0/VELZ_LESO_RATE)


extern tdObject_t posZ_TD;
extern nlsefObject_t posZ_nlsef;
extern nlsefObject_t velZ_nlsef;
extern lesoObject_3rd_t posZ_LESO;
extern lesoObject_2rd_t velZ_LESO;

// extern lesoObject_3rd_t posZ_LESO;

void  posZ_adrc_init(adrcInit_t* param);
void  velZ_adrc_init(adrcInit_t *param);
void  pos_adrc_reset(adrcObject_t* adrcobject);
float adrc_VelControl(void);
float adrc_PosControl(PidObject* pid, float error);
void  posZ_adrc_writeToConfigParam(void);
void  posZ_td_states_set(const float x1, const float x2);
void  posZ_transient_process_update(setpoint_t* setpoint);
void  posZ_state_estimate(sensorData_t* sensorData, state_t* state, float u);
void  velZ_ESO_estimate(float input, float output);
#endif
