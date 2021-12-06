#ifndef _POSITION_ADRC_H_
#define _POSITION_ADRC_H_

#include "ADRC.h"
#include "stabilizer_types.h"
#include "stabilizer.h"


#define POSZ_ADRC_RATE           POS_PID_RATE
#define POSZ_ADRC_DT             (1.0/POSZ_ADRC_RATE)

#define VELZ_ADRC_RATE           VEL_PID_RATE
#define VELZ_ADRC_DT             (1.0/VELZ_ADRC_RATE)

#define VEL_ESO_RATE             RATE_500_HZ
#define VEL_ESO_DT               (1.0/VEL_ESO_RATE)

// #define POSZ_LESO_RATE           RATE_1000_HZ
// #define POSZ_LESO_DT             (1.0/POSZ_LESO_RATE)

// #define VELZ_LESO_RATE          RATE_500_HZ
// #define VELZ_LESO_DT            (1.0/VELZ_LESO_RATE)


extern tdObject_t posZ_TD;
extern tdObject_t velZ_TD;
extern nlsefObject_t posZ_nlsef;
extern nlsefObject_t velZ_nlsef;
extern lesoObject_3rd_t posZ_LESO;
extern lesoObject_2rd_t velZ_LESO;

// extern lesoObject_3rd_t posZ_LESO;

void  posZ_adrc_init(adrcInit_t* param);
void  velZ_adrc_init(adrcInit_t* param);
void  pos_adrc_reset(void);
void  vel_adrc_reset(void);
float adrc_VelControl(const float x1, const float x2,setpoint_t* setpoint);
float adrc_PosControl(float x1, float x2,setpoint_t* setpoint);
void  posZ_adrc_writeToConfigParam(void);
void  posZ_td_states_set(const float x1, const float x2);
void  posZ_transient_process_update(setpoint_t* setpoint);
void  velZ_transient_process_update(setpoint_t* setpoint);
void  posZ_state_estimate(sensorData_t* sensorData, state_t* state, float u);
void velZ_ESO_estimate(float u,float x);
#endif
