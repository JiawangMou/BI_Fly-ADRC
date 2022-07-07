#ifndef _POSITION_ADRC_H_
#define _POSITION_ADRC_H_

#include "ADRC.h"
#include "stabilizer_types.h"
#include "stabilizer.h"


// #define POSZ_ADRC_RATE           POS_PID_RATE
// #define POSZ_ADRC_DT             (1.0f/POSZ_ADRC_RATE)

// #define VELZ_ADRC_RATE           VEL_PID_RATE
// #define VELZ_ADRC_DT             (1.0f/VELZ_ADRC_RATE)

#define POSZ_TD_RATE             RATE_1000_HZ
#define POSZ_TD_DT               (1.0f/POSZ_TD_RATE)
#define ATTITUDE_TD_RATE         RATE_1000_HZ
#define ATTITUDE_TD_DT           (1.0f/ATTITUDE_TD_RATE)

#define VELZ_LOOP_RATE           RATE_250_HZ
#define VELZ_LOOP_DT             (1.0f/VELZ_LOOP_RATE)

// #define VELZ_TD_RATE             RATE_500_HZ
// #define VELZ_TD_DT               (1.0f/VELZ_TD_RATE)

// #define VEL_ESO_RATE             RATE_1000_HZ
// #define VEL_ESO_DT               (1.0f/VEL_ESO_RATE)

// #define POSZ_LESO_RATE           RATE_1000_HZ
// #define POSZ_LESO_DT             (1.0/POSZ_LESO_RATE)

// #define VELZ_LESO_RATE          RATE_1000_HZ
// #define VELZ_LESO_DT            (1.0f/VELZ_LESO_RATE)



extern tdObject_t posZ_TD;
// extern tdObject_t velZ_TD;
// extern nlsefObject_t posZ_nlsef;
// extern nlsefObject_t velZ_nlsef;
// extern lesoObject_3rd_t posZ_LESO;
// extern lesoObject_2rd_t velZ_LESO;

// extern lesoObject_3rd_t posZ_LESO;

// void  posZ_adrc_init(adrcInit_t* param);
// void  velZ_adrc_init(adrcInit_t* param);
// void  pos_adrc_reset(void);
// void  vel_adrc_reset(void);
// float adrc_VelControl(const float x1, const float x2, setpoint_t* setpoint);
void adrc_VelControl(const float desired_vel, const float vel, float *ADRC_u0);
// float adrc_PosControl(float x1, float x2, setpoint_t* setpoint);
void  posZ_adrc_writeToConfigParam(void);
void  td_states_set(tdObject_t *td,const float x1,const float x2);
void  posZ_transient_process_update(setpoint_t* setpoint);
// void  velZ_transient_process_update(setpoint_t* setpoint);
// void  posZ_state_estimate(sensorData_t* sensorData, state_t* state, float u);
// void  velZ_ESO_estimate(control_t* control, state_t* state);
lesoObject_2rd_t getVelZ_leso(void);
nlsefObject_t getVelZ_nlsef(void);
float getPosZ_TD_x1(void);
void positionADRCinit(void);
#endif
