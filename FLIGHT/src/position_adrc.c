#include "ADRC.h"
#include "pid.h"
#include "config_param.h"
#include "position_adrc.h"
#include "model.h"
#include "maths.h"


#define POSZ_INTEGRAL 8.0f
#define POSZ_U_LPF_CUTOFF_FREQ 80

#define VELZ_INTEGRAL 120.0f

tdObject_t posZ_TD;


nlsefObject_t velZ_nlsef;
// lesoObject_3rd_t posZ_LESO;
// lesoObject_2rd_t velZ_LESO;

// static float pos_integral = 0;
// static float vel_integral = 0;

Tf_t u_tf;

#define  UTF_ORDER 2  // 这里的阶数是系统实际阶数+1
 
float32_t u_tf_x[UTF_ORDER] = {0};
float32_t u_tf_y[UTF_ORDER-1] = {0};
float32_t u_tf_num[UTF_ORDER] = {0,0.0601}; //Fs = 1000 这里的系数与采样率有关，一定注意！！
float32_t u_tf_dec[UTF_ORDER-1] = {-0.9399};//Fs = 1000 这里的系数与采样率有关，一定注意！！


// void posZ_adrc_init(adrcInit_t *param)
// {
//     td_init(&posZ_TD,&param->td,POSZ_TD_DT);
//     // leso_3rd_init(&posZ_LESO, &param->leso,POSZ_LESO_DT);
//     // nlsef_toc_init(&adrcobject->nlsef_TOC,&param->nlsef_TOC,nlsefDt);
//     // lpf2pInit(&posZ_LESO.uLpf, POSZ_LESO_RATE, POSZ_U_LPF_CUTOFF_FREQ);
// }

// void velZ_adrc_init(adrcInit_t *param)
// {

//     nlsef_init(&velZ_nlsef,&param->nlsef,VELZ_LOOP_DT);
//     // leso_init(&velZ_LESO, &param->leso,VELZ_LESO_DT);
//     // vel_integral = 0.0f;
//     // u_tf.order= UTF_ORDER;
//     // u_tf.decp = u_tf_dec;
//     // u_tf.nump = u_tf_num;
//     // u_tf.x    = u_tf_x;
//     // u_tf.y    = u_tf_y;
// }
// void pos_adrc_reset(void)
// {
//     posZ_adrc_init(&configParam.adrcPosZ);
// }
// void vel_adrc_reset(void)
// {
//     velZ_adrc_init(&configParam.adrcVelZ);
// }
// float adrc_VelControl(const float x1, const float x2,setpoint_t *setpoint)
// {
//     velZ_nlsef.e1 = velZ_TD.x1 - velZ_LESO.z1;
//     velZ_nlsef.e2 = velZ_TD.x2 - x2; 
//     vel_integral += velZ_nlsef.e1 * VELZ_ADRC_DT;
	
// 	//积分限幅
// 	if (vel_integral > velZ_nlsef.I_limit)
// 	{
// 		vel_integral = velZ_nlsef.I_limit;
// 	}
// 	else if (vel_integral < -velZ_nlsef.I_limit)
// 	{
// 		vel_integral = -velZ_nlsef.I_limit;
// 	}  
//     return  adrc_nlsef(&velZ_nlsef) + vel_integral * velZ_nlsef.beta_I;
// }
void adrc_VelControl(const float desired_vel, const float vel, float *ADRC_u0)
{
    velZ_nlsef.e1 = desired_vel - vel;
    velZ_nlsef.e2 = 0;
    *(ADRC_u0+3) = velZ_nlsef.beta_1 * velZ_nlsef.e1;
}

// float adrc_PosControl(float x1,float x2, setpoint_t *setpoint)
// {
//     posZ_nlsef.e1 = posZ_TD.x1 - x1;
//     posZ_nlsef.e2 = posZ_TD.x2 - x2;
//     pos_integral  += posZ_nlsef.e1 * posZ_nlsef.h;
	
// 	//积分限幅
// 	if (pos_integral > posZ_nlsef.I_limit)
// 	{
// 		pos_integral = posZ_nlsef.I_limit;
// 	}
// 	else if (pos_integral < -posZ_nlsef.I_limit)
// 	{
// 		pos_integral = -posZ_nlsef.I_limit;
// 	}
//     return  adrc_nlsef(&posZ_nlsef) + pos_integral * posZ_nlsef.beta_I;
//     // return posZ_TD.x2;
// }
void posZ_adrc_writeToConfigParam(void)
{
    configParam.adrcPosZ.td.N0        = posZ_TD.N0;
    configParam.adrcPosZ.td.r         = posZ_TD.r;

    // configParam.adrcVelZ.td.N0        = velZ_TD.N0;
    // configParam.adrcVelZ.td.r         = velZ_TD.r;

    // configParam.adrcPosZ.nlsef.alpha1  = posZ_nlsef.alpha1;
    // configParam.adrcPosZ.nlsef.alpha2  = posZ_nlsef.alpha2;
    // configParam.adrcPosZ.nlsef.beta_1  = posZ_nlsef.beta_1;
    // configParam.adrcPosZ.nlsef.beta_2  = posZ_nlsef.beta_2;
    // configParam.adrcPosZ.nlsef.beta_I  = posZ_nlsef.beta_I;
    // configParam.adrcPosZ.nlsef.zeta    = posZ_nlsef.zeta;
    // configParam.adrcPosZ.nlsef.N1      = posZ_nlsef.N1;
    // configParam.adrcPosZ.nlsef.I_limit = posZ_nlsef.I_limit;

    configParam.adrcVelZ.nlsef.alpha1  = velZ_nlsef.alpha1;
    configParam.adrcVelZ.nlsef.alpha2  = velZ_nlsef.alpha2;
    configParam.adrcVelZ.nlsef.beta_1  = velZ_nlsef.beta_1;
    configParam.adrcVelZ.nlsef.beta_2  = velZ_nlsef.beta_2;
    configParam.adrcVelZ.nlsef.beta_I  = velZ_nlsef.beta_I;
    configParam.adrcVelZ.nlsef.zeta    = velZ_nlsef.zeta;
    configParam.adrcVelZ.nlsef.N1      = velZ_nlsef.N1;
    configParam.adrcVelZ.nlsef.I_limit = velZ_nlsef.I_limit;

    // configParam.adrcVelZ.leso.w0 = velZ_LESO.w0;
}

void td_states_set(tdObject_t *td,const float x1,const float x2)
{
    td->x1 = x1; //跟踪微分期状态量更新
    td->x2 = x2; //跟踪微分期状态量微分项更新
}

void posZ_transient_process_update(setpoint_t *setpoint)
{
    adrc_td(&posZ_TD, setpoint->position.z);
}
// void velZ_transient_process_update(setpoint_t *setpoint)
// {
//     adrc_td(&velZ_TD, setpoint->velocity.z);
// }
// void posZ_state_estimate(sensorData_t* sensorData, state_t* state, float u)
// {
//     adrc_leso_3rd(&posZ_LESO,sensorData->zrange.distance, u);    
// }
// void velZ_ESO_estimate(control_t* control,state_t* state)
// {
//     float Beta_01 = 2 * velZ_LESO.w0;
//     float Beta_02 = velZ_LESO.w0 * velZ_LESO.w0;
//     float u = TfApply(&u_tf,control->thrust / 60000.0f);
//     float a = control->a;
//     float b = control->b;   
//     velZ_LESO.e = velZ_LESO.z1 - state->velocity.z;
//     if(control->thrust > 5.0f)   
//         velZ_LESO.z1 += (velZ_LESO.z2 - Beta_01 * velZ_LESO.e + (a*u*u + b*u )*100.0f/MASS - G ) * velZ_LESO.h;
//     else
//         velZ_LESO.z1 += (velZ_LESO.z2 - Beta_01 * velZ_LESO.e ) * velZ_LESO.h;
//     // adrcobject->z1 += (adrcobject->z2 - Beta_01 * e + adrcobject->b0 *  adrcobject->u) * adrcobject->h;
//     velZ_LESO.z2 += -Beta_02 * velZ_LESO.e * velZ_LESO.h;
//     velZ_LESO.disturb = constrainf(velZ_LESO.z2,-400.0f,400.0f);
// }

// lesoObject_2rd_t getVelZ_leso(void)
// {
//     return velZ_LESO;
// }

nlsefObject_t getVelZ_nlsef(void)
{
    return velZ_nlsef;
}

float getPosZ_TD_x1(void)
{
    return posZ_TD.x1;
}
void positionADRCinit(void)
{
    td_init(&posZ_TD,&configParam.adrcPosZ.td,POSZ_TD_DT);
    nlsef_init(&velZ_nlsef,&configParam.adrcVelZ.nlsef,VELZ_LOOP_DT);
}
