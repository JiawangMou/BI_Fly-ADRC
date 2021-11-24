#include "ADRC.h"
#include "pid.h"
#include "config_param.h"
#include "position_adrc.h"




#define POSZ_U_LPF_CUTOFF_FREQ 80
tdObject_t posZ_TD;
// nlsefObject_t posZ_nlsef;
lesoObject_3rd_t posZ_LESO;

void pos_adrc_init(adrcInit_t *param)
{
    td_init(&posZ_TD,&param->td,POSZ_TD_DT);
    leso_3rd_init(&posZ_LESO, &param->leso,POSZ_LESO_DT);
    // nlsef_toc_init(&adrcobject->nlsef_TOC,&param->nlsef_TOC,nlsefDt);
    // nlsef_init(&adrcobject->nlsef,&param->nlsef,nlsefDt);
    lpf2pInit(&posZ_LESO.uLpf, POSZ_LESO_RATE, POSZ_U_LPF_CUTOFF_FREQ);
}
void pos_adrc_reset(adrcObject_t *adrcobject)
{
    // /*****安排过度过程*******/
    // fhan_Input->TD_input = 0;
    // fhan_Input->x1       = 0; //跟踪微分期状态量
    // fhan_Input->x2       = 0; //跟踪微分期状态量微分项

    // fhan_Input->fh = 0; //最速微分加速度跟踪量
    // /*****扩张状态观测器*******/
    // /******已系统输出y和输入u来跟踪估计系统状态和扰动*****/
    // fhan_Input->z1      = 0;
    // fhan_Input->z2      = 0;
    // fhan_Input->e       = 0; //系统状态误差
    // fhan_Input->y       = 0; //系统输出量
    // fhan_Input->fe      = 0;
    // fhan_Input->fe1     = 0;

    /**********系统状态误差反馈率*********/
    // fhan_Input->e0 = 0; //状态误差积分项
    // fhan_Input->e1 = 0; //状态偏差
    // fhan_Input->e2 = 0; //状态量微分项

    // adrcobject->nlsef_TOC.u0  = 0; //非线性组合系统输出
    // adrcobject->nlsef.u0  = 0; //非线性组合系统输出
    // adrcobject->u  = 0; //带扰动补偿后的输出
}
float adrc_VelControl( PidObject* pid,setpoint_t *setpoint)
{
    return pidUpdate(pid, setpoint->velocity.z - posZ_LESO.z2);			/*pid更新*/
}
float adrc_PosControl(PidObject* pid,setpoint_t *setpoint)
{
    return pidUpdate(pid, setpoint->position.z - posZ_LESO.z1);			/*pid更新*/
}
void posZ_adrc_writeToConfigParam(void)
{
    configParam.adrcPosZ.td.N0   = posZ_TD.N0;
    configParam.adrcPosZ.td.r    = posZ_TD.r;
    configParam.adrcPosZ.leso.b0 = posZ_LESO.b0;
    configParam.adrcPosZ.leso.w0 = posZ_LESO.w0;
}

void posZ_td_states_set(const float x1,const float x2)
{
    posZ_TD.x1 = x1; //跟踪微分期状态量更新
    posZ_TD.x2 = x2; //跟踪微分期状态量微分项更新
}

void posZ_transient_process_update(setpoint_t *setpoint)
{
    adrc_td(&posZ_TD, setpoint->pos_desired.z);
    setpoint->position.z = posZ_TD.x1;
    setpoint->velocity.z = posZ_TD.x2;
    setpoint->acc.z      = posZ_TD.fh;
}

void posZ_state_estimate(sensorData_t* sensorData, state_t* state, float u)
{
    adrc_leso_3rd(&posZ_LESO,sensorData->zrange.distance, u);
    // state->position.z = posZ_LESO.z1;
    // state->velocity.z = posZ_LESO.z2;    
}
