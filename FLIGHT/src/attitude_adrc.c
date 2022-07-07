#include "ADRC.h"
#include "config_param.h"
#include "attitude_adrc.h"
#include "arm_math.h"
#include "axis.h"




#define U_LPF_CUTOFF_FREQ 50

tdObject_t Roll_td;
tdObject_t Pitch_td;
tdObject_t Yaw_td;

// adrcObject_t ADRCRatePitch;
// adrcObject_t ADRCRateRoll;
// adrcObject_t ADRCRateYaw;

// adrcObject_t ADRCAngleRoll;
// adrcObject_t ADRCAnglePitch;
// adrcObject_t ADRCAngleYaw;


// void adrc_init(adrcObject_t *adrcobject,adrcInit_t *param, float tdDt,float lesoDt,float nlsefDt)
// {
//     td_init(&adrcobject->td,&param->td,tdDt);
//     leso_init(&adrcobject->leso, &param->leso,lesoDt);
//     // nlsef_toc_init(&adrcobject->nlsef_TOC,&param->nlsef_TOC,nlsefDt);
//     nlsef_init(&adrcobject->nlsef,&param->nlsef,nlsefDt);
//     lpf2pInit(&adrcobject->leso.uLpf, 1.0f/lesoDt, U_LPF_CUTOFF_FREQ);
//     adrcobject->u = 0;
// }

// void adrc_reset(adrcObject_t *adrcobject)
// {

//     // /*****安排过度过程*******/
//     // fhan_Input->TD_input = 0;
//     // fhan_Input->x1       = 0; //跟踪微分期状态量
//     // fhan_Input->x2       = 0; //跟踪微分期状态量微分项

//     // fhan_Input->fh = 0; //最速微分加速度跟踪量
//     // /*****扩张状态观测器*******/
//     // /******已系统输出y和输入u来跟踪估计系统状态和扰动*****/
//     // fhan_Input->z1      = 0;
//     // fhan_Input->z2      = 0;
//     // fhan_Input->e       = 0; //系统状态误差
//     // fhan_Input->y       = 0; //系统输出量
//     // fhan_Input->fe      = 0;
//     // fhan_Input->fe1     = 0;

//     /**********系统状态误差反馈率*********/
//     // fhan_Input->e0 = 0; //状态误差积分项
//     // fhan_Input->e1 = 0; //状态偏差
//     // fhan_Input->e2 = 0; //状态量微分项

//     // adrcobject->nlsef_TOC.u0  = 0; //非线性组合系统输出
//     adrcobject->nlsef.u0  = 0; //非线性组合系统输出
//     adrcobject->u  = 0; //带扰动补偿后的输出

// }

// void ADRC_RateControl(adrcObject_t *adrcObject,float desired_rate,float rate)
// {
// 	//TD  这里TD由于刷新频率高不放在ADRC_RateControl中
// 	//LESO 这里LESO由于刷新频率高不放在ADRC_RateControl中
//     /********状态误差反馈率***/
//     // fhan_Input->e0 += fhan_Input->e1 * fhan_Input->h1; //状态积分项
//     // fhan_Input->e0 = Constrain_Float(fhan_Input->e0, -10.0f, 10.0f);

//     //nlsef_TOC
//     // adrcObject->nlsef_TOC.e1 = adrcObject->td.x1; //状态偏差项
//     // adrcObject->nlsef_TOC.e2 = adrcObject->td.x2; //状态微分项
//     //nlsef
//     adrcObject->nlsef.e1 = desired_rate - rate; //状态偏差项
//     adrcObject->nlsef.e2 = 0; //状态微分项
//     /********NLSEF*******/
//     //nlsef_TOC
//     // adrcObject->nlsef_TOC.u0 = adrc_TOCnlsef(&adrcObject->nlsef_TOC);
//     //nlsef
//     adrcObject->nlsef.u0 = adrc_nlsef(&adrcObject->nlsef);
//     /**********扰动补偿*******/
//     // adrcObject->u = (adrcObject->nlsef.u0 - 0.5f * adrcObject->leso.z2) / adrcObject->leso.b0;
//     // // adrcObject->u = adrcObject->nlsef.u0 / adrcObject->leso.b0;
//     // //实际输出
//     // adrcObject->u = Constrain_Float(adrcObject->u, -32767, 32767);
// }

// void ADRC_AngleControl(adrcObject_t *adrcObject,float expect_ADRC,float feedback)
// {
// 	adrcObject->td.x1 = feedback;
// 	adrc_td( &adrcObject->td, expect_ADRC);
// }


void attitudeADRCwriteToConfigParam(void)
{
    // configParam.adrcAngle.pitch.leso.b0      = ADRCAnglePitch.leso.b0;
    // configParam.adrcAngle.pitch.leso.w0      = ADRCAnglePitch.leso.w0;
    // configParam.adrcAngle.pitch.nlsef.alpha1 = ADRCAnglePitch.nlsef.alpha1;
    // configParam.adrcAngle.pitch.nlsef.alpha2 = ADRCAnglePitch.nlsef.alpha2;
    // configParam.adrcAngle.pitch.nlsef.beta_1 = ADRCAnglePitch.nlsef.beta_1;
    // configParam.adrcAngle.pitch.nlsef.beta_2 = ADRCAnglePitch.nlsef.beta_2;
    // configParam.adrcAngle.pitch.nlsef.N1     = ADRCAnglePitch.nlsef.N1;
    // configParam.adrcAngle.pitch.nlsef.zeta   = ADRCAnglePitch.nlsef.zeta;
    configParam.Pitch_td.N0        = Pitch_td.N0;
    configParam.Pitch_td.r         = Pitch_td.r;

    // configParam.adrcAngle.roll.leso.b0      = ADRCAngleRoll.leso.b0;
    // configParam.adrcAngle.roll.leso.w0      = ADRCAngleRoll.leso.w0;
    // configParam.adrcAngle.roll.nlsef.alpha1 = ADRCAngleRoll.nlsef.alpha1;
    // configParam.adrcAngle.roll.nlsef.alpha2 = ADRCAngleRoll.nlsef.alpha2;
    // configParam.adrcAngle.roll.nlsef.beta_1 = ADRCAngleRoll.nlsef.beta_1;
    // configParam.adrcAngle.roll.nlsef.beta_2 = ADRCAngleRoll.nlsef.beta_2;
    // configParam.adrcAngle.roll.nlsef.N1     = ADRCAngleRoll.nlsef.N1;
    // configParam.adrcAngle.roll.nlsef.zeta   = ADRCAngleRoll.nlsef.zeta;
    configParam.Roll_td.N0        = Roll_td.N0;
    configParam.Roll_td.r         = Roll_td.r;

    // configParam.adrcRate.pitch.leso.b0      = ADRCRatePitch.leso.b0;
    // configParam.adrcRate.pitch.leso.w0      = ADRCRatePitch.leso.w0;
    // configParam.adrcRate.pitch.nlsef.alpha1 = ADRCRatePitch.nlsef.alpha1;
    // configParam.adrcRate.pitch.nlsef.alpha2 = ADRCRatePitch.nlsef.alpha2;
    // configParam.adrcRate.pitch.nlsef.beta_1 = ADRCRatePitch.nlsef.beta_1;
    // configParam.adrcRate.pitch.nlsef.beta_2 = ADRCRatePitch.nlsef.beta_2;
    // configParam.adrcRate.pitch.nlsef.N1     = ADRCRatePitch.nlsef.N1;
    // configParam.adrcRate.pitch.nlsef.zeta   = ADRCRatePitch.nlsef.zeta;
    configParam.Yaw_td.N0        = Yaw_td.N0;
    configParam.Yaw_td.r         = Yaw_td.r;
    // configParam.adrcRate.roll.leso.b0       = ADRCRateRoll.leso.b0;
    // configParam.adrcRate.roll.leso.w0       = ADRCRateRoll.leso.w0;
    // configParam.adrcRate.roll.nlsef.alpha1  = ADRCRateRoll.nlsef.alpha1;
    // configParam.adrcRate.roll.nlsef.alpha2  = ADRCRateRoll.nlsef.alpha2;
    // configParam.adrcRate.roll.nlsef.beta_1  = ADRCRateRoll.nlsef.beta_1;
    // configParam.adrcRate.roll.nlsef.beta_2  = ADRCRateRoll.nlsef.beta_2;
    // configParam.adrcRate.roll.nlsef.N1      = ADRCRateRoll.nlsef.N1;
    // configParam.adrcRate.roll.nlsef.zeta    = ADRCRateRoll.nlsef.zeta;
    // configParam.adrcRate.roll.td.N0         = ADRCRateRoll.td.N0;
    // configParam.adrcRate.roll.td.r          = ADRCRateRoll.td.r;
}

// void attitudeRateADRC(Axis3f *actualRate, attitude_t *desiredRate, float32_t *ADRC_u0)
// {
//     ADRC_RateControl(&ADRCRateRoll,desiredRate->roll,actualRate->x);
//     ADRC_RateControl(&ADRCRatePitch,desiredRate->pitch,actualRate->y);
//     ADRC_RateControl(&ADRCRateYaw,desiredRate->yaw,actualRate->z);
//     *ADRC_u0 = ADRCRateRoll.nlsef.u0;
//     *(ADRC_u0+1) = ADRCRatePitch.nlsef.u0;
//     *(ADRC_u0+2) = ADRCRateYaw.nlsef.u0;

//     // output->pitch = pidOutLimit(pidUpdate(&pidRatePitch, desiredRate->pitch - actualRate->y));
//     // output->roll = pidOutLimit(ADRCRateRoll.u);
//     // output->yaw = pidOutLimit(pidUpdate(&pidRateYaw, desiredRate->yaw - actualRate->z));
// }

void attitudeTD(setpoint_t *setpoint)
{
    adrc_td(&Roll_td, setpoint->attitude.roll);
    adrc_td(&Pitch_td, setpoint->attitude.pitch);
    adrc_td(&Yaw_td, setpoint->attitude.yaw);
    setpoint->attitudeDesired.roll = Roll_td.x1;
    setpoint->attitudeDesired.pitch = Pitch_td.x1;
    setpoint->attitudeDesired.yaw = Yaw_td.x1;    
}

void attitudeADRCinit(void)
{
    td_init(&Roll_td,&configParam.Roll_td,ANGLE_TD_DT);
    td_init(&Pitch_td,&configParam.Pitch_td,ANGLE_TD_DT);
    td_init(&Yaw_td,&configParam.Yaw_td,ANGLE_TD_DT);

    // nlsef_init(&ADRCRateRoll.nlsef,&configParam.adrcRate.roll.nlsef,RATE_LOOP_DT);
    // nlsef_init(&ADRCRatePitch.nlsef,&configParam.adrcRate.pitch.nlsef,RATE_LOOP_DT);
    // nlsef_init(&ADRCRateYaw.nlsef,&configParam.adrcRate.yaw.nlsef,RATE_LOOP_DT);
}
