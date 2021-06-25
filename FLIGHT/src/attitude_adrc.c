#include "ADRC.h"
#include "config_param.h"
#include "attitude_adrc.h"



#define U_LPF_CUTOFF_FREQ 100


adrcObject_t ADRCAnglePitch;
adrcObject_t ADRCAngleRoll;
adrcObject_t ADRCRatePitch;
adrcObject_t ADRCRateRoll;

void adrc_init(adrcObject_t *adrcobject,adrcInit_t *param, float tdDt,float lesoDt,float nlsefDt)
{
    td_init(&adrcobject->td,&param->td,tdDt);
    leso_init(&adrcobject->leso, &param->leso,lesoDt);
    // nlsef_toc_init(&adrcobject->nlsef_TOC,&param->nlsef_TOC,nlsefDt);
    nlsef_init(&adrcobject->nlsef,&param->nlsef,nlsefDt);
    lpf2pInit(&adrcobject->leso.uLpf, 1.0f/lesoDt, U_LPF_CUTOFF_FREQ);
    adrcobject->u = 0;
}

void adrc_reset(adrcObject_t *adrcobject)
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
    adrcobject->nlsef.u0  = 0; //非线性组合系统输出
    adrcobject->u  = 0; //带扰动补偿后的输出

}

void ADRC_RateControl(adrcObject_t *adrcObject,float expect_ADRC,float feedback_ADRC)
{
	//TD  这里TD由于刷新频率高不放在ADRC_RateControl中
	//LESO 这里LESO由于刷新频率高不放在ADRC_RateControl中
    /********状态误差反馈率***/
    // fhan_Input->e0 += fhan_Input->e1 * fhan_Input->h1; //状态积分项
    // fhan_Input->e0 = Constrain_Float(fhan_Input->e0, -10.0f, 10.0f);

    //nlsef_TOC
    // adrcObject->nlsef_TOC.e1 = adrcObject->td.x1; //状态偏差项
    // adrcObject->nlsef_TOC.e2 = adrcObject->td.x2; //状态微分项
    //nlsef
    adrcObject->nlsef.e1 = adrcObject->td.x1; //状态偏差项
    adrcObject->nlsef.e2 = adrcObject->td.x2; //状态微分项
    /********NLSEF*******/
    //nlsef_TOC
    // adrcObject->nlsef_TOC.u0 = adrc_TOCnlsef(&adrcObject->nlsef_TOC);
    //nlsef
    adrcObject->nlsef.u0 = adrc_nlsef(&adrcObject->nlsef);
    /**********扰动补偿*******/
    // adrcObject->u = (adrcObject->nlsef.u0 - 0.5f * adrcObject->leso.z2) / adrcObject->leso.b0;
    adrcObject->u = adrcObject->nlsef.u0 / adrcObject->leso.b0;
    //实际输出
    adrcObject->u = Constrain_Float(adrcObject->u, -32767, 32767);
}

void ADRC_AngleControl(adrcObject_t *adrcObject,float expect_ADRC,float feedback)
{
	adrcObject->td.x1 = feedback;
	adrc_td( &adrcObject->td, expect_ADRC);
}


void attitudeADRCwriteToConfigParam(void)
{
    configParam.adrcAngle.pitch.leso.b0      = ADRCAnglePitch.leso.b0;
    configParam.adrcAngle.pitch.leso.w0      = ADRCAnglePitch.leso.w0;
    configParam.adrcAngle.pitch.nlsef.alpha1 = ADRCAnglePitch.nlsef.alpha1;
    configParam.adrcAngle.pitch.nlsef.alpha2 = ADRCAnglePitch.nlsef.alpha2;
    configParam.adrcAngle.pitch.nlsef.beta_1 = ADRCAnglePitch.nlsef.beta_1;
    configParam.adrcAngle.pitch.nlsef.beta_2 = ADRCAnglePitch.nlsef.beta_2;
    configParam.adrcAngle.pitch.nlsef.N1     = ADRCAnglePitch.nlsef.N1;
    configParam.adrcAngle.pitch.nlsef.zeta   = ADRCAnglePitch.nlsef.zeta;
    configParam.adrcAngle.pitch.td.N0        = ADRCAnglePitch.td.N0;
    configParam.adrcAngle.pitch.td.r         = ADRCAnglePitch.td.r;

    configParam.adrcAngle.roll.leso.b0      = ADRCAngleRoll.leso.b0;
    configParam.adrcAngle.roll.leso.w0      = ADRCAngleRoll.leso.w0;
    configParam.adrcAngle.roll.nlsef.alpha1 = ADRCAngleRoll.nlsef.alpha1;
    configParam.adrcAngle.roll.nlsef.alpha2 = ADRCAngleRoll.nlsef.alpha2;
    configParam.adrcAngle.roll.nlsef.beta_1 = ADRCAngleRoll.nlsef.beta_1;
    configParam.adrcAngle.roll.nlsef.beta_2 = ADRCAngleRoll.nlsef.beta_2;
    configParam.adrcAngle.roll.nlsef.N1     = ADRCAngleRoll.nlsef.N1;
    configParam.adrcAngle.roll.nlsef.zeta   = ADRCAngleRoll.nlsef.zeta;
    configParam.adrcAngle.roll.td.N0        = ADRCAngleRoll.td.N0;
    configParam.adrcAngle.roll.td.r         = ADRCAngleRoll.td.r;

    configParam.adrcRate.pitch.leso.b0      = ADRCRatePitch.leso.b0;
    configParam.adrcRate.pitch.leso.w0      = ADRCRatePitch.leso.w0;
    configParam.adrcRate.pitch.nlsef.alpha1 = ADRCRatePitch.nlsef.alpha1;
    configParam.adrcRate.pitch.nlsef.alpha2 = ADRCRatePitch.nlsef.alpha2;
    configParam.adrcRate.pitch.nlsef.beta_1 = ADRCRatePitch.nlsef.beta_1;
    configParam.adrcRate.pitch.nlsef.beta_2 = ADRCRatePitch.nlsef.beta_2;
    configParam.adrcRate.pitch.nlsef.N1     = ADRCRatePitch.nlsef.N1;
    configParam.adrcRate.pitch.nlsef.zeta   = ADRCRatePitch.nlsef.zeta;
    configParam.adrcRate.pitch.td.N0        = ADRCRatePitch.td.N0;
    configParam.adrcRate.pitch.td.r         = ADRCRatePitch.td.r;

    configParam.adrcRate.roll.leso.b0       = ADRCRateRoll.leso.b0;
    configParam.adrcRate.roll.leso.w0       = ADRCRateRoll.leso.w0;
    configParam.adrcRate.roll.nlsef.alpha1  = ADRCRateRoll.nlsef.alpha1;
    configParam.adrcRate.roll.nlsef.alpha2  = ADRCRateRoll.nlsef.alpha2;
    configParam.adrcRate.roll.nlsef.beta_1  = ADRCRateRoll.nlsef.beta_1;
    configParam.adrcRate.roll.nlsef.beta_2  = ADRCRateRoll.nlsef.beta_2;
    configParam.adrcRate.roll.nlsef.N1      = ADRCRateRoll.nlsef.N1;
    configParam.adrcRate.roll.nlsef.zeta    = ADRCRateRoll.nlsef.zeta;
    configParam.adrcRate.roll.td.N0         = ADRCRateRoll.td.N0;
    configParam.adrcRate.roll.td.r          = ADRCRateRoll.td.r;
}
