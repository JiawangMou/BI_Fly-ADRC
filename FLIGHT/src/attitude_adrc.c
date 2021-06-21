#include "attitude_adrc.h"
#include "ADRC.h"

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

    // /*****���Ź��ȹ���*******/
    // fhan_Input->TD_input = 0;
    // fhan_Input->x1       = 0; //����΢����״̬��
    // fhan_Input->x2       = 0; //����΢����״̬��΢����

    // fhan_Input->fh = 0; //����΢�ּ��ٶȸ�����
    // /*****����״̬�۲���*******/
    // /******��ϵͳ���y������u�����ٹ���ϵͳ״̬���Ŷ�*****/
    // fhan_Input->z1      = 0;
    // fhan_Input->z2      = 0;
    // fhan_Input->e       = 0; //ϵͳ״̬���
    // fhan_Input->y       = 0; //ϵͳ�����
    // fhan_Input->fe      = 0;
    // fhan_Input->fe1     = 0;

    /**********ϵͳ״̬������*********/
    // fhan_Input->e0 = 0; //״̬��������
    // fhan_Input->e1 = 0; //״̬ƫ��
    // fhan_Input->e2 = 0; //״̬��΢����

    // adrcobject->nlsef_TOC.u0  = 0; //���������ϵͳ���
    adrcobject->nlsef.u0  = 0; //���������ϵͳ���
    adrcobject->u  = 0; //���Ŷ�����������

}

void ADRC_RateControl(adrcObject_t *adrcObject,float expect_ADRC,float feedback_ADRC)
{
	//TD  ����TD����ˢ��Ƶ�ʸ߲�����ADRC_RateControl��
	//LESO ����LESO����ˢ��Ƶ�ʸ߲�����ADRC_RateControl��
    /********״̬������***/
    // fhan_Input->e0 += fhan_Input->e1 * fhan_Input->h1; //״̬������
    // fhan_Input->e0 = Constrain_Float(fhan_Input->e0, -10.0f, 10.0f);

    //nlsef_TOC
    // adrcObject->nlsef_TOC.e1 = adrcObject->td.x1; //״̬ƫ����
    // adrcObject->nlsef_TOC.e2 = adrcObject->td.x2; //״̬΢����
    //nlsef
    adrcObject->nlsef.e1 = adrcObject->td.x1; //״̬ƫ����
    adrcObject->nlsef.e2 = adrcObject->td.x2; //״̬΢����
    /********NLSEF*******/
    //nlsef_TOC
    // adrcObject->nlsef_TOC.u0 = adrc_TOCnlsef(&adrcObject->nlsef_TOC);
    //nlsef
    adrcObject->nlsef.u0 = adrc_nlsef(&adrcObject->nlsef);
    /**********�Ŷ�����*******/
    // adrcObject->u = (adrcObject->nlsef.u0 - 0.5f * adrcObject->leso.z2) / adrcObject->leso.b0;
    adrcObject->u = adrcObject->nlsef.u0 / adrcObject->leso.b0;
    //ʵ�����
    adrcObject->u = Constrain_Float(adrcObject->u, -32767, 32767);
}

void ADRC_AngleControl(adrcObject_t *adrcObject,float expect_ADRC,float feedback)
{
	adrcObject->td.x1 = feedback;
	adrc_td( &adrcObject->td, expect_ADRC);
}
