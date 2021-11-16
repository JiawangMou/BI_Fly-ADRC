#include "ADRC.h"
#include "pm.h"
#include <math.h>
#include "maths.h"
#include "filter.h"

#define U_LPF_CUTOFF_FREQ 100

// const float ADRC_Unit[3][16]=
// {
// /*TD跟踪微分器   改进最速TD,h0=N*h      扩张状态观测器ESO           扰动补偿     非线性组合*/
// /*  r     h      N                  beta_01   beta_02    beta_03     b0     beta_0   beta_1     beta_2     N1     C    alpha1  alpha2  zeta  b*/
//  {6000000 ,0.002 , 2,               300,      30000,     1000000,    30.0,    0.002,   350.0,     0.025,      5,    5,    0.8,   1.6,    50,    0.0004372},
//  {6000000 ,0.002 , 2,               300,      30000,     1000000,    30.0,    0.002,   350.0,     0.025,      5,    5,    0.8,   1.6,    50,    0.0004205},
//  {1000000 ,0.002 , 2,               300,      4000,      10000,      0.001,   0.002,   120.0,     0.0001,     5,    5,    0.8,   1.5,    50,    0},
// };
// const float ADRC_Unit[3][11]=
// {
// /*TD跟踪微分器      改进最速TD,h0=N*h    扩张状态观测器ESO     扰动补偿 非线性组合*/
// /*  r       h       N0                  beta_01   beta_02     b0       r1       h1         N1   C    zeta*/
//  {6000000  ,0.002 , 2,                  500,      85000,      0.26,    20000,   0.002,     2,   2,   50},
//  {6000000  ,0.002 , 2,                  500,      85000,      0.26,    20000,   0.002,     2,   2,   50},
//  {10000    ,0.002 , 2,                  300,      4000,       1.0,     3000,    0.002,     2,   5,   50},
// };
// const float ADRC_Unit[3][10]=
// {
// /*TD跟踪微分器      改进最速TD,h0=N*h    扩张状态观测器ESO     扰动补偿 非线性组合*/
// /*  r       h       N0                  w0           b0        r1       h1      N1      C    zeta*/
//  {800000   ,0.001 , 1,                 300,         0.25,    8000,     0.004,   30.0,   1.2,   50},
//  {800000   ,0.001 , 1,                 300,         0.25,    8000,     0.004,   30.0,   1.2,   50},
//  {10000    ,0.001 , 2,                  300,        1.0,    3000,      0.002,    2.0,   1.0,   50},
// };

// const float ADRC_Unit[3][10]=
// {
// /*TD跟踪微分器      改进最速TD,h0=N*h    扩张状态观测器ESO     扰动补偿 非线性组合*/
// /*  r       h       N0                  w0           b0        r1       h1      N1      C    zeta*/
//  {800000   ,0.001 , 1,                 800,         0.033,    1100.0,  0.004,   72.0,   0.04,   50},
//  {800000   ,0.001 , 1,                 800,         0.033,    1100.0,  0.004,   72.0,   0.04,   50},
//  {10000    ,0.001 , 2,                 300,         1.0,      3000,    0.002,   2.0,    1.0,    50},
// };

// const float ADRC_Unit[3][10]=
// {
// /*TD跟踪微分器      改进最速TD,h0=N*h    扩张状态观测器ESO     扰动补偿 非线性组合*/
// /*  r       h       N0                  w0           b0        r1       h1      N1      C    zeta*/
//  {16000000   ,0.001 , 2,                 600,        0.26,   4000.0,  0.004,   40.0,   0.064,   50},
//  {16000000   ,0.001 , 2,                 600,        0.26,   4000.0,  0.004,   40.0,   0.064,   50},
//  {10000    ,0.001 , 1,                 300,         1.0,      3000,    0.002,   2.0,    1.0,    50},
// };
float Constrain_Float(float amt, float low, float high){
  return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

int16_t adrc_sign(float val)
{
	if(val >= 0.0f)
		return 1.0f;
	else
		return -1.0f;
}

// int16_t Fsg_ADRC(float x,float d)
// {
//   int16_t output=0;
//   output=(Sign_ADRC(x+d)-Sign_ADRC(x-d))/2;
//   return output;
// }

void td_init(tdObject_t *tdobject,tdParam_t *tdparam, float tdDt)
{
    /*****TD*******/
    tdobject->TD_input = 0;
    tdobject->x1       = 0; //跟踪微分期状态量
    tdobject->x2       = 0; //跟踪微分期状态量微分项
    tdobject->r        = tdparam->r; //时间尺度
    tdobject->h        = tdDt; //ADRC系统积分时间
    tdobject->N0       = tdparam->N0; //跟踪微分器解决速度超调h0=N*h
    tdobject->fh       = 0;
}
void td_states_update(tdObject_t *tdobject,const float x1,const float x2)
{
    tdobject->x1       = x1; //跟踪微分期状态量
    tdobject->x2       = x2; //跟踪微分期状态量微分项
}
void leso_init(lesoObject_t *lesoobject, lesoParam_t *lesoparam, float lesoDt)
{
    /*****LESO*******/
    lesoobject->z1      = 0;
    lesoobject->z2      = 0;
    lesoobject->e       = 0; //系统状态误差
    lesoobject->b0      = lesoparam->b0;
    lesoobject->h       = lesoDt;
    lesoobject->w0      = lesoparam->w0;

}
void nlsef_toc_init(nlsef_TOCObject_t *nlsef_tocobject, nlsef_TOCParam_t * nlsef_tocparam,float nlsefDt)
{
    /**********NLSEF*********/
//    adrcobject->nlsef_TOC.e0 = 0; //状态误差积分项
    nlsef_tocobject->r  = nlsef_tocparam->r; //状态偏差
    nlsef_tocobject->h  = nlsefDt; //状态量微分项
    nlsef_tocobject->N1 = nlsef_tocparam->N1;
    nlsef_tocobject->e1 = 0;
    nlsef_tocobject->e2 = 0; //非线性组合系统输出
    nlsef_tocobject->c  = nlsef_tocparam->c;
    nlsef_tocobject->u0 = 0; //扰动补偿
}

void nlsef_init(nlsefObject_t *nlsef_object, nlsefParam_t *nlsef_param, float nlsefDt)
{
    nlsef_object->N1 = nlsef_param->N1; //跟踪微分器解决速度超调h1=N1*h
    nlsef_object->beta_1 = nlsef_param->beta_1;
    nlsef_object->beta_2 = nlsef_param->beta_2;
    nlsef_object->zeta = nlsef_param->zeta;
    nlsef_object->alpha1 = nlsef_param->alpha1;
    nlsef_object->alpha2 = nlsef_param->alpha2;

    nlsef_object->h = nlsefDt;
    nlsef_object->u0 = 0; //扰动补偿
    nlsef_object->e1 = 0;
    nlsef_object->e2 = 0; //非线性组合系统输出
    nlsef_object->e1_out = 0;
    nlsef_object->e2_out = 0;
}





// //ADRC最速跟踪微分器TD，改进的算法fhan
// void Fhan_ADRC(Fhan_Data *fhan_Input,float expect_ADRC)//安排ADRC过度过程
// {
//   float d=0,a0=0,y=0,a1=0,a2=0,a=0;
//   float x1_delta=0;//ADRC状态跟踪误差项
//   fhan_Input->TD_input = expect_ADRC;
//   x1_delta=fhan_Input->x1-expect_ADRC;//用x1-v(k)替代x1得到离散更新公式
//   fhan_Input->h0=fhan_Input->N0*fhan_Input->h;//用h0替代h，解决最速跟踪微分器速度超调问题
//   d=fhan_Input->r*fhan_Input->h0*fhan_Input->h0;//d=rh^2;
//   a0=fhan_Input->h0*fhan_Input->x2;//a0=h*x2
//   y=x1_delta+a0;//y=x1+a0
//   a1=sqrt(d*(d+8*ABS(y)));//a1=sqrt(d*(d+8*ABS(y))])
//   a2=a0+Sign_ADRC(y)*(a1-d)/2;//a2=a0+sign(y)*(a1-d)/2;
//   a=(a0+y)*Fsg_ADRC(y,d)+a2*(1-Fsg_ADRC(y,d));
//   fhan_Input->fh = -fhan_Input->r * (a / d) * Fsg_ADRC(a, d) - fhan_Input->r * Sign_ADRC(a) * (1 - Fsg_ADRC(a, d)); //得到最速微分加速度跟踪量
//   fhan_Input->x1 += fhan_Input->h * fhan_Input->x2;             //跟新最速跟踪状态量x1
//   fhan_Input->x2 += fhan_Input->h * fhan_Input->fh;             //跟新最速跟踪状态量微分x2
// }

float adrc_fhan(float x1, float x2, float r0, float h0)
{
	float d = h0 * h0 * r0;
	float a0 = h0 * x2;
	float y = x1 + a0;
	float a1 = sqrtf(d*(d + 8.0f*fabsf(y)));
	float a2 = a0 + adrc_sign(y)*(a1-d)*0.5f;
	float sy = (adrc_sign(y+d) - adrc_sign(y-d))*0.5f;
	float a = (a0 + y - a2)*sy + a2;
	float sa = (adrc_sign(a+d) - adrc_sign(a-d))*0.5f;
	
	return -r0*(a/d - adrc_sign(a))*sa - r0*adrc_sign(a);
}

void adrc_td(tdObject_t *td,float v)
{
    td->fh = adrc_fhan(td->x1 - v, td->x2, td->r, td->h * td->N0);
    td->x1 += td->h * td->x2;
    td->x2 += td->h  * td->fh;
    td->TD_input = v;
}

//原点附近有连线性段的连续幂次函数
float Fal_ADRC(float e,float alpha,float zeta)
{
	if(fabsf(e) <= zeta){
		return e / (powf(zeta, 1.0f-alpha));
	}else{
		return powf(fabsf(e), alpha) * adrc_sign(e);
	}
}
//LESO
void adrc_leso(lesoObject_t* adrcobject,float expect_val, float u)
{
    float Beta_01 = 2 * adrcobject->w0;
    float Beta_02 = adrcobject->w0 * adrcobject->w0;

    adrcobject->e = adrcobject->z1 - expect_val;
    adrcobject->z1 += (adrcobject->z2 - Beta_01 * adrcobject->e + adrcobject->b0 * lpf2pApply(&adrcobject->uLpf, u)) * adrcobject->h;
    // adrcobject->z1 += (adrcobject->z2 - Beta_01 * e + adrcobject->b0 *  adrcobject->u) * adrcobject->h;
    adrcobject->z2 += -Beta_02 * adrcobject->e * adrcobject->h;
}

/************扩张状态观测器********************/
//状态观测器参数beta01=1/h  beta02=1/(3*h^2)  beta03=2/(8^2*h^3) 我这里取得是1/20*h^3 ...
void adrc_eso(nlesoObject_t *nlesoObject,float expect_val,float u)
{
  // float bat = pmGetBatteryVoltage();
  nlesoObject->e = nlesoObject->z1 - expect_val; //状态误差

  nlesoObject->fe =Fal_ADRC(nlesoObject->e,0.5,nlesoObject->h);//非线性函数，提取跟踪状态与当前状态误差
  // nlesoObject->fe1=Fal_ADRC(nlesoObject->e,0.25,nlesoObject->h);

//   /*************扩展状态量更新**********/
//   nlesoObject->z1+=nlesoObject->h*(nlesoObject->z2-nlesoObject->beta_01*nlesoObject->e);
//   nlesoObject->z2+=nlesoObject->h*(nlesoObject->z3
//                                  -nlesoObject->beta_02*nlesoObject->fe
//                                    +nlesoObject->b*bat*nlesoObject->u);
//  //ESO估计状态加速度信号，进行扰动补偿，传统MEMS陀螺仪漂移较大，估计会产生漂移
//   nlesoObject->z3+=nlesoObject->h*(-nlesoObject->beta_03*nlesoObject->fe1);
  /*************扩展状态量更新**********/
  nlesoObject->z1+=nlesoObject->h*(nlesoObject->z2 - nlesoObject->beta_01*nlesoObject->e + nlesoObject->b0*lpf2pApply(&nlesoObject->uLpf, u));
  // nlesoObject->z1+=nlesoObject->h*(nlesoObject->z2 - nlesoObject->beta_01*nlesoObject->e);
  nlesoObject->z2+=nlesoObject->h*(-nlesoObject->beta_02*nlesoObject->fe);
  // nlesoObject->z2 = Constrain_Float(nlesoObject->z2,-40000,40000);

}

float adrc_TOCnlsef(nlsef_TOCObject_t* nlsef_t)
{
    return -adrc_fhan(nlsef_t->e1, nlsef_t->c * nlsef_t->e2, nlsef_t->r, nlsef_t->h * nlsef_t->N1);
}

float adrc_nlsef(nlsefObject_t *nlsef_t)
{
    // nlsef_t->e1_out = 50.0f*nlsef_t->beta_1*Fal_ADRC(nlsef_t->e1/50.0f,nlsef_t->alpha1,nlsef_t->zeta);
    // nlsef_t->e2_out = nlsef_t->beta_2*Fal_ADRC(nlsef_t->e2,nlsef_t->alpha2,nlsef_t->zeta);
    nlsef_t->e1_out = nlsef_t->beta_1 * nlsef_t->e1;
    nlsef_t->e2_out = nlsef_t->beta_2 * nlsef_t->e2;
    return  nlsef_t->e1_out + nlsef_t->e2_out;
}
// void Nolinear_Conbination_ADRC(Fhan_Data *fhan_Input)
// {
//   float temp_e2=0;
//   temp_e2=Constrain_Float(fhan_Input->e2,-3000,3000);
//   fhan_Input->beta_1_temp = fhan_Input->beta_1*Fal_ADRC(fhan_Input->e1,fhan_Input->alpha1,fhan_Input->zeta);
//   fhan_Input->beta_2_temp = fhan_Input->beta_2*Fal_ADRC(temp_e2,fhan_Input->alpha2,fhan_Input->zeta);
//   // fhan_Input->beta_1_temp = fhan_Input->beta_1*fhan_Input->e1;
//   // fhan_Input->beta_2_temp = fhan_Input->beta_2*temp_e2;
//   fhan_Input->u0 = fhan_Input->beta_1_temp + fhan_Input->beta_2_temp;
// }

//void Nolinear_Conbination_ADRC(Fhan_Data *fhan_Input)
//{
//  float temp_e2=0;
//  temp_e2=Constrain_Float(fhan_Input->e2,-3000,3000);
//  fhan_Input->beta_1_temp = fhan_Input->beta_1*fhan_Input->e1;
//  fhan_Input->beta_2_temp = fhan_Input->beta_2*fhan_Input->e2;
//  // fhan_Input->beta_1_temp = fhan_Input->beta_1*fhan_Input->e1;
//  // fhan_Input->beta_2_temp = fhan_Input->beta_2*temp_e2;
//  fhan_Input->u0 = fhan_Input->beta_1_temp + fhan_Input->beta_2_temp;
//}



