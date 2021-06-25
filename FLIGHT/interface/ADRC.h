#ifndef _ADRC_H_
#define _ADRC_H_

#include "stm32f4xx.h"
#include "filter.h"
#include "config_param.h"
// typedef struct
// {
// /*****安排过度过程*******/
// float TD_input;
// float x1;//跟踪微分期状态量
// float x2;//跟踪微分期状态量微分项
// float r;//时间尺度
// float h;//ADRC系统积分时间
// float N0;//跟踪微分器解决速度超调h0=N*h

// float h0;
// float fh;//最速微分加速度跟踪量
// /*****扩张状态观测器*******/
// /******已系统输出y和输入u来跟踪估计系统状态和扰动*****/
// float z1;
// float z2;
// float e;//系统状态误差
// float y;//系统输出量
// float fe;
// float fe1;
// float beta_01;
// float beta_02;
// float beta_03;
// float b;

// /**********系统状态误差反馈率*********/
// float e0;//状态误差积分项
// float e1;//状态偏差
// float e2;//状态量微分项
// float u0;//非线性组合系统输出
// float u;//带扰动补偿后的输出
// float b0;//扰动补偿

// float w0;
// // /*********第一种组合形式*********/
// // float beta_0;//线性
// // float beta_1;//非线性组合参数
// // float beta_2;//u0=beta_1*e1+beta_2*e2+(beta_0*e0);
// // /*********第二种组合形式*********/
// // float alpha1;//u0=beta_1*fal(e1,alpha1,zeta)+beta_2*fal(e2,alpha2,zeta)
// // float alpha2;//0<alpha1<1<alpha2
// // float zeta;//线性段的区间长度
// /*********第三种组合形式*********/
// float h1;//u0=-fhan(e1,e2,r,h1);
// float N1;//跟踪微分器解决速度超调h1=N1*h
// float r1;
// /*********第四种组合形式*********/
// float c;//u0=-fhan(e1,c*e2,r,h1);

// float beta_1_temp;
// float beta_2_temp;


// lpf2pData uLpf;

// }Fhan_Data;

typedef struct 
{
/*****安排过度过程*******/
	float TD_input;
	float x1;//跟踪微分期状态量
	float x2;//跟踪微分期状态量微分项
	float r;//时间尺度
	float h;//ADRC系统积分时间
	float N0;//跟踪微分器解决速度超调h0=N*h	
}tdObject_t;

typedef struct 
{
	float h;//u0=-fhan(e1,e2,r,h1);
	float N1;//跟踪微分器解决速度超调h1=N1*h
	float r1;
	float beta_1;
	float beta_2;
	float zeta;
	float alpha1;
	float alpha2;
    float u0;
	float e1;
    float e2;
	float e1_out;
    float e2_out;
}nlsefObject_t;

//最速控制(time-optimal control:TOC)  TODO: 最速开关控制的英文缩写看是不是改一下
typedef struct 
{
	float r;//时间尺度
	float h;//ADRC系统积分时间
	float N1;//跟踪微分器解决速度超调h0=N*h	
    float e1;
    float e2;
    float c;
    float u0;
}nlsef_TOCObject_t;


typedef struct 
{
	float z1;
	float z2;
	float e;//系统状态误差
	float b0;
	float h;
	float w0;
	lpf2pData uLpf;//ESO中考虑执行器的延迟
}lesoObject_t;

typedef struct 
{
    float z1;
	float z2;
	float e;//系统状态误差
	float b0;
	float h;
	float beta_01;
	float beta_02;
	float zeta;
	float alpha1;
	float alpha2;
	float fe;
    lpf2pData uLpf;//ESO中考虑执行器的延迟
}nlesoObject_t;

typedef struct 
{
	tdObject_t td;
//	nlsef_TOCObject_t nlsef_TOC;
	nlsefObject_t nlsef;
	lesoObject_t leso;
    float u;

}adrcObject_t;

void leso_init(lesoObject_t *lesoobject, lesoParam_t *lesoparam, float lesoDt);
void td_init(tdObject_t *tdobject, tdParam_t *tdparam, float ldtDt);
void nlsef_toc_init(nlsef_TOCObject_t *nlsef_tocobject, nlsef_TOCParam_t *nlsef_tocparam, float nlsefDt);
void nlsef_init(nlsefObject_t *nlsef_object, nlsefParam_t *nlsef_param, float nlsefDt);
void adrc_leso(lesoObject_t *adrcobject, float expect_val, float u);
float adrc_TOCnlsef(nlsef_TOCObject_t *nlsef_t);
float adrc_nlsef(nlsefObject_t *nlsef_t);
void adrc_eso(nlesoObject_t *nlesoObject, float expect_val, float u);
void adrc_td(tdObject_t *td, float v);
float Constrain_Float(float amt, float low, float high);

// void ADRCGet(Fhan_Data *fhan_Input,Fhan_Data *fhan_Ouput);

#endif

