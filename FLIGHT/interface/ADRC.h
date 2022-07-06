#ifndef _ADRC_H_
#define _ADRC_H_

#include "stm32f4xx.h"
#include "filter.h"
#include "config_param.h"
#include "arm_math.h"

#if defined(__CC_ARM) 
	#pragma anon_unions
#endif

typedef struct 
{
/*****安排过度过程*******/
	float TD_input;
	union 
	{
		struct 
		{
			float x1;//跟踪微分期状态量
			float x2;//跟踪微分期状态量微分项
			float fh;
		};
		float x[3];
	};
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
	float beta_I;
	float I_limit;
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
	float disturb;
	lpf2pData uLpf;//ESO中考虑执行器的延迟
}lesoObject_2rd_t;
typedef struct 
{
	float z1;
	float z2;
	float z3;
	float e;//系统状态误差
	float b0;
	float h;
	float w0;
	lpf2pData uLpf;//ESO中考虑执行器的延迟
}lesoObject_3rd_t;
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
	lesoObject_2rd_t leso;
    float u;

}adrcObject_t;

void  leso_init(lesoObject_2rd_t* lesoobject, lesoParam_t* lesoparam, float lesoDt);
void  leso_3rd_init(lesoObject_3rd_t* lesoobject, lesoParam_t* lesoparam, float lesoDt);
void  td_init(tdObject_t* tdobject, tdParam_t* tdparam, float ldtDt);
void  td_states_update(tdObject_t* tdobject, const float x1, const float x2);
void  nlsef_toc_init(nlsef_TOCObject_t* nlsef_tocobject, nlsef_TOCParam_t* nlsef_tocparam, float nlsefDt);
void  nlsef_init(nlsefObject_t* nlsef_object, nlsefParam_t* nlsef_param, float nlsefDt);
void  adrc_leso(lesoObject_2rd_t* adrcobject, const float expect_val, const float u);
void  adrc_leso_3rd(lesoObject_3rd_t* lesoobject, const float expect_val, const float u);
float adrc_TOCnlsef(nlsef_TOCObject_t* nlsef_t);
float adrc_nlsef(nlsefObject_t *nlsef);
void  adrc_eso(nlesoObject_t* nlesoObject, float expect_val, float u);
void  adrc_td(tdObject_t* td, float v);
float Constrain_Float(float amt, float low, float high);

// void ADRCGet(Fhan_Data *fhan_Input,Fhan_Data *fhan_Ouput);

#endif

