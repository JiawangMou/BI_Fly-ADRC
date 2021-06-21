#ifndef __CONFIG_PARAM_H
#define __CONFIG_PARAM_H
#include "sys.h"
#include <stdbool.h>

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 配置参数驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/
												
typedef struct 
{
	float kp;
	float ki;
	float kd;
	float outputLimit;
} pidInit_t;

typedef struct
{
	pidInit_t roll;
	pidInit_t pitch;	
	pidInit_t yaw;	
} pidParam_t;

typedef struct
{
	pidInit_t vx;
	pidInit_t vy;
	pidInit_t vz;
	
	pidInit_t x;
	pidInit_t y;
	pidInit_t z;
} pidParamPos_t;

typedef struct
{
	int16_t accZero[3];
	int16_t accGain[3];
	bool bias_isfound;
} accBias_t;

typedef struct
{
	int16_t magZero[3];
} magBias_t;

typedef struct 
{
    int16_t rollDeciDegrees;
    int16_t pitchDeciDegrees;
    int16_t yawDeciDegrees;
} boardAlignment_t;

typedef struct 
{
	u16 s_left;
	u16 s_right;
	u16 s_middle;	
}Servo_initpos;

// typedef struct
// {
// 	float x;
// 	float y;
// 	float z;
// }Bias_Value;
// typedef struct
// {
// 	Bias_Value bias_value;
// 	bool bias_isfound;
// }Bias;
typedef struct 
{
/*****安排过度过程*******/
	float r;//时间尺度
	float N0;//跟踪微分器解决速度超调h0=N*h	
}tdParam_t;

typedef struct 
{
	float N1;//跟踪微分器解决速度超调h1=N1*h
	float beta_1;
	float beta_2;
	float zeta;
	float alpha1;
	float alpha2;
}nlsefParam_t;

//最速控制(time-optimal control:TOC)  TODO:最速开关控制的英文缩写看是不是改一下
typedef struct 
{
	float r;//时间尺度
	float N1;//跟踪微分器解决速度超调h0=N*h	
	float c;//阻尼系数
}nlsef_TOCParam_t;

typedef struct 
{
	float b0;
	float w0;
}lesoParam_t;

typedef struct 
{
	float b0;
	float beta_01;
	float beta_02;
}nlesoParam_t;

typedef struct 
{
	tdParam_t td;
	// nlsef_TOCParam_t nlsef_TOC;
	nlsefParam_t nlsef;
	lesoParam_t leso;
}adrcInit_t;

typedef struct 
{
	adrcInit_t roll;
	adrcInit_t pitch;
}adrcParam_t;
typedef struct	
{
	u8 version;				/*软件版本号*/
	pidParam_t pidAngle;	/*角度PID*/	
	pidParam_t pidRate;		/*角速度PID*/	
	pidParamPos_t pidPos;	/*位置PID*/
//	accBias_t accBias;		/*加速度校准值*/
//	magBias_t magBias;		/*磁力计校准值*/
	adrcParam_t adrcAngle;	/*角度ADRC*/
	adrcParam_t adrcRate;	/*角速度ADRC*/
	float trimP;			/*pitch微调*/
	float trimR;			/*roll微调*/
	u16 thrustBase;			/*油门基础值*/
	Servo_initpos servo_initpos;	/*舵机初始值*/
	accBias_t accBias;
	u8 cksum;				/*校验*/
} configParam_t;



extern configParam_t configParam;

void configParamInit(void);	/*参数配置初始化*/
void configParamTask(void* param);	/*参数配置任务*/
bool configParamTest(void);

void configParamGiveSemaphore(void);
void resetConfigParamPID(void);
void saveConfigAndNotify(void);
void changeServoinitpos_configParam(u16 s1,u16 s2,u16 s3);
u16 getservoinitpos_configParam(u8 pwm_id);
accBias_t getaccbias_configParam( void );
void accbias_writeFlash(void);

#endif /*__CONFIG_PARAM_H */

