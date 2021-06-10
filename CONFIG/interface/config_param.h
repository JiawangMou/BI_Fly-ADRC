#ifndef __CONFIG_PARAM_H
#define __CONFIG_PARAM_H
#include "sys.h"
#include <stdbool.h>

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * ���ò�����������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) �������������ӿƼ����޹�˾ 2014-2024
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
/*****���Ź��ȹ���*******/
	float r;//ʱ��߶�
	float N0;//����΢��������ٶȳ���h0=N*h	
}tdParam_t;

typedef struct 
{
	float N1;//����΢��������ٶȳ���h1=N1*h
	float r1;
	float beta_1;
	float beta_2;
	float zeta;
	float alpha1;
	float alpha2;
}nlsefParam_t;

//���ٿ���(time-optimal control:TOC)  TODO:���ٿ��ؿ��Ƶ�Ӣ����д���ǲ��Ǹ�һ��
typedef struct 
{
	float r;//ʱ��߶�
	float N1;//����΢��������ٶȳ���h0=N*h	
	float c;//����ϵ��
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
	nlsef_TOCParam_t nlsef_TOC;
	lesoParam_t leso;
}adrcInit_t;

typedef struct 
{
	adrcInit_t roll;
	adrcInit_t pitch;
}adrcParam_t;
typedef struct	
{
	u8 version;				/*�����汾��*/
	pidParam_t pidAngle;	/*�Ƕ�PID*/	
	pidParam_t pidRate;		/*���ٶ�PID*/	
	pidParamPos_t pidPos;	/*λ��PID*/
//	accBias_t accBias;		/*���ٶ�У׼ֵ*/
//	magBias_t magBias;		/*������У׼ֵ*/
	adrcParam_t adrcAngle;	/*�Ƕ�ADRC*/
	adrcParam_t adrcRate;	/*���ٶ�ADRC*/
	float trimP;			/*pitch΢��*/
	float trimR;			/*roll΢��*/
	u16 thrustBase;			/*���Ż���ֵ*/
	Servo_initpos servo_initpos;	/*�����ʼֵ*/
	accBias_t accBias;
	u8 cksum;				/*У��*/
} configParam_t;



extern configParam_t configParam;

void configParamInit(void);	/*�������ó�ʼ��*/
void configParamTask(void* param);	/*������������*/
bool configParamTest(void);

void configParamGiveSemaphore(void);
void resetConfigParamPID(void);
void saveConfigAndNotify(void);
void changeServoinitpos_configParam(u16 s1,u16 s2,u16 s3);
u16 getservoinitpos_configParam(u8 pwm_id);
accBias_t getaccbias_configParam( void );
void accbias_writeFlash(void);

#endif /*__CONFIG_PARAM_H */
