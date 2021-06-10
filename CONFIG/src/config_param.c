#include <stdbool.h>
#include <string.h>
#include "math.h"
#include "config.h"
#include "config_param.h"
#include "watchdog.h"
#include "stmflash.h"
#include "delay.h"
#include "sensors.h"

/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "motors.h"

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

#define VERSION 33 /*13 ��ʾV3.3*/

configParam_t configParam;

#ifdef BI_Fly_1
static configParam_t configParamDefault =
	{
		.version = VERSION, /*�����汾��*/

		.pidAngle = /*�Ƕ�PID*/
		{
			.roll =
				{
					.kp = 10.0,
					.ki = 0.0,
					.kd = 0.0,
					.outputLimit = 0,
				},
			.pitch =
				{
					.kp = 8.0,
					.ki = 0.0,
					.kd = 0.0,
					.outputLimit = 0,
				},
			.yaw =
				{
					.kp = 5.0,
					.ki = 0.0,
					.kd = 0.0,
					.outputLimit = 0,
				},
		},
		.pidRate = /*���ٶ�PID*/
		{
			.roll =
				{
					.kp = 50.0,
					.ki = 0.0,
					.kd = 1.0,
					.outputLimit = 0,
				},
			.pitch =
				{
					.kp = 70.0,
					.ki = 0.0,
					.kd = 0.5,
					.outputLimit = 0,
				},
			.yaw =
				{
					.kp = 30.0,
					.ki = 0.0,
					.kd = 0.0,
					.outputLimit = 0,
				},
		},
		.pidPos = /*λ��PID*/
		{
			.vx =
				{
					.kp = 4.5,
					.ki = 0.0,
					.kd = 0.0,
					.outputLimit = 120.0f,
				},
			.vy =
				{
					.kp = 4.5,
					.ki = 0.0,
					.kd = 0.0,
					.outputLimit = 120.0f,
				},
			.vz =
				{
					.kp = 80.0,
					.ki = 130.0,
					.kd = 10.0,
					.outputLimit = 65500.0f,
				},

			.x =
				{
					.kp = 4.0,
					.ki = 0.0,
					.kd = 0.6,
					.outputLimit = 1200.0f,
				},
			.y =
				{
					.kp = 4.0,
					.ki = 0.0,
					.kd = 0.6,
					.outputLimit = 1200.0f,
				},
			.z =
				{
					.kp = 300.0,
					.ki = 0.0,
					.kd = 100.0,
					.outputLimit = 65500.0f,
				},
		},

		.servo_initpos =
			{//��Կ��ư忴
				.s_left = 1500,
				.s_right = 1500,
				.s_middle = 1600,
			},
		.accBias = 
		{
			.accZero = {32, -39, -92},
			.accGain = {2049, 2045, 2068},
			.bias_isfound = true,
		},
		.adrcRate=
		{
			.roll=
			{
				.td=
				{
					.r  = 8000000,
					.N0 = 2,
				},
				.nlsef_TOC=
				{
					.r  = 4000.0,
					.N1 = 70.0,
					.c  = 0.0364,
				},
				.leso=
				{
					.b0 = 0.26,
					.w0 = 600,
				},

			},
			.pitch=
			{
				.td=
				{
					.r  = 8000000,
					.N0 = 2,
				},
				.nlsef_TOC=
				{
					.r  = 4000.0,
					.N1 = 40.0,
					.c  = 0.064,
				},
				.leso=
				{
					.b0 = 0.26,
					.w0 = 600,
				},
			},
		},
		.adrcAngle=
		{
			.roll=
			{
				.td=
				{
					.r  = 0,
					.N0 = 0,
				},
				.nlsef_TOC=
				{
					.r  = 0,
					.N1 = 0,
					.c  = 0,
				},
				.leso=
				{
					.b0 = 0,
					.w0 = 0,
				},

			},
			.pitch=
			{
				.td=
				{
					.r  = 0,
					.N0 = 0,
				},
				.nlsef_TOC=
				{
					.r  = 0,
					.N1 = 0,
					.c  = 0,
				},
				.leso=
				{
					.b0 = 0,
					.w0 = 0,
				},
			},
		},
		.trimP = 0.f,		 /*pitch΢��*/
		.trimR = 0.f,		 /*roll΢��*/
		.thrustBase = 34000, /*�������Ż���ֵ*/
};
#endif

#ifdef BI_Fly_2
static configParam_t configParamDefault =
	{
		.version = VERSION, /*�����汾��*/

		.pidAngle = /*�Ƕ�PID*/
		{
			.roll =
				{
					.kp = 10.0,
					.ki = 0.0,
					.kd = 0.0,
					.outputLimit = 0,
				},
			.pitch =
				{
					.kp = 8.0,
					.ki = 0.0,
					.kd = 0.0,
					.outputLimit = 0,
				},
			.yaw =
				{
					.kp = 5.0,
					.ki = 0.0,
					.kd = 0.0,
					.outputLimit = 0,
				},
		},
		.pidRate = /*���ٶ�PID*/
		{
			.roll =
				{
					.kp = 50.0,
					.ki = 0.0,
					.kd = 1.0,
					.outputLimit = 0,
				},
			.pitch =
				{
					.kp = 70.0,
					.ki = 0.0,
					.kd = 0.5,
					.outputLimit = 0,
				},
			.yaw =
				{
					.kp = 20.0,
					.ki = 0.0,
					.kd = 0.0,
					.outputLimit = 0,
				},
		},
		.pidPos = /*λ��PID*/
		{
			.vx =
				{
					.kp = 4.5,
					.ki = 0.0,
					.kd = 0.0,
				},
			.vy =
				{
					.kp = 4.5,
					.ki = 0.0,
					.kd = 0.0,
				},
			.vz =
				{
					.kp = 80.0,
					.ki = 130.0,
					.kd = 10.0,
				},

			.x =
				{
					.kp = 4.0,
					.ki = 0.0,
					.kd = 0.6,
				},
			.y =
				{
					.kp = 4.0,
					.ki = 0.0,
					.kd = 0.6,
				},
			.z =
				{
					.kp = 4.0,
					.ki = 1.0,
					.kd = 1.0,
				},
		},

		.servo_initpos =
		{
			.s_left = 1500,
			.s_right = 1500,
			.s_middle = 1500,
		},
		.accBias = 
		{
			.accZero = {32, -39, -92},
			.accGain = {2049, 2045, 2068},
			.bias_isfound = true,
		},
		.adrcRate=
		{
			.roll=
			{
				.td=
				{
					.r  = 16000000,
					.N0 = 2,
				},
				.nlsef_TOC=
				{
					.r  = 4000.0,
					.N1 = 40.0,
					.c  = 0.064,
				},
				.leso=
				{
					.b0 = 0.26,
					.w0 = 600,
				},

			},
			.pitch=
			{
				.td=
				{
					.r  = 16000000,
					.N0 = 2,
				},
				.nlsef_TOC=
				{
					.r  = 4000.0,
					.N1 = 40.0,
					.c  = 0.064,
				},
				.leso=
				{
					.b0 = 0.26,
					.w0 = 600,
				},
			},
		},
		.adrcAngle=
		{
			.roll=
			{
				.td=
				{
					.r  = 0,
					.N0 = 0,
				},
				.nlsef_TOC=
				{
					.r  = 0,
					.N1 = 0,
					.c  = 0,
				},
				.leso=
				{
					.b0 = 0,
					.w0 = 0,
				},

			},
			.pitch=
			{
				.td=
				{
					.r  = 0,
					.N0 = 0,
				},
				.nlsef_TOC=
				{
					.r  = 0,
					.N1 = 0,
					.c  = 0,
				},
				.leso=
				{
					.b0 = 0,
					.w0 = 0,
				},
			},
		},		
		.trimP = 0.f,		 /*pitch΢��*/
		.trimR = 0.f,		 /*roll΢��*/
		.thrustBase = 44500, /*�������Ż���ֵ*/
};
#endif

static u32 lenth = 0;
static bool isInit = false;
static bool isConfigParamOK = false;

static SemaphoreHandle_t xSemaphore = NULL;

static u8 configParamCksum(configParam_t *data)
{
	int i;
	u8 cksum = 0;
	u8 *c = (u8 *)data;
	size_t len = sizeof(configParam_t);

	for (i = 0; i < len; i++)
		cksum += *(c++);
	cksum -= data->cksum;

	return cksum;
}

void configParamInit(void) /*�������ó�ʼ��*/
{
	if (isInit)
		return;

	lenth = sizeof(configParam);
	lenth = lenth / 4 + (lenth % 4 ? 1 : 0);

	STMFLASH_Read(CONFIG_PARAM_ADDR, (u32 *)&configParam, lenth);

	if (configParam.version == VERSION) /*�汾��ȷ*/
	{
		if (configParamCksum(&configParam) == configParam.cksum) /*У����ȷ*/
		{
			//			printf("Version V%1.1f check [OK]\r\n", configParam.version / 10.0f);
			isConfigParamOK = true;
		}
		else
		{
			//			printf("Version check [FAIL]\r\n");
			isConfigParamOK = false;
		}
	}
	else /*�汾����*/
	{
		isConfigParamOK = false;
	}

	if (isConfigParamOK == false) /*���ò�������д��Ĭ�ϲ���*/
	{
		memcpy((u8 *)&configParam, (u8 *)&configParamDefault, sizeof(configParam));
		configParam.cksum = configParamCksum(&configParam);			   /*����У��ֵ*/
		STMFLASH_Write(CONFIG_PARAM_ADDR, (u32 *)&configParam, lenth); /*д��stm32 flash*/
		isConfigParamOK = true;
	}

	xSemaphore = xSemaphoreCreateBinary();

	isInit = true;
}

void configParamTask(void *param)
{
	u8 cksum = 0;

	while (1)
	{
		xSemaphoreTake(xSemaphore, portMAX_DELAY);
		cksum = configParamCksum(&configParam); /*����У��*/

		if (configParam.cksum != cksum)
		{
			configParam.cksum = cksum;				/*����У��*/
			//watchdogInit(500);					/*����ʱ��Ƚϳ������Ź�ʱ�����ô�һЩ*/
			STMFLASH_Write(CONFIG_PARAM_ADDR, (u32 *)&configParam, lenth); /*д��stm32 flash*/
			//watchdogInit(WATCHDOG_RESET_MS);		/*�������ÿ��Ź�*/
		}
	}
}

bool configParamTest(void)
{
	return isInit;
}

void configParamGiveSemaphore(void)
{
	xSemaphoreGive(xSemaphore);
}

void resetConfigParamPID(void)
{
	configParam.pidAngle = configParamDefault.pidAngle;
	configParam.pidRate = configParamDefault.pidRate;
	configParam.pidPos = configParamDefault.pidPos;
}

void saveConfigAndNotify(void)
{
	u8 cksum = configParamCksum(&configParam); /*����У��*/
	if (configParam.cksum != cksum)
	{
		configParam.cksum = cksum;									   /*����У��*/
		STMFLASH_Write(CONFIG_PARAM_ADDR, (u32 *)&configParam, lenth); /*д��stm32 flash*/
	}
}

void changeServoinitpos_configParam(u16 s1, u16 s2, u16 s3)
{
	configParam.servo_initpos.s_left = s1;
	configParam.servo_initpos.s_right = s2;
	configParam.servo_initpos.s_middle = s3;
}

u16 getservoinitpos_configParam(u8 pwm_id)
{
	u16 value = 0;
	switch (pwm_id)
	{
	case PWM_LEFT:
		value = configParam.servo_initpos.s_left;
		break;
	case PWM_RIGHT:
		value = configParam.servo_initpos.s_right;
		break;
	case PWM_MIDDLE:
		value = configParam.servo_initpos.s_middle;
		break;
	}
	return value;
}

accBias_t getaccbias_configParam( void )
{
	return configParam.accBias;
}

void accbias_writeFlash(void)
{
	Axis3f temp=getaccBias();
	configParam.accBias.accZero[0] = (int16_t)temp.x;
	configParam.accBias.accZero[1] = (int16_t)getaccBias().y;
	configParam.accBias.accZero[2] = (int16_t)getaccBias().z;	
	configParam.accBias.bias_isfound = true;
	configParam.accBias.accGain[0] = (int16_t)getaccScale().x;
	configParam.accBias.accGain[1] = (int16_t)getaccScale().y;	
	configParam.accBias.accGain[2] = (int16_t)getaccScale().z;
}
