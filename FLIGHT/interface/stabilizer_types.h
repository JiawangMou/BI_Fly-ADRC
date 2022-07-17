#ifndef __STABILIZER_TYPES_H
#define __STABILIZER_TYPES_H
#include "sys.h"
#include "config.h"
#include <stdbool.h>
#include "sensors_types.h"
#include "arm_math.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * �ṹ�����Ͷ���	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) �������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

#if defined(__CC_ARM) 
	#pragma anon_unions
#endif

typedef struct  
{
	u32 timestamp;	/*ʱ���*/
	union 
	{
		struct 
		{
			float roll;
			float pitch;
			float yaw;
		};
		float axis[3];
	};
} attitude_t;

struct  vec3_s 
{
	u32 timestamp;	/*ʱ���*/

	float x;
	float y;
	float z;
};

typedef struct vec3_s point_t;
typedef struct vec3_s velocity_t;
typedef struct vec3_s acc_t;

/* Orientation as a quaternion */
typedef struct quaternion_s 
{
	uint32_t timestamp;

	union 
	{
		struct 
		{
			float q0;
			float q1;
			float q2;
			float q3;
		};
		struct 
		{
			float x;
			float y;
			float z;
			float w;
		};
	};
} quaternion_t;

typedef struct toaMeasurement_s 
{
	int8_t senderId;
	float x, y, z;
	int64_t rx, tx;
} toaMeasurement_t;

typedef struct tdoaMeasurement_s {
  point_t anchorPosition[2];
  float distanceDiff;
  float stdDev;
} tdoaMeasurement_t;

typedef struct positionMeasurement_s 
{
	union 
	{
		struct 
		{
			float x;
			float y;
			float z;
		};
		float pos[3];
	};
	float stdDev;
} positionMeasurement_t;

typedef struct distanceMeasurement_s 
{
	union 
	{
		struct 
		{
			float x;
			float y;
			float z;
		};
		float pos[3];
	};
	float distance;
	float stdDev;
} distanceMeasurement_t;

typedef struct zRange_s 
{
	uint32_t timestamp;	//ʱ���
	float distance;		//��������
	float distance_uncomp;
	float rawdata;
	float quality;		//���Ŷ�
} zRange_t;

/** Flow measurement**/
typedef struct flowMeasurement_s 
{
	uint32_t timestamp;
	union 
	{
		struct 
		{
			float dpixelx;  // Accumulated pixel count x
			float dpixely;  // Accumulated pixel count y
		};
		float dpixel[2];  // Accumulated pixel count
	};
	float stdDevX;      // Measurement standard deviation
	float stdDevY;      // Measurement standard deviation
	float dt;           // Time during which pixels were accumulated
} flowMeasurement_t;


/** TOF measurement**/
typedef struct tofMeasurement_s 
{
	uint32_t timestamp;
	float distance;
	float stdDev;
} tofMeasurement_t;

typedef struct
{
	float pressure;
	float temperature;
	float asl;
} baro_t;
typedef struct
{
	Axis3i16 offset;
	Axis3u16 radius;
} mag_calib;

typedef struct
{
	Axis3f acc;
	Axis3f gyro;  // Unit: °/s
	Axis3f gyro_R;  //rad  Unit: /s 
	Axis3f mag;
	baro_t baro;
	point_t position;
	zRange_t zrange;
	mag_calib mag_calibration;
	Axis3f velocity;   
} sensorData_t;

typedef struct
{
	attitude_t attitude;
	attitude_t attitude_R;
	quaternion_t attitudeQuaternion;
	point_t position;
	velocity_t velocity;
	acc_t acc;
	bool isRCLocked;
} state_t;

enum dir_e
{
	CENTER=0,
	FORWARD,
	BACK,
	LEFT,
	RIGHT,
};

enum u_id
{
	T_l=0,
	T_r,
	beta_l,
	beta_r,
};



typedef enum
{
	modeDisable = 0,/*�ر�ģʽ*/
	modeAbs,		/*����ֵģʽ*/
	modeVelocity	/*����ģʽ*/
} mode_e;

typedef struct
{
	mode_e x;
	mode_e y;
	mode_e z;
	mode_e roll;
	mode_e pitch;
	mode_e yaw;
}mode_t;

#ifdef USE_MBD
typedef struct
{
    attitude_t attitude;        // deg
	attitude_t attitudedesired; // deg
    attitude_t attitudeRate;    // deg/s
    point_t    position;        // cm
    velocity_t velocity;        // transient velocity setpoint cm/s
    Axis3f     acc;
    mode_t     mode;
    float      thrust;
} setpoint_t;
#else
typedef struct
{
	attitude_t attitude;		// deg	
	attitude_t attitudeRate;	// deg/s
	point_t position;         	// m
	velocity_t velocity;      	// m/s
	mode_t mode;
	float thrust;
} setpoint_t;
#endif
#ifdef USE_MBD
typedef struct 
{
	float vel;
	float pos;
	float MBD;
}thrust_t;

typedef struct
{
	s16 roll;
	s16 pitch;
	s16 yaw;
	float thrust;
	thrust_t thrust_part;
	enum dir_e flipDir;		/*翻滚方向*/
	float32_t Tao_Fz[4];    //Tao_Fz[0]~Tao_Fz[2]： Tao, unit: g*cm^2/s^2;   Tao_Fz[3] ：Fz,  unit: g*cm/s^2; 
	float U[4];				//U[0]~U[3]: T_lcos(beta_l), T_lsin(beta_l),T_rcos(beta_r),T_rsin(beta_r)  unit: mN; 
	float actuator[4];  	//T_l  T_r  beta_l  beta_r	 unit: mN, rad 
	float actuator_Tf[4];	//T_l  T_r  beta_l  beta_r   unit: mN, rad 
	arm_matrix_instance_f32 mat_Tao_Fz_41;
	arm_matrix_instance_f32 mat_actuator_41;
}control_t;
#else

typedef struct
{
	s16 roll;
	s16 pitch;
	s16 yaw;
	float thrust;
	enum dir_e flipDir;		/*��������*/
} control_t;

#endif
typedef struct
{
	union 
	{
		struct 
		{
			s16 sensorsAcquire_tick;
			s16 imuUpdate_tick;
			s16 positionEstimate_tick;
			s16 commanderGetSetpoint_tick;
			s16 getOpFlowData_tick;
			s16 flyerFlipCheck_tick;
			s16 stateControl_tick;
			s16 motorControl_tick;
		};
		s16 stabilizer_tick[8];
	};
} Debug_stabi_tick_t;


#define RATE_5_HZ		5
#define RATE_10_HZ		10
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000

#define MAIN_LOOP_RATE 	RATE_1000_HZ
#define MAIN_LOOP_DT	(u32)(1000/MAIN_LOOP_RATE)	/*��λms*/

#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (MAIN_LOOP_RATE / RATE_HZ)) == 0)


#endif


