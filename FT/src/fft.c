#include "fft.h"
#include "arm_const_structs.h"
#include "arm_math.h"
#include "uart_3.h"
#include "axis.h"
#include "dyn_notch_filter.h"
/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"


/*********************************************************************************
  *Copyright(C),
  *FileName:  fft.c
  *Author:  Jiawang Mou
  *Version:  V1.0
  *Date:  2021.01.01
  *Description:
  *Others:  //其他内容说明
  *Function List:  //主要函数列表，每条记录应包含函数名及功能简要说明
     1.…………
     2.…………
  *History:  //修改历史记录列表，每条修改记录应包含修改日期、修改者及修改内容简介
     1.Date:
       Author:
       Modification:
     2.…………
**********************************************************************************/

#define FFT_LENGTH 1024
#define FFT_LOOP_HZ 1000 //单位：Hz
#define FFT_LOOP_DT_US (u32)(1e6f / FFT_LOOP_HZ) //单位 ：us
#define FFT_LOOP_DT_mS (u32)(1e3f / FFT_LOOP_HZ) //单位 ：ms

float32_t fft_inputbuf[FFT_LENGTH];  // FFT输入数组
float32_t fft_outputbuf[FFT_LENGTH]; // FFT输出数组
float32_t mag_outputbuf[FFT_LENGTH];
extern const arm_rfft_fast_instance_f32 arm_rfft_fast_sR_f32_len1024;
arm_rfft_fast_instance_f32              S;
arm_cfft_radix4_instance_f32            scfft;
u16                                     i          = 0;
u32                                     lastTime   = 0;
u32                                     curentTime = 0;
u16                                     dutytime   = 0;
unsigned char*                          p;

void fftTask(void* param)
{
    arm_status status = arm_rfft_fast_init_f32(&S, FFT_LENGTH);
    //arm_cfft_radix4_init_f32(&scfft,FFT_LENGTH,0,1);//初始化scfft结构体，设定FFT相关参数
    u32 lastWakeTime = getSysTickCnt();
    dynNotchInit(&configParam.dynNotchConfig ,FFT_LOOP_DT_US);
    
    while (1) {
      
    vTaskDelayUntil(&lastWakeTime, FFT_LOOP_DT_mS); /*1ms周期*/
		// for (i = 0; i < FFT_LENGTH; i++) //生成信号序列
		// {
		// 	fft_inputbuf[i] = 10 + 2 * arm_sin_f32(2 * PI * i * 10 / FFT_LENGTH) + 
    //                                5 * arm_sin_f32(2 * PI * i * 50 / FFT_LENGTH) + 
    //                               10 * arm_cos_f32(2 * PI * i * 300 / FFT_LENGTH); //信号实部，直流分量100,1HZ信号幅值为10，50HZ信号幅值为20，300HZ信号幅值为30。
		// 	// fft_inputbuf[2*i+1]=0;//信号虚部，全部为0
		// }
        lastTime = getSysTickCnt();
        dynNotchUpdate(Y);
//         arm_rfft_fast_f32(&S, fft_inputbuf, fft_outputbuf, 0);
//         // for (i = 0; i < FFT_LENGTH; i++) {
//         //     printf("%f\r\n", fft_outputbuf[i]);
//         // }

//         arm_cmplx_mag_f32(fft_outputbuf, mag_outputbuf, FFT_LENGTH);
//         // for (i = 0; i < FFT_LENGTH; i++) {
//         //     printf("%f\r\n", mag_outputbuf[i]);
//         // }
//         // arm_cfft_f32(&arm_cfft_sR_f32_len1024,fft_inputbuf,0,1);
//         // arm_cmplx_mag_f32(fft_inputbuf, fft_outputbuf, FFT_LENGTH);
//         // arm_cfft_radix4_f32(&scfft,fft_inputbuf);	//FFT计算（基4）
//         // arm_cmplx_mag_f32(fft_inputbuf,fft_outputbuf,FFT_LENGTH);	//把运算结果复数求模得幅值
// //        printf("FFTout\r\n");
// //        for (i = 0; i < FFT_LENGTH / 2; i++) {
// //            printf("%f\r\n", fft_outputbuf[i]);
// //        }
        curentTime = getSysTickCnt();
        dutytime   = curentTime - lastTime;
    }
    vTaskDelete(NULL);
}
