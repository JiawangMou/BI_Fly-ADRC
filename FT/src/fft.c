#include "fft.h"
#include "arm_const_structs.h"
#include "arm_math.h"
#include "uart_3.h"
#include "axis.h"
#include "dyn_notch_filter.h"
/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"


/*********************************************************************************
  *Copyright(C),
  *FileName:  fft.c
  *Author:  Jiawang Mou
  *Version:  V1.0
  *Date:  2021.01.01
  *Description:
  *Others:  //��������˵��
  *Function List:  //��Ҫ�����б�ÿ����¼Ӧ���������������ܼ�Ҫ˵��
     1.��������
     2.��������
  *History:  //�޸���ʷ��¼�б�ÿ���޸ļ�¼Ӧ�����޸����ڡ��޸��߼��޸����ݼ��
     1.Date:
       Author:
       Modification:
     2.��������
**********************************************************************************/

#define FFT_LENGTH 1024
#define FFT_LOOP_HZ 1000 //��λ��Hz
#define FFT_LOOP_DT_US (u32)(1e6f / FFT_LOOP_HZ) //��λ ��us
#define FFT_LOOP_DT_mS (u32)(1e3f / FFT_LOOP_HZ) //��λ ��ms

float32_t fft_inputbuf[FFT_LENGTH];  // FFT��������
float32_t fft_outputbuf[FFT_LENGTH]; // FFT�������
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
    //arm_cfft_radix4_init_f32(&scfft,FFT_LENGTH,0,1);//��ʼ��scfft�ṹ�壬�趨FFT��ز���
    u32 lastWakeTime = getSysTickCnt();
    dynNotchInit(&configParam.dynNotchConfig ,FFT_LOOP_DT_US);
    
    while (1) {
      
    vTaskDelayUntil(&lastWakeTime, FFT_LOOP_DT_mS); /*1ms����*/
		// for (i = 0; i < FFT_LENGTH; i++) //�����ź�����
		// {
		// 	fft_inputbuf[i] = 10 + 2 * arm_sin_f32(2 * PI * i * 10 / FFT_LENGTH) + 
    //                                5 * arm_sin_f32(2 * PI * i * 50 / FFT_LENGTH) + 
    //                               10 * arm_cos_f32(2 * PI * i * 300 / FFT_LENGTH); //�ź�ʵ����ֱ������100,1HZ�źŷ�ֵΪ10��50HZ�źŷ�ֵΪ20��300HZ�źŷ�ֵΪ30��
		// 	// fft_inputbuf[2*i+1]=0;//�ź��鲿��ȫ��Ϊ0
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
//         // arm_cfft_radix4_f32(&scfft,fft_inputbuf);	//FFT���㣨��4��
//         // arm_cmplx_mag_f32(fft_inputbuf,fft_outputbuf,FFT_LENGTH);	//��������������ģ�÷�ֵ
// //        printf("FFTout\r\n");
// //        for (i = 0; i < FFT_LENGTH / 2; i++) {
// //            printf("%f\r\n", fft_outputbuf[i]);
// //        }
        curentTime = getSysTickCnt();
        dutytime   = curentTime - lastTime;
    }
    vTaskDelete(NULL);
}
