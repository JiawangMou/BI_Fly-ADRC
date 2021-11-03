#ifndef __MOTORS_H
#define __MOTORS_H
#include "sys.h"
#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * �����������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

/*�Ĵ���--- 180M��Ƶ�£�APB1 Timer clk = 90Mzh,2��Ƶ  8λ�������ԼΪ351K PWM */
/*���  --- 180M��Ƶ��,180��Ƶ  3333Ԥװ�� ���300HZ PWM */
#define TIM_CLOCK_HZ 90000000
#define MOTORS_PWM_BITS 8
#define MOTORS_PWM_PERIOD ((1 << MOTORS_PWM_BITS) - 1)
#define MOTORS_PWM_PRESCALE 0

#ifdef PCBV4_5

#define MOTORS_2_PWM_PRESCALE 1     // TIM8 ��APB2 Timer Clock 180M ��Ҫ1��Ƶ
#define TIM_MOTOR TIM4
#define TIM_MOTOR_2 TIM8
// Pin Defines
#define PWMPD12 0
#define PWMPD13 1
#define PWMPC7 5

#else
#define PWMPD12 0

#endif


#define SERVOS_PWM_PRESCALE (90 - 1)
#define SERVOS_PWM_PERIOD 3333

#define TIM_SERVO TIM3

//#define ENABLE_THRUST_BAT_COMPENSATED /*ʹ�ܵ�����Ų���*/



#define NBR_OF_MOTORS 7
#define PWMF1 PWMPD12
#define PWMF2 PWMPC7
#define PWM_LEFT 2
#define PWM_RIGHT 3
#define PWM_MIDDLE 4
#define PWMR PWMPD13

#define MOTORS_TEST_RATIO (u16)(0.2 * (1 << 16)) //20%
#define MOTORS_TEST_ON_TIME_MS 50
#define MOTORS_TEST_DELAY_TIME_MS 150

#define MOTORS_MAXPWM (0.4 * 65535)

#ifdef FOUR_WING
#define SERVO_MAXPWM 2050 //2.1ms
#define SERVO_MINPWM 950  //0.9ms
#define SERVO_HalfRANGE ((SERVO_MAXPWM - SERVO_MINPWM) / 2)

#elif defined DOUBLE_WING

#define SERVO_HalfRANGE 350
#define SERVO_MAXPWM 2100 //2.1ms
#define SERVO_MINPWM 900  //0.9ms
#endif
 
void motorsInit(void);                    /*�����ʼ��*/
bool motorsTest(void);                    /*�������*/
void motorsSetRatio(u32 id, u16 ithrust); /*���õ��ռ�ձ�*/
void servoSetPWM(u8 id, u16 value);
u32 servoPWMLimit(u16 value); /*����ʵ����װ��ֵ��900~2100��Χ��*/

#endif /* __MOTORS_H */
