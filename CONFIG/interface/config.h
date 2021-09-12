#ifndef __CONFIG_H
#define __CONFIG_H
#include "nvic.h"
#include "stdio.h"	/*printf 调用*/

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 配置文件代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

#define BOOTLOADER_SIZE		(16*1024)	
//#define BOOTLOADER_SIZE		0	
#define CONFIG_PARAM_SIZE	(16*1024)

#define CONFIG_PARAM_ADDR 	(FLASH_BASE + BOOTLOADER_SIZE)	/*16K bootloader*/
#define FIRMWARE_START_ADDR (FLASH_BASE + BOOTLOADER_SIZE + CONFIG_PARAM_SIZE)	/*16K bootloader+ 16 模拟eeprom*/


#define DEG2RAD		0.017453293f	/* 度转弧度 π/180 */
#define RAD2DEG		57.29578f		/* 弧度转度 180/π */

#define P_NAME "BI_Fly"
#define MCU_ID_ADDRESS          0x1FFF7A10
#define MCU_FLASH_SIZE_ADDRESS  0x1FFF7A22

//FOUR_WING 四翅  DOUBLE_WING 二翅
#define FOUR_WING  


//自定义的一些配置宏用于完成代码的切换

//#define ENABLE_GET_TASK_STATUS     //查看系统每个线程大概占用了多少CPU的资源

// BOARD_VERTICAL 板子竖着放置  BOARD_HORIZONTAL 板子水平放置
#define BOARD_VERTICAL            //板子竖着放置


#define PCBV4_5                     //PCBV4.5将电池的电压检测换到主控上来检测，检测引脚为PA5 模拟输入 PA4 灌电流，电机的引脚换了

//其中一个电机的引脚换成了PC7
//#define PC7_OUT_ENABLE              

//ADRC_CONTROL ADRC控制姿态Roll轴  PID_CONTROL PID控制姿态
#define PID_CONTROL


#define USE_DYN_NOTCH_FILTER

// #define TEST


#endif /* __CONFIG_H */
