/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    pwm.h
  Author:      汪贤云
  Version:     1.0
  Date:        2026-09-21
  Description: PWM 驱动对外接口，提供 LED 调光与电机调速两路 PWM。
               LED：PF9 / TIM14_CH1，500Hz；电机：PE5、PE6 / TIM9_CH1、CH2，20kHz。
               电机接口的 speed 参数直接表示占空比百分数（0~100），
               其上限由调用方保证：本工程 main.c 最大只给 58%，
               因为母线 12.6V x 58% = 7.3V 已接近电机额定电压 7.4V。
  Function List:
    1. PWM_LED_Init()      LED 调光 PWM 初始化（与 led.c 互斥）
    2. PWM_Motor_Config()  电机 PWM 初始化（含 PA8 STBY 使能）
    3. Motor_Config()      运行期设置转速与方向
  History:
    <author>   <time>      <version>   <desc>
    汪贤云     2026-09-21    1.0         创建文件
    汪贤云     2026-10-16    1.1         按《华为编程规范》补充中文注释
    汪贤云     2026-10-16    1.1         按《华为编程规范》补充中文注释
    汪贤云     2026-10-16    1.1         按《华为编程规范》补充中文注释
***********************************************************/
#ifndef _PWM_H_
#define _PWM_H_
#include "stm32f4xx.h"  /* 标准外设库声明 */
void PWM_LED_Init(void);      /* LED 调光 PWM 初始化（PF9 / TIM14_CH1） */
void PWM_Motor_Config(void);  /* 电机 PWM 初始化（PE5/PE6 -> TIM9 CH1/CH2） */
void Motor_Config(uint16_t speed,uint16_t dir); /* 设置占空比（百分数）与转向 */

#endif  /* __PWM_H 头文件结束 */
