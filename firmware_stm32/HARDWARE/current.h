/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    current.h
  Author:      汪贤云
  Version:     1.2
  Date:        2026-10-19
  Description: ACS712 电流检测接口。传感器输出接 PA5（ADC1_IN5）。
               换算：I = (Vout - 零点) / 0.185，其中 0.185V/A 为 5A 版灵敏度、
               零点理论值为 VCC/2 = 2.5V（不是 1.65V）。
               零点在上电时校准一次，并在电机确认停转时缓慢跟踪温漂。
               【模块定位：只作辅助观测量，不参与 AI 判定】
               ------------------------------------------------------
               电流值只用于 OLED 显示、遥测上报、传感器健康判据、功率回路排障四处。
               AI 的唯一输入是 feature_r 模块产出的 R = 转速 / 母线电压，电流不进模型：
               电流未除掉母线电压与占空比，换电池换档位后阈值都要重标；R 已除掉电压。
               实测 R 的两簇完全不重叠，电流的两簇仍有约 16 mA 重叠区。
  Function List:
    1. Current_Init()        初始化 PA5 为模拟输入
    2. Current_Calibrate()   停机校准零点
    3. Current_GetAmp()      取当前电流(A)
    4. Current_TrackZero()   停机时跟踪零点
    5. Current_IsValid()     查询数据是否可信
  History:
    <author>   <time>      <version>   <desc>
    汪贤云     2026-10-16    1.0         创建文件并按华为规范补充注释
    汪贤云     2026-10-19    1.2         明确模块定位：只作辅助观测量，不进入 AI 判定
***********************************************************/

#ifndef __CURRENT_H
#define __CURRENT_H  /* 防止重复包含 */

#include "stm32f4xx.h"  /* 标准外设库声明 */

/* ==========================================================================
 * 第 5 步 · 电流检测（ACS712-05B 模块 -> PA5 / ADC1_IN5）
 * --------------------------------------------------------------------------
 * 接线（模块不是裸芯片）：
 *   IP+  -> 电池正极        IP-  -> D153C 的 VIN（串进主电流路）
 *   VCC  -> 5V（必须 5V）    GND  -> 共地
 *   OUT  -> PA5（P11 第 3 脚）
 *
 * 换算：
 *   Vout = 2.5V + 0.185V/A * I   （5A 版，零点 = VCC/2 = 2.5V，不是 1.65V）
 *   I = (Vout - 零点) / 0.185
 *
 * 零点必须每次上电校准一次（停转时做），否则几十毫安的漂移会让屏显电流出现静差。
 * ========================================================================== */

#define ACS712_SENSITIVITY   0.185f    /* V/A，5A 版固定值 */
#define ACS712_VREF          3.30f     /* ADC 参考电压 = VDDA = 3.3V */
#define ACS712_ADC_FULL      4095.0f   /* 12 位 ADC 满量程（0~4095） */
#define ACS712_ADC_AVG       64U       /* 零点校准取 64 次平均 */
#define ACS712_ZERO_MIN     2.10f     /* 无电流时的输出下限：低于此值视为异常 */
#define ACS712_ZERO_MAX     2.90f

void  Current_Init(void);              /* 配 PA5 为模拟输入（ADC1 已在 vbat.c 开好） */
float Current_Calibrate(void);         /* 停转时校准零点，返回零点电压(V) */
float Current_GetAmp(void);            /* 当前电流，单位 A */
void  Current_TrackZero(void);         /* 停机状态下的零点缓慢跟踪 */
uint8_t Current_IsValid(void);        /* 1 = ACS712 在位且最近一次读取可用 */

#endif
