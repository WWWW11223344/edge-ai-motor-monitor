/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    feature_r.h
  Author:      汪贤云
  Version:     1.0
  Date:        2026-10-19
  Description: 转速单变量特征 R 的接口定义。
               R = 转速(rpm) / 母线电压(V)，单位 rpm/V。
               本模块只负责按 100 ms 节拍把 R 攒成一个定长序列，
               判定与模型推理由 ai_monitor 层负责。
  Function List:
    1. feature_r_reset()          清空窗口
    2. feature_r_tick()           每 10 ms 调一次，内部按 100 ms 节拍推点
    3. feature_r_get()            取窗口内 32 点 R 序列（时间顺序：旧 -> 新）
    4. feature_r_current()        取最近一次 R（供屏显）
    5. feature_r_current_valid()  取最近一次 R 是否处于有效运行区间
    6. feature_r_fill()           取窗口已攒点数（0 ~ FEATURE_R_WINDOW）
  History:
    <author>   <time>      <version>   <desc>
    汪贤云     2026-10-19    1.0         创建文件，转速单变量方案
***********************************************************/

#ifndef __FEATURE_R_H
#define __FEATURE_R_H  /* 防止重复包含 */

#include <stddef.h>    /* 提供 size_t */
#include <stdint.h>    /* 提供 uint8_t / uint32_t */

/* ==========================================================================
 * 参数（改了就得重新导训练数据并重刷模型，不要随手改）
 * --------------------------------------------------------------------------
 *   FEATURE_R_WINDOW   32     窗口点数 = 模型输入长度
 *   FEATURE_R_PUSH_MS  100    推点周期，与编码器采样周期一致
 *                             所以窗长 = 32 x 100 ms = 3.2 s
 *   FEATURE_R_RPM_MIN  20     低于此转速视为停转（与 main.c 的 20 rpm 门限一致）
 *   FEATURE_R_VBAT_MIN 1.0    低于此电压视为分压通路失效（分母不可信）
 * ========================================================================== */
#define FEATURE_R_WINDOW        32U
#define FEATURE_R_PUSH_MS       100U
#define FEATURE_R_RPM_MIN       20.0f
#define FEATURE_R_VBAT_MIN      1.0f

void    feature_r_reset(void);        /* 清空窗口（不动节拍时基） */

uint8_t feature_r_tick(float rpm, float vbat_v, uint8_t vbat_valid,
                       uint32_t now_ms);   /* 1 = 本次真的推入了一个新点 */

uint8_t feature_r_get(float *output, size_t capacity);  /* 1 = 窗口已满且已写出 */

float   feature_r_current(void);      /* 最近一次 R，无效时返回上一个有效值 */
uint8_t feature_r_current_valid(void);/* 1 = 当前处于有效运行区间 */
uint8_t feature_r_fill(void);         /* 已攒点数 0 ~ FEATURE_R_WINDOW */

#endif  /* __FEATURE_R_H 头文件结束 */
