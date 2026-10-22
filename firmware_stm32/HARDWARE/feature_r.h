/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    feature_r.h
  Author:      汪贤云
  Version:     1.0
  Date:        2026-10-19
  Description: 转速单变量特征 R 的接口定义。
               R = 转速(rpm) / 母线电压(V)，单位 rpm/V。
               本模块按 100 ms 节拍把 R 攒成定长序列（供屏显与规则判据），
               另把 q = rpm / vbat 的 8 点滚动峰峰值攒成 32 点窗口喂 AI，
               判定与模型推理由 ai_monitor 层负责。
  Function List:
    1. feature_r_reset()          清空窗口
    2. feature_r_tick()           每 10 ms 调一次，内部按 100 ms 节拍推点
    3. feature_r_get()            取窗口内 32 点滚动峰峰值（时间顺序：旧 -> 新）
    4. feature_r_current()        取最近一次 R（供屏显）
    5. feature_r_current_valid()  取最近一次 R 是否处于有效运行区间
    6. feature_r_fill()           取峰峰值窗口已攒点数（0 ~ FEATURE_R_WINDOW）
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
 *   FEATURE_R_DUTY_REF 58.0   归一化基准档位：1 档 58% 系数为 1.0，2 档 64% 为 8/14
 *   FEATURE_R_DUTY_MIN 40.0   低于此占空比落在启动死区，R 无物理意义，丢窗
 * ========================================================================== */
#define FEATURE_R_WINDOW        32U
#define FEATURE_R_PUSH_MS       100U
#define FEATURE_R_RPM_MIN       20.0f
#define FEATURE_R_VBAT_MIN      1.0f

#define FEATURE_R_RIPPLE_SPAN   8U      /* 滚动峰峰值的跳样跳数：8 点 = 0.8 s。
                                       * 为什么不用整窗 32 点：太长会把瞬时卡滞淡化，太短又容易受噪声影响。 */

/* 档位归一化常数。空载下 R = 转速/母线电压 对占空比是线性的，
 * 且在 50% 附近过零（实测 rpm = 17.5 x (duty - 50)），所以
 *   R = (rpm / vbat) x (REF - KNEE) / (duty - KNEE)
 * 才是把档位彻底除掉的正确形式。
 * 实测两档（归一后均值）：58% -> 13.054，64% -> 13.878，残差 6.3%，
 * 仍大于模型带宽，故模型必须用两档数据一起训；旧公式乘 (REF / duty) 为
 * 13.21 / 21.27 / 23.83，差 61% —— 这才是必须改成膝点模型的原因。
 * 58% 档下两式系数均为 1.0，所以 1 档行为完全不变。判决改用 dR 不受其影响。 */
#define FEATURE_R_DUTY_REF      58.0f   /* 归一化基准档位：除以它是为了让基准档的 R 和原始 rpm/V 相等 */
#define FEATURE_R_DUTY_KNEE     50.0f   /* 占空比膝点：实测 rpm = 17.5 x (duty - 50)，
                                         * 50% 以下电机根本不转，是死区不是工作区。 */
#define FEATURE_R_DUTY_MIN      40.0f   /* 低于此占空比不再算数，R 无意义。
                                         * 2026-10-21 从 45 降到 40：否则低速档 48% 会被
                                         * 误判成无效而丢窗；40 是实测的起动死区边界。 */

void    feature_r_reset(void);        /* 清空窗口（不动节拍时基） */

uint8_t feature_r_tick(float rpm, float vbat_v, uint8_t vbat_valid,
                       float duty_pct, uint32_t now_ms);  /* 1 = 本次真的推入了一个新点 */

/* 1 = 峰峰值窗口已满且已写出（约需 3.9 s） */
uint8_t feature_r_get(float *output, size_t capacity);

float   feature_r_current(void);      /* 最近一次 R，无效时返回上一个有效值 */
uint8_t feature_r_current_valid(void);/* 1 = 当前处于有效运行区间 */
uint8_t feature_r_fill(void);         /* 已攒点数 0 ~ FEATURE_R_WINDOW */
/* 窗口内 R 的相对峰峰值（跳变幅度），单位 %。
 * 2026-10-21 新增（按导师思路）：绝对 R 值随档位变，但跳变幅度不随档位变，
 * 所以它是唯一能跨档位通用的判据：稳定运行时只有 2~4%，
 * 捏一下停一下地制造跳变时会顶到 15~25%。窗口未满时无效。 */
float   feature_r_ripple(void);       /* 跳变幅度(%)，窗口满才有意义 */
uint8_t feature_r_ripple_valid(void); /* 1 = 窗口已满，幅度可用 */
float   feature_r_ripple_abs(void);   /* 绝对跳变幅度(rpm/V)，跨档位通用的主判据 */


#endif  /* __FEATURE_R_H 头文件结束 */
