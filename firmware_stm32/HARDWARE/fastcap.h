/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    fastcap.h
  Author:      汪贤云
  Version:     1.0
  Date:        2026-10-14
  Description: 与 PWM 同步的高速电流采样前端（自研新增模块，尚未并入主循环）。
               解决的问题：原有链路是 100Hz 软件轮询 + 采样后 8 点平均，
               Nyquist 仅 50Hz，而 TIM9 载波 20kHz 正好是其 200 倍，纹波会折叠到 DC，
               AI 只能看到幅值、看不到波形结构。
               本模块用 ADC2 + DMA 以 20kHz 连续采样，每 10ms 恰好一帧 200 点，
               200 点滑动平均的零点正好落在 100Hz 整数倍上，可完整吃掉载波与谐波；
               再输出去均值后的交流 RMS、峰峰值与过零插值计数作为波形特征。
               另提供 2000 点（100ms）原始块抓取，可用 J-Link 直接读 RAM 做离线 FFT。
               初始化顺序约束：必须在 Vbat_Init() 之后调用（ADC 公共预分频由它配置）。
  Function List:
    1. fastcap_init()        初始化 TIM3 + ADC2 + DMA2 循环搬运
    2. fastcap_poll()        主循环取帧，输出一帧的处理结果
    3. fastcap_align_to_pwm() 标定采样相位，使采样点落在 PWM 导通窗口中部
    4. fastcap_arm_raw() / fastcap_is_raw_ready() / fastcap_get_raw() 原始块抓取
    5. fastcap_get_frames() / fastcap_get_overruns() / fastcap_get_status() 运行统计
    6. fastcap_get_last_block() 取最近一帧 200 点原始样本（供波形导出）
  History:
    <author>   <time>      <version>   <desc>
    汪贤云     2026-10-16    1.0         创建文件并按华为规范补充注释
***********************************************************/

/* ==========================================================================
 * fastcap.h  --- 与 PWM 同步的高速电流采样前端（暂存模块，尚未并入工程）
 * --------------------------------------------------------------------------
 * 归属：本项目自研新增模块。文件编码 GBK（与工程内其它 .c/.h 一致，Keil 按 ANSI 打开）。
 *
 * 解决的问题（见 outputs/sampling_audit_20261014/report.md）：
 *   现有链路是「100 Hz 软件轮询 + 采样后 8 点滑动平均」：
 *     - Nyquist 只有 50 Hz；
 *     - TIM9 载波 20 kHz 正好是 100 Hz 的 200 倍，斩波纹波折叠到 DC；
 *     - 8 点平均是采样之后的滤波，压不掉混叠，还把通带压到约 5.6 Hz。
 *   结果：AI 只能看到幅值，看不到任何波形结构。
 *
 * 本模块提供：
 *   1) ADC2 + DMA 以 20 kHz（= PWM 频率）连续采样 PA5/ADC2_IN5；
 *   2) 每 10 ms 恰好一帧 200 点；一帧 200 点滑动平均的零点正好落在
 *      100 Hz 的整数倍上 —— 20 kHz 载波、其各次谐波以及 100 Hz 栅格全部被
 *      零点吃掉，这是「与采样率同源的噪声如何变成 DC」的正解；
 *   3) 去均值后的交流 RMS / 峰值 / 过零插值计数 —— 换向纹波与齿轮啮合频率
 *      落在这个通道里，AI 才有波形特征可用；
 *   4) 一次性原始块抓取（2048 点 = 102.4 ms），供 J-Link 直接读 RAM，
 *      主机做 FFT 出图，作为「确实存在 200~600 Hz 分量」的现场证据。
 *
 * 未做（刻意留给下一阶段）：
 *   - 未改 ADC1 的单次转换路径，vbat.c / current.c 保持原样；
 *   - 未接入主循环、未接入 feature_r、未接入遥测；
 *   - 未做抽取级联的抗混叠设计之外的频谱特征（Goertzel/FFT）。
 *
 * 硬件前置条件（详见 staging_notes.md）：
 *   A. 【不改线】用户 2026-10-14 明确维持现有接线：ACS712 继续串在电池与
 *      驱动板之间。本模块因而不依赖「传感器移到电机引线」，改为把采样点相位
 *      锁定在 PWM 的 ON 窗口中部：
 *        - ON 窗口内（AIN1 = PWM 导通）母线电流就是绕组电流，是连续量；
 *        - 采样点离两个开关沿各约 14 us，避开开关振铃；
 *        - 相位由 fastcap_align_to_pwm() 标定一次；TIM3 与 TIM9 同源 168 MHz、
 *          周期完全相同，标定后不会漂移。
 *      代价与残留风险见 staging_notes.md 第 1、4 节，必须实测确认。
 *   B. 【不改线】不再要求外部 RC。抗混叠改由软件承担（尚未实现），
 *      够不够用必须用原始块 FFT 实测判定。
 *
 * 初始化顺序约束：
 *   Vbat_Init() 必须先调用（它配置 ADC 公共预分频 ADC_Prescaler_Div4 = 21 MHz，
 *   该寄存器 ADC123 共用）。本模块刻意不重复调用 ADC_CommonInit，避免覆盖。
 * ========================================================================== */

#ifndef FASTCAP_H
#define FASTCAP_H   /* 防止重复包含 */

#include "stm32f4xx.h"   /* 标准外设库声明 */

#ifdef __cplusplus  /* C++ 下按 C 方式链接 */
extern "C" {        /* 保持 C 符号名 */
#endif               /* __cplusplus 条件编译段结束 */

/* ---- 采样参数 ---- */
#define FASTCAP_FS_HZ          20000U   /* ADC 采样率，等于 TIM9 载波频率 */
#define FASTCAP_BLOCK_LEN      200U     /* 每帧 200 点 = 10 ms = 恰好一次主循环采样 */
#define FASTCAP_RAW_LEN        2000U    /* 原始抓取长度 = 100 ms @20 kHz（10 整帧） */
#define FASTCAP_DC_SMOOTH_FRAMES 8U     /* 抽取之后（100 Hz 域）的平滑帧数，等价旧的 8 点平均，
                                         * 但此处已经过抗混叠，不再是「先混叠后平均」 */

/* ---- 状态位 ---- */
#define FASTCAP_ST_OK          0x00U    /* 正常 */
#define FASTCAP_ST_NO_ADC      0x01U    /* ADC2 校准未完成 */
#define FASTCAP_ST_OVERRUN     0x02U    /* 主循环没跟上（整帧被覆盖）或 ADC OVR 已自恢复 */
#define FASTCAP_ST_RAIL        0x04U    /* 原始值贴到 0 或 4095，前端饱和/未接 */

/* 一帧 10 ms 的处理结果。电流换算沿用 current.c 的 0.185 V/A 和已标定零点，
 * 本模块只输出原始 ADC 码，避免两份零点标定互相打架。 */
typedef struct {
    uint16_t dc_raw;        /* 本帧 200 点平均：零点落在 100 Hz 整数倍上 */
    uint16_t dc_smooth_raw; /* dc_raw 再经 8 帧（80 ms）平滑，供幅值特征使用 */
    uint16_t ac_rms_raw;    /* 去均值后交流分量 RMS（码） */
    uint16_t ac_peak_raw;   /* 去均值后峰峰值（码） */
    uint16_t ac_cross10;    /* 过零点 x10 计数（线性插值，用于纹波频率估计） */
    uint16_t raw_min;       /* 本帧原始最小值 */
    uint16_t raw_max;       /* 本帧原始最大值 */
    uint8_t  status;        /* FASTCAP_ST_* 位组合 */
} FastcapSample;

/* ---- J-Link 只读快照区 ----
 * 按本工程既有做法（main.c 里 g_scan_duty 的注释），这几个符号刻意不加 static，
 * 以便调试时用符号名直接读内存，确认采集到底跑了没有、块有没有被覆盖。
 * 写入只发生在模块内部；调用方只读，不要写。 */
extern uint16_t g_fastcap_raw[FASTCAP_RAW_LEN];   /* 抓满后为 2000 点连续原始样本 */
extern volatile uint32_t g_fastcap_frames;        /* 已完成帧数 */
extern volatile uint32_t g_fastcap_overruns;      /* 被 DMA 覆盖而丢掉的帧数 */
extern volatile uint32_t g_fastcap_ovr;           /* ADC OVR 自恢复次数 */
extern volatile uint8_t  g_fastcap_status;        /* FASTCAP_ST_* 位组合 */
extern volatile uint32_t g_fastcap_alignTim9;     /* 对齐瞬间读到的 TIM9->CNT */
extern volatile uint16_t g_fastcap_alignTim3;     /* 对齐时写入 TIM3->CNT 的值 */

/* 初始化：TIM3 20 kHz 触发 + ADC2 + DMA2 Stream2/Channel1 循环搬运。
 * 必须在 Vbat_Init() 之后调用。无返回值：失败状态通过 fastcap_get_status() 读取。 */
void fastcap_init(void);  /* 初始化 TIM3 触发 + ADC2 + DMA2 循环搬运（须在 Vbat_Init() 之后调用） */

/* 由主循环每约 10 ms 调用一次；有新帧时填入 out 并返回 1，否则返回 0。
 * out 在返回 1 之前不被修改。上下文：主循环可调用，不可在 ISR 中调用。 */
uint8_t fastcap_poll(FastcapSample *out);  /* 主循环调用：取一帧 200 点的处理结果，成功返回 1，未就绪返回 0 */

/* 启动一次原始块抓取（FASTCAP_RAW_LEN 点连续样本）。抓取期间不能丢帧。 */
void fastcap_arm_raw(void);  /* 启动一次原始块抓取（2000 点） */

/* 把 TIM3 的采样相位对齐到 TIM9 的 ON 窗口中部（说明见 fastcap.c）。
 * 必须在 PWM_Motor_Config() 启动 TIM9 之后调用；fastcap_init() 内部已调用一次。
 * 若之后有人重置了 TIM9 计数器，需要重新调用。
 * 返回 1 = 已写入对齐值；是否真的落在 ON 窗口内仍需按 staging_notes.md 验收。 */
uint8_t fastcap_align_to_pwm(void);  /* 标定采样相位：把采样点锁在 PWM 导通窗口中部，返回 1 成功 */

/* 原始块是否已经抓满（1 = 可以读）。 */
uint8_t fastcap_is_raw_ready(void);  /* 查询原始块是否已抓满 */

/* 取原始块首地址与长度；未抓满时返回 NULL 并把 *len 置 0。缓冲区归模块所有，
 * 调用者只读，不释放，下次 fastcap_arm_raw() 后可被覆盖。 */
const uint16_t *fastcap_get_raw(uint32_t *len);  /* 取原始块首地址，len 回填点数；仅供只读使用 */

/* 运行计数：完整帧数 / 因主循环过慢被覆盖的帧数 / 初始化状态。 */
uint32_t fastcap_get_frames(void);  /* 取已完成帧数 */
uint32_t fastcap_get_overruns(void);  /* 取因 DMA 覆盖而丢弃的帧数 */
uint8_t  fastcap_get_status(void);  /* 取状态位（FASTCAP_ST_* 的组合） */

/* 取最近完成一帧的 200 点原始样本，供波形导出使用。
 * 每次 fastcap_poll() 成功返回 1 时，本缓冲区被刷新为该帧内容。
 * 返回：指向 FASTCAP_BLOCK_LEN 个 uint16 样本的只读指针，永不返回 NULL。
 * 注意：内容会被下一次 fastcap_poll() 覆盖，调用方必须「取到就立刻用」。
 *       上下文：主循环。不可在中断中读取，否则可能读到半新半旧的帧。
 */
const uint16_t *fastcap_get_last_block(void);  /* 取最近一帧 200 点原始样本（软件快照，只读） */

#ifdef __cplusplus
}
#endif  /* __cplusplus 条件编译段结束 */
#endif /* FASTCAP_H 头文件结束 */
