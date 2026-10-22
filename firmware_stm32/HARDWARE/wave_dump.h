/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    wave_dump.h
  Author:      汪贤云
  Version:     1.0
  Date:        2026-10-19
  Description: 20 kHz 电流波形的上位机导出通道（USART1 + DMA2）。
               把 fastcap 每 10 ms 采到的一帧 200 点原始 ADC 样本，按 100 ms
               分频通过 USART1（PA9，波特率 115200）以 418 字节定长二进制帧
               发给电脑，供 NanoEdge AI Studio 训练与离线频谱分析使用。
               只发不收：本通道不配置 USART1 接收中断，也不解析任何上位机命令。
  Function List:
    1. WaveDump_Init()        初始化 USART1 + DMA2 Stream7 并复位统计
    2. WaveDump_Poll()        主循环调用，按占空比与分频决定是否导出一帧
    3. WaveDump_SetEnabled()  手动开关导出（0 = 停，1 = 开）
    4. WaveDump_IsEnabled()   查询当前是否允许导出
    5. WaveDump_GetSent()     取已成功发送的帧数
    6. WaveDump_GetDropped()  取因 DMA 未发完而丢弃的帧数
  History:
    <author>   <time>      <version>   <desc>
    汪贤云     2026-10-19    1.0         创建文件，按华为规范编写注释
***********************************************************/

/* ==========================================================================
 * wave_dump.h  --- 波形导出通道对外接口
 * --------------------------------------------------------------------------
 * 为什么要单独建一个模块，而不是把 printf 塞进 fastcap：
 *   fastcap.c 的职责是「采样与统计」，它不应当知道数据往哪里去；
 *   导出属于通信职责，放到独立模块后，将来要换传输介质（USB / SPI / SD 卡）
 *   只改本模块，不动采样前端。
 *
 * 为什么用二进制而不是 CSV：
 *   一帧 200 点。CSV 每点约 5 个字符，一帧超过 1000 字节；
 *   二进制每点 2 字节，整帧 418 字节。115200 波特率下前者要 87 ms，
 *   后者只要 36 ms，后者才能稳定地在 100 ms 周期内发完。
 *
 * 帧格式（小端，共 418 字节）：
 *   偏移   长度   内容
 *   0      2      帧头 0xA5 0x5A
 *   2      1      版本号，当前 0x01
 *   3      2      点数，固定 200
 *   5      4      帧序号，上电从 0 开始，每发一帧加 1
 *   9      1      fastcap 状态位（FASTCAP_ST_*）
 *   10     2      本帧直流分量 dc_raw（ADC 码）
 *   12     2      本帧原始最小值
 *   14     2      本帧原始最大值
 *   16     400    200 个 uint16 原始 ADC 码
 *   416    2      CRC16-CCITT，覆盖偏移 2 起共 414 字节
 * ========================================================================== */

#ifndef WAVE_DUMP_H
#define WAVE_DUMP_H   /* 防止重复包含 */

#include "stm32f4xx.h"      /* 标准外设库声明 */
#include "fastcap.h"        /* 需要 FastcapSample 类型 */

#ifdef __cplusplus  /* C++ 下按 C 方式链接 */
extern "C" {        /* 保持 C 符号名 */
#endif               /* __cplusplus 条件编译段结束 */

/* ---- 传输参数 ---- */
#define WAVE_DUMP_BAUD          115200U   /* USART1 波特率，CH340C 与常见串口助手都支持 */
#define WAVE_DUMP_DECIM         10U       /* 分频：每 10 帧取 1 帧，即 100 ms 导出一次 */
#define WAVE_DUMP_FRAME_SIZE    418U      /* 整帧字节数，见文件头帧格式表 */
#define WAVE_DUMP_POINTS        200U      /* 每帧点数，必须等于 FASTCAP_BLOCK_LEN */

/* ---- 编译期自检：点数必须与采样前端一致，改了一边忘了另一边会在这里报错 ---- */
#if (WAVE_DUMP_POINTS != FASTCAP_BLOCK_LEN)
#error "WAVE_DUMP_POINTS 必须等于 FASTCAP_BLOCK_LEN，否则帧内点数与偏移全部对不上"
#endif

/* 初始化：配置 PA9 为 USART1_TX，DMA2 Stream7/Channel4 搬运。
 * 必须在 SysTick_Init() 之后调用（本模块不依赖延时，但工程惯例如此）。
 * 初始化完成后默认允许导出，由 WaveDump_Poll() 里的占空比条件把关。 */
void WaveDump_Init(void);

/* 主循环调用：有新采样帧时传进来，按分频与占空比决定是否发送一帧。
 * dutyPercent：当前占空比百分数，为 0 表示电机停转，此时不导出（停转波形无意义）。
 * sample：fastcap 刚产出的一帧统计结果，直流/极值/状态位会被写进帧头。
 * 上下文：只能在主循环调用，不可在中断里调用。 */
void WaveDump_Poll(uint8_t dutyPercent, const FastcapSample *sample);

/* 手动开关：0 = 停止导出，1 = 允许导出。上电默认为 1。 */
void WaveDump_SetEnabled(uint8_t enabled);

/* 查询当前开关状态：1 = 允许导出。 */
uint8_t WaveDump_IsEnabled(void);

/* 取已成功交给 DMA 发送的帧数。 */
uint32_t WaveDump_GetSent(void);

/* 取因上一帧尚未发完而被丢弃的帧数。该值为 0 才说明链路有余量。 */
uint32_t WaveDump_GetDropped(void);

#ifdef __cplusplus
}
#endif  /* __cplusplus 条件编译段结束 */
#endif /* WAVE_DUMP_H 头文件结束 */
