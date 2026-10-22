/* 2026-10-05：USART3 上只发不收的 M2 遥测通道，没有接收路径，也下发不了电机指令。 */
/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    telemetry.h
  Author:      汪贤云
  Version:     1.0
  Date:        2026-09-27
  Description: 遥测发送接口。每 10ms 把当前测量值打包为 42 字节二进制帧，
               经 USART3(PB10, 57600 8N1) 发给 D1 mini，再由 D1 转成网页与 USB 数据。
               本通道只发送不接收，不会驱动 D1 侧的引脚。
               帧格式定义见 m2_protocol.h。
  Function List:
    1. telemetry_init()             初始化 USART3
    2. telemetry_poll()             主循环调用，打包并发送一帧
  History:
    <author>   <time>      <version>   <desc>
    汪贤云     2026-09-27    1.0         创建文件
***********************************************************/
#ifndef TELEMETRY_H
#define TELEMETRY_H
#include "ui.h"  /* 遥测组帧要借用 UI_Data 快照 */

/* 只能在主循环里调用；只读借用 UI 快照，绝不修改它。 */
/* 【接口清单】
 *   telemetry_init()             上电初始化 USART3（只调一次）
 *   telemetry_poll(data, nowMs)  主循环调用；nowMs 传 millis 值，
 *                                函数内部用它做 10ms 节流和填 uptimeMs 字段
 * 【线程安全说明】data 指向的 UI 快照只在主循环里被读，
 *   发送缓冲由 TXE 中断独占 —— 两边分工明确，不需要加锁。 */
void telemetry_init(void);
void telemetry_poll(const UI_Data *data, uint32_t nowMs);

#endif  /* __TELEMETRY_H 头文件结束 */
