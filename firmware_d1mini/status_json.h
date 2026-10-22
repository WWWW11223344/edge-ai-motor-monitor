/* 2026-09-27：可移植、可在主机侧单测的 JSON 输出（供只读监控页使用）。 */
#ifndef STATUS_JSON_H   /* 头文件保护：防止重复包含 */
#define STATUS_JSON_H   /* 定义保护宏 */
#include "m2_protocol.h"   /* 需要 M2Receiver 类型定义 */
#include "raw_history.h"   /* 需要 M2History 类型：dR 要回看最近 32 帧 */

/* Codex, 2026-10-07：USB_END 覆盖「区间拒收」类别。 */
#define TELEMETRY_VERSION "d1-telemetry-2.1.11"   /* 协议版本号，随固件一起发布 */
/* 2026-10-21：dR（窗内 rpm/V 绝对峰峰值）由 D1 侧就地算出，不占 42 字节帧的槽位。
 * 与板上 feature_r_ripple_abs() 同一口径：最近 32 帧、q = rpm / vbat、取 max - min。
 * history 传 NULL 时该字段输出 null（老调用方不受影响）。 */
#define STATUS_DR_WINDOW 32U   /* dR 窗口帧数，与板上 FEATURE_R_WINDOW 一致（32 点 x 100 ms = 3.2 s） */
#define STATUS_DR_DUTY_MIN 50U   /* 占空比低于此值落在起动死区，R 与 dR 都无意义 */

/* 返回不含结尾 NUL 的字节数，失败返回 0；本响应中不出现任何凭据。 */
size_t status_json(const M2Receiver *receiver, uint32_t nowMs, char *buffer, size_t capacity);   /* 生成状态 JSON（基础字段） */
size_t status_json_with_diagnostics(const M2Receiver *receiver, const M2History *history,   /* 带诊断信息的重载版本：收帧器 + 最近帧历史 */
    uint32_t nowMs,
    uint32_t uartOverflows, uint32_t uartBacklogEvents, uint32_t uartBacklogBytes,   /* 入参：串口溢出次数、积压事件数、积压字节数 */
    uint32_t externalDiscards, char *buffer, size_t capacity);   /* 入参：外部丢弃计数，以及输出缓冲与容量 */
#endif   /* STATUS_JSON_H 结束 */
