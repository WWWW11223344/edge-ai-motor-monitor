/* 2026-09-27：可移植、可在主机侧单测的 JSON 输出（供只读监控页使用）。 */
#ifndef STATUS_JSON_H   /* 头文件保护：防止重复包含 */
#define STATUS_JSON_H   /* 定义保护宏 */
#include "m2_protocol.h"   /* 需要 M2Receiver 类型定义 */

/* Codex, 2026-10-07：USB_END 覆盖「区间拒收」类别。 */
#define TELEMETRY_VERSION "d1-telemetry-2.1.11"   /* 协议版本号，随固件一起发布 */
/* 返回不含结尾 NUL 的字节数，失败返回 0；本响应中不出现任何凭据。 */
size_t status_json(const M2Receiver *receiver, uint32_t nowMs, char *buffer, size_t capacity);   /* 生成状态 JSON（基础字段） */
size_t status_json_with_diagnostics(const M2Receiver *receiver, uint32_t nowMs,   /* 带诊断信息的重载版本 */
    uint32_t uartOverflows, uint32_t uartBacklogEvents, uint32_t uartBacklogBytes,   /* 入参：串口溢出次数、积压事件数、积压字节数 */
    uint32_t externalDiscards, char *buffer, size_t capacity);   /* 入参：外部丢弃计数，以及输出缓冲与容量 */
#endif   /* STATUS_JSON_H 结束 */
