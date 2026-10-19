/* ==========================================================================
 *  status_json.cpp  --  把「当前状态」拼成一段 JSON（给网页和调试工具看）
 *
 *  【谁在用】
 *    ① D1 的 GET /status 接口
 *    ② 手机网页每秒拿来刷新一次读数（monitor_page.h 里的 refresh()）
 *
 *  【两条铁律，全文件都围着它们转】
 *    铁律一：无效 / 过期的测量值一律输出 null，绝不输出 0。
 *            0 会被上位机当成真实测量值，画出假曲线；null 前端会显示「--」。
 *    铁律二：只有链路状态是 live（3 秒内有新帧）时，才允许输出具体数值；
 *            stale（超时）时所有测量值都变 null，但计数器照常给 ——
 *            计数器是「历史累计」，跟当前这一帧是否新鲜无关。
 *
 *  【输出字段一览】
 *    version            固件/协议版本字符串
 *    link               waiting / live / stale（链路状态）
 *    age_ms             距最近一帧过了多少毫秒（没有帧则为 null）
 *    level / duty       当前档位 / 占空比百分数
 *    feature_valid_mask 6 个测量值的有效位（63 = 0x3F = 全有效）
 *    current_mA / accel_x_mg / accel_y_mg / accel_z_mg / rpm_x10 / battery_mV   （6 个测量值字段名）
 *                       6 个测量值（注意单位：毫安、毫克、转速x10、毫伏）
 *    ai_phase / ai_alarm  板上 AI 阶段 / 是否报警（0/1）
 *    frames / rejected  累计收下的帧数 / 被拒收的帧数
 *    reject_*          按原因分类的拒收计数（判断「是谁把数据弄丢了」的关键）
 *    uart_*            串口层诊断：FIFO 溢出、上游积压、外部丢弃
 *    duplicates/resyncs 重复帧 / 序号乱掉的次数
 * ========================================================================== */
/* 2026-10-05：无效与过期的 M2 测量值一律输出 JSON null，绝不输出 0。 */
#include "status_json.h"
#include <stdio.h>
#include <string.h> /* memcpy 声明，用来把 "null" 写进输出缓冲 */

/* 【整个函数只在这一个地方被调用：主循环里的 HTTP 处理器】
 * 因为调用点唯一，格式化过程中接收状态不会被别的代码改动，
 * 所以这里读到的是一份「静止的快照」，不用加锁。 */
size_t status_json_with_diagnostics(const M2Receiver *receiver, uint32_t nowMs,
    uint32_t uartOverflows, uint32_t uartBacklogEvents, uint32_t uartBacklogBytes,
    uint32_t externalDiscards, char *buffer, size_t capacity) /* 续行：外部丢弃计数 / 输出缓冲指针 / 缓冲容量 */
{
    char values[6][12];   /* 6 个测量值各自的十进制文本 */
    char age[12] = "null";   /* 数据年龄(ms)，默认 null */
    if (receiver == NULL || buffer == NULL || capacity == 0U) {   /* 入参合法性检查 */
        return 0U;   /* 参数非法：输出 0 字节 */
    }
/* 先算链路状态（waiting / live / stale）和两个派生量：
 *   live  = 3 秒内有新帧（只有 live 才给具体数值）
 *   link  = 给页面显示的字符串（waiting=还没收到过，stale=通信中断）
 * 然后把各种「待填」的文本先设成 null，后面只有 live 才覆盖成真实数字。
 * 这么写的目的是：任何一条路径忘了填，输出也是 null 而不是垃圾值。 */
    const M2Link state = m2_link(receiver, nowMs);
    const bool live = state == M2_LIVE;
    const char *link = state == M2_WAITING ? "waiting" : (live ? "live" : "stale"); /* 链路状态文本：等待 / 正常 / 过期 */
    char levelText[5] = "null";   /* 档位文本，默认 null */
    char dutyText[5] = "null";   /* 占空比文本，默认 null */
    char aiPhaseText[12] = "null";   /* AI 阶段文本，默认 null */
    int aiAlarm = 0;   /* 报警标志，默认 0 */
/* 【数据年龄】有帧才输出年龄（毫秒），没有帧就保持 null。
 * 页面用它显示「数据年龄 x.x 秒」，一眼能看出数据是不是卡住了。 */
    if (receiver->hasFrame) {
        snprintf(age, sizeof(age), "%lu", (unsigned long)(uint32_t)(nowMs - receiver->lastNewMs));
    }
/* 【只有 live 才填具体数值】
 * 档位、占空比、AI 阶段、报警标志都在这段里填：
 *   ai_phase 用 0..3 查表成 off/learning/detecting/error 四个词；
 *   ai_alarm 只看 aiState 的 bit2。
 * 链路 stale 时这些保持 null，页面显示「--」，不会拿过期数字骗人。 */
    if (live) {
        snprintf(levelText, sizeof(levelText), "%u", (unsigned)receiver->latest.level);
        snprintf(dutyText, sizeof(dutyText), "%u", (unsigned)receiver->latest.duty); /* 占空比按十进制输出 */
        static const char *const aiPhaseNames[4] = {"off", "learning", "detecting", "error"};   /* 阶段码 0~3 对应的短词 */
        const unsigned aiPhase = receiver->latest.aiState & 0x03U;   /* 取低 2 位得到阶段码 */
        snprintf(aiPhaseText, sizeof(aiPhaseText), "%s", aiPhaseNames[aiPhase]);   /* 阶段码查表转文本 */
        aiAlarm = (receiver->latest.aiState & 0x04U) != 0U ? 1 : 0;   /* 看 bit2 判断是否报警 */
    }
/* 【6 个测量值的一个个处理】规则很简单但很关键：
 *   链路是 live 且 validMask 这一位是 1 -> 输出数字
 *   否则                               -> 输出 null
 * 千万别为了「看起来连续」把无效值补成 0 —— 那不是测量，是编数据。 */
    for (unsigned i = 0U; i < 6U; ++i) {
        if (live && (receiver->latest.validMask & (1U << i)) != 0U) {
            snprintf(values[i], sizeof(values[i]), "%ld", (long)receiver->latest.value[i]); /* 该测量值有效：输出十进制整数 */
        } else {   /* 链路非 live，或该测量值无效 */
            memcpy(values[i], "null", 5U);   /* 写 null，绝不写 0 */
        }
    }
/* 按固定模板把 JSON 拼进调用方给的 buffer：
 * 返回长度必须 >0 且小于容量，否则本函数返回 0（调用方回 500）。
 * 【为什么要做长度判断】snprintf 会在截断时返回「本该写多少字节」，
 * 一旦 >= capacity 就说明这段 JSON 不完整，绝不能把半截 JSON 发出去，
 * 前端 JSON.parse 会直接抛错。 */
    const int length = snprintf(buffer, capacity,
        "{\"version\":\"%s\",\"link\":\"%s\",\"age_ms\":%s,\"level\":%s,\"duty\":%s,"   /* 版本 / 链路状态 / 数据年龄 / 档位 / 占空比 */
        "\"feature_valid_mask\":%u,\"current_mA\":%s,\"accel_x_mg\":%s,\"accel_y_mg\":%s,\"accel_z_mg\":%s,\"rpm_x10\":%s,\"battery_mV\":%s,"   /* 有效位掩码 + 6 个测量值 */
        "\"ai_phase\":\"%s\",\"ai_alarm\":%d,"   /* AI 阶段与报警标志 */
        "\"frames\":%lu,\"rejected\":%lu,\"reject_version\":%lu,\"reject_length\":%lu,"   /* 收帧数、总拒收数与前两类拒收 */
        "\"reject_format\":%lu,\"reject_crc\":%lu,\"reject_range\":%lu,\"reject_other\":%lu,"   /* 格式 / CRC / 范围 / 其它四类拒收 */
        "\"uart_overflows\":%lu,\"uart_backlog_events\":%lu,\"uart_backlog_bytes\":%lu,"   /* 串口层诊断计数 */
        "\"external_discards\":%lu,\"duplicates\":%lu,\"resyncs\":%lu}",   /* 外部丢弃 / 重复帧 / 重新同步 */
        TELEMETRY_VERSION, link, age, levelText, dutyText,   /* 对应参数：版本、链路、年龄、档位、占空比 */
        live ? (unsigned)receiver->latest.validMask : 0U,   /* 有效位掩码：非 live 时给 0 */
        values[0], values[1], values[2], values[3], values[4], values[5],   /* 6 个测量值文本 */
        aiPhaseText, aiAlarm,   /* AI 阶段文本与报警标志 */
        (unsigned long)receiver->accepted, (unsigned long)receiver->rejected,   /* 累计接收 / 累计拒收 */
        (unsigned long)receiver->rejectVersion, (unsigned long)receiver->rejectLength,   /* 拒收分类：版本 / 长度 */
        (unsigned long)receiver->rejectFormat, (unsigned long)receiver->rejectCrc,   /* 拒收分类：格式 / CRC */
        (unsigned long)receiver->rejectRange, (unsigned long)receiver->rejectOther,   /* 拒收分类：范围 / 其它 */
        (unsigned long)uartOverflows, (unsigned long)uartBacklogEvents,   /* 串口溢出次数 / 积压事件数 */
        (unsigned long)uartBacklogBytes, (unsigned long)externalDiscards,   /* 积压字节数 / 外部丢弃数 */
        (unsigned long)receiver->duplicates, (unsigned long)receiver->resyncs);   /* 重复帧数 / 重新同步次数 */
    if (length < 0 || (size_t)length >= capacity) {   /* 被截断或格式化失败 */
        buffer[0] = '\0';   /* 清空输出，保证绝不发出半截 JSON */
        return 0U;   /* 返回 0 字节（调用方回 500） */
    }
    return (size_t)length;   /* 返回实际写入的字节数 */
}

/* 兼容用的简化入口：不带串口诊断计数的版本（内部就是传 0 给诊断参数）。
 * 老的测试代码用它，正式固件走的是上面那个带诊断的版本。 */
size_t status_json(const M2Receiver *receiver, uint32_t nowMs, char *buffer, size_t capacity)
{
    return status_json_with_diagnostics(receiver, nowMs, 0U, 0U, 0U, 0U, buffer, capacity); /* 4 个串口诊断计数全部传 0，与旧版行为一致 */
}
