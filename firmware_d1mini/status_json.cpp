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
 *    feature_valid_mask 有效位掩码。历史满值是 63（0x3F），但加速度三位已停用，
 *                       当前实际满值是 49（0x31 = 电流 + 转速 + 母线）
 *    current_mA / accel_x_mg / accel_y_mg / accel_z_mg / rpm_x10 / battery_mV   （6 个测量值字段名）
 *                       加速度三个字段（accel_*_mg）自 2026-10-19 起恒为 null，网页已不再展示
 *                       6 个测量值（注意单位：毫安、毫克、转速x10、毫伏）
 *    dr_x1000            窗内 rpm/V 的绝对峰峰值（dR）放大 1000 倍的整数；
 *                        与板上 OLED 上轮显的「dR」同一口径，阈值 1000 即 1.0 rpm/V。
 *                        由 D1 侧用最近 32 帧就地算出 —— 42 字节帧里没有它的槽位，
 *                        而改协议会让 STM32 固件与本页数据同时对不上，不划算。
 *                        窗口未满 / 死区 / 转速或电压无效时输出 null（绝不输出 0）。
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

/* dR 就地计算的实现放在文件末尾，这里先声明（本文件内私有） */
static bool dr_from_history(const M2History *history, long *drX1000);   /* 前向声明：算窗内 rpm/V 峰峰值并放大 1000 倍 */

/* 【整个函数只在这一个地方被调用：主循环里的 HTTP 处理器】
 * 因为调用点唯一，格式化过程中接收状态不会被别的代码改动，
 * 所以这里读到的是一份「静止的快照」，不用加锁。 */
size_t status_json_with_diagnostics(const M2Receiver *receiver, const M2History *history, uint32_t nowMs,
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
    char drText[14] = "null";   /* dR（x1000）文本，默认 null */
    char uptimeText[14] = "null";   /* 板上运行毫秒数，默认 null（页面折成秒，与 OLED 的 T:xxx s 对齐） */
    int aiAlarm = 0;   /* 报警标志，默认 0 */
/* 【dR】只有链路 live 时才可能算出来：
 *   非 live 时历史里存的是过期帧，拿它算峰峰值等于拿旧数据编新结论。
 *   算不出来（窗口未满 / 死区 / 缺路）就保持 null，页面显示「--」。 */
    if (live) {
        long drX1000 = 0L;   /* dR 放大 1000 倍后的整数 */
        if (dr_from_history(history, &drX1000)) {   /* 窗口满足且各路都有效 */
            snprintf(drText, sizeof(drText), "%ld", drX1000);   /* 输出整数，页面再除 1000 */
        }
        snprintf(uptimeText, sizeof(uptimeText), "%lu", /* 板上运行毫秒数 */
                 (unsigned long)(uint32_t)receiver->latest.uptimeMs);
    }
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
        "\"dr_x1000\":%s,"   /* 窗内 rpm/V 峰峰值 dR（x1000），与板上 OLED 轮显的 dR 同口径 */
        "\"uptime_ms\":%s,"   /* 板上运行毫秒数（页面折成秒，对齐 OLED 的 T:xxx s） */
        "\"ai_phase\":\"%s\",\"ai_alarm\":%d,"   /* AI 阶段与报警标志 */
        "\"frames\":%lu,\"rejected\":%lu,\"reject_version\":%lu,\"reject_length\":%lu,"   /* 收帧数、总拒收数与前两类拒收 */
        "\"reject_format\":%lu,\"reject_crc\":%lu,\"reject_range\":%lu,\"reject_other\":%lu,"   /* 格式 / CRC / 范围 / 其它四类拒收 */
        "\"uart_overflows\":%lu,\"uart_backlog_events\":%lu,\"uart_backlog_bytes\":%lu,"   /* 串口层诊断计数 */
        "\"external_discards\":%lu,\"duplicates\":%lu,\"resyncs\":%lu}",   /* 外部丢弃 / 重复帧 / 重新同步 */
        TELEMETRY_VERSION, link, age, levelText, dutyText,   /* 对应参数：版本、链路、年龄、档位、占空比 */
        live ? (unsigned)receiver->latest.validMask : 0U,   /* 有效位掩码：非 live 时给 0 */
        values[0], values[1], values[2], values[3], values[4], values[5],   /* 6 个测量值文本 */
        drText,   /* dR（x1000），无效时为 null */
        uptimeText,   /* 板上运行毫秒数 */
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

/* --------------------------------------------------------------------------
 *  dR 就地计算：回看最近 32 帧，算 q = rpm / vbat 的峰峰值（放大 1000 倍取整）
 *
 *  【为什么在 D1 侧算，而不是让 STM32 发过来】
 *    42 字节帧的 6 个槽位里，加速度三位（MPU6050 摘除后）虽然空着，
 *    但要启用它们就得同时改 STM32 固件、协议版本与两侧解析；
 *    而 D1 本来就有 256 帧的环形缓冲（约 25 秒），窗口所需的 32 帧唾手可得。
 *    在接收端算，只动 ESP8266 一侧，STM32 那边一个字节都不用改。
 *
 *  【与板上口径必须一致的三件事】
 *    ① 窗口长度 32 帧（板上 FEATURE_R_WINDOW = 32，100 ms 一点，共 3.2 s）
 *    ② q = rpm / vbat 在原始空间算，不做膝点归一化 ——
 *       板上 s_rippleAbs 的推导式化简后正好等于 q 的峰峰值（窗内 duty 恒定）
 *    ③ duty <= 50 落在起动死区，R 无意义，整窗判无效（与板上丢窗规则一致）
 *
 *  【返回 false 的每一种情况都输出 null，不输出 0】
 *    窗口没攒满、占空比在死区、转速或电压任一路无效、电压为 0（不能做分母）。
 * -------------------------------------------------------------------------- */
static bool dr_from_history(const M2History *history, long *drX1000)   /* 入参：帧历史；出参：dR x1000 */
{
    bool haveValue = false;   /* 是否已经取到第一个有效点 */
    float qmin = 0.0f;   /* 窗内 q 的最小值 */
    float qmax = 0.0f;   /* 窗内 q 的最大值 */
    float peak;   /* 峰峰值 */
    if (history == NULL || drX1000 == NULL) {   /* 入参检查 */
        return false;   /* 没有历史或没有出参缓冲：判无效 */
    }
    if (history->count < STATUS_DR_WINDOW) {   /* 窗口还没攒满 */
        return false;   /* 与板上一致：未满窗不给结论，避免把攒窗阶段当异常 */
    }
    /* 环形缓冲里「最近 32 帧」的起点：head 指向下一个写入位，往前回退 32 个 */
    const size_t start = (history->head + M2_HISTORY_CAPACITY - STATUS_DR_WINDOW) % M2_HISTORY_CAPACITY;
    for (size_t k = 0U; k < STATUS_DR_WINDOW; ++k) {   /* 从最旧到最新逐个检查 */
        const M2Frame *frame = &history->frames[(start + k) % M2_HISTORY_CAPACITY];   /* 取这一帧 */
        if (frame->duty <= STATUS_DR_DUTY_MIN) {   /* 占空比落在起动死区 */
            return false;   /* R 与 dR 都没有物理意义，整窗作废 */
        }
        if ((frame->validMask & M2_VALID_SPEED) == 0U) {   /* 转速无效 */
            return false;   /* 缺一路就整窗作废，不用缺帧拼窗口 */
        }
        if ((frame->validMask & M2_VALID_BATTERY) == 0U) {   /* 母线电压无效 */
            return false;   /* 同上 */
        }
        if (frame->value[5] <= 0) {   /* 母线电压为 0 或负 */
            return false;   /* 不能做分母 */
        }
        /* q = (rpm_x10 / 10) / (mV / 1000) = rpm_x10 * 100 / mV */
        const float q = ((float)frame->value[4] * 100.0f) / (float)frame->value[5];
        if (!haveValue) {   /* 第一个有效点：同时当最小值与最大值 */
            qmin = q;
            qmax = q;
            haveValue = true;
        } else {   /* 之后的点：各自更新极值 */
            if (q < qmin) { qmin = q; }
            if (q > qmax) { qmax = q; }
        }
    }
    if (!haveValue) {   /* 理论上到不了这里（窗口已满且每帧都过了检查） */
        return false;   /* 保险起见仍判无效 */
    }
    peak = qmax - qmin;   /* 峰峰值 */
    if (peak < 0.0f) {   /* 浮点异常保护 */
        return false;   /* 负值无意义 */
    }
    *drX1000 = (long)(peak * 1000.0f + 0.5f);   /* 放大 1000 倍并四舍五入成整数 */
    return true;   /* dR 可用 */
}

/* 兼容用的简化入口：不带串口诊断计数的版本（内部就是传 0 给诊断参数）。
 * 老的测试代码用它，正式固件走的是上面那个带诊断的版本。
 * 这里传 NULL 历史：dR 字段输出 null，老调用方的行为完全不变。 */
size_t status_json(const M2Receiver *receiver, uint32_t nowMs, char *buffer, size_t capacity)
{
    return status_json_with_diagnostics(receiver, NULL, nowMs, 0U, 0U, 0U, 0U, buffer, capacity); /* 历史传 NULL + 4 个串口诊断计数传 0 */
}
