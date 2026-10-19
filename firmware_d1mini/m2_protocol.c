/* ==========================================================================
 *  m2_protocol.c（D1 侧）  --  和 STM32 上那份是「同一个协议」的实现
 *
 *  【和 STM32 侧的区别】算法完全一样（编码、解码、CRC、状态机都同源），
 *    但这份代码跑在 ESP8266 上，职责偏向「收」：
 *      - m2_feed()   逐字节收帧（D1 的主职，STM32 侧基本不用）
 *      - m2_encode() 用来把收到的帧原样打包，走 USB 转发给电脑（抓包时用）
 * 帧格式（42 字节、A5 5A 开头、CRC 收尾）和每个字节的含义
 * 见 STM32 工程的 m2_protocol.c 顶部那张偏移表 —— 两边必须一致，改一边就等于改协议。
 *
 *  【两份代码为什么要「同源」】只要有一边算错，CRC 就会对不上，
 *    表现是「STM32 明明在发，D1 一直 rejected++」，很难查。
 *    所以两边的缩放、取整、CRC 多项式、拒收分类都保持一致。
 * ========================================================================== */
/* 2026-10-05：有界 M2 编解码；无效字段靠掩码始终保持无效。 */
#include "m2_protocol.h"

/* 缩放取整函数，和 STM32 侧那份逐字相同 —— 这是故意的：
 * 两边算法但凡有一点差别，同一个物理量在两头算出的整数就会不一样，
 * 排查问题时会被误导。 */
/* 缩放函数必须与 STM32 编码端逐字节一致。 */
bool m2_scale(float value, uint32_t factor, int32_t *scaled)
{
    double product = (double)value * (double)factor; /* 用 double 相乘，避免先转 int 丢精度 */
    if (scaled == 0 || factor == 0U ||   /* 输出指针为空或放大倍数为 0 */
        !(product > -2147483648.5 && product < 2147483647.5)) {   /* 越界，或为 NaN / 无穷大 */
        return false;   /* 拒绝非法输入 */
    }
    product += product < 0.0 ? -0.5 : 0.5;   /* 四舍五入 */
    *scaled = (int32_t)product;   /* 截断成 int32 */
    return true;   /* 转换成功 */
}

static void put_u32(uint8_t *out, uint32_t value)   /* 小端写入 4 字节无符号数 */
{
    out[0] = (uint8_t)value;   /* 最低字节 */
    out[1] = (uint8_t)(value >> 8);   /* 次低字节 */
    out[2] = (uint8_t)(value >> 16);   /* 次高字节 */
    out[3] = (uint8_t)(value >> 24);   /* 最高字节 */
}

static uint32_t get_u32(const uint8_t *in)   /* 小端读出 4 字节无符号数 */
{
    return (uint32_t)in[0] |   /* 最低字节 */
           ((uint32_t)in[1] << 8) |   /* 次低字节 */
           ((uint32_t)in[2] << 16) |   /* 次高字节 */
           ((uint32_t)in[3] << 24);   /* 最高字节 */
}

static void put_i32(uint8_t *out, int32_t value)   /* 小端写入 4 字节有符号数 */
{
    put_u32(out, (uint32_t)value);   /* 按位原样写入，符号在解读时体现 */
}

static int32_t get_i32(const uint8_t *in)   /* 小端读出 4 字节有符号数 */
{
    return (int32_t)get_u32(in);   /* 按位原样读回 */
}

/* CRC-CCITT-FALSE（多项式 0x1021，初值 0xFFFF）：逐字节异或进高 8 位，
 * 再做 8 次「看最高位决定是否异或多项式」的移位。
 * 发送方算出来附在帧尾，接收方用同样算法重算一遍，对不上就整帧丢弃。
 * 参数 data 为空时直接返回初值，调用方靠比对结果自然发现不对。 */
uint16_t m2_crc(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU; /* CRC-CCITT 初值 0xFFFF */
    size_t i;   /* 字节索引 */
    unsigned bit;   /* 位索引 0~7 */
    if (data == 0) { return crc; }   /* 空指针直接返回初值 */
    for (i = 0U; i < length; ++i) {   /* 逐字节喂入 */
        crc ^= (uint16_t)((uint16_t)data[i] << 8);   /* 当前字节异或到高 8 位 */
        for (bit = 0U; bit < 8U; ++bit) {   /* 逐位移出，共 8 次 */
            crc = (uint16_t)(((crc & 0x8000U) != 0U) ?   /* 先看最高位 */
                (((uint32_t)crc << 1) ^ 0x1021U) : ((uint32_t)crc << 1));   /* 为 1 则异或多项式 0x1021，为 0 只左移 */
        }
    }
    return crc;   /* 返回 16 位校验值 */
}

/* 【编码】把 M2Frame 打包成 42 字节，用在 USB 抓包转发上：
 *  D1 收到一帧 -> 原样重新编码成 42 字节 -> 从 USB 发给电脑。
 * 为什么不是「把收到的原始字节直接转发」？因为接收状态机是逐字节吃进去的，
 * 没保留原始字节；用同一份编码函数重打包，结果和原始发送内容完全一致（可验证）。
 * 开头同样做范围体检（档位<=4、占空比<=88、有效位和 AI 状态不越界）。 */
size_t m2_encode(const M2Frame *frame, uint8_t *buffer, size_t capacity)
{
    uint16_t crc; /* 待写入帧尾的校验值 */
    unsigned i;   /* 6 个测量值的循环下标 */
    if (frame == 0 || buffer == 0 || capacity < M2_FRAME_SIZE ||   /* 入参检查：指针与容量 */
        frame->level > 4U || frame->duty > 88U ||   /* 档位与占空比范围检查 */
        frame->validMask > M2_VALID_ALL || frame->aiState > M2_AI_STATE_MAX) {   /* 有效位掩码与 AI 状态范围检查 */
        return 0U;   /* 参数非法则返回 0 */
    }
    buffer[0] = M2_SYNC0;   /* 字节 0：帧头第 1 字节 */
    buffer[1] = M2_SYNC1;   /* 字节 1：帧头第 2 字节 */
    buffer[2] = M2_VERSION;   /* 字节 2：协议版本 */
    buffer[3] = M2_PAYLOAD_SIZE;   /* 字节 3：负载长度 */
    put_u32(&buffer[4], frame->seq);   /* 字节 4~7：帧序号（小端） */
    put_u32(&buffer[8], frame->uptimeMs);   /* 字节 8~11：运行毫秒（小端） */
    buffer[12] = frame->level;   /* 字节 12：档位 */
    buffer[13] = frame->duty;   /* 字节 13：占空比 */
    buffer[14] = frame->validMask;   /* 字节 14：有效位掩码 */
    buffer[15] = frame->aiState;   /* 字节 15：AI 阶段与报警 */
    for (i = 0U; i < 6U; ++i) { put_i32(&buffer[16U + 4U * i], frame->value[i]); }   /* 字节 16~39：6 个测量值，各 4 字节小端 */
    crc = m2_crc(&buffer[2], 38U);   /* CRC 覆盖从版本字节起的 38 字节 */
    buffer[40] = (uint8_t)crc;   /* 字节 40：CRC 低字节 */
    buffer[41] = (uint8_t)(crc >> 8);   /* 字节 41：CRC 高字节 */
    return M2_FRAME_SIZE;   /* 返回整帧长度 */
}

/* 拒收原因分类，和 STM32 侧同一个枚举。D1 上这些计数非常重要：
 * 网页 /status 和 /raw 都会把它们报出来，用来判断「是 STM32 没发好，
 * 还是 D1 这边串口时序被打乱了」——
 * 例如 rejectCrc 一直涨，基本就是软件串口的位采样被 WiFi 中断干扰了。 */
typedef enum {
    M2_REJECT_NONE = 0, /* 未分类（仅占位，正常不会用到） */
    M2_REJECT_VERSION, /* 版本号不符（第 3 字节） */
    M2_REJECT_LENGTH,   /* 长度不符（第 4 字节长度字段） */
    M2_REJECT_FORMAT,   /* 格式不符（帧头或位域） */
    M2_REJECT_CRC,   /* CRC 校验失败 */
    M2_REJECT_RANGE,   /* 数值超出范围 */
    M2_REJECT_OTHER   /* 其它 */
} M2RejectReason;   /* M2RejectReason 枚举结束 */

static M2RejectReason m2_decode_reason(const uint8_t *data, size_t length, M2Frame *frame)   /* 完整校验：返回拒收原因，全部通过则为 NONE */
{
    M2Frame parsed;   /* 解析结果先放本地，全部通过才写回 */
    uint16_t expected;   /* 帧里携带的 CRC */
    unsigned i;   /* 测量值循环下标 */
    if (data == 0 || frame == 0) { return M2_REJECT_OTHER; }   /* 空指针 */
    if (length != M2_FRAME_SIZE) { return M2_REJECT_LENGTH; }   /* 长度必须正好等于整帧长度 */
    if (data[0] != M2_SYNC0 || data[1] != M2_SYNC1) { return M2_REJECT_FORMAT; }   /* 帧头必须匹配 */
    if (data[2] != M2_VERSION) { return M2_REJECT_VERSION; }   /* 版本必须匹配 */
    if (data[3] != M2_PAYLOAD_SIZE) { return M2_REJECT_LENGTH; }   /* 负载长度字段必须匹配 */
    if (data[14] > M2_VALID_ALL || data[15] > M2_AI_STATE_MAX) { return M2_REJECT_FORMAT; }   /* 有效位掩码与 AI 状态不得越界 */
    expected = (uint16_t)data[40] | ((uint16_t)data[41] << 8);   /* 取出帧尾 CRC（小端） */
    if (m2_crc(&data[2], 38U) != expected) { return M2_REJECT_CRC; }   /* 本地重算并比对，不符即整帧作废 */
    parsed.seq = get_u32(&data[4]);   /* 帧序号 */
    parsed.uptimeMs = get_u32(&data[8]);   /* 运行毫秒 */
    parsed.level = data[12];   /* 档位 */
    parsed.duty = data[13];   /* 占空比 */
    parsed.aiState = data[15];   /* AI 状态 */
    if (parsed.level > 4U || parsed.duty > 88U) { return M2_REJECT_RANGE; }   /* 数值范围复查 */
    parsed.validMask = data[14];   /* 有效位掩码 */
    for (i = 0U; i < 6U; ++i) { parsed.value[i] = get_i32(&data[16U + 4U * i]); }   /* 6 个测量值 */
    *frame = parsed;   /* 全部通过，写回输出参数 */
    return M2_REJECT_NONE;   /* 解码成功 */
}

bool m2_decode(const uint8_t *data, size_t length, M2Frame *frame)   /* 对外接口：只关心成功与否 */
{
    return m2_decode_reason(data, length, frame) == M2_REJECT_NONE;   /* NONE 表示全部校验通过 */
}

static void record_reject(M2Receiver *receiver, M2RejectReason reason)   /* 按原因分类累加拒收计数 */
{
    if (receiver == 0) { return; }   /* 空指针保护 */
    ++receiver->rejected;   /* 总拒收数加一 */
    switch (reason) {   /* 再按具体原因分类计数 */
    case M2_REJECT_VERSION: ++receiver->rejectVersion; break;   /* 版本不符 */
    case M2_REJECT_LENGTH: ++receiver->rejectLength; break;   /* 长度不符 */
    case M2_REJECT_FORMAT: ++receiver->rejectFormat; break;   /* 格式不符 */
    case M2_REJECT_CRC: ++receiver->rejectCrc; break;   /* CRC 错 */
    case M2_REJECT_RANGE: ++receiver->rejectRange; break;   /* 数值越界 */
    default: ++receiver->rejectOther; break;   /* 其它 */
    }
}

void m2_reset(M2Receiver *receiver)   /* 复位接收状态（统计计数不清） */
{
    if (receiver != 0) {   /* 空指针保护 */
        receiver->used = 0U;   /* 已收字节数归零 */
        receiver->expected = 0U;   /* 预期长度归零 */
    }
}

void m2_discard(M2Receiver *receiver)   /* 丢弃当前半截帧 */
{
    if (receiver != 0) {   /* 空指针保护 */
        m2_reset(receiver);   /* 先复位状态 */
        record_reject(receiver, M2_REJECT_OTHER);   /* 记一次「其它」类拒收 */
    }
}

/* 【逐字节收帧状态机】D1 这边最重要的函数，每收到一个字节就调一次。
 * 用 used 记「已经收了几字节」：
 *    used==0：等 0xA5（帧头1），别的字节一律无视
 *    used==1：等 0x5A（帧头2）；又见 0xA5 就覆盖（处理 A5 A5 5A）；其它 -> 归零
 *    used>=2：往后存，顺路在第 3、4 字节校验版本号和载荷长度（提前排除明显的错帧）
 *    收满 42：交给 m2_decode_reason 做完整校验（长度/帧头/版本/位域/CRC/数值范围）
 *            通过就更新 latest、记录时间戳、accepted++，返回 true
 *            —— 返回 true 的这一刻，ino 里的主循环才会把这帧转发 USB 并存进历史缓冲
 * 【三个额外的判断，都是实战踩出来的】
 *   duplicates：seq 和 uptimeMs 都和上一帧一样 -> 重复帧（串口重发/回绕），丢弃但计数
 *   resyncs   ：seq 比上一帧小 -> 序号乱了（对方重启过），记一次但仍然收下
 *   载荷里的 0xA5：used>=2 之后一律当数据处理，不会被误当帧头
 * 【如果 used 已经 >= 42 还没结束】说明上一帧没收干净，直接丢弃重新找帧头。 */
bool m2_feed(M2Receiver *receiver, uint8_t byte, uint32_t nowMs)
{
    M2Frame next; /* 拼装中的下一帧，全部校验通过后才提交给 latest */
    if (receiver == 0) { return false; }   /* 空指针保护 */
    if (receiver->used == 0U) {   /* 还没开始收帧 */
        if (byte == M2_SYNC0) { receiver->buffer[0] = byte; receiver->used = 1U; }   /* 见到 0xA5 才开始攒帧 */
        return false;   /* 尚未成帧 */
    }
    if (receiver->used == 1U) {   /* 已有帧头第 1 字节 */
        if (byte == M2_SYNC1) { receiver->buffer[1] = byte; receiver->used = 2U; }   /* 见到 0x5A：帧头齐了 */
        else if (byte == M2_SYNC0) { receiver->buffer[0] = byte; }   /* 又是 0xA5：当成新的帧头1（处理 A5 A5 5A） */
        else { receiver->used = 0U; }   /* 其它字节：丢掉重来 */
        return false;   /* 尚未成帧 */
    }
    if (receiver->used >= M2_FRAME_MAX) { m2_discard(receiver); return false; }   /* 收超长了还没结束：丢帧重来 */
    receiver->buffer[receiver->used++] = byte;   /* 存入缓冲并递增计数 */
    if (receiver->used == 3U && byte != M2_VERSION) {   /* 第 3 字节（版本）不符合 */
        m2_reset(receiver); record_reject(receiver, M2_REJECT_VERSION); return false;   /* 复位并记一次版本拒收 */
    }
    if (receiver->used == 4U) {   /* 第 4 字节（负载长度） */
        if (byte != M2_PAYLOAD_SIZE) {   /* 长度字段不等于约定值 */
            m2_reset(receiver); record_reject(receiver, M2_REJECT_LENGTH); return false;   /* 复位并记一次长度拒收 */
        }
        receiver->expected = M2_FRAME_SIZE;   /* 长度合法，记下本帧预期总长 */
    }
    if (receiver->expected != 0U && receiver->used == receiver->expected) {   /* 正好收满一整帧 */
        const M2RejectReason reason = m2_decode_reason(receiver->buffer, M2_FRAME_SIZE, &next);   /* 交给完整校验，拿到拒收原因 */
        m2_reset(receiver);   /* 先复位，准备下一帧 */
        if (reason != M2_REJECT_NONE) {   /* 校验没通过 */
            record_reject(receiver, reason);   /* 按原因计数 */
            return false;   /* 本帧不计入统计 */
        }
        if (receiver->hasFrame && next.seq == receiver->latest.seq &&   /* 与上一帧的序号、时间戳都相同 */
            next.uptimeMs == receiver->latest.uptimeMs) { ++receiver->duplicates; return false; }   /* 判为重复帧：丢弃但计数 */
        if (receiver->hasFrame && next.seq < receiver->latest.seq) { ++receiver->resyncs; }   /* 序号回退 = 对端重启，记一次重新同步 */
        receiver->latest = next;   /* 采纳为最新快照 */
        receiver->lastNewMs = nowMs;   /* 记录收到时刻 */
        receiver->hasFrame = true;   /* 标记已有可用帧 */
        ++receiver->accepted;   /* 累计成功接收 */
        return true;   /* 通知调用方：这一帧有效 */
    }
    return false;   /* 还没收满 */
}

/* 链路状态判断：从来没收到过 -> WAITING；超过 3000ms 没新帧 -> STALE；否则 LIVE。
 * 用无符号相减算时间差，millis() 回绕时依然正确。
 * 注意 STALE 时数值字段会全部变成 null（见 status_json.cpp），
 * 这样「通信断了」和「值是 0」在界面上完全区分得开。 */
M2Link m2_link(const M2Receiver *receiver, uint32_t nowMs)
{
    if (receiver == 0 || !receiver->hasFrame) { return M2_WAITING; } /* 从未收到过任何帧 -> WAITING */
    return (uint32_t)(nowMs - receiver->lastNewMs) > M2_TIMEOUT_MS ? M2_STALE : M2_LIVE;   /* 超时判 STALE，否则 LIVE */
}
