/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    m2_protocol.c
  Author:      汪贤云
  Version:     1.0
  Date:        2026-10-05
  Description: M2 二进制遥测协议的编解码实现，STM32 与 D1 mini 共用同一套算法。
               一帧固定 42 字节：2 字节帧头(0xA5 0x5A)、版本、载荷长度、
               帧序号、开机毫秒、档位、占空比、有效位、AI 状态、
                6 个 int32 测量值（电流mA / X,Y,Z加速度mg / 转速x10 / 电压mV）、2 字节 CRC。
               加速度三槽自 2026-10-19 起不再写入（MPU6050 已摘除），槽位保留以兼容帧格式。
               CRC 覆盖自版本字段起的 38 字节，不含帧头与 CRC 自身。
               接收端以「收满 42 字节」为前提切帧，故帧长不可变。
  Function List:
    1. put_u32()/get_u32()/put_i32()/get_i32()  小端字节序读写
    2. m2_crc()        计算 CCITT-FALSE 校验值
    3. m2_scale()      浮点按系数转整数（含溢出保护）
    4. m2_encode()     M2Frame 打包为 42 字节
    5. m2_decode()     42 字节还原为 M2Frame
    6. m2_feed()       逐字节接收状态机
    7. m2_link()       链路状态判定
  History:
    <author>   <time>      <version>   <desc>
    汪贤云     2026-10-16    1.0         创建文件并按华为规范补充注释
***********************************************************/
/* 2026-10-05：定长（有界）的 M2 编解码器；无效字段靠有效位掩码保持「无效」。 */
#include "m2_protocol.h"  /* 协议常量与结构体定义 */

/*-----------------------------------------------------------
  Function:      put_u32
  Description:   把 32 位无符号整数按小端顺序写入 4 个字节（低字节在前）。
  Calls:         无
  Called By:     put_i32()、m2_encode()
  Input:         out —— 4 字节缓冲区首地址；value —— 待写入的数值
  Output:        out 指向的 4 个字节
  Return:        无
  Others:        手工拆字节使字节序完全由代码决定，不依赖编译器与平台。
-----------------------------------------------------------*/
static void put_u32(uint8_t *out, uint32_t value)
{
    out[0] = (uint8_t)value;  /* 最低字节 */
    out[1] = (uint8_t)(value >> 8);  /* 次低字节 */
    out[2] = (uint8_t)(value >> 16);  /* 次高字节 */
    out[3] = (uint8_t)(value >> 24);  /* 最高字节 */
}

/*-----------------------------------------------------------
  Function:      get_u32
  Description:   把 4 个小端字节拼回 32 位无符号整数。
  Calls:         无
  Called By:     get_i32()、m2_decode_reason()
  Input:         in —— 指向 4 个字节
  Output:        无
  Return:        拼装后的 32 位数值
  Others:        每次移位前必须先转成 uint32_t，否则 uint8_t 左移 16/24 位会丢失高位。
-----------------------------------------------------------*/
static uint32_t get_u32(const uint8_t *in)
{
    return (uint32_t)in[0] |  /* 最低字节 */
           ((uint32_t)in[1] << 8) |  /* 次低字节 */
           ((uint32_t)in[2] << 16) |  /* 次高字节 */
           ((uint32_t)in[3] << 24);  /* 最高字节 */
}

/*-----------------------------------------------------------
  Function:      put_i32
  Description:   把 32 位有符号整数按小端写入 4 个字节，直接复用无符号版本。
  Calls:         put_u32()
  Called By:     m2_encode()
  Input:         out —— 4 字节缓冲区；value —— 待写入的有符号数值
  Output:        out 指向的 4 个字节
  Return:        无
  Others:        补码表示下有符号与无符号位模式相同，可直接转无符号处理。
-----------------------------------------------------------*/

static void put_i32(uint8_t *out, int32_t value)
{
    put_u32(out, (uint32_t)value);  /* 补码位模式相同，直接转无符号 */
}

/*-----------------------------------------------------------
  Function:      get_i32
  Description:   把 4 个小端字节拼回 32 位有符号整数，直接复用无符号版本。
  Calls:         get_u32()
  Called By:     m2_decode_reason()
  Input:         in —— 指向 4 个字节
  Output:        无
  Return:        拼装后的 32 位有符号数值
  Others:        补码表示下可直接由无符号结果强制转换得到。
-----------------------------------------------------------*/

static int32_t get_i32(const uint8_t *in)
{
    return (int32_t)get_u32(in);  /* 补码位模式相同，直接转有符号 */
}

/*-----------------------------------------------------------
  Function:      m2_crc
  Description:   计算 CRC-CCITT-FALSE 校验值（多项式 0x1021、初值 0xFFFF）。
  Calls:         无
  Called By:     m2_encode()、m2_decode_reason()
  Input:         data —— 数据首地址；length —— 参与计算的字节数
  Output:        无
  Return:        16 位校验值；data 为空指针时返回初值 0xFFFF
  Others:        校验范围为版本字段起 38 字节，不含帧头与校验值本身。
-----------------------------------------------------------*/
uint16_t m2_crc(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;  /* CCITT-FALSE 初值 */
    size_t i;  /* 字节下标 */
    unsigned bit;  /* 位下标 */

    if (data == 0)  /* 空指针 */
    {
        return crc;  /* 返回初值 */
    }
    for (i = 0U; i < length; ++i)  /* 逐字节处理 */
    {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);  /* 当前字节异或到高 8 位 */
        for (bit = 0U; bit < 8U; ++bit)  /* 逐位处理 */
        {
            crc = (uint16_t)(((crc & 0x8000U) != 0U) ?  /* 最高位为 1 时 */
                (((uint32_t)crc << 1) ^ 0x1021U) : ((uint32_t)crc << 1));  /* 左移后异或多项式 0x1021 */
        }
    }
    return crc;  /* 返回 16 位校验值 */
}

/*-----------------------------------------------------------
  Function:      m2_scale
  Description:   把浮点测量值乘以给定系数并四舍五入为整数，用于按固定倍数
                 把物理量压成整数写入协议字段（如电流 A 转 mA 用 1000）。
  Calls:         无
  Called By:     telemetry.c 的 telemetry_poll()
  Input:         value —— 待转换的浮点值；factor —— 放大倍数（不能为 0）
  Output:        scaled —— 转换结果，仅在返回 1 时有效
  Return:        1 = 成功；0 = 参数非法或结果超出 int32 范围
  Others:        NaN 与无穷大因范围判断不成立而被拒绝，调用方据此不置有效位。
-----------------------------------------------------------*/
/* 在把缩放结果转成整数之前，先拒绝 NaN/无穷大与整数溢出。 */
bool m2_scale(float value, uint32_t factor, int32_t *scaled)
{
    double product = (double)value * (double)factor;  /* 用 double 乘，避免提前溢出 */

    if (scaled == 0 || factor == 0U ||  /* 输出指针为空或倍数为 0 */
        !(product > -2147483648.5 && product < 2147483647.5))  /* 或越界/NaN（比较不成立即拒绝） */
    {
        return false;  /* 转换失败 */
    }
    product += product < 0.0 ? -0.5 : 0.5;  /* 四舍五入（远离零） */
    *scaled = (int32_t)product;  /* 写回整数结果 */
    return true;  /* 转换成功 */
}

/*-----------------------------------------------------------
  Function:      m2_encode
  Description:   把 M2Frame 打包成 42 字节定长帧并附加 CRC 校验值。
  Calls:         put_u32()、put_i32()、m2_crc()
  Called By:     telemetry.c 的 telemetry_poll()
  Input:         frame —— 待打包数据；buffer —— 输出缓冲；capacity —— 缓冲容量
  Output:        buffer 中写入完整 42 字节帧
  Return:        成功返回 M2_FRAME_SIZE(42)；参数非法或数值越界返回 0
  Others:        入参体检在编码前完成，越界数据一律拒发，避免送出违规帧。
-----------------------------------------------------------*/
/* 定长编码让每帧的 UART 占用时间恒定，100 帧/秒也稳稳扛得住。 */
size_t m2_encode(const M2Frame *frame, uint8_t *buffer, size_t capacity)
{
    uint16_t crc;  /* 校验值 */
    unsigned i;  /* 测量值下标 */

    if (frame == 0 || buffer == 0 || capacity < M2_FRAME_SIZE ||  /* 指针与容量检查 */
        frame->level > 4U || frame->duty > 88U ||  /* 档位/占空比越界 */
        frame->validMask > M2_VALID_ALL || frame->aiState > M2_AI_STATE_MAX)  /* 有效位 / AI 状态越界 */
    {
        return 0U;  /* 拒发，返回长度 0 */
    }

    buffer[0] = M2_SYNC0;  /* 帧头 0xA5 */
    buffer[1] = M2_SYNC1;  /* 帧头 0x5A */
    buffer[2] = M2_VERSION;  /* 协议版本 */
    buffer[3] = M2_PAYLOAD_SIZE;  /* 载荷长度 36 */
    put_u32(&buffer[4], frame->seq);  /* [4..7] 帧序号 */
    put_u32(&buffer[8], frame->uptimeMs);  /* [8..11] 开机毫秒 */
    buffer[12] = frame->level;  /* [12] 档位 */
    buffer[13] = frame->duty;  /* [13] 占空比 */
    buffer[14] = frame->validMask;  /* [14] 有效位 */
    buffer[15] = frame->aiState;  /* [15] AI 状态 */
    for (i = 0U; i < 6U; ++i)  /* [16..39] 6 个测量值 */
    {
        put_i32(&buffer[16U + 4U * i], frame->value[i]);  /* 每个占 4 字节，小端 */
    }
    crc = m2_crc(&buffer[2], 38U);  /* CRC 覆盖「版本」起 38 字节 */
    buffer[40] = (uint8_t)crc;  /* CRC 低字节 */
    buffer[41] = (uint8_t)(crc >> 8);  /* CRC 高字节 */
    return M2_FRAME_SIZE;  /* 返回整帧长度 42 */
}

/*-----------------------------------------------------------
  Function:      m2_decode_reason
  Description:   严格解析一帧数据，并返回具体的失败原因供上层分类计数。
  Calls:         get_u32()、get_i32()、m2_crc()
  Called By:     m2_decode()、m2_feed()
  Input:         data —— 42 字节帧；length —— 长度（必须等于 42）
  Output:        frame —— 解析结果，仅在返回 M2_REJECT_NONE 时有效
  Return:        M2RejectReason 枚举（NONE/VERSION/LENGTH/FORMAT/CRC/RANGE/OTHER）
  Others:        检查顺序由外到内，使一帧只归因于第一层失败，避免计数互相污染。
-----------------------------------------------------------*/
/* 严格解码器：先按「成帧层」分类失败原因，再做 CRC 与范围检查。 */
typedef enum {
    M2_REJECT_NONE = 0,  /* 无错误 */
    M2_REJECT_VERSION,  /* 版本不符 */
    M2_REJECT_LENGTH,  /* 长度不符 */
    M2_REJECT_FORMAT,  /* 帧头/字段格式错 */
    M2_REJECT_CRC,  /* CRC 校验失败 */
    M2_REJECT_RANGE,  /* 数值越界 */
    M2_REJECT_OTHER  /* 其它原因 */
} M2RejectReason;  /* 拒收原因分类 */

static M2RejectReason m2_decode_reason(const uint8_t *data, size_t length, M2Frame *frame)
{
    M2Frame parsed;  /* 解析结果暂存（成功才发布） */
    uint16_t expected;  /* 帧内携带的 CRC */
    unsigned i;  /* 测量值下标 */

    /* 2026-10-07：让拒收计数只归因到第一层失败，避免分类互相污染。 */
    if (data == 0 || frame == 0)  /* 指针为空 */
    {
        return M2_REJECT_OTHER;  /* 归入「其它原因」 */
    }
    if (length != M2_FRAME_SIZE) { return M2_REJECT_LENGTH; }  /* 长度必须正好 42 */
    if (data[0] != M2_SYNC0 || data[1] != M2_SYNC1) { return M2_REJECT_FORMAT; }  /* 帧头两个字节必须都对 */
    if (data[2] != M2_VERSION) { return M2_REJECT_VERSION; }  /* 版本必须匹配 */
    if (data[3] != M2_PAYLOAD_SIZE) { return M2_REJECT_LENGTH; }  /* 载荷长度字段必须匹配 */
    if (data[14] > M2_VALID_ALL || data[15] > M2_AI_STATE_MAX) { return M2_REJECT_FORMAT; }  /* 有效位 / AI 状态越界 */
    expected = (uint16_t)data[40] | ((uint16_t)data[41] << 8);  /* 小端拼出帧内 CRC */
    if (m2_crc(&data[2], 38U) != expected)  /* 本地重算 CRC 比对 */
    {
        return M2_REJECT_CRC;  /* 归入 CRC 失败 */
    }

    parsed.seq = get_u32(&data[4]);  /* 帧序号 */
    parsed.uptimeMs = get_u32(&data[8]);  /* 开机毫秒 */
    parsed.level = data[12];  /* 档位 */
    parsed.duty = data[13];  /* 占空比 */
    parsed.aiState = data[15];  /* AI 状态 */
    if (parsed.level > 4U || parsed.duty > 88U)  /* 档位 / 占空比越界 */
    {
        return M2_REJECT_RANGE;  /* 归入数值越界 */
    }
    parsed.validMask = data[14];  /* 有效位 */
    for (i = 0U; i < 6U; ++i)  /* 6 个测量值 */
    {
        parsed.value[i] = get_i32(&data[16U + 4U * i]);  /* 小端拼回 int32 */
    }
    *frame = parsed;  /* 全部检查通过才发布结果 */
    return M2_REJECT_NONE;  /* 解析成功 */
}

/*-----------------------------------------------------------
  Function:      m2_decode
  Description:   解析一帧数据，仅给出成功或失败的布尔结果。
  Calls:         m2_decode_reason()
  Called By:     无
  Input:         data —— 42 字节帧；length —— 长度
  Output:        frame —— 解析结果，仅在返回 true 时有效
  Return:        true = 解析成功；false = 解析失败
  Others:        需要具体失败原因时改用 m2_decode_reason()。
-----------------------------------------------------------*/
bool m2_decode(const uint8_t *data, size_t length, M2Frame *frame)
{
    return m2_decode_reason(data, length, frame) == M2_REJECT_NONE;  /* 只看是否成功，不关心原因 */
}

/*-----------------------------------------------------------
  Function:      record_reject
  Description:   记录一次拒收：总数加一，并按原因累加对应的分类计数。
  Calls:         无
  Called By:     m2_discard()、m2_feed()
  Input:         receiver —— 接收器状态；reason —— 拒收原因
  Output:        更新 receiver 的 rejected 与各分类计数
  Return:        无
  Others:        这些计数最终会出现在 D1 的 /status、/raw 与 USB 抓包报告中。
-----------------------------------------------------------*/
static void record_reject(M2Receiver *receiver, M2RejectReason reason)
{
    if (receiver == 0) { return; }  /* 入参为空 */
    ++receiver->rejected;  /* 拒收总数加一 */
    switch (reason)
    {
    case M2_REJECT_VERSION: ++receiver->rejectVersion; break;  /* 版本不符 */
    case M2_REJECT_LENGTH: ++receiver->rejectLength; break;  /* 长度不符 */
    case M2_REJECT_FORMAT: ++receiver->rejectFormat; break;  /* 格式错 */
    case M2_REJECT_CRC: ++receiver->rejectCrc; break;  /* CRC 失败 */
    case M2_REJECT_RANGE: ++receiver->rejectRange; break;  /* 数值越界 */
    default: ++receiver->rejectOther; break;  /* 其它 */
    }
}

/*-----------------------------------------------------------
  Function:      m2_reset
  Description:   复位收帧状态机，丢弃正在接收的半截帧。
  Calls:         无
  Called By:     m2_discard()、m2_feed()
  Input:         receiver —— 接收器状态
  Output:        清零 used 与 expected
  Return:        无
  Others:        不清 latest / hasFrame 及各计数器，避免一次丢帧导致显示空白。
-----------------------------------------------------------*/
void m2_reset(M2Receiver *receiver)
{
    if (receiver != 0)  /* 入参有效 */
    {
        receiver->used = 0U;  /* 已收字节数清零 */
        receiver->expected = 0U;  /* 本帧期望长度清零 */
    }
}

/*-----------------------------------------------------------
  Function:      m2_discard
  Description:   丢弃半截帧，并记一次「其它原因」的拒收。
  Calls:         m2_reset()、record_reject()
  Called By:     m2_feed()、接收端的空闲超时处理
  Input:         receiver —— 接收器状态
  Output:        清空收帧缓冲
  Return:        无
  Others:        典型触发：半帧静置超过 30ms 无新字节（对端复位或断线）。
-----------------------------------------------------------*/
/* 丢掉半截帧，同时保留最近一次已接受的快照。 */
void m2_discard(M2Receiver *receiver)
{
    if (receiver != 0)  /* 入参有效 */
    {
        m2_reset(receiver);  /* 丢弃半截帧 */
        record_reject(receiver, M2_REJECT_OTHER);  /* 记一次「其它原因」拒收 */
    }
}

/*-----------------------------------------------------------
  Function:      m2_feed
  Description:   逐字节收帧状态机。按已收字节数推进：等帧头1 -> 等帧头2 ->
                 累积载荷 -> 收满 42 字节后整体校验，校验通过则更新最新帧。
  Calls:         m2_decode_reason()、m2_reset()、m2_discard()、record_reject()
  Called By:     接收侧串口搬运函数（D1 的 recv_uart，每个字节调用一次）
  Input:         receiver —— 接收器状态；byte —— 本次收到的字节；nowMs —— 当前毫秒
  Output:        更新 receiver 的缓冲、latest 与各计数
  Return:        true = 本次凑齐并校验通过一整帧；false = 尚未成帧或被拒收
  Others:        1) 载荷内部出现帧头字节不会被误判，used>=2 后一律按数据处理；
                                  2) 序号与时间戳均与上一帧相同的重复帧计入 duplicates 并丢弃；
                                  3) 序号回退计入 resyncs，但该帧仍被接受。
-----------------------------------------------------------*/
/* 逐字节状态机；载荷内部出现的帧头字节按普通数据处理。 */
bool m2_feed(M2Receiver *receiver, uint8_t byte, uint32_t nowMs)
{
    M2Frame next;  /* 刚收到的这一帧 */

    if (receiver == 0)  /* 入参为空 */
    {
        return false;  /* 直接返回 */
    }
    if (receiver->used == 0U)  /* 状态 0：等帧头 1 */
    {
        if (byte == M2_SYNC0)  /* 收到 0xA5 */
        {
            receiver->buffer[0] = byte;  /* 存进缓冲 */
            receiver->used = 1U;  /* 进入状态 1 */
        }
        return false;  /* 本字节处理完毕 */
    }
    if (receiver->used == 1U)  /* 状态 1：等帧头 2 */
    {
        if (byte == M2_SYNC1)  /* 收到 0x5A，帧头凑齐 */
        {
            receiver->buffer[1] = byte;  /* 存进缓冲 */
            receiver->used = 2U;  /* 进入状态 2 */
        }
        else if (byte == M2_SYNC0)  /* 又是一个 0xA5：重新对齐 */
        {
            receiver->buffer[0] = byte;  /* 覆盖首字节，仍停在状态 1 */
        }
        else
        {
            receiver->used = 0U;  /* 其它字节：整帧作废，重新等 */
        }
        return false;  /* 本字节处理完毕 */
    }
    if (receiver->used >= M2_FRAME_MAX)  /* 超出缓冲区（正常不会发生） */
    {
        m2_discard(receiver);  /* 丢弃并计数 */
        return false;  /* 直接返回 */
    }
    receiver->buffer[receiver->used++] = byte;  /* 存入并前移收帧位置 */
    if (receiver->used == 3U && byte != M2_VERSION)  /* 第 3 字节必须是版本号 */
    {
        m2_reset(receiver);  /* 复位状态机 */
        record_reject(receiver, M2_REJECT_VERSION);  /* 记版本不符 */
        return false;  /* 本帧作废 */
    }
    if (receiver->used == 4U)  /* 第 4 字节是载荷长度 */
    {
        if (byte != M2_PAYLOAD_SIZE)  /* 长度字段不符 */
        {
            m2_reset(receiver);  /* 复位状态机 */
            record_reject(receiver, M2_REJECT_LENGTH);  /* 记长度不符 */
            return false;  /* 本帧作废 */
        }
        receiver->expected = M2_FRAME_SIZE;  /* 确认本帧共 42 字节 */
    }
    if (receiver->expected != 0U && receiver->used == receiver->expected)  /* 已收满一整帧 */
    {
        const M2RejectReason reason = m2_decode_reason(receiver->buffer, M2_FRAME_SIZE, &next);  /* 整体校验并解析 */
        m2_reset(receiver);  /* 无论成败都复位，准备下一帧 */
        if (reason != M2_REJECT_NONE)  /* 校验不通过 */
        {
            record_reject(receiver, reason);  /* 按原因分类计数 */
            return false;  /* 本帧丢弃 */
        }
        if (receiver->hasFrame && next.seq == receiver->latest.seq &&  /* 序号与时间戳都与上一帧相同 */
            next.uptimeMs == receiver->latest.uptimeMs)  /* 判定为重复帧 */
        {
            ++receiver->duplicates;  /* 重复帧计数加一 */
            return false;  /* 丢弃重复帧 */
        }
        if (receiver->hasFrame && next.seq < receiver->latest.seq)  /* 序号回退：对端可能重启过 */
        {
            ++receiver->resyncs;  /* 重新同步计数加一（该帧仍接受） */
        }
        receiver->latest = next;  /* 发布最新帧 */
        receiver->lastNewMs = nowMs;  /* 记录收到时刻 */
        receiver->hasFrame = true;  /* 标记已收到过帧 */
        ++receiver->accepted;  /* 接收成功计数加一 */
        return true;  /* 本次凑齐并校验通过 */
    }
    return false;  /* 尚未收满 42 字节 */
}

/*-----------------------------------------------------------
  Function:      m2_link
  Description:   判断链路当前状态：从未收到帧为 WAITING，超过 3 秒无新帧为 STALE，
                 否则为 LIVE。
  Calls:         无
  Called By:     显示与上报逻辑（D1 的 status_json.cpp 等）
  Input:         receiver —— 接收器状态；nowMs —— 当前毫秒
  Output:        无
  Return:        M2_WAITING / M2_LIVE / M2_STALE
  Others:        使用无符号相减计算时间差，在毫秒计数回绕时仍保持正确。
-----------------------------------------------------------*/
/* 用无符号相减算时间差，毫秒计数回绕时判定仍然正确。 */
M2Link m2_link(const M2Receiver *receiver, uint32_t nowMs)
{
    if (receiver == 0 || !receiver->hasFrame)  /* 入参为空或从未收到过帧 */
    {
        return M2_WAITING;  /* 等待中 */
    }
    return (uint32_t)(nowMs - receiver->lastNewMs) > M2_TIMEOUT_MS ?  /* 距上次收到帧的时间差 */
        M2_STALE : M2_LIVE;  /* 超时 = STALE，否则 LIVE */
}
