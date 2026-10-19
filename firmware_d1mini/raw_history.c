/* ==========================================================================
 *  raw_history.c  --  最近 256 帧的「环形缓冲」（历史数据仓库）
 *
 *  【它解决什么问题】网页 GET /raw 要能一次拿出一批历史帧给上位机分析，
 *    但 ESP8266 内存很小，不可能无限存。做法是环形缓冲：
 *    固定 256 个格子，收满一圈后新数据覆盖最老的（老的自然被淘汰）。
 *
 *  【两个游标，别搞混】
 *    head  ：下一个要写的位置（写指针），每收一帧 +1 再对 256 取模
 *    count ：当前实际存了几帧（从 0 涨到 256 就不再涨）
 *    最老的一帧位置 = (head + 256 - count) % 256
 *
 *  【为什么要专门写这个文件（而不在 ino 里草草了事）】上位机靠 seq（帧序号）
 *    来判断「有没有漏收」，所以这里必须把三件事说清楚：
 *      ① 帧是按序号严格递增进来的；
 *      ② 中间缺号 = 真的漏收了（sequence gap）；
 *      ③ 数据被覆盖淘汰了 = overflow，和「漏收」是两回事，不能混为一谈。
 * ========================================================================== */
/* Codex, 2026-10-07：保留「向前跳号」的事实，但不重放旧的原始帧。 */
#include "raw_history.h"

/* 【判断「序号 a 是否在 b 之后」】不能直接比大小，因为 seq 是 uint32，会回绕。
 * 做法：算差值 delta = a - b（无符号减法），
 *   delta 在 (0, 2^31) 之间   -> a 在 b 之后（正常前进）
 *   delta == 0               -> 就是同一帧
 *   delta 很大（接近 2^32）  -> 说明 a 其实在 b 之前（回绕了）
 * 这样即使跑很久、序号回绕几次，顺序判断依然正确。 */
static bool seq_is_after(uint32_t current, uint32_t reference)
{
    const uint32_t delta = current - reference; /* 无符号相减：序号回绕时差值自然很大，据此判断先后 */
    return delta != 0U && delta < UINT32_C(0x80000000);   /* delta 非 0 且小于 2^31 即视为「在参考帧之后」 */
}

/* 【收一帧进缓冲】
 * ① 如果缓冲里已经有数据，先看新帧序号是不是真的在最后一帧之后：
 *    不是（序号变小或相等）-> 说明 STM32 重启过、或者又转回去了，
 *    此时把整个缓冲清空重新开始 —— 混着两段不同批次的数据会让
 *    /raw 的增量拉取逻辑（after=N）彻底混乱，宁可清掉。
 * ② 缓冲满了（count==256）时：把即将被覆盖那一格的序号记进 evictedSeq，
 *    并置 hasEvicted —— 这是「真的丢过数据」的唯一证据，
 *    后面 /raw 靠它判断是「上游漏发」还是「我们这边淘汰了」。
 * ③ 写入 head 位置，head 前移一格（取模 256 回到头部）。 */
/* 发送方重启即开启一段新的保留数据；uint32 回绕仍保持有序。 */
void m2h_push(M2History *history, const M2Frame *frame)
{
    if (history == NULL || frame == NULL) { /* 空指针保护：任一为 NULL 直接返回 */
        return;   /* 空指针保护 */
    }
    if (history->count != 0U) {   /* 缓冲里已经有帧 */
        const size_t latest = (history->head + M2_HISTORY_CAPACITY - 1U) % M2_HISTORY_CAPACITY;   /* 最后一帧所在下标 */
        if (!seq_is_after(frame->seq, history->frames[latest].seq)) {   /* 新帧序号没有前进（对端重启或回绕） */
            history->head = 0U;   /* 清空缓冲，重新开始一段 */
            history->count = 0U;   /* 帧数归零 */
            history->hasEvicted = false;   /* 清掉淘汰标志 */
        }
    }
    if (history->count == M2_HISTORY_CAPACITY) {   /* 缓冲已满：这一格即将被覆盖 */
        history->evictedSeq = history->frames[history->head].seq;   /* 记下被挤出的序号，这是「真丢过数据」的证据 */
        history->hasEvicted = true;   /* 置淘汰标志 */
    } else {   /* 未满则只需增加计数 */
        ++history->count;   /* 帧数加一 */
    }
    history->frames[history->head] = *frame;   /* 整帧写入写指针位置 */
    history->head = (history->head + 1U) % M2_HISTORY_CAPACITY;   /* 写指针环形前进一格 */
}

/* 【查询：算出「这次要返回哪一段」】不做拷贝，只算 start / count 两个下标。
 * 三种模式：
 *   M2_RAW_TAIL  ：要最新的 limit 帧 -> 从 (head - limit) 开始
 *   M2_RAW_AFTER ：要序号 > after 的帧 -> 从最老的开始逐个找，
 *                  找到第一个「序号在 after 之后」的位置就停下，
 *                  剩下全部给它
 *   默认（OLDEST）：从最老的一帧开始给
 * 【两个布尔标志的含义，别弄混】
 *   sequenceGap  ：找到的第一个帧的序号 != after+1，说明中间真的漏收了
 *   overflow     ：缓冲淘汰过数据，而且被淘汰的序号是 after+1 之后的
 *                  -> 那些帧不是漏收，是被我们覆盖掉了（信息不同，结论也不同）
 * 【最后的 limit 裁剪】无论哪种模式，一次最多给 query->limit 帧。 */
/* 缺少的期望帧才算「缺口」；「被覆盖」必须有淘汰证据。 */
bool m2h_select(const M2History *history, const M2RawQuery *query, M2RawSlice *slice)
{
    M2RawSlice result = {0U, 0U, false, false}; /* 结果先全零：起始下标 / 帧数 / 是否淘汰 / 是否断号 */
    size_t oldest;   /* 最老一帧的下标 */
    if (history == NULL || query == NULL || slice == NULL || query->limit == 0U ||   /* 入参合法性检查 */
        query->limit > M2_HISTORY_CAPACITY || history->count > M2_HISTORY_CAPACITY ||   /* limit 不得为 0、不得超过容量；count 与 head 必须在范围内 */
        history->head >= M2_HISTORY_CAPACITY || query->mode > M2_RAW_TAIL ||   /* 模式必须在枚举范围内 */
        query->mode < M2_RAW_OLDEST) {   /* 续：模式下界检查 */
        return false;   /* 参数非法 */
    }
    oldest = (history->head + M2_HISTORY_CAPACITY - history->count) % M2_HISTORY_CAPACITY;   /* 最老一帧位置 = (head + 容量 - count) 对容量取模 */
    result.start = oldest;   /* 默认从最老一帧开始给 */
    result.count = history->count;   /* 先假设把现有帧全部给出 */
    if (query->mode == M2_RAW_TAIL && result.count > query->limit) {   /* TAIL 模式且现有帧数超过 limit */
        result.start = (history->head + M2_HISTORY_CAPACITY - query->limit) % M2_HISTORY_CAPACITY;   /* 改成从 (head - limit) 开始 */
        result.count = query->limit;   /* 只给 limit 帧 */
    } else if (query->mode == M2_RAW_AFTER) {   /* AFTER 模式：找第一个序号在 after 之后的帧 */
        size_t offset;   /* 相对最老帧的偏移 */
        result.count = 0U;   /* 先置 0，找到再赋值 */
        /* 即使 after+1 已被拒收，也要选中它的第一个后继帧。 */
        for (offset = 0U; offset < history->count; ++offset) {
            const size_t index = (oldest + offset) % M2_HISTORY_CAPACITY;
            if (seq_is_after(history->frames[index].seq, query->after)) { /* 找到第一个序号排在 after 之后的帧 */
                const uint32_t expected = query->after + 1U;   /* 期望的下一帧序号 */
                result.start = index;   /* 从这一帧开始给 */
                result.count = history->count - offset;   /* 剩下全部给它 */
                result.sequenceGap = history->frames[index].seq != expected;   /* 首帧序号不等于期望值 = 中间真的漏收了 */
                result.overflow = history->hasEvicted &&   /* 判断是否属于「被我们覆盖掉」 */
                    (history->evictedSeq == expected || seq_is_after(history->evictedSeq, expected));   /* 被淘汰的序号等于或晚于期望值 */
                break;   /* 找到即跳出 */
            }
        }
    }
    if (result.count > query->limit) {   /* 最终裁剪：一次最多给 limit 帧 */
        result.count = query->limit;   /* 截到 limit */
    }
    *slice = result;   /* 写回查询结果 */
    return true;   /* 查询成功 */
}

/* 【真正把帧拷出来】把选中的那一段复制到调用方给的数组里。
 * 【为什么必须「先选后拷」两次调用】拷贝要花时间，而这段时间里
 * UART 还在往环形缓冲里推新帧（head 会动）。所以：
 *   先用 m2h_select 把「要哪一段」定下来（只记下标），
 *   再在拷贝时按固定的 start/count 去取 —— 这样即使期间 head 变了，
 *   拷出来的也是一份前后一致的快照（不会一半新一半旧）。
 * 这里把参数检查做得很足：start 越界、count 超过容量、
 * 或者要求跨过环形边界却算不出来的情况，一律返回 false（宁可报错，不要拷出垃圾）。
 * 注意拷贝循环里用 (start + index) % 256 取模，跨过下标 0 的切片也能正确取到。 */
/* Codex, 2026-10-07：UART 正在更新环形缓冲时，冻结本次 HTTP 响应的内容。 */
bool m2h_copy(const M2History *history, const M2RawSlice *slice,
    M2Frame *frames, size_t capacity)
{
    size_t oldest;   /* 最老一帧下标 */
    size_t offset;   /* 切片起点相对最老帧的偏移 */
    size_t index;   /* 拷贝循环下标 */
    if (history == NULL || slice == NULL || frames == NULL ||   /* 入参合法性检查 */
        history->head >= M2_HISTORY_CAPACITY || history->count > M2_HISTORY_CAPACITY ||   /* head 与 count 必须在范围内 */
        slice->start >= M2_HISTORY_CAPACITY || slice->count > history->count ||   /* 切片起点合法，且帧数不超过现有帧数 */
        slice->count > capacity) {   /* 调用方缓冲要装得下 */
        return false;   /* 参数非法 */
    }
    oldest = (history->head + M2_HISTORY_CAPACITY - history->count) % M2_HISTORY_CAPACITY;   /* 同上：定位最老一帧 */
    offset = (slice->start + M2_HISTORY_CAPACITY - oldest) % M2_HISTORY_CAPACITY;   /* 把切片起点换算成相对偏移 */
    if (slice->count != 0U && (offset >= history->count ||   /* 偏移越界，或切片越过已存数据末尾 */
        slice->count > history->count - offset)) {   /* 续：切片长度校验 */
        return false;   /* 越界，拒绝拷贝 */
    }
    /* 写入之前先校验所有边界，包括跨过下标 0 的切片。 */
    for (index = 0U; index < slice->count; ++index) {
        frames[index] = history->frames[(slice->start + index) % M2_HISTORY_CAPACITY];
    }
    return true;   /* 拷贝完成 */
}
