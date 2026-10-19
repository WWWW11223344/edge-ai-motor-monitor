/* Codex, 2026-10-07：区分「接收器漏帧」与「原始历史被覆盖」两种情况。 */
#ifndef RAW_HISTORY_H   /* 头文件保护：防止重复包含 */
#define RAW_HISTORY_H   /* 定义保护宏 */
#include "m2_protocol.h"   /* 需要 M2Frame / M2Receiver 类型 */

#ifdef __cplusplus   /* C++ 编译时按 C 方式链接 */
extern "C" {   /* 保持 C 符号名 */
#endif   /* 条件编译段结束 */

#define M2_HISTORY_CAPACITY 256U   /* 历史缓冲容量：256 帧 */

typedef struct {   /* 环形历史缓冲 */
    M2Frame frames[M2_HISTORY_CAPACITY];   /* 帧数组，环形复用 */
    size_t head;   /* 下一个写入位置 */
    size_t count;   /* 当前已存帧数（未满时小于容量） */
    uint32_t evictedSeq;   /* 最近一次被挤出的帧序号 */
    bool hasEvicted;   /* 是否发生过挤出 */
} M2History;   /* M2History 结构体结束 */

typedef enum { M2_RAW_OLDEST, M2_RAW_AFTER, M2_RAW_TAIL } M2RawMode;   /* 查询模式：最早一批 / 指定序号之后 / 最新若干帧 */

typedef struct {   /* 原始数据查询条件 */
    M2RawMode mode;   /* 查询模式 */
    uint32_t after;   /* 只取该序号之后的帧（仅 AFTER 模式使用） */
    size_t limit;   /* 最多返回多少帧 */
} M2RawQuery;   /* M2RawQuery 结构体结束 */

typedef struct {   /* 查询结果切片 */
    size_t start;   /* 起始下标 */
    size_t count;   /* 帧数 */
    bool overflow;   /* 是否因超出 limit 而被截断 */
    bool sequenceGap;   /* 帧序号是否不连续（中间有缺口） */
} M2RawSlice;   /* M2RawSlice 结构体结束 */

/* 清零初始化历史缓冲。仅主循环调用；内部整帧拷贝，不保留外部指针。 */
void m2h_push(M2History *history, const M2Frame *frame);   /* 压入一帧并维护环形缓冲 */
/* 只读、不阻塞。入参必须非空；只有返回成功时 slice 才有效。 */
bool m2h_select(const M2History *history, const M2RawQuery *query, M2RawSlice *slice);   /* 按查询条件选出切片 */
/* 仅主循环调用：切片选出后要立刻拷贝，之后再有 push 内容就变了。 */
/* 输出缓冲不得与 history 重叠；拷贝成功后，后续 push 不会再影响该快照。 */
bool m2h_copy(const M2History *history, const M2RawSlice *slice,   /* 把选出的切片拷贝到调用方缓冲 */
    M2Frame *frames, size_t capacity);   /* 目标帧数组与容量 */

#ifdef __cplusplus   /* C++ 编译时按 C 方式链接 */
}
#endif   /* 条件编译段结束 */
#endif   /* RAW_HISTORY_H 结束 */
