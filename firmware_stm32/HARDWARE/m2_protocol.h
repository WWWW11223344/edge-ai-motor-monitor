/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    m2_protocol.h
  Author:      汪贤云
  Version:     1.0
  Date:        2026-10-05
  Description: M2 二进制遥测协议规格书，供 STM32 与 D1 mini 双方共同遵守。
               一帧固定 42 字节，各字节含义如下（偏移从 0 开始）：
               [0]=0xA5 [1]=0x5A 帧头，[2]=0x01 版本，[3]=36 载荷长度，
               [4..7] 帧序号 seq，[8..11] 开机毫秒 uptimeMs，
               [12] 档位 level，[13] 占空比 duty(0~88)，[14] 有效位 validMask，
               [15] AI 状态 aiState，[16..39] 6 个 int32 测量值（小端）：
               电流mA / 加速度X,Y,Z(mg) / 转速x10 / 母线电压mV，[40..41] CRC 校验。
               CRC 覆盖 [2] 起 38 字节，不含帧头与 CRC 本身。
  Function List:
    1. m2_crc()        计算 CCITT-FALSE 校验值
    2. m2_scale()      浮点数按系数转整数（含溢出保护）
    3. m2_encode()     M2Frame 打包为 42 字节
    4. m2_decode()     42 字节还原为 M2Frame
    5. m2_feed()       逐字节接收状态机
    6. m2_link()       判断链路 waiting/live/stale
  History:
    <author>   <time>      <version>   <desc>
    汪贤云     2026-10-05    1.0         创建文件
***********************************************************/
#ifndef M2_PROTOCOL_H
#define M2_PROTOCOL_H

#include <stdbool.h>   /* bool 类型 */
#include <stddef.h>   /* size_t 类型 */
#include <stdint.h>   /* 定长整数类型 */

#ifdef __cplusplus  /* C++ 下按 C 方式链接 */
extern "C" {        /* 保持 C 符号名 */
#endif               /* __cplusplus 条件编译段结束 */

/* 【协议常量】
 *   M2_SYNC0/1     帧头两个字节。接收方靠它们重新同步（找不到 0xA5 0x5A 就不认）
 *   M2_VERSION     协议版本。版本不匹配的帧直接丢弃，防止新旧固件混跑时读错字段
 *   M2_PAYLOAD_SIZE 载荷长度 = 36（从 seq 到第 6 个测量值）
 *   M2_FRAME_SIZE   整帧 42 字节 = 2 帧头 + 1 版本 + 1 长度 + 36 载荷 + 2 CRC
 *   M2_TIMEOUT_MS   超过 3 秒没收到新帧，就判定链路断了（界面显示"通信中断"） */
#define M2_SYNC0           0xA5U
#define M2_SYNC1           0x5AU
#define M2_VERSION         0x01U         /* 协议版本，版本不符的帧直接丢弃 */
#define M2_PAYLOAD_SIZE    36U       /* 载荷长度：从 seq 到第 6 个测量值 */
#define M2_FRAME_SIZE      42U       /* 整帧长度 */
#define M2_FRAME_MAX       M2_FRAME_SIZE  /* 接收缓冲上限 */
#define M2_TIMEOUT_MS      3000U     /* 超过此时长没新帧即判定链路断开 */

/* 【有效位 validMask：哪几个测量值这次是真的】
 *    bit0 电流  bit1 X  bit2 Y  bit3 Z  bit4 转速  bit5 电压
 * 某位是 0 表示「这一路这次没采到 / 数据无效」，
 * 接收方必须显示成空（null），绝不能当 0 —— 0 会被误读成真实测量值。
 * M2_VALID_ALL(0x3F) 表示 6 路全部有效。 */
#define M2_VALID_CURRENT   (1U << 0)
#define M2_VALID_ACCEL_X   (1U << 1)
#define M2_VALID_ACCEL_Y   (1U << 2)  /* bit2：Y 轴加速度 */
#define M2_VALID_ACCEL_Z   (1U << 3)  /* bit3：Z 轴加速度 */
#define M2_VALID_SPEED     (1U << 4)  /* bit4：转速 */
#define M2_VALID_BATTERY   (1U << 5)  /* bit5：母线电压 */
#define M2_VALID_ALL       0x3FU        /* 6 路测量值全部有效 */

/* 【AI 状态 aiState：借用第 15 字节】
 *    bit0..1  阶段：0=未启用 1=学习中 2=检测中 3=错误
 *    bit2     报警标志：1 = 当前处于报警状态
 * 老版本固件把第 15 字节固定写 0（等价于"未启用"），
 * 所以新固件解析旧帧也不会出错；大于 7 的值一律判为非法帧。 */
#define M2_AI_OFF          0U
#define M2_AI_LEARNING     1U
#define M2_AI_DETECTING    2U          /* 检测中 */
#define M2_AI_ERROR        3U          /* 模型出错 */
#define M2_AI_PHASE_MASK   0x03U       /* 取阶段值的掩码 */
#define M2_AI_ALARM_BIT    0x04U       /* 报警标志位 */
#define M2_AI_STATE_MAX    0x07U       /* 合法最大值，超过即判为非法帧 */

/* M2Frame：一帧测量数据的内存表示，发送端填充、接收端解析 */
typedef struct {
    uint32_t seq;          /* 帧序号，上电从 0 开始每发一帧 +1 */
    uint32_t uptimeMs;     /* 发送时刻的开机毫秒数 */
    uint8_t level;        /* 档位：0 或 1 */
    uint8_t duty;         /* 实际输出占空比百分数 0~88 */
    uint8_t validMask;    /* 6 路测量值的有效位，见 M2_VALID_xxx */
    uint8_t aiState;      /* 板上 AI 状态：bit0~1 阶段，bit2 报警 */
    int32_t value[6];     /* 6 个测量值，顺序见文件头 */
} M2Frame;

/* M2Receiver：接收端状态机，记录收帧进度与各类计数 */
typedef struct {
    uint8_t buffer[M2_FRAME_MAX]; /* 拼帧暂存区，最多 M2_FRAME_MAX 字节 */
    uint8_t used;         /* 当前已接收的字节数 */
    uint8_t expected;     /* 本帧总长度，帧头确认后为 M2_FRAME_SIZE */
    M2Frame latest;      /* 最近一次校验通过的整帧 */
    uint32_t lastNewMs;   /* 收到最近一帧的时刻，用于判断链路状态 */
    uint32_t accepted;    /* 累计接收成功的帧数 */
    uint32_t rejected;    /* 累计拒收的帧数 */
    uint32_t rejectVersion;
    uint32_t rejectLength; /* 因帧长不符被拒的累计次数 */
    uint32_t rejectFormat;      /* 帧格式不合法的累计次数 */
    uint32_t rejectCrc;         /* CRC 校验失败的累计次数 */
    uint32_t rejectRange;       /* 数值越界的累计次数 */
    uint32_t rejectOther;       /* 其它原因的累计次数 */
    uint32_t duplicates;        /* 重复帧的累计次数 */
    uint32_t resyncs;          /* 重新同步的累计次数 */
    bool hasFrame;       /* 是否已经收到过至少一帧 */
} M2Receiver;

typedef enum { M2_WAITING, M2_LIVE, M2_STALE } M2Link;  /* 链路状态：等待 / 在线 / 超时 */

/* 【接口清单】编解码、校验、状态机都在 m2_protocol.c 里实现，两边（STM32 和 D1）
 * 用的是同一套算法，保证同一条数据在两头的结果完全一致：
 *   m2_crc()     算 CCITT-FALSE 校验
 *   m2_scale()   浮点 x 系数 -> 整数（带 NaN/溢出保护）
 *   m2_encode()  结构体 -> 42 字节
 *   m2_decode()  42 字节 -> 结构体（失败返回 false）
 *   m2_feed()    逐个字节喂进状态机；返回 true 表示凑齐并校验通过了一帧
 *   m2_link()    判断链路当前是 waiting / live / stale
 *   m2_reset()/m2_discard()  丢掉半截帧
 * 【调用约束】只在主循环里用，不要在中断里调用（内部没有做并发保护）。 */
uint16_t m2_crc(const uint8_t *data, size_t length);
/* 浮点遥测值的有限性缩放；失败时输出保持不变。 */
bool m2_scale(float value, uint32_t factor, int32_t *scaled);
size_t m2_encode(const M2Frame *frame, uint8_t *buffer, size_t capacity);
bool m2_decode(const uint8_t *data, size_t length, M2Frame *frame);  /* 整帧解码，失败返回 false */
void m2_discard(M2Receiver *receiver);  /* 丢弃半截帧 */
void m2_reset(M2Receiver *receiver);  /* 复位接收器（清空缓冲，保留诊断计数） */
bool m2_feed(M2Receiver *receiver, uint8_t byte, uint32_t nowMs);  /* 逐字节收帧；返回 true 表示凑齐并校验通过一帧 */
M2Link m2_link(const M2Receiver *receiver, uint32_t nowMs);  /* 判定链路状态 */

#ifdef __cplusplus  /* C++ 下按 C 方式链接 */
}
#endif  /* __cplusplus 条件编译段结束 */
#endif  /* M2_PROTOCOL_H 头文件结束 */
