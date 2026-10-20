/* 2026-10-05：M2 二进制原始遥测协议，服务于板上 AI 的特征链路。
 * 2026-10-19 起 AI 输入改为「32 点转速单变量 R」，帧格式保持不变；
 * 加速度三槽（bit1~bit3）随之停写，恒为 0。 */
#ifndef M2_PROTOCOL_H   /* 头文件保护：防止重复包含 */
#define M2_PROTOCOL_H   /* 定义保护宏 */

#include <stdbool.h>   /* 布尔类型 */
#include <stddef.h>   /* size_t 类型 */
#include <stdint.h>   /* 定宽整数类型 */

#ifdef __cplusplus   /* C++ 编译时按 C 方式链接 */
extern "C" {   /* 保持 C 符号名 */
#endif   /* 条件编译段结束 */

#define M2_SYNC0           0xA5U   /* 帧头第 1 字节 */
#define M2_SYNC1           0x5AU   /* 帧头第 2 字节 */
#define M2_VERSION         0x01U   /* 协议版本 */
#define M2_PAYLOAD_SIZE    36U   /* 负载字节数（不含帧头与校验） */
#define M2_FRAME_SIZE      42U   /* 整帧字节数 */
#define M2_FRAME_MAX       M2_FRAME_SIZE   /* 接收缓冲上限，与整帧长度一致 */
#define M2_TIMEOUT_MS      3000U   /* 超过此时长未收到新帧即判定链路超时 */
#define M2_VALID_CURRENT   (1U << 0)   /* 有效位 0：电流 */
#define M2_VALID_ACCEL_X   (1U << 1)   /* 有效位 1：加速度 X（2026-10-19 起恒为 0） */
#define M2_VALID_ACCEL_Y   (1U << 2)   /* 有效位 2：加速度 Y（2026-10-19 起恒为 0） */
#define M2_VALID_ACCEL_Z   (1U << 3)   /* 有效位 3：加速度 Z（2026-10-19 起恒为 0） */
#define M2_VALID_SPEED     (1U << 4)   /* 有效位 4：转速 */
#define M2_VALID_BATTERY   (1U << 5)   /* 有效位 5：母线电压 */
#define M2_VALID_ALL       0x3FU   /* 历史定义：6 个测量值全有效（现只用 bit0/4/5） */

/* 保留控制字节 15：承载板上 AI 的阶段与报警标志。 */
#define M2_AI_OFF          0U   /* AI 未启用 */
#define M2_AI_LEARNING     1U   /* AI 学习中 */
#define M2_AI_DETECTING    2U   /* AI 检测中 */
#define M2_AI_ERROR        3U   /* AI 出错 */
#define M2_AI_PHASE_MASK   0x03U   /* 取阶段值的掩码（低 2 位） */
#define M2_AI_ALARM_BIT    0x04U   /* 报警标志位 */
#define M2_AI_STATE_MAX    0x07U   /* 状态字节最大值，用于合法性校验 */

typedef struct {   /* M2 定长数据帧 */
    uint32_t seq;   /* 帧序号，每发一帧加一 */
    uint32_t uptimeMs;   /* 板端运行毫秒数（上电起算） */
    uint8_t level;   /* 档位 0~4 */
    uint8_t duty;   /* 占空比百分数 0~88 */
    uint8_t validMask;   /* 有效位掩码（现只用 bit0 电流 / bit4 转速 / bit5 母线） */
    uint8_t aiState;   /* AI 阶段 + 报警标志 */
    int32_t value[6];   /* 6 个测量值；加速度三槽已停写，槽位保留以兼容帧格式 */
} M2Frame;   /* M2Frame 结构体结束 */

typedef struct {   /* 接收器状态（逐字节状态机） */
    uint8_t buffer[M2_FRAME_MAX];   /* 攒帧缓冲 */
    uint8_t used;   /* 已收到字节数 */
    uint8_t expected;   /* 本帧预期总长度 */
    M2Frame latest;   /* 最近一次校验通过的帧 */
    uint32_t lastNewMs;   /* 最近一次收到新帧的时刻 */
    uint32_t accepted;   /* 累计成功接收帧数 */
    uint32_t rejected;   /* 累计拒收帧数 */
    uint32_t rejectVersion;   /* 拒收分类：版本不符 */
    uint32_t rejectLength;   /* 拒收分类：长度不符 */
    uint32_t rejectFormat;   /* 拒收分类：格式不符 */
    uint32_t rejectCrc;   /* 拒收分类：CRC 校验错 */
    uint32_t rejectRange;   /* 拒收分类：数值超出范围 */
    uint32_t rejectOther;   /* 拒收分类：其它 */
    uint32_t duplicates;   /* 重复帧计数 */
    uint32_t resyncs;   /* 序号或时间回退（重新同步）次数 */
    bool hasFrame;   /* 是否已有可用帧 */
} M2Receiver;   /* M2Receiver 结构体结束 */

typedef enum { M2_WAITING, M2_LIVE, M2_STALE } M2Link;   /* 链路状态：等待首帧 / 在线 / 已超时 */

uint16_t m2_crc(const uint8_t *data, size_t length);   /* 计算 CRC-CCITT-FALSE */
bool m2_scale(float value, uint32_t factor, int32_t *scaled);   /* 浮点按系数缩放到 int32，含溢出与 NaN 检查 */
size_t m2_encode(const M2Frame *frame, uint8_t *buffer, size_t capacity);   /* 编码一帧，返回实际写入字节数 */
bool m2_decode(const uint8_t *data, size_t length, M2Frame *frame);   /* 解码一帧，失败返回 false */
void m2_discard(M2Receiver *receiver);   /* 丢弃当前半截帧并记一次拒收 */
void m2_reset(M2Receiver *receiver);   /* 复位接收器的全部状态 */
bool m2_feed(M2Receiver *receiver, uint8_t byte, uint32_t nowMs);   /* 逐字节喂入状态机 */
M2Link m2_link(const M2Receiver *receiver, uint32_t nowMs);   /* 判定链路状态 */

#ifdef __cplusplus   /* C++ 编译时按 C 方式链接 */
}
#endif   /* 条件编译段结束 */
#endif   /* M2_PROTOCOL_H 结束 */
