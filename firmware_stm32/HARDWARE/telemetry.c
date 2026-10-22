/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    telemetry.c
  Author:      汪贤云
  Version:     1.1
  Date:        2026-09-27
  Description: 遥测发送模块：每 10ms 把测量值打包成 42 字节二进制帧，
               经 USART3(PB10, AF7, 57600 8N1) 发给 D1 mini。
               为不阻塞主循环，发送采用 TXE 中断逐字节搬运：
               主循环只负责填好发送缓冲并打开中断。
               本通道只发送、不接收，不会驱动 D1 侧的引脚。
               2026-10-19：振动传感器已移除，value[1..3] 三路加速度不再写入，
               对应 validMask 位恒为 0，D1 侧按「无效」显示 null（帧格式不变）。
  Function List:
    1. telemetry_init()    初始化 USART3 发送
    2. telemetry_poll()    主循环组帧并启动发送
    3. USART3_IRQHandler() 发送中断服务函数
  History:
    <author>   <time>      <version>   <desc>
    汪贤云     2026-10-16    1.0         创建文件并按华为规范补充注释
    汪贤云     2026-10-19    1.1         停写三路加速度，删除 accel_raw_to_mg()
***********************************************************/
/* 2026-10-07：原理图上 PB10 对应 P9 第 30 脚 -> D1 mini 的 D6；57600 8N1 这一对已实测通过。 */
#include "telemetry.h"
#include "m2_protocol.h"
#include "ai_monitor.h"  /* 只取 AI 阶段与报警标志，不喂数据 */
#include "stm32f4xx_usart.h"  /* USART3 寄存器与库函数 */

static uint8_t g_txBuffer[M2_FRAME_SIZE];  /* 发送缓冲（一整帧 42 字节） */
static volatile uint16_t g_txLength;  /* 本帧待发字节数；0 = 空闲 */
static volatile uint16_t g_txIndex;  /* 已发出的字节位置 */
static uint32_t g_lastMs;  /* 上次组帧的时刻（10ms 节流用） */
static uint32_t g_seq;  /* 帧序号，每发一帧 +1 */
static volatile uint32_t g_skipped;  /* 因上一帧未发完而跳过的次数 */

#define TELEMETRY_PERIOD_MS 10U  /* 组帧周期：10ms 一帧 */

/*-----------------------------------------------------------
  Function:      telemetry_init
  Description:   初始化遥测串口：把 PB10 配置为 USART3_TX（复用 AF7），
                 波特率 57600、8 位数据、无校验、1 位停止位，仅开启发送方向。
  Calls:         GPIO_Init()、GPIO_PinAFConfig()、USART_Init()、NVIC_SetPriority()、USART_Cmd()
  Called By:     main.c 初始化部分（只调用一次）
  Input:         无
  Output:        无
  Return:        无
  Others:        1) USART3 中断优先级设为最低(15)，发送不得挤占采样与 AI 计算；
                                  2) TXE 中断先关闭，等有整帧数据时再由 telemetry_poll 打开；
                                  3) 8N1 参数必须与 D1 侧一致，否则收到的是乱码。
-----------------------------------------------------------*/
/* PB10 的复用功能是 AF7。库函数按实际 APB1 时钟算波特率，与定时器时钟无关。 */
void telemetry_init(void)
{
    GPIO_InitTypeDef gpio;  /* GPIO 配置结构体 */
    USART_InitTypeDef uart;  /* 串口配置结构体 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);  /* 开 GPIOB 时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);  /* 开 USART3 时钟（挂 APB1） */
    GPIO_StructInit(&gpio);  /* 先填默认值 */
    gpio.GPIO_Pin = GPIO_Pin_10;  /* 用 PB10 作 TX */
    gpio.GPIO_Mode = GPIO_Mode_AF;  /* 复用模式 */
    gpio.GPIO_Speed = GPIO_Speed_2MHz;  /* 低速档即可（57600 波特） */
    gpio.GPIO_OType = GPIO_OType_PP;  /* 推挽输出 */
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;  /* 不用内部上下拉 */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);  /* PB10 复用为 USART3 */
    GPIO_Init(GPIOB, &gpio);  /* 写入 GPIOB 配置 */
    USART_StructInit(&uart);  /* 先填默认值（8 位、无校验、1 停止位） */
    uart.USART_BaudRate = 57600U;  /* 波特率必须与 D1 侧一致 */
    uart.USART_Mode = USART_Mode_Tx;  /* 只开发送方向 */
    USART_Init(USART3, &uart);  /* 写入 USART3 配置 */
    USART_ITConfig(USART3, USART_IT_TXE, DISABLE);  /* 先关 TXE 中断，等有整帧再开 */
    g_txLength = 0U;  /* 发送缓冲空闲 */
    g_txIndex = 0U;  /* 发送位置归零 */
    g_lastMs = 0U;  /* 节流计时归零 */
    g_seq = 0U;  /* 帧序号从 0 开始 */
    g_skipped = 0U;  /* 跳过计数清零 */
    /* 取本实现支持的最低 NVIC 优先级，与现有优先级分组无关 */
    NVIC_SetPriority(USART3_IRQn, 15U);
    NVIC_ClearPendingIRQ(USART3_IRQn); /* 先清掉可能残留的挂起标志，避免一开中断就立刻进一次 */
    NVIC_EnableIRQ(USART3_IRQn);  /* 使能 USART3 中断 */
    USART_Cmd(USART3, ENABLE);  /* 启动 USART3 */
}

/*-----------------------------------------------------------
  Function:      telemetry_poll
  Description:   主循环每 10ms 调用一次：按 10ms 节流组帧，把 UI 快照中的测量值
                 填充进 M2Frame，编码后交给 TXE 中断发送。
  Calls:         m2_scale()、m2_encode()、ai_monitor_get_phase()、USART_ITConfig()
  Called By:     main.c 主循环
  Input:         data —— 只读的 UI 数据快照；nowMs —— 当前毫秒
  Output:        无
  Return:        无
  Others:        1) 上一帧未发完时直接跳过并计入 skipped，绝不等待；
                                  2) 占空比直接读 TIM9 的 CCR1+CCR2，保证与引脚实际输出一致；
                                  3) 本帧只做上报，不再喂给板上 AI；AI 走 feature_r 的 32 点窗；
                                  4) 发布发送缓冲前用两道 __DMB() 保证中断不会读到半新半旧的缓冲。
-----------------------------------------------------------*/
void telemetry_poll(const UI_Data *data, uint32_t nowMs)
{
    M2Frame frame = {0};  /* 本帧数据，先清零 */
    uint32_t duty;  /* 实际占空比（从 TIM9 读回） */
    size_t length;  /* 编码后的帧长 */
    if (data == 0 || (uint32_t)(nowMs - g_lastMs) < TELEMETRY_PERIOD_MS)  /* 入参为空，或未到 10ms 节拍 */
    {
        return;  /* 本轮不发 */
    }
    g_lastMs = nowMs;  /* 记下本次组帧时刻 */
    if (g_txLength != 0U)  /* 上一帧还没发完 */
    {
        ++g_skipped;  /* 计入跳过次数 */
        return;  /* 绝不阻塞等待 */
    }
    /* 现有 PWM 是 ARR=99 且只驱动一路通道；界面 L4 的占空比只是历史显示值 */
    duty = TIM9->CCR1 + TIM9->CCR2;
    if (TIM9->ARR != 99U || duty > 88U || data->level > 4U)
    {
        ++g_skipped;  /* 参数异常：跳过本帧 */
        return;  /* 直接返回 */
    }
    frame.seq = g_seq;  /* 帧序号 */
    frame.uptimeMs = nowMs;  /* 发送时刻（开机毫秒） */
    frame.level = data->level;  /* 档位 */
    frame.duty = (uint8_t)duty;  /* 实际输出占空比 */
    if (data->current_valid != 0U &&  /* 电流有效才写入 */
        m2_scale(data->current_a, 1000U, &frame.value[0]))  /* A -> mA（含溢出保护） */
    {
        frame.validMask |= M2_VALID_CURRENT;  /* 置电流有效位 */
    }
    if (data->speed_valid != 0U &&  /* 转速有效才写入 */
        m2_scale(data->speed_rpm, 10U, &frame.value[4]))  /* 转速 ×10 后存入 */
    {
        frame.validMask |= M2_VALID_SPEED;  /* 置转速有效位 */
    }
    if (data->vbat_valid != 0U &&  /* 电压有效才写入 */
        m2_scale(data->vbat_v, 1000U, &frame.value[5]))  /* 电压换算成毫伏后存入 */
    {
        frame.validMask |= M2_VALID_BATTERY;  /* 置电压有效位 */
    }
    /* 2026-10-19：板上 AI 改由 main.c 直接驱动 feature_r 的 R 窗口，
     * 不再从遥测帧取数，所以这里只保留「阶段 + 报警」的读取。 */
    /* 2026-10-15：把 AI 阶段与事件级报警写进保留的第 15 字节，
     * 手机端就能显示「学习中 / 检测中 / 报警」。 */
    frame.aiState = (uint8_t)ai_monitor_get_phase() & M2_AI_PHASE_MASK;
    if (ai_monitor_is_alarm() != 0U)
    {
        frame.aiState |= M2_AI_ALARM_BIT;  /* 置报警标志位 */
    }
    length = m2_encode(&frame, g_txBuffer, sizeof(g_txBuffer));  /* 编码成一整帧 */
    if (length == 0U)  /* 编码失败（容量不足等） */
    {
        ++g_skipped;  /* 计入跳过 */
        return;  /* 直接返回 */
    }
    ++g_seq;  /* 序号自增（给下一帧用） */
    g_txIndex = 0U;  /* 从第 0 字节开始发 */
    __DMB();
    g_txLength = (uint16_t)length;  /* 发布本帧长度（此后中断开始搬字节） */
    __DMB();
    USART_ITConfig(USART3, USART_IT_TXE, ENABLE);  /* 打开 TXE 中断，正式开始发送 */
}

/*-----------------------------------------------------------
  Function:      USART3_IRQHandler
  Description:   USART3 发送中断服务函数。每次把一个字节写入数据寄存器，
                 整帧发完后关闭 TXE 中断并清空发送长度标志。
  Calls:         USART_GetITStatus()、USART_SendData()、USART_ITConfig()
  Called By:     硬件在发送寄存器空时自动进入
  Input:         无
  Output:        无
  Return:        无
  Others:        1) 中断内只做搬运，禁止 printf、延时、读传感器等耗时操作；
                                  2) 必须先关中断再清长度标志，否则可能漏掉一次中断状态。
-----------------------------------------------------------*/
void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_TXE) != RESET) {  /* TXE：发送寄存器已空 */
        if (g_txIndex < g_txLength) {  /* 还有字节没发完 */
            USART_SendData(USART3, (uint16_t)(uint8_t)g_txBuffer[g_txIndex]);  /* 写入下一个字节 */
            ++g_txIndex;  /* 发送位置前移 */
        }
        if (g_txIndex >= g_txLength) {  /* 整帧发完 */
            USART_ITConfig(USART3, USART_IT_TXE, DISABLE);  /* 先关 TXE 中断 */
            __DMB();
            g_txLength = 0U;  /* 再清长度，标记发送缓冲空闲 */
        }
    }
}
