/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    wave_dump.c
  Author:      汪贤云
  Version:     1.0
  Date:        2026-10-19
  Description: 20 kHz 电流波形导出通道的实现（USART1 + DMA2 Stream7）。
               帧格式与设计理由见 wave_dump.h 文件头。
  Function List:
    1. WaveDump_Init()        配置 PA9 / USART1 / DMA2 Stream7
    2. WaveDump_Poll()        按 100 ms 分频导出一帧 200 点波形
    3. WaveDump_SetEnabled() / WaveDump_IsEnabled() 开关与查询
    4. WaveDump_GetSent() / WaveDump_GetDropped() 统计
  History:
    <author>   <time>      <version>   <desc>
    汪贤云     2026-10-19    1.0         创建文件，按华为规范编写注释
***********************************************************/

#include "wave_dump.h"
#include "m2_protocol.h"    /* 复用 m2_crc()，避免再造一份 CRC 实现 */

/* ---- 帧内偏移（与 wave_dump.h 的帧格式表一一对应） ---- */
#define WAVE_DUMP_OFS_SYNC0      0U     /* 帧头第 1 字节 */
#define WAVE_DUMP_OFS_SYNC1      1U     /* 帧头第 2 字节 */
#define WAVE_DUMP_OFS_VERSION    2U     /* 协议版本 */
#define WAVE_DUMP_OFS_POINTS     3U     /* 点数，2 字节 */
#define WAVE_DUMP_OFS_SEQ        5U     /* 帧序号，4 字节 */
#define WAVE_DUMP_OFS_STATUS     9U     /* fastcap 状态位 */
#define WAVE_DUMP_OFS_DC        10U     /* 直流分量，2 字节 */
#define WAVE_DUMP_OFS_MIN       12U     /* 帧内最小值，2 字节 */
#define WAVE_DUMP_OFS_MAX       14U     /* 帧内最大值，2 字节 */
#define WAVE_DUMP_OFS_DATA      16U     /* 200 点数据起点 */
#define WAVE_DUMP_OFS_CRC      416U     /* CRC 起点 */
#define WAVE_DUMP_CRC_SPAN     414U     /* CRC 覆盖长度 = 416 - 2 */

#define WAVE_DUMP_VERSION       0x01U   /* 当前帧版本 */
#define WAVE_DUMP_SYNC0         0xA5U   /* 帧头第 1 字节 */
#define WAVE_DUMP_SYNC1         0x5AU   /* 帧头第 2 字节 */

/* ---- 硬件资源 ---- */
#define WAVE_DUMP_UART          USART1              /* 板载 CH340C 接 PA9 / PA10 */
#define WAVE_DUMP_DMA_STREAM    DMA2_Stream7        /* USART1_TX 固定映射到 DMA2 Stream7 */
#define WAVE_DUMP_DMA_CHANNEL   DMA_Channel_4       /* 与上一条对应，不可改 */
#define WAVE_DUMP_DMA_FLAG_TC   DMA_FLAG_TCIF7      /* 传输完成标志 */

/* ---- 模块状态 ---- */
static uint8_t  g_frame[WAVE_DUMP_FRAME_SIZE];  /* 待发送的整帧，DMA 的内存源 */
static uint32_t g_seq;                          /* 帧序号，每发一帧加 1 */
static uint32_t g_sent;                         /* 成功启动发送的帧数 */
static uint32_t g_dropped;                      /* 因 DMA 忙而丢弃的帧数 */
static uint8_t  g_decimCount;                   /* 分频计数器 */
static uint8_t  g_enabled;                      /* 1 = 允许导出 */
static uint8_t  g_inited;                       /* 1 = 已完成初始化 */

/* ---- 文件私有函数原型 ---- */
static void cfg_gpio(void);                     /* 配置 PA9 为 USART1_TX */
static void cfg_uart(void);                     /* 配置 USART1 参数 */
static void cfg_dma(void);                      /* 配置 DMA2 Stream7 搬运 */
static void put_u16(uint8_t *dst, uint16_t value);  /* 小端写 2 字节 */
static void put_u32(uint8_t *dst, uint32_t value);  /* 小端写 4 字节 */
static void build_frame(const FastcapSample *sample);  /* 组装一帧 */
static void start_send(void);                   /* 启动一次 DMA 发送 */

/*-----------------------------------------------------------
  Function:      cfg_gpio
  Description:   把 PA9 配置为复用推挽输出，复用功能 AF7 = USART1。
                 PA10（RX）不配置：本通道只发不收，少配一根线少一个干扰源。
  Calls:         RCC_AHB1PeriphClockCmd / GPIO_Init / GPIO_PinAFConfig
  Called By:     WaveDump_Init
  Input:         无
  Output:        无
  Return:        无
  Others:        PA9 / PA10 在旧版工程里一直空着，不存在引脚冲突。
-----------------------------------------------------------*/
static void cfg_gpio(void)
{
    GPIO_InitTypeDef gpio = {0};   /* 必须清零，避免栈上垃圾写进 CR 寄存器 */

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);   /* 打开 GPIOA 时钟 */

    gpio.GPIO_Pin   = GPIO_Pin_9;               /* 只用 PA9 = USART1_TX */
    gpio.GPIO_Mode  = GPIO_Mode_AF;             /* 复用模式 */
    gpio.GPIO_OType = GPIO_OType_PP;            /* 推挽输出 */
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;             /* 上拉，空闲时保持高电平 */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;         /* 115200 用 50MHz 绰绰有余 */
    GPIO_Init(GPIOA, &gpio);                    /* 写入寄存器 */

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);   /* PA9 复用为 USART1 */
}

/*-----------------------------------------------------------
  Function:      cfg_uart
  Description:   配置 USART1 为 8 位数据、1 位停止、无校验、只发模式，
                并打开 DMA 发送请求。波特率取 WAVE_DUMP_BAUD。
  Calls:         RCC_APB2PeriphClockCmd / USART_Init / USART_DMACmd / USART_Cmd
  Called By:     WaveDump_Init
  Input:         无
  Output:        无
  Return:        无
  Others:        115200 下 418 字节约 36 ms，小于 100 ms 的导出周期，链路有余量。
-----------------------------------------------------------*/
static void cfg_uart(void)
{
    USART_InitTypeDef uart = {0};   /* 清零，未赋值字段保持 0 */

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);   /* USART1 挂在 APB2（84MHz） */

    uart.USART_BaudRate            = WAVE_DUMP_BAUD;                        /* 115200 */
    uart.USART_WordLength          = USART_WordLength_8b;                   /* 8 位数据 */
    uart.USART_StopBits            = USART_StopBits_1;                      /* 1 位停止 */
    uart.USART_Parity              = USART_Parity_No;                       /* 无校验 */
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;        /* 无硬件流控 */
    uart.USART_Mode                = USART_Mode_Tx;                         /* 只发不收 */
    USART_Init(WAVE_DUMP_UART, &uart);                                      /* 写入寄存器 */

    USART_DMACmd(WAVE_DUMP_UART, USART_DMAReq_Tx, ENABLE);   /* 允许 TX 走 DMA */
    USART_Cmd(WAVE_DUMP_UART, ENABLE);                       /* 使能串口 */
}

/*-----------------------------------------------------------
  Function:      cfg_dma
  Description:   配置 DMA2 Stream7 / Channel4，把 g_frame 搬到 USART1->DR。
                普通模式（非循环），每帧搬运 WAVE_DUMP_FRAME_SIZE 字节。
  Calls:         RCC_AHB1PeriphClockCmd / DMA_Init
  Called By:     WaveDump_Init
  Input:         无
  Output:        无
  Return:        无
  Others:        DMA2 Stream2/Channel1 已被 fastcap 占用，本模块用的是 Stream7，
                两者是不同的流，可以同时工作。
-----------------------------------------------------------*/
static void cfg_dma(void)
{
    DMA_InitTypeDef dma = {0};   /* 清零，避免未赋值字段留下垃圾 */

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);   /* 打开 DMA2 时钟 */

    dma.DMA_Channel            = WAVE_DUMP_DMA_CHANNEL;                         /* 通道 4 */
    dma.DMA_PeripheralBaseAddr = (uint32_t)(uintptr_t)&(WAVE_DUMP_UART->DR);    /* 外设端 = USART1 数据寄存器 */
    dma.DMA_Memory0BaseAddr    = (uint32_t)(uintptr_t)g_frame;                  /* 内存端 = 帧缓冲 */
    dma.DMA_DIR                = DMA_DIR_MemoryToPeripheral;                    /* 内存到外设 */
    dma.DMA_BufferSize         = WAVE_DUMP_FRAME_SIZE;                          /* 一次搬一整帧 */
    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;                     /* 外设地址固定 */
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;                          /* 内存地址递增 */
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;                   /* 外设按字节 */
    dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;                       /* 内存按字节 */
    dma.DMA_Mode               = DMA_Mode_Normal;                               /* 普通模式，发完就停 */
    dma.DMA_Priority           = DMA_Priority_Medium;                           /* 中等优先级，不抢 ADC */
    dma.DMA_FIFOMode           = DMA_FIFOMode_Disable;                          /* 不用 FIFO */
    dma.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;                    /* FIFO 关闭时该值无影响 */
    dma.DMA_MemoryBurst        = DMA_MemoryBurst_Single;                        /* 单次传输 */
    dma.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;                    /* 单次传输 */
    DMA_Init(WAVE_DUMP_DMA_STREAM, &dma);                                       /* 写入寄存器 */
}

/*-----------------------------------------------------------
  Function:      put_u16
  Description:   以小端顺序把 16 位整数写进字节缓冲。
  Calls:         无
  Called By:     build_frame
  Input:         dst：目标地址；value：待写入的值
  Output:        无
  Return:        无
  Others:        上位机按小端解析，两边必须一致。
-----------------------------------------------------------*/
static void put_u16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);          /* 低字节在前 */
    dst[1] = (uint8_t)((value >> 8) & 0xFFU);   /* 高字节在后 */
}

/*-----------------------------------------------------------
  Function:      put_u32
  Description:   以小端顺序把 32 位整数写进字节缓冲。
  Calls:         无
  Called By:     build_frame
  Input:         dst：目标地址；value：待写入的值
  Output:        无
  Return:        无
  Others:        无
-----------------------------------------------------------*/
static void put_u32(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);          /* 第 1 字节 */
    dst[1] = (uint8_t)((value >> 8) & 0xFFU);   /* 第 2 字节 */
    dst[2] = (uint8_t)((value >> 16) & 0xFFU);  /* 第 3 字节 */
    dst[3] = (uint8_t)((value >> 24) & 0xFFU);  /* 第 4 字节 */
}

/*-----------------------------------------------------------
  Function:      build_frame
  Description:   按 wave_dump.h 的帧格式把一帧数据组装进 g_frame。
  Calls:         put_u16 / put_u32 / m2_crc / fastcap_get_last_block
  Called By:     WaveDump_Poll
  Input:         sample：本帧的统计结果（直流、极值、状态位）
  Output:        g_frame 被整帧填满
  Return:        无
  Others:        sample 为 NULL 时按全零填写，不在这里做空指针保护，
                 因为上层 WaveDump_Poll 已经判过。
-----------------------------------------------------------*/
static void build_frame(const FastcapSample *sample)
{
    const uint16_t *block;   /* 最近一帧 200 点原始样本 */
    uint32_t i;              /* 样本索引 */
    uint16_t crc;            /* 计算出的 CRC */

    block = fastcap_get_last_block();   /* 取软件快照，长度固定 200 点 */

    g_frame[WAVE_DUMP_OFS_SYNC0]   = WAVE_DUMP_SYNC0;                       /* 帧头第 1 字节 */
    g_frame[WAVE_DUMP_OFS_SYNC1]   = WAVE_DUMP_SYNC1;                       /* 帧头第 2 字节 */
    g_frame[WAVE_DUMP_OFS_VERSION] = WAVE_DUMP_VERSION;                     /* 版本号 */
    put_u16(&g_frame[WAVE_DUMP_OFS_POINTS], WAVE_DUMP_POINTS);              /* 点数 = 200 */
    put_u32(&g_frame[WAVE_DUMP_OFS_SEQ], g_seq);                            /* 帧序号 */
    g_frame[WAVE_DUMP_OFS_STATUS] = sample->status;                         /* fastcap 状态位 */
    put_u16(&g_frame[WAVE_DUMP_OFS_DC], sample->dc_raw);                    /* 直流分量 */
    put_u16(&g_frame[WAVE_DUMP_OFS_MIN], sample->raw_min);                  /* 帧内最小值 */
    put_u16(&g_frame[WAVE_DUMP_OFS_MAX], sample->raw_max);                  /* 帧内最大值 */

    for (i = 0U; i < WAVE_DUMP_POINTS; i++)   /* 逐点写 200 个原始码 */
    {
        put_u16(&g_frame[WAVE_DUMP_OFS_DATA + (2U * i)], block[i]);         /* 每点 2 字节 */
    }

    crc = m2_crc(&g_frame[2], WAVE_DUMP_CRC_SPAN);                          /* 覆盖版本到最后一个点 */
    put_u16(&g_frame[WAVE_DUMP_OFS_CRC], crc);                              /* 写 CRC */

    g_seq++;   /* 序号递增，供上位机发现丢帧 */
}

/*-----------------------------------------------------------
  Function:      start_send
  Description:   启动一次 DMA 发送。若上一帧还没发完则直接丢弃本帧。
  Calls:         DMA_GetCmdStatus / DMA_Cmd / DMA_ClearFlag / DMA_SetCurrDataCounter
  Called By:     WaveDump_Poll
  Input:         无
  Output:        无
  Return:        无
  Others:        每 100 ms 只有 36 ms 在发送，正常情况下不会出现忙状态；
                 一旦出现，说明波特率被改低了或者主循环时序变了，count 会暴露它。
-----------------------------------------------------------*/
static void start_send(void)
{
    if (DMA_GetCmdStatus(WAVE_DUMP_DMA_STREAM) != DISABLE)   /* 上一帧还在搬运 */
    {
        g_dropped++;   /* 如实计数，不覆盖正在发送的缓冲 */
        return;        /* 本帧放弃 */
    }

    DMA_ClearFlag(WAVE_DUMP_DMA_STREAM, WAVE_DUMP_DMA_FLAG_TC);   /* 清掉上一轮的完成标志 */
    DMA_SetCurrDataCounter(WAVE_DUMP_DMA_STREAM, WAVE_DUMP_FRAME_SIZE);   /* 重装传输长度 */
    DMA_Cmd(WAVE_DUMP_DMA_STREAM, ENABLE);   /* 启动本次搬运 */

    g_sent++;   /* 成功启动一次，计数加一 */
}

/*-----------------------------------------------------------
  Function:      WaveDump_Init
  Description:   初始化波形导出通道：引脚、串口、DMA，并复位统计与开关。
  Calls:         cfg_gpio / cfg_uart / cfg_dma
  Called By:     main（上电初始化阶段，位于 fastcap_init() 之后）
  Input:         无
  Output:        无
  Return:        无
  Others:        必须在 fastcap_init() 之后调用：本模块的 build_frame() 依赖
                 fastcap_get_last_block() 返回有效缓冲。重复调用是幂等的。
-----------------------------------------------------------*/
void WaveDump_Init(void)
{
    if (g_inited != 0U)   /* 已经初始化过 */
    {
        return;           /* 直接返回，避免重复配置串口打乱正在发送的帧 */
    }

    cfg_gpio();   /* PA9 -> USART1_TX */
    cfg_uart();   /* 115200 8N1，只发 */
    cfg_dma();    /* DMA2 Stream7 搬运 */

    g_seq        = 0UL;   /* 帧序号从 0 开始 */
    g_sent       = 0UL;   /* 已发送清零 */
    g_dropped    = 0UL;   /* 丢弃计数清零 */
    g_decimCount = 0U;    /* 分频计数清零 */
    g_enabled    = 1U;    /* 上电默认允许导出 */
    g_inited     = 1U;    /* 标记初始化完成 */
}

/*-----------------------------------------------------------
  Function:      WaveDump_Poll
  Description:   主循环调用。每收到一帧新采样就累加分频计数，
                满 WAVE_DUMP_DECIM 次（即 100 ms）才组装并发送一帧。
  Calls:         build_frame / start_send
  Called By:     main（fastcap_poll() 返回 1 时调用）
  Input:         dutyPercent：当前占空比，0 表示停转；sample：本帧统计结果
  Output:        无
  Return:        无
  Others:        1) 占空比为 0 时不导出：停转时电流恒为零，波形没有分析价值；
                 2) sample 为 NULL 时直接返回，不进入组装流程。
-----------------------------------------------------------*/
void WaveDump_Poll(uint8_t dutyPercent, const FastcapSample *sample)
{
    if ((g_inited == 0U) || (g_enabled == 0U))   /* 未初始化或已被关闭 */
    {
        return;                                   /* 不发 */
    }
    if (dutyPercent == 0U)   /* 电机停转 */
    {
        g_decimCount = 0U;   /* 分频计数归零，下次启动从完整周期开始 */
        return;              /* 停转波形不导出 */
    }
    if (sample == NULL)      /* 上层没给统计结果 */
    {
        return;              /* 不发 */
    }

    if (g_decimCount < (WAVE_DUMP_DECIM - 1U))   /* 还没攒够一个导出周期 */
    {
        g_decimCount++;   /* 计数加一，等下一帧 */
        return;           /* 本帧不导出 */
    }
    g_decimCount = 0U;   /* 攒满，重新开始计数 */

    build_frame(sample);   /* 组装整帧 */
    start_send();          /* 交给 DMA 发送 */
}

/*-----------------------------------------------------------
  Function:      WaveDump_SetEnabled
  Description:   打开或关闭波形导出。
  Calls:         无
  Called By:     调试流程
  Input:         enabled：0 = 关闭，非 0 = 打开
  Output:        无
  Return:        无
  Others:        关闭后不再组装新帧，但已经交给 DMA 的那一帧仍会发完。
-----------------------------------------------------------*/
void WaveDump_SetEnabled(uint8_t enabled)
{
    g_enabled = (enabled != 0U) ? 1U : 0U;   /* 归一化成 0 / 1 */
    g_decimCount = 0U;                        /* 切换时重置分频相位 */
}

/*-----------------------------------------------------------
  Function:      WaveDump_IsEnabled
  Description:   查询当前是否允许导出。
  Calls:         无
  Called By:     调试流程
  Input:         无
  Output:        无
  Return:        1 = 允许导出，0 = 已关闭
  Others:        无
-----------------------------------------------------------*/
uint8_t WaveDump_IsEnabled(void)
{
    return g_enabled;   /* 返回开关状态 */
}

/*-----------------------------------------------------------
  Function:      WaveDump_GetSent
  Description:   取已成功启动发送的帧数。
  Calls:         无
  Called By:     调试流程
  Input:         无
  Output:        无
  Return:        累计帧数
  Others:        与上位机收到的帧数比对，可以判断有没有丢包。
-----------------------------------------------------------*/
uint32_t WaveDump_GetSent(void)
{
    return g_sent;   /* 返回累计发送帧数 */
}

/*-----------------------------------------------------------
  Function:      WaveDump_GetDropped
  Description:   取因 DMA 忙而被丢弃的帧数。
  Calls:         无
  Called By:     调试流程
  Input:         无
  Output:        无
  Return:        累计丢弃帧数
  Others:        正常应为 0；不为 0 说明发送周期与波特率不匹配。
-----------------------------------------------------------*/
uint32_t WaveDump_GetDropped(void)
{
    return g_dropped;   /* 返回累计丢弃帧数 */
}
