/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    encoder.c
  Author:      汪贤云
  Version:     1.0
  Date:        2026-09-22
  Description: 编码器测速实现。使用 TIM4 编码器接口模式对 PB6/PB7 的正交脉冲硬件计数，
               不占用 CPU；每 100ms 在 1ms 中断内读取计数差并换算为转速。
               机械参数：编码器 13 线、四倍频、减速比 20，即每转 1040 个计数。
               换算：rpm = 计数差 x 60000 / (1040 x 100) = 计数差 x 0.576923。
  Function List:
    1. Encoder_Init()      配置引脚与 TIM4 编码器模式
    2. Encoder_Tick1ms()   1ms 节拍，满 100ms 采样一次
    3. Encoder_GetRpm()    取转速
    4. Encoder_GetDelta()  取计数差
    5. Encoder_GetCount()  取计数器绝对值
    6. Encoder_GetMs()     取运行毫秒数
  History:
    <author>   <time>      <version>   <desc>
    汪贤云     2026-10-16    1.0         创建文件并按华为规范补充注释
***********************************************************/

#include "encoder.h"

/* ==========================================================================
 * 第 4 步 · 编码器测速的实现
 * --------------------------------------------------------------------------
 * 两个函数分工：
 *      Encoder_Init()      --- 配置引脚 + TIM4 编码器模式（只在上电时跑一次）
 *      Encoder_Tick1ms()   --- TIM7 每 1ms 调一次，攒够 100ms 就读一次计数器
 *
 * 为什么采样要放在中断里做？
 *   因为主循环还要刷 OLED，一次刷屏要几十毫秒。如果在主循环里"到点读一次"，
 *   实际读取时刻会跟着刷屏前后漂移，100ms 的窗口变成 100~160ms，
 *   算出来的转速就会忽高忽低。放进 1ms 中断里读，周期误差只有几十微秒，
 *   转速立刻就稳了。读一次计数器就是读个寄存器，纳秒级，不影响中断实时性。
 * ========================================================================== */

static volatile int16_t  s_delta    = 0;        /* 最近 100ms 的计数差 */
static volatile float    s_rpm      = 0.0f;     /* 最近算出的转速 */
static volatile uint32_t s_ms       = 0U;       /* 1ms 计数（等于上电毫秒数） */

static int16_t           s_last_cnt = 0;        /* 上一次采样时的计数器值 */

/* -------------------------------------------------------------------------
 * 初始化：PB6 / PB7 复用成 TIM4_CH1 / CH2，TIM4 进编码器模式
 * ------------------------------------------------------------------------- */
/*-----------------------------------------------------------
  Function:      Encoder_Init
  Description:   初始化编码器测速：把 PB6/PB7 复用为 TIM4_CH1/CH2，并将 TIM4 配置为
                 编码器接口模式（TI1 与 TI2 双边沿计数，即四倍频）。
  Calls:         RCC_APB1PeriphClockCmd()、GPIO_PinAFConfig()、GPIO_Init()、TIM_TimeBaseInit()、TIM_EncoderInterfaceConfig()、TIM_ICInit()
  Called By:     main.c 上电初始化（只调用一次）
  Input:         无
  Output:        无
  Return:        无
  Others:        1) PB6/PB7 使用内部上拉，因编码器多为集电极开路输出；
                                  2) 输入滤波器设为 10 个采样周期，用于滤除换向尖峰；
                                  3) 时基 PSC=0、ARR=65535，让 16 位计数器自由计数。
-----------------------------------------------------------*/

void Encoder_Init(void)
{
    GPIO_InitTypeDef        GPIO_InitStructure;    /* GPIO 引脚配置结构体 */
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;    /* 时基配置结构体 */
    TIM_ICInitTypeDef       TIM_ICInitStructure;    /* 输入捕获配置结构体 */

    /* --- 1. 开时钟：TIM4 挂在 APB1 上，PB 口挂在 AHB1 上 --- */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); /* 使能 GPIOB 时钟（编码器 A/B 接在 PB6 / PB7） */

    /* --- 2. 把 PB6 / PB7 的复用功能切到 TIM4 --- */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4); /* PB6 切到 TIM4（编码器 A 相） */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4); /* PB7 切到 TIM4（编码器 B 相） */

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;  /* 一次配 PB6 和 PB7 */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;       /* 复用模式 */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; /* 输出速度档（配成复用输入后不敏感） */
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;       /* 编码器多为集电极开路，需要上拉 */
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* --- 3. 时基：预分频 0（每个边沿都算数），周期给满 65535 让 16 位计数器自由跑 --- */
    TIM_TimeBaseStructure.TIM_Prescaler     = 0;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up; /* 编码器模式下方向由 A/B 相位决定，此项仅占位 */
    TIM_TimeBaseStructure.TIM_Period        = 65535;        /* 周期给满，让 16 位计数器自由跑 */
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  /* 时钟分割 DIV1（不做数字滤波） */
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);  /* 写入 TIM4 时基 */

    /* --- 4. 关键一句：配成编码器接口，TI1 + TI2 都当计数源，两相都按上升沿 -> 四倍频 --- */
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); /* 续行：TI1 / TI2 都按上升沿 -> 两相都计数 -> 四倍频 */

    /* --- 5. 输入滤波：电机换向时线上有尖峰毛刺，滤掉 10 个采样周期再认 --- */
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; /* 先配通道 1 */
    TIM_ICInitStructure.TIM_ICFilter = 10;  /* 输入滤波：连续 10 个采样周期一致才认 */
    TIM_ICInit(TIM4, &TIM_ICInitStructure);  /* 写入 TIM4 通道 1 */

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;  /* 切换成通道 2（滤波参数沿用） */
    TIM_ICInit(TIM4, &TIM_ICInitStructure);  /* 写入 TIM4 通道 2 */

    /* --- 6. 计数器清零后启动 --- */
    TIM_SetCounter(TIM4, 0);
    s_last_cnt = 0; /* 上次计数一并清零，防止首帧算出离谱转速 */
    TIM_Cmd(TIM4, ENABLE);  /* 启动 TIM4，开始硬件计数 */
}

/* -------------------------------------------------------------------------
 * 读一次计数器，算出这一段的转速（只在 1ms 中断里被调用）
 * ------------------------------------------------------------------------- */
/*-----------------------------------------------------------
  Function:      Encoder_Sample
  Description:   读取一次 TIM4 计数器，计算与上次采样的差值并换算为转速。
  Calls:         TIM_GetCounter()
  Called By:     Encoder_Tick1ms()（每 100ms 调用一次）
  Input:         无
  Output:        更新静态变量 s_delta、s_rpm 与 s_last_cnt
  Return:        无
  Others:        两个 int16 相减可正确处理计数器 65535 到 0 的回绕；
-----------------------------------------------------------*/

static void Encoder_Sample(void)
{
    int16_t now   = (int16_t)TIM_GetCounter(TIM4);   /* 读当前计数（截成有符号便于求差） */
    /* 两个 int16 相减：即使计数器从 65535 回绕到 0，结果依然正确 */
    int16_t delta = (int16_t)(now - s_last_cnt);
    float   rpm; /* 本次换算出的转速 */

    s_last_cnt = now;    /* 更新「上次计数」 */
    s_delta    = delta;    /* 发布本次计数差 */

    rpm = (float)delta * ENC_RPM_PER_CNT;    /* 乘换算系数得到 rpm */
    if (rpm < 0.0f)    /* 反转为负 */
    {
        rpm = -rpm;                 /* 先只关心转多快，方向等做闭环控制时再用 */
    }
    s_rpm = rpm; /* 发布给 UI 与遥测读取 */
}

/* -------------------------------------------------------------------------
 * TIM7 的中断服务函数里每 1ms 调一次这里
 * ------------------------------------------------------------------------- */
/*-----------------------------------------------------------
  Function:      Encoder_Tick1ms
  Description:   1ms 定时节拍：累加运行毫秒数，每满 100ms 采样一次编码器。
  Calls:         Encoder_Sample()
  Called By:     stm32f4xx_it.c 中的 TIM7_IRQHandler（每 1ms 一次）
  Input:         无
  Output:        无
  Return:        无
  Others:        放在中断中采样可避免主循环刷屏造成的周期漂移，保证 100ms 窗口准确。
-----------------------------------------------------------*/

void Encoder_Tick1ms(void)
{
    s_ms++;    /* 毫秒计数加一 */

    if ((s_ms % ENC_SAMPLE_MS) == 0U)      /* 攒够 100ms */
    {
        Encoder_Sample();                  /* 立刻采样，周期最准 */
    }
}

/*-----------------------------------------------------------
  Function:      Encoder_GetRpm
  Description:   取最近一次计算出的转速。
  Calls:         无
  Called By:     main.c 主循环、telemetry.c 组帧
  Input:         无
  Output:        无
  Return:        转速，单位 rpm，恒为非负值
  Others:        无
-----------------------------------------------------------*/

float Encoder_GetRpm(void)
{
    return s_rpm;    /* 返回最近一次转速 */
}

/*-----------------------------------------------------------
  Function:      Encoder_GetDelta
  Description:   取最近 100ms 内的编码器计数差（带符号，可反映转向）。
  Calls:         无
  Called By:     main.c 主循环
  Input:         无
  Output:        无
  Return:        最近 100ms 的计数差
  Others:        无
-----------------------------------------------------------*/

int16_t Encoder_GetDelta(void)
{
    return s_delta;    /* 返回最近 100ms 计数差 */
}

/* 取 TIM4 计数器的当前值 = 输出轴的"绝对位置"（16 位，转满 65536 回绕）。
 * 用途：① 标定"输出轴转一整圈 = 1040 个计数"；② 以后做位置 / 闭环控制。
 * 注意它和 Encoder_GetDelta() 不是一回事：Delta 是最近 100ms 的增量，
 * Count 是开机以来的累计位置（会一直加/减，只在 65536 处回绕）。 */
/*-----------------------------------------------------------
  Function:      Encoder_GetCount
  Description:   取 TIM4 计数器的当前值，即输出轴的绝对位置。
  Calls:         无
  Called By:     标定与后续位置闭环控制
  Input:         无
  Output:        无
  Return:        计数器当前值（16 位，转满 65536 回绕）
  Others:        与 Encoder_GetDelta() 不同：本函数返回累计位置，不是增量。
-----------------------------------------------------------*/

int16_t Encoder_GetCount(void)
{
    return (int16_t)TIM_GetCounter(TIM4);    /* 直接读 TIM4 计数器 */
}

/*-----------------------------------------------------------
  Function:      Encoder_GetMs
  Description:   取上电以来的毫秒数。
  Calls:         无
  Called By:     需要运行时间的场合
  Input:         无
  Output:        无
  Return:        毫秒数
  Others:        无
-----------------------------------------------------------*/

uint32_t Encoder_GetMs(void)
{
    return s_ms;    /* 返回上电毫秒数 */
}
