/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    vbat.c
  Author:      汪贤云
  Version:     1.0
  Date:        2026-09-22
  Description: 母线电压采集实现。D153C 驱动板将母线电压经 10k+1k 分压（1/11）后
               由 H2 第 5 脚引出，接至 PA4（ADC1_IN4）。
               换算：VM = raw / 4095 x 3.3V x 11；量程上限 36.3V、分辨率约 8.9mV。
               采集采用软件触发 + 轮询 EOC 的方式，不用 DMA 与中断（8 次转换不足 200us）。
               本文件同时提供 ADC1 通用单通道读取接口，供电流采集复用。
  Function List:
    1. Vbat_Init()          初始化 PA4 与 ADC1
    2. Vbat_GetVolt()        取母线电压(V)
    3. Vbat_GetRaw()         取原始 ADC 值
    4. Vbat_IsValid()        查询数据是否可信
    5. ADC1_ReadChannel()    ADC1 通用单通道读取
    6. ADC1_LastReadOk()     查询最近一次转换结果
  History:
    <author>   <time>      <version>   <desc>
    汪贤云     2026-10-16    1.0         创建文件并按华为规范补充注释
***********************************************************/

#include "vbat.h"
#include "Systick.h"  /* Delay_ms() 需要 */

#define ADC1_CONVERT_TIMEOUT    100000UL  /* EOC 轮询次数上限，约几十毫秒量级 */
/* 分压电路接好时，读数应落在 4~16V 对应码值的合理区间内。 */
#define VBAT_PRESENT_MIN_RAW    450U
#define VBAT_PRESENT_MAX_RAW    1900U

static uint8_t s_adc_last_ok = 0U;  /* 最近一次 ADC 转换是否成功 */
static uint8_t s_vbat_present = 0U;  /* 分压电路是否在位 */
static uint8_t s_vbat_valid = 0U;  /* 母线电压读数是否可信 */

static void Vbat_ConfigPin(uint32_t mode, uint32_t pupd);  /* 配置 PA4 的工作模式与上下拉 */
static uint8_t Vbat_CheckPresent(void);  /* 检测分压电路是否接上 */

/* ==========================================================================
 * 母线电压采集的实现
 * --------------------------------------------------------------------------
 * 就三件事：把 PA4 配成模拟输入 -> 打开 ADC1 -> 软件触发读一次再乘 11。
 * 没有用 DMA、没有用中断：主循环 200ms 才刷一次屏，8 次转换加起来
 * 不到 200us，占空比可忽略，没必要把简单问题复杂化。
 * ========================================================================== */

/*-----------------------------------------------------------
  Function:      Vbat_Init
  Description:   初始化母线电压采集：打开 GPIOA 与 ADC1 时钟，把 PA4 配置为模拟输入，
                 并将 ADC1 设为 12 位、单次转换、480 周期采样时间后使能。
  Calls:         RCC_AHB1PeriphClockCmd()、RCC_APB2PeriphClockCmd()、Vbat_ConfigPin()、ADC_CommonInit()、ADC_Init()、ADC_Cmd()
  Called By:     main.c 上电初始化（只调用一次）
  Input:         无
  Output:        无
  Return:        无
  Others:        1) PA4 必须关闭内部上下拉：内部上拉约 40k 会与 D153C 的 1k 分压电阻并联，
                                     使读数整体偏高；
                                  2) ADC 时钟 = APB2/4 = 21MHz，低于手册上限 36MHz；
                                  3) 采样时间取最慢档，因分压源阻抗约 0.9k，需要充分充电。
-----------------------------------------------------------*/

void Vbat_Init(void)
{
    ADC_CommonInitTypeDef ADC_CommonInitStructure = {0};  /* ADC 公共配置，必须清零 */
    ADC_InitTypeDef       ADC_InitStructure       = {0};  /* ADC1 私有配置，必须清零 */

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  /* 开 GPIOA 时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,  ENABLE);  /* 开 ADC1 时钟（ADC 挂在 APB2） */

    /* PA4 = ADC1_IN4。
     * 模拟输入必须显式关掉上下拉：内部上拉约 40k，会和 D153C 的
     * 1k 分压电阻并联，把读数整体拉高一大截。 */
    Vbat_ConfigPin(GPIO_Mode_AN, GPIO_PuPd_NOPULL);

    /* ADC 时钟 = APB2(84MHz) / 4 = 21MHz，低于数据手册上限 36MHz */
    ADC_CommonInitStructure.ADC_Mode             = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler        = ADC_Prescaler_Div4; /* ADC 时钟 4 分频 -> 21MHz，低于手册上限 36MHz */
    ADC_CommonInitStructure.ADC_DMAAccessMode    = ADC_DMAAccessMode_Disabled;  /* 单通道采集，不用 DMA */
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;  /* 双重采样间隔，本工程用不到 */
    ADC_CommonInit(&ADC_CommonInitStructure);  /* 写入 ADC 公共寄存器 */

    ADC_InitStructure.ADC_Resolution           = ADC_Resolution_12b;  /* 12 位分辨率（0~4095） */
    ADC_InitStructure.ADC_ScanConvMode         = DISABLE;   /* 只采一个通道 */
    ADC_InitStructure.ADC_ContinuousConvMode   = DISABLE;   /* 要的时候才采一次 */
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_ExternalTrigConv     = ADC_ExternalTrigConv_T1_CC1; /* 外部触发源（边沿触发已关闭，实际走软件触发） */
    ADC_InitStructure.ADC_DataAlign            = ADC_DataAlign_Right;  /* 结果右对齐 */
    ADC_InitStructure.ADC_NbrOfConversion      = 1;  /* 规则组只排 1 个通道 */
    ADC_Init(ADC1, &ADC_InitStructure);  /* 写入 ADC1 配置 */

    /* 读取端对每一次单次转换都要轮询 EOC，这里把「每个规则通道都产生 EOC」
     * 这个约定显式写出来，即便转换序列长度只有 1。 */
    ADC_EOCOnEachRegularChannelCmd(ADC1, ENABLE);

    /* 采样时间取最慢的 480 周期：分压源阻抗有 10k||1k ≈ 0.9k，
     * 采样电容充得慢，时间给不够读数会系统性偏低。 */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_480Cycles);

    ADC_Cmd(ADC1, ENABLE);  /* 使能 ADC1 */

    /* 上电后 ADC 要 ~1us 稳定，这里多等 10ms 保险，
     * 免得第一次读回来的是个假值。 */
    Delay_ms(10U);

    s_vbat_present = Vbat_CheckPresent();  /* 上电先探一次分压电路在不在 */
    s_vbat_valid = 0U;  /* 有效性等真正读到合理值再置 1 */
}

/*-----------------------------------------------------------
  Function:      Vbat_ConfigPin
  Description:   按传入的模式与上下拉配置 PA4 引脚。
  Calls:         GPIO_Init()
  Called By:     Vbat_Init()、Vbat_CheckPresent()
  Input:         mode —— GPIO 工作模式；pupd —— 内部上下拉设置
  Output:        无
  Return:        无
  Others:        抽成独立函数是为了让「模拟输入」与「推挽输出放电」两种用法复用同一段代码。
-----------------------------------------------------------*/

static void Vbat_ConfigPin(uint32_t mode, uint32_t pupd)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};  /* GPIO 配置结构体 */

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;  /* 操作 PA4 */
    GPIO_InitStructure.GPIO_Mode = (GPIOMode_TypeDef)mode;  /* 工作模式由调用者指定 */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  /* 推挽（仅在输出模式有效） */
    GPIO_InitStructure.GPIO_PuPd = (GPIOPuPd_TypeDef)pupd;  /* 上下拉由调用者指定 */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  /* 中速档 */
    GPIO_Init(GPIOA, &GPIO_InitStructure);  /* 写入 GPIOA 配置 */
}

/*-----------------------------------------------------------
  Function:      Vbat_CheckPresent
  Description:   检测分压电路是否真正接上：先将 PA4 拉低放电（悬空线放电后仍接近 0V），
                 再切回模拟输入读取电压，判断其是否落在 4~16V 对应的码值区间内。
  Calls:         Vbat_ConfigPin()、GPIO_ResetBits()、Delay_ms()、ADC1_ReadChannel()、ADC1_LastReadOk()
  Called By:     Vbat_Init()、Vbat_GetVolt()
  Input:         无
  Output:        无
  Return:        1 = 检测到分压电路；0 = 未接线或读数异常
  Others:        用于区分「没接线」与「电压确实为 0」，避免把悬空引脚的漂移值当成电压。
-----------------------------------------------------------*/

static uint8_t Vbat_CheckPresent(void)
{
    uint16_t raw;  /* 本次读回的原始码 */

    /* 先把 PA4 节点放电。悬空的引线在这一步之后仍接近 0V，
     * 而 D153C 的分压网络会重新充到母线分压值。 */
    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    Vbat_ConfigPin(GPIO_Mode_OUT, GPIO_PuPd_NOPULL); /* 切成普通输出才能主动拉低放电 */
    GPIO_ResetBits(GPIOA, GPIO_Pin_4);  /* 转成输出后再拉低一次，确保放干净 */
    Delay_ms(2U);  /* 放电等待 */

    Vbat_ConfigPin(GPIO_Mode_AN, GPIO_PuPd_NOPULL);  /* 切回模拟输入 */
    Delay_ms(2U);  /* 等分压网络重新充电 */
    raw = ADC1_ReadChannel(ADC_Channel_4);  /* 读一次 PA4 */

    if (ADC1_LastReadOk() == 0U)  /* 转换超时 */
    {
        return 0U;  /* 读不到就判为未接线 */
    }

    return ((raw >= VBAT_PRESENT_MIN_RAW) &&  /* 码值落在合理区间才算接上 */
            (raw <= VBAT_PRESENT_MAX_RAW)) ? 1U : 0U;  /* 区间之外一律判为未接线 */
}

/*-----------------------------------------------------------
  Function:      ADC1_ReadChannel
  Description:   ADC1 通用单通道读取：切换通道、软件触发一次转换、轮询等待结束并取回结果。
  Calls:         ADC_RegularChannelConfig()、ADC_SoftwareStartConv()、ADC_GetFlagStatus()、ADC_GetConversionValue()
  Called By:     Vbat_GetRaw()、Vbat_CheckPresent()、current.c 的电流采样
  Input:         ch —— ADC 通道号（母线电压用 ADC_Channel_4，电流用 ADC_Channel_5）
  Output:        无
  Return:        转换结果（0~4095）；超时返回 0
  Others:        1) 每次转换前先清 SR，避免上一次遗留的状态影响本次判断；
                                  2) 转换期间保持 ADC 使能，不要在转换过程中关闭外设；
                                  3) 返回 0 不代表失败，必须再用 ADC1_LastReadOk() 确认。
-----------------------------------------------------------*/

uint16_t ADC1_ReadChannel(uint8_t ch)
{
    uint16_t raw;  /* 转换结果 */
    uint32_t timeout = ADC1_CONVERT_TIMEOUT;  /* 超时计数初值 */

    /* PA4/PA5 共用同一个已使能的 ADC1。两次单次转换之间保持外设使能：
     * 在本型号上，转换进行中关闭 ADC 会留下 STRT 置位却没有新 EOC 的状态。 */

    /* 切换下一个通道之前，先清掉规则组所有已锁存的状态位；
     * 这同时会清掉「上次转换被中断或复位」留下的 STRT 残留。 */
    ADC1->SR = 0U;
    ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles); /* 规则组第 1 个位置放本通道，采样时间 480 周期 */
    ADC_SoftwareStartConv(ADC1);  /* 软件触发一次转换 */

    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)  /* 轮询等待转换结束 */
    {
        if (timeout == 0U)  /* 等超时了 */
        {
            s_adc_last_ok = 0U;  /* 标记本次失败 */
            return 0U;  /* 返回 0，调用方须查 ADC1_LastReadOk() 确认 */
        }
        timeout--;  /* 递减超时计数 */
    }

    s_adc_last_ok = 1U;  /* 转换成功 */
    raw = ADC_GetConversionValue(ADC1);   /* 读 DR 会清掉 EOC */

    /* 单次转换期间若出现 OVR（数据被覆盖），把它清掉 */
    if (ADC_GetFlagStatus(ADC1, ADC_FLAG_OVR) != RESET)
    {
        ADC_ClearFlag(ADC1, ADC_FLAG_OVR);  /* 清掉 OVR 标志 */
    }

    return raw;  /* 返回 12 位结果 */
}

/*-----------------------------------------------------------
  Function:      ADC1_LastReadOk
  Description:   查询最近一次 ADC1 转换是否在超时前完成。
  Calls:         无
  Called By:     Vbat_GetVolt()、Vbat_CheckPresent()、current.c
  Input:         无
  Output:        无
  Return:        1 = 转换成功；0 = 超时未完成
  Others:        无
-----------------------------------------------------------*/

uint8_t ADC1_LastReadOk(void)
{
    return s_adc_last_ok;  /* 返回最近一次转换状态 */
}

/*-----------------------------------------------------------
  Function:      Vbat_GetRaw
  Description:   读取母线电压通道的原始 ADC 值（未乘分压比）。
  Calls:         ADC1_ReadChannel()
  Called By:     标定与诊断
  Input:         无
  Output:        无
  Return:        原始 ADC 值（0~4095）
  Others:        无
-----------------------------------------------------------*/

uint16_t Vbat_GetRaw(void)
{
    return ADC1_ReadChannel(ADC_Channel_4);  /* 读 PA4（母线电压通道） */
}

/*-----------------------------------------------------------
  Function:      Vbat_GetVolt
  Description:   取母线电压：先确认分压电路在位，再做 8 次采样取平均并按分压比换算为伏特。
  Calls:         Vbat_CheckPresent()、Vbat_GetRaw()、ADC1_LastReadOk()
  Called By:     main.c 主循环、telemetry.c 组帧
  Input:         无
  Output:        无
  Return:        母线电压(V)；未接分压或结果超出 4~16V 时返回 0.0f
  Others:        换算公式：VM = 平均码 / 4095 x 3.3V x 11；任一环节失败即置无效标志并返回 0.0f。
-----------------------------------------------------------*/

float Vbat_GetVolt(void)
{
    uint32_t sum = 0U;  /* 8 次采样累加和 */
    uint8_t i;  /* 循环下标 */
    float volt;  /* 换算后的电压 */

    s_vbat_present = Vbat_CheckPresent();  /* 每次取值前都确认分压电路在位 */
    if (s_vbat_present == 0U)  /* 没接线 */
    {
        s_vbat_valid = 0U;  /* 置为无效 */
        return 0.0f;  /* 返回 0，屏上显示 -- */
    }

    for (i = 0U; i < VBAT_ADC_AVG; i++)  /* 采 8 次取平均，压掉随机噪声 */
    {
        sum += Vbat_GetRaw();  /* 累加本次原始码 */
        if (ADC1_LastReadOk() == 0U)  /* 任一次转换失败 */
        {
            s_vbat_valid = 0U;  /* 整体置为无效 */
            return 0.0f;  /* 不用半截数据出结果 */
        }
        /* 采样窗口本身已足够让这个低阻分压网络稳定，无需额外延时 */
    }

    volt = ((float)sum / (float)VBAT_ADC_AVG) / VBAT_ADC_FULL  /* 平均码 -> 电压：除满量程，再乘 3.3V 与分压比 11 */
           * VBAT_VREF * VBAT_DIV_RATIO;

    if ((volt < VBAT_VALID_MIN) || (volt > VBAT_VALID_MAX))  /* 越界即判为不可信 */
    {
        s_vbat_valid = 0U;  /* 越界：置无效 */
        return 0.0f;  /* 返回 0 */
    }

    s_vbat_valid = 1U;  /* 一切正常：置有效 */
    return volt;  /* 返回电压值（V） */
}

/*-----------------------------------------------------------
  Function:      Vbat_IsValid
  Description:   查询母线电压数据当前是否可信。
  Calls:         无
  Called By:     main.c（决定显示数值还是 --）、telemetry.c（决定是否置有效位）
  Input:         无
  Output:        无
  Return:        1 = 可信；0 = 缺失或超出范围
  Others:        无
-----------------------------------------------------------*/

uint8_t Vbat_IsValid(void)
{
    return s_vbat_valid;  /* 返回有效性标志 */
}
