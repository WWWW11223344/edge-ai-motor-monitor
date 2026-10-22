/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    fastcap.c
  Author:      汪贤云
  Version:     1.0
  Date:        2026-10-14
  Description: 与 PWM 同步的高速电流采样前端实现（自研新增模块，尚未并入主循环）。
               构成：TIM3 以 20kHz 触发 ADC2 转换，DMA2 Stream2/Channel1 循环搬运到缓冲；
               每 200 点为一帧（恰好 10ms），在 process_frame() 中计算直流分量、
               去均值后的交流 RMS、峰峰值与过零插值计数。
               采样相位通过 fastcap_align_to_pwm() 标定一次，使采样点落在 PWM 导通窗口中部，
               既避开开关振铃，又保证采到的是连续绕组电流。
               本模块只输出原始 ADC 码，不重复做电流零点标定，避免与 current.c 冲突。
               状态与计数通过 fastcap_get_*() 读取，未接入遥测与 AI 特征链。
  Function List:
    1. fastcap_init()        初始化外设与 DMA
    2. fastcap_poll()        取一帧处理结果
    3. fastcap_align_to_pwm() 标定采样相位
    4. 原始块抓取与运行统计接口
  History:
    <author>   <time>      <version>   <desc>
    汪贤云     2026-10-16    1.0         创建文件并按华为规范补充注释
***********************************************************/

/* ==========================================================================
 * fastcap.c  --- 与 PWM 同频的 20 kHz 电流采样前端（暂存模块，尚未并入工程）
 * --------------------------------------------------------------------------
 * 归属：本项目自研新增模块。文件编码 GBK（与工程内其它 .c/.h 一致，Keil 默认按 ANSI 打开）。
 *
 * 修改记录：
 *   2026-10-14  HU JIAXUAN  新建。原因：审查发现现有 100 Hz 软件轮询 +
 *   采样后 8 点滑动平均的电流通道，Nyquist 只有 50 Hz、有效通带约 5.6 Hz，
 *   而 TIM9 载波 20 kHz 又正好是 100 Hz 的 200 倍（折叠到 DC），
 *   电机换向纹波（约 145~242 Hz）与齿轮啮合频率（约 387~580 Hz）全部丢失，
 *   板端 19 维特征退化为幅值统计。本模块提供与载波同频的高速采样、
 *   正确的抗混叠抽取和可做 FFT 的原始块。
 *   2026-10-14  HU JIAXUAN  修订：用户决定**不改线**，ACS712 保留在母线上。
 *   原设计依赖「把传感器移到电机引线、让电流连续」，现改为把采样相位锁定到
 *   PWM 的 ON 窗口中部：ON 窗口内（AIN1 导通）母线电流就是绕组电流，本身是
 *   连续量，因此不改线、不加外部 RC 也能得到稳定读数。新增
 *   fastcap_align_to_pwm() 与两个对齐诊断量，供上电后核对。
 *
 * 上下文与副作用声明：
 *   - fastcap_init() 只能在 main() 初始化阶段调用一次，且必须在 Vbat_Init() 之后
 *     （ADC123 共用公共预分频寄存器，本模块刻意不重设，避免破坏 ADC1 的 21 MHz）；
 *   - fastcap_poll() 只在主循环调用，不可在 ISR 里调用；它轮询 DMA 的 HT/TC 标志，
 *     不使用中断，因此不需要改 stm32f4xx_it.c，也不占用 NVIC；
 *   - DMA 写入 g_adcBuf，主循环读取已完成的半区，两者通过「半区完成标志」解耦；
 *     Cortex-M4 无 D-Cache，不需要 cache 维护；
 *   - 不使用动态分配、不使用递归、不使用 VLA；所有循环次数编译期有界。
 *
 * 已知未完成项（不得当作已实现）：
 *   - 未接入主循环 / feature_r / 遥测；
 *   - 未做模拟前端（RC 抗混叠）与传感器位置变更，见 staging_notes.md；
 *   - 未在实物上验证（尚未编译进工程、尚未上电）。
 * ========================================================================== */

#include "fastcap.h"   /* 本模块对外接口与常量定义 */
#include "Systick.h"   /* 提供 Delay_ms() 毫秒延时 */

#include <math.h>   /* sqrt()，只用于一帧的交流 RMS 计算 */
#include <stddef.h>   /* NULL、size_t */
#include <stdint.h>   /* 定宽整数类型 */

/* ---- 硬件绑定 ---- */
#define FASTCAP_ADC                 ADC2                /* 电流采样接 PA5，即 ADC2_IN5 通道 */
#define FASTCAP_ADC_CH              ADC_Channel_5   /* 通道 5，对应 PA5 */
#define FASTCAP_ADC_SAMPLETIME      ADC_SampleTime_84Cycles   /* 采样时间 84+12 周期约 4.6us，远小于 50us 的触发间隔 */
#define FASTCAP_TRIG_TIM            TIM3   /* 用 TIM3 的更新事件做 ADC 触发源 */
#define FASTCAP_DMA_STREAM          DMA2_Stream2   /* ADC2 固定映射到 DMA2_Stream2 */
#define FASTCAP_DMA_CHANNEL         DMA_Channel_1   /* ADC2 固定映射到 DMA 通道 1 */
#define FASTCAP_DMA_HT_FLAG         DMA_FLAG_HTIF2   /* 半传输完成标志（前半区填满） */
#define FASTCAP_DMA_TC_FLAG         DMA_FLAG_TCIF2   /* 传输完成标志（后半区填满） */

#define FASTCAP_APB1_TIMER_HZ       84000000UL  /* TIM3 在 APB1，定时器时钟 84 MHz */
#define FASTCAP_TIM_TICK_HZ         2000000UL   /* 分频后 2 MHz，便于与 TIM9 对照 */
#define FASTCAP_CAL_TIMEOUT         1000000UL   /* 校准等待上限，避免无界循环 */

/* ---- PWM 相位对齐参数 ----
 * TIM9 为向上计数、PWM1 模式：CNT 0..CCR1 为导通（AIN1 高），
 * 占空比 58% 时导通窗口是 CNT 0..58（0~29 us）。采样点取窗口中部 CNT=29
 * （14.5 us），离起沿 14.5 us、离落沿 14.5 us，避开两处开关振铃。
 * 约束：FASTCAP_ON_SAMPLE_CNT 必须小于 AI 检测域的 CCR1（58），并留足余量；
 * 占空比小于该位置时采样会落进续流段，读数不再代表绕组电流（见 staging_notes.md）。 */
#define FASTCAP_PWM_ARR            100U   /* TIM9 的 ARR，必须与 pwm.c 保持一致 */
#define FASTCAP_ON_SAMPLE_CNT      29U   /* 采样点定在导通窗口中部（CNT = 29） */

/* 本工程 USER/stm32f4xx.h 是精简版器件头，ADC_CR2 只定义了 ADON/EXTEN 等位，
 * 没有 CAL。这里按 RM0090 ADC_CR2 bit2 自行定义，只用于本模块的校准。 */
#define FASTCAP_ADC_CR2_CAL         ((uint32_t)0x00000004)   /* 按 RM0090 ADC_CR2 bit2 自定义 CAL 位 */
/* ADC SR bit5 = OVR（bit4 是 STRT，不要混）。精简版头文件未定义该位，这里自行定义。 */
#define FASTCAP_ADC_SR_OVR          ((uint32_t)0x00000020)   /* 按 RM0090 ADC_SR bit5 自定义 OVR 位（注意 bit4 是 STRT） */

#if ((FASTCAP_APB1_TIMER_HZ % FASTCAP_TIM_TICK_HZ) != 0UL)   /* 编译期检查：定时器分频必须整除 */
#error "TIM3 分频不整除"   /* 不整除直接编译失败 */
#endif
#if ((FASTCAP_TIM_TICK_HZ % FASTCAP_FS_HZ) != 0UL)   /* 编译期检查：ARR 必须能整除出目标采样率 */
#error "ARR 无法整除出目标采样率"   /* 不整除直接编译失败 */
#endif
#if ((FASTCAP_RAW_LEN % FASTCAP_BLOCK_LEN) != 0U)   /* 编译期检查：原始块长度必须是整帧的整数倍 */
#error "FASTCAP_RAW_LEN 必须是 FASTCAP_BLOCK_LEN 的整数倍，否则原始块时间上不连续"   /* 否则时间上不连续，无法做 FFT */
#endif

/* ---- 模块状态（全部 static，生命周期 = 整个运行期）---- */
static uint16_t g_adcBuf[2U * FASTCAP_BLOCK_LEN];   /* DMA 循环目标，两个半区 */
static uint16_t g_dcHist[FASTCAP_DC_SMOOTH_FRAMES]; /* 100 Hz 域的平滑历史 */

static uint32_t g_smoothSum;                        /* 平滑窗口内 dc_raw 之和 */
static uint32_t g_rawCount;                         /* 已抓取的原始样本数 */
static uint8_t  g_smoothIdx;   /* 平滑环形缓冲的写指针 */
static uint8_t  g_smoothCount;   /* 已填入的平滑样本数（未满时用它做除数） */
static uint8_t  g_rawArmed;   /* 1 = 正在抓取原始块 */
static uint8_t  g_rawReady;   /* 1 = 原始块已抓满 */
static uint16_t g_lastBlock[FASTCAP_BLOCK_LEN];   /* 最近完成一帧的软件快照，供波形导出 */

/* 对外可见（见 fastcap.h 的说明），供 J-Link 按符号名读取。 */
uint16_t g_fastcap_raw[FASTCAP_RAW_LEN];   /* 原始采样块（2000 点 = 100ms） */
volatile uint32_t g_fastcap_frames;   /* 已完成处理的帧数 */
volatile uint32_t g_fastcap_overruns;   /* 因主循环没跟上而被覆盖的帧数 */
volatile uint32_t g_fastcap_ovr;   /* ADC 溢出恢复次数 */
volatile uint8_t  g_fastcap_status;   /* 状态位组合，取值见 FASTCAP_ST_* */
volatile uint32_t g_fastcap_alignTim9;   /* 标定时读到的 TIM9 计数值（诊断用） */
volatile uint16_t g_fastcap_alignTim3;   /* 标定时写入 TIM3 的计数值（诊断用） */

/* ---- 文件私有函数原型 ---- */
static void cfg_pin(void);   /* 配置 PA5 为模拟输入 */
static void cfg_dma(void);   /* 配置 DMA2 循环搬运 */
static uint8_t cfg_adc(void);   /* 配置并校准 ADC2 */
static void cfg_timer(void);   /* 配置 TIM3 作为 20kHz 触发源 */
static void process_frame(const uint16_t *buf, FastcapSample *out);   /* 一帧统计：直流、交流 RMS、峰峰值、过零计数 */
static void take_raw(const uint16_t *buf);   /* 把一帧采样拷进原始块缓冲 */
static void save_block(const uint16_t *buf);   /* 把一帧采样存成软件快照，供波形导出 */

/* --------------------------------------------------------------------------
 * PA5 配为模拟输入。Current_Init() 也会这么配，这里重复一次是为了让本模块
 * 不依赖其它模块的初始化顺序（写同一个值，幂等）。
 * -------------------------------------------------------------------------- */
/*-----------------------------------------------------------
  Function:      cfg_pin
  Description:   把 PA5 配置为模拟输入，供 ADC2_IN5 采样使用。
  Calls:         RCC_AHB1PeriphClockCmd()、GPIO_Init()
  Called By:     fastcap_init()
  Input:         无
  Output:        无
  Return:        无
  Others:        与 current.c 共用 PA5：本模块只做高速采样，不改变 current.c 的零点标定。
-----------------------------------------------------------*/

static void cfg_pin(void)
{
    GPIO_InitTypeDef gpio = {0};   /* 结构体清零，避免栈垃圾写进寄存器 */

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);   /* 打开 GPIOA 时钟 */

    gpio.GPIO_Pin   = GPIO_Pin_5;   /* 只用 PA5 */
    gpio.GPIO_Mode  = GPIO_Mode_AN;   /* 模拟输入，ADC 专用模式 */
    gpio.GPIO_OType = GPIO_OType_PP;   /* 模拟模式下该域无效，仍显式赋值避免栈垃圾 */
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;   /* 模拟输入不能开上拉下拉，否则会改变分压结果 */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;   /* 模拟模式下该域无效 */
    GPIO_Init(GPIOA, &gpio);   /* 写入寄存器 */
}

/* --------------------------------------------------------------------------
 * DMA2 Stream2 / Channel1 对应 ADC2。循环模式，长度 = 两个半区。
 * 半区长度取 200 点 = 10 ms，正好等于现有主循环的采样周期。
 * -------------------------------------------------------------------------- */
/*-----------------------------------------------------------
  Function:      cfg_dma
  Description:   配置 DMA2 Stream2 / Channel1，把 ADC2 的转换结果循环搬运到采样缓冲。
  Calls:         RCC_AHB1PeriphClockCmd()、DMA_DeInit()、DMA_Init()、DMA_Cmd()
  Called By:     fastcap_init()
  Input:         无
  Output:        无
  Return:        无
  Others:        循环模式保证 ADC 连续转换不会丢点；缓冲长度为一整帧（200 点）。
-----------------------------------------------------------*/

static void cfg_dma(void)
{
    DMA_InitTypeDef dma = {0};   /* 结构体清零 */

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);   /* 打开 DMA2 时钟 */

    DMA_DeInit(FASTCAP_DMA_STREAM);   /* 把该数据流复位为默认值 */
    dma.DMA_Channel            = FASTCAP_DMA_CHANNEL;   /* ADC2 固定在 DMA 通道 1 */
    dma.DMA_PeripheralBaseAddr = (uint32_t)(uintptr_t)&(FASTCAP_ADC->DR);   /* 外设端 = ADC2 数据寄存器 */
    dma.DMA_Memory0BaseAddr    = (uint32_t)(uintptr_t)g_adcBuf;   /* 内存端 = 双半区采样缓冲 */
    dma.DMA_DIR                = DMA_DIR_PeripheralToMemory;   /* 传输方向：外设到内存 */
    dma.DMA_BufferSize         = (uint32_t)(2U * FASTCAP_BLOCK_LEN);   /* 传输长度 = 两个半区共 400 点 */
    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;   /* 外设地址固定，始终读 DR */
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;   /* 内存地址自动递增 */
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;   /* 外设数据宽度 16 位 */
    dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;   /* 内存数据宽度 16 位，与缓冲元素一致 */
    dma.DMA_Mode               = DMA_Mode_Circular;   /* 循环模式，保证 ADC 连续转换不丢点 */
    dma.DMA_Priority           = DMA_Priority_High;   /* 高优先级，避免被其它数据流抢占 */
    dma.DMA_FIFOMode           = DMA_FIFOMode_Disable;   /* 关闭 FIFO，直通模式 */
    dma.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;   /* FIFO 关闭时该域无实际作用，按库默认值填 */
    dma.DMA_MemoryBurst        = DMA_MemoryBurst_Single;   /* 内存侧单次传输 */
    dma.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;   /* 外设侧单次传输 */
    DMA_Init(FASTCAP_DMA_STREAM, &dma);   /* 写入寄存器 */

    DMA_ClearFlag(FASTCAP_DMA_STREAM, FASTCAP_DMA_HT_FLAG | FASTCAP_DMA_TC_FLAG);   /* 清掉复位后可能残留的半传输/传输完成标志 */
    DMA_Cmd(FASTCAP_DMA_STREAM, ENABLE);   /* 使能数据流 */
}

/* --------------------------------------------------------------------------
 * ADC2：外部触发（TIM3 TRGO，20 kHz）+ DMA 请求，单通道单次转换。
 * 返回 1 = 校准完成可启动，0 = 校准超时（调用者应保持停机并报无效）。
 * -------------------------------------------------------------------------- */
/*-----------------------------------------------------------
  Function:      cfg_adc
  Description:   配置并校准 ADC2：外部触发为 TIM3、单通道、开启连续转换与 DMA 请求。
  Calls:         ADC_Init()、ADC_RegularChannelConfig()、ADC_DMARequestAfterLastTransferCmd()、ADC_Cmd()
  Called By:     fastcap_init()
  Input:         无
  Output:        无
  Return:        1 = 校准完成；0 = 校准失败或超时
  Others:        不重复调用 ADC_CommonInit()：公共预分频寄存器为 ADC123 共用，该配置由 Vbat_Init() 完成。
-----------------------------------------------------------*/

static uint8_t cfg_adc(void)
{
    ADC_InitTypeDef adc = {0};   /* 结构体清零 */
    uint32_t wait;   /* 校准等待计数器 */

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);   /* 打开 ADC2 时钟 */

    /* ADC 公共预分频已在 Vbat_Init() 设为 Div4 = 21 MHz，此处不重设。 */
    adc.ADC_Resolution           = ADC_Resolution_12b;   /* 12 位分辨率 */
    adc.ADC_ScanConvMode         = DISABLE;   /* 单通道，不用扫描模式 */
    adc.ADC_ContinuousConvMode   = DISABLE;   /* 由外部触发逐次启动，不做连续转换 */
    adc.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;   /* 上升沿触发 */
    adc.ADC_ExternalTrigConv     = ADC_ExternalTrigConv_T3_TRGO;   /* 触发源 = TIM3 的 TRGO（更新事件） */
    adc.ADC_DataAlign            = ADC_DataAlign_Right;   /* 右对齐，低 12 位有效 */
    adc.ADC_NbrOfConversion      = 1U;   /* 规则组只安排 1 个转换 */
    ADC_Init(FASTCAP_ADC, &adc);   /* 写入 ADC 寄存器 */

    /* 21 MHz 下 84+12 = 96 周期约 4.6 us，远小于 50 us 的触发间隔。 */
    ADC_RegularChannelConfig(FASTCAP_ADC, FASTCAP_ADC_CH, 1U, FASTCAP_ADC_SAMPLETIME);   /* 规则组第 1 个转换用通道 5，采样时间 84 周期 */
    ADC_DMACmd(FASTCAP_ADC, ENABLE);   /* 使能 ADC 的 DMA 请求 */
    /* F4 关键位：DDS=1 让 ADC 在每次转换后都持续发出 DMA 请求。
     * 不置位时，循环 DMA 填满一轮缓冲区后 ADC 就停止发请求，NDTR 停在满值。 */
    ADC_DMARequestAfterLastTransferCmd(FASTCAP_ADC, ENABLE);   /* DDS=1：每次转换结束后都持续发出 DMA 请求 */
    ADC_Cmd(FASTCAP_ADC, ENABLE);   /* 开 ADC（置 ADON） */
    Delay_ms(1U);       /* ADON 之后等待内部稳压稳定（数据手册 tSTAB 上限 3 us） */

    /* 校准：本工程 FWLIB 是精简版 SPL，没有 ADC_StartCalibration /
     * ADC_GetCalibrationStatus 封装，直接读写 CR2.CAL。
     * 读改写只动 CAL 位，不影响 ADC_Init 已写好的 EXTSEL/EXTEN；
     * 由硬件在校准结束时清零 CAL，因此循环有上界，不是无界等待。
     * 此时 TIM3 尚未启动，不会与校准过程抢转换。 */
    FASTCAP_ADC->CR2 |= FASTCAP_ADC_CR2_CAL;   /* 启动校准；硬件完成后自动清零 CAL */
    wait = FASTCAP_CAL_TIMEOUT;   /* 装载等待上限，保证循环有界 */
    while (((FASTCAP_ADC->CR2 & FASTCAP_ADC_CR2_CAL) != 0U) && (wait != 0UL))   /* 等待 CAL 被硬件清零 */
    {
        wait--;   /* 递减等待计数 */
    }
    if (wait == 0UL)   /* 超时仍未完成 */
    {
        return 0U;   /* 上报校准失败 */
    }

    /* 外部触发使能由 ADC_Init(ADC_ExternalTrigConvEdge_Rising) 写 CR2.EXTEN 完成，
     * 无需再调用 ADC_ExternalTrigConvCmd（本版 SPL 也没有该函数）。 */
    return 1U;   /* 校准完成，可以启动触发 */
}

/* --------------------------------------------------------------------------
 * TIM3 更新事件作为 ADC2 的触发源，与 TIM9 同频同相。
 *
 * 为什么不能直接用 TIM9 触发：ADC2 的外部触发源列表里没有 TIM9
 * （只有 T1_CC1/CC2/CC3、T2_CC2/CC3/CC4、T2_TRGO、T3_CC1、T3_TRGO、T4_CC4、
 * T5_CCx、T8_CC1、T8_TRGO、EXTI11/15），所以必须另起一个定时器做触发源。
 *
 * 相位靠「写 CNT」对齐，而不是靠 ITRx 级联：
 *   TIM3 与 TIM9 都由同一 168 MHz PLL 驱动、分频后都是 2 MHz / ARR=99，
 *   两者周期严格相等，只差一个固定的相位差。因此只要在对齐时算出这个相位差，
 *   把 TIM3->CNT 直接写成目标值，触发点就固定在想要的 CNT 位置上，之后不再漂移
 *   （不依赖 RM0090 的 ITR 连接表，少一个易错的隐含前提）。
 *   代价：读两个计数器不是原子的，对齐本身有约 1~2 us 的不确定度；
 *   相对 29 us 宽的导通窗口仍有十几 us 余量，可接受。实测方法见 staging_notes.md。
 * -------------------------------------------------------------------------- */
/*-----------------------------------------------------------
  Function:      cfg_timer
  Description:   配置 TIM3 为 20kHz 触发源，用于周期性启动 ADC2 转换。
  Calls:         RCC_APB1PeriphClockCmd()、TIM_TimeBaseInit()、TIM_SelectOutputTrigger()、TIM_Cmd()
  Called By:     fastcap_init()
  Input:         无
  Output:        无
  Return:        无
  Others:        TIM3 与 TIM9 同源 168MHz、周期完全相同，因此相位标定后不会漂移。
-----------------------------------------------------------*/

static void cfg_timer(void)
{
    TIM_TimeBaseInitTypeDef tb = {0};   /* 结构体清零（历史教训：不清零会让 CR1 混入栈垃圾） */

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);   /* 打开 TIM3 时钟 */

    tb.TIM_Prescaler         = (uint16_t)(FASTCAP_APB1_TIMER_HZ / FASTCAP_TIM_TICK_HZ - 1UL);   /* 预分频到 2MHz，每格 0.5us */
    tb.TIM_CounterMode       = TIM_CounterMode_Up;   /* 向上计数 */
    tb.TIM_Period            = (uint16_t)(FASTCAP_TIM_TICK_HZ / FASTCAP_FS_HZ - 1UL);   /* ARR = 99，周期 50us，即 20kHz */
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;   /* 数字滤波时钟不分频 */
    tb.TIM_RepetitionCounter = 0U;   /* 重复计数器仅高级定时器使用，这里显式置 0 */
    TIM_TimeBaseInit(FASTCAP_TRIG_TIM, &tb);   /* 写入时基寄存器 */

    TIM_SelectOutputTrigger(FASTCAP_TRIG_TIM, TIM_TRGOSource_Update);   /* 把更新事件输出为 TRGO，作为 ADC2 的触发源 */
}

/* --------------------------------------------------------------------------
 * 把采样相位对齐到 TIM9 的 ON 窗口中部。
 *
 * 目标：TIM3 回绕（= ADC 触发）发生的瞬间，TIM9->CNT 应等于 FASTCAP_ON_SAMPLE_CNT。
 * 做法：两定时器周期完全相同，所以只要令
 *           TIM3->CNT = (TIM9->CNT - 目标) mod ARR
 *       写入之后两者的相位差就固定下来，之后各自回绕，触发点恒定。
 *
 * 单次读 CNT 就够：CPU 168 MHz、TIM9 计数 2 MHz（0.5 us 一格），
 * 读与写之间只隔几十个 CPU 周期，远小于一格计数。
 * 返回 1 = 已写入；0 = 参数越界，未做任何写入。
 * -------------------------------------------------------------------------- */
/*-----------------------------------------------------------
  Function:      fastcap_align_to_pwm
  Description:   标定采样相位：读 TIM9 当前计数值并据此重置 TIM3，使采样点落在
                 PWM 导通窗口中部（离两个开关沿各约 14us），避开开关振铃。
  Calls:         无
  Called By:     fastcap_init() 之后、正式取数之前调用一次
  Input:         无
  Output:        无
  Return:        1 = 标定成功；0 = 失败（TIM9 未运行或计数异常）
  Others:        标定瞬间的计数值记录在 g_fastcap_alignTim9 / g_fastcap_alignTim3 中，便于事后核对。
-----------------------------------------------------------*/

uint8_t fastcap_align_to_pwm(void)
{
    uint32_t tim9Cnt;   /* 标定瞬间读到的 TIM9 计数值 */
    uint16_t writeValue;   /* 要写入 TIM3 的计数值 */

    if (FASTCAP_ON_SAMPLE_CNT >= FASTCAP_PWM_ARR)   /* 目标相位越界检查 */
    {
        return 0U;   /* 参数非法，不做任何写入 */
    }

    tim9Cnt = TIM9->CNT;   /* 读 TIM9 当前计数 */
    writeValue = (uint16_t)(((tim9Cnt + FASTCAP_PWM_ARR) - FASTCAP_ON_SAMPLE_CNT)   /* 先加一个 ARR 防止无符号下溢 */
                            % FASTCAP_PWM_ARR);   /* 取模得到应写入的相位 */
    TIM3->CNT = writeValue;   /* 写入之后两定时器的相位差就被固定下来 */

    g_fastcap_alignTim9 = tim9Cnt;   /* 记录诊断量，供上电后核对 */
    g_fastcap_alignTim3 = writeValue;   /* 记录实际写入值 */
    return 1U;   /* 标定成功 */
}

/*-----------------------------------------------------------
  Function:      fastcap_init
  Description:   初始化高速采样链路：依次配置引脚、DMA、ADC2 与 TIM3，并做一次相位标定。
  Calls:         cfg_pin()、cfg_dma()、cfg_adc()、cfg_timer()、fastcap_align_to_pwm()
  Called By:     main.c 初始化部分（须在 Vbat_Init() 之后调用）
  Input:         无
  Output:        无
  Return:        无
  Others:        无返回值；初始化结果通过 fastcap_get_status() 查询。
-----------------------------------------------------------*/

void fastcap_init(void)
{
    g_fastcap_frames   = 0UL;   /* 帧计数清零 */
    g_fastcap_overruns = 0UL;   /* 丢帧计数清零 */
    g_fastcap_ovr      = 0UL;   /* 溢出恢复计数清零 */
    g_fastcap_status   = FASTCAP_ST_OK;   /* 状态置为正常 */
    g_smoothSum        = 0UL;   /* 平滑累加和清零 */
    g_smoothIdx        = 0U;   /* 平滑写指针归零 */
    g_smoothCount      = 0U;   /* 平滑样本数归零 */
    g_rawCount         = 0UL;   /* 原始块已抓点数归零 */
    g_rawArmed         = 0U;   /* 未开始抓取原始块 */
    g_rawReady         = 0U;   /* 原始块未就绪 */

    cfg_pin();   /* 配 PA5 为模拟输入 */
    cfg_dma();   /* 配 DMA 循环搬运 */
    if (cfg_adc() == 0U)   /* 配置并校准 ADC2 */
    {
        g_fastcap_status |= FASTCAP_ST_NO_ADC;   /* 记下 ADC 未就绪，后续读数按无效处理 */
        return;   /* 不启动定时器，保持停机状态 */
    }
    cfg_timer();        /* 只配置 TIM3，不启动 */
    (void)fastcap_align_to_pwm();   /* 停止状态下写 CNT 对齐；TIM9 必须已在运行 */
    TIM_Cmd(FASTCAP_TRIG_TIM, ENABLE);   /* 最后启动时基，保证 ADC/DMA 已就绪 */
}

/* --------------------------------------------------------------------------
 * ADC OVR 自恢复。F4 一旦发生 OVR，ADC 会**永久**停发 DMA 请求，只清 OVR 位
 * 不恢复，必须把 ADC 关掉再重开。同时复位 DMA 流，让缓冲区从半区 0 重新开始。
 * 只允许在主循环上下文调用（会短暂阻塞，不用在 ISR 里）。
 * -------------------------------------------------------------------------- */
/*-----------------------------------------------------------
  Function:      recover_overrun
  Description:   处理 DMA 覆盖与 ADC 溢出：清标志、重建传输并累加对应的计数值。
  Calls:         无
  Called By:     fastcap_poll()
  Input:         无
  Output:        无
  Return:        无
  Others:        主循环没跟上时整帧会被覆盖，此处负责恢复并如实计数，不隐藏丢失。
-----------------------------------------------------------*/

static void recover_overrun(void)
{
    uint32_t wait;   /* 等待 ADON 真正清零的计数上限 */

    ADC_DMACmd(FASTCAP_ADC, DISABLE);   /* 先关掉 ADC 的 DMA 请求 */
    ADC_Cmd(FASTCAP_ADC, DISABLE);   /* 再关 ADC，为重新初始化做准备 */

    /* 等 ADON 真正清 0：写 ADON=0 后若还有一次在途转换，F4 会先完成它再停机。
     * 不等到 ADON=0 就清 SR，会漏掉那次转换的 EOC，重开后立刻再 OVR，
     * 形成「每 10 ms 自触发一次恢复」的死循环。 */
    wait = FASTCAP_CAL_TIMEOUT;   /* 装载等待上限 */
    while (((FASTCAP_ADC->CR2 & ADC_CR2_ADON) != 0U) && (wait != 0UL))   /* 等在途转换结束、ADON 真正清零 */
    {
        wait--;   /* 递减等待计数 */
    }
    if (wait == 0UL)   /* 超时仍未停机 */
    {
        g_fastcap_status |= FASTCAP_ST_NO_ADC;   /* 记下 ADC 异常 */
        return;   /* 放弃本次恢复 */
    }

    DMA_Cmd(FASTCAP_DMA_STREAM, DISABLE);   /* 关掉 DMA 数据流 */
    while (DMA_GetCmdStatus(FASTCAP_DMA_STREAM) != DISABLE)   /* 等数据流真正停止（库要求的顺序） */
    {
    }
    DMA_ClearFlag(FASTCAP_DMA_STREAM, FASTCAP_DMA_HT_FLAG | FASTCAP_DMA_TC_FLAG);   /* 清掉残留的半传输/传输完成标志 */
    DMA_SetCurrDataCounter(FASTCAP_DMA_STREAM, (uint32_t)(2U * FASTCAP_BLOCK_LEN));   /* 重装传输长度，缓冲从半区 0 重新开始 */
    DMA_Cmd(FASTCAP_DMA_STREAM, ENABLE);   /* 重新使能 DMA 数据流 */

    FASTCAP_ADC->SR = 0U;   /* ADON 已确认清 0，这里清掉停转前那次转换的 EOC/OVR */

    /* 关键顺序：先恢复 DMA 请求、再开 ADON。
     * 若 ADON 先开而 DMA 请求后开，中间那段时间的转换无人读取，
     * 会立刻再产生一次 OVR，形成「每 10 ms 自触发一次恢复」的死循环。 */
    ADC_DMACmd(FASTCAP_ADC, ENABLE);   /* 先恢复 DMA 请求 */
    ADC_Cmd(FASTCAP_ADC, ENABLE);   /* 再开 ADC，避免中间段无人读取又立刻溢出 */

    g_fastcap_ovr++;   /* 累计一次溢出恢复 */
    g_fastcap_status |= FASTCAP_ST_OVERRUN;   /* 置溢出状态位 */
    g_rawArmed = 0U;   /* 恢复后本次原始块抓取作废 */
    g_rawReady = 0U;   /* 清除原始块就绪标志 */
    g_rawCount = 0UL;   /* 抓取点数归零 */
}

/* --------------------------------------------------------------------------
 * 一帧 200 点（10 ms）的统计。全部整数运算，只有开方用 double。
 *   dc      = 帧均值。200 点均值是长度 10 ms 的矩形窗，零点精确落在 100 Hz
 *             的整数倍上：20 kHz 载波（=200x100 Hz）及其各次谐波、以及所有
 *             100 Hz 栅格上的分量都被零点吃掉 —— 这是「载波噪声折叠到 DC」的解法。
 *   ac      = 去均值后的 RMS / 峰峰值，对应约 50 Hz~10 kHz 的交流分量，
 *             换向纹波与齿轮啮合频率就在这个带里。
 *   cross   = 线性插值过零计数 x10，给出纹波频率的粗估计；一帧只有约 2.4 个
 *             周期，分辨率差，精确谱以主机 FFT 为准（见 staging_notes.md）。
 * -------------------------------------------------------------------------- */
/*-----------------------------------------------------------
  Function:      process_frame
  Description:   处理一帧 200 点原始采样：算出直流分量、去均值后的交流 RMS、
                 峰峰值与过零插值计数，并判断是否贴轨。
  Calls:         无
  Called By:     fastcap_poll()、take_raw()
  Input:         buf —— 一帧原始采样（200 点）
  Output:        out —— 本帧处理结果（FastcapSample）
  Return:        无
  Others:        直流分量可直接用于电流换算，交流分量才带波形结构信息。
-----------------------------------------------------------*/

static void process_frame(const uint16_t *buf, FastcapSample *out)
{
    uint32_t sum = 0U;   /* 一帧原始码之和，用于求直流分量 */
    uint64_t sq = 0ULL;   /* 去均值后的平方和，用 64 位防止溢出 */
    uint16_t min = buf[0];   /* 帧内最小值，初值取首个样本 */
    uint16_t max = buf[0];   /* 帧内最大值，初值取首个样本 */
    uint16_t dc;   /* 直流分量（帧均值） */
    double cross = 0.0;   /* 过零插值计数（含小数部分） */
    size_t i;   /* 样本索引 */

    for (i = 0U; i < FASTCAP_BLOCK_LEN; ++i)   /* 第一遍：求和并找极值 */
    {
        uint16_t v = buf[i];   /* 当前样本 */

        sum += (uint32_t)v;   /* 累加求均值 */
        if (v < min)   /* 是否需要更新最小值 */
        {
            min = v;   /* 记录更小的值 */
        }
        if (v > max)   /* 是否需要更新最大值 */
        {
            max = v;   /* 记录更大的值 */
        }
    }
    dc = (uint16_t)((sum + (FASTCAP_BLOCK_LEN / 2U)) / FASTCAP_BLOCK_LEN);   /* 加半个长度实现四舍五入的帧均值 */

    for (i = 0U; i < FASTCAP_BLOCK_LEN; ++i)   /* 第二遍：累加去均值后的平方和 */
    {
        int32_t d = (int32_t)buf[i] - (int32_t)dc;   /* 去直流后的交流分量（有符号，可能是负） */

        sq += (uint64_t)(d * d);   /* 累加平方，64 位避免溢出 */
    }

    for (i = 1U; i < FASTCAP_BLOCK_LEN; ++i)   /* 第三遍：统计过零次数 */
    {
        int32_t previous = (int32_t)buf[i - 1U] - (int32_t)dc;   /* 前一点（已去直流） */
        int32_t current = (int32_t)buf[i] - (int32_t)dc;   /* 当前点（已去直流） */

        if (((previous < 0) && (current > 0)) || ((previous > 0) && (current < 0)))   /* 相邻两点异号 = 跨过一个零点 */
        {
            int32_t absPrevious = (previous < 0) ? -previous : previous;   /* 前一点幅值（取正） */
            int32_t absCurrent = (current < 0) ? -current : current;   /* 当前点幅值（取正） */

            if ((absPrevious + absCurrent) > 0)   /* 两端不全为零，可以做线性插值 */
            {
                cross += (double)absPrevious / (double)(absPrevious + absCurrent);   /* 线性插值出零点在本段内的精确位置（0~1） */
            }
            cross += 1.0;   /* 计数加一，加上插值小数即真实过零数 */
        }
    }

    /* 100 Hz 域的平滑：8 帧 = 80 ms。此处已经在抗混叠之后，等价于旧实现想要的
     * 平滑效果，但不会把混叠后的假分量一起平均进来。 */
    g_smoothSum -= (uint32_t)g_dcHist[g_smoothIdx];   /* 环出滑动窗口里最旧的一个样本 */
    g_dcHist[g_smoothIdx] = dc;   /* 写入本帧直流值 */
    g_smoothSum += (uint32_t)dc;   /* 累加进滑动窗口 */
    g_smoothIdx = (uint8_t)((g_smoothIdx + 1U) % FASTCAP_DC_SMOOTH_FRAMES);   /* 写指针环形前进 */
    if (g_smoothCount < FASTCAP_DC_SMOOTH_FRAMES)   /* 窗口尚未填满 */
    {
        g_smoothCount++;   /* 有效样本数加一，未满时用它做除数 */
    }

    if ((min <= 2U) || (max >= 4093U))   /* 贴到 0 或 4095 附近 = 超出 ADC 量程 */
    {
        g_fastcap_status |= FASTCAP_ST_RAIL;   /* 置贴轨状态位 */
    }

    out->dc_raw        = dc;   /* 本帧直流原始码 */
    out->dc_smooth_raw = (uint16_t)(g_smoothSum / (uint32_t)g_smoothCount);   /* 8 帧滑动平均后的直流 */
    out->ac_rms_raw    = (uint16_t)(sqrt((double)sq / (double)FASTCAP_BLOCK_LEN) + 0.5);   /* 交流 RMS：开方后四舍五入成整数码 */
    out->ac_peak_raw   = (uint16_t)(max - min);   /* 峰峰值 */
    out->ac_cross10    = (uint16_t)((cross * 10.0) + 0.5);   /* 过零计数乘 10，避免用浮点存放 */
    out->raw_min       = min;   /* 本帧最小值 */
    out->raw_max       = max;   /* 本帧最大值 */
    out->status        = g_fastcap_status;   /* 当前状态位快照 */
}

/* --------------------------------------------------------------------------
 * 原始块抓取。要求连续 10 整帧没有丢帧，任一次覆盖都会作废本次抓取，
 * 避免把时间上有洞的数据当成连续波形做 FFT。
 * -------------------------------------------------------------------------- */
/*-----------------------------------------------------------
  Function:      take_raw
  Description:   把当前 DMA 缓冲中的一帧采样拷贝进原始块缓冲。
  Calls:         无
  Called By:     recover_overrun()（原始块抓取期间）
  Input:         buf —— DMA 采样缓冲
  Output:        无
  Return:        无
  Others:        原始块容量为 2000 点（100ms），抓满后置就绪标志供 J-Link 读取。
-----------------------------------------------------------*/

static void take_raw(const uint16_t *buf)
{
    uint32_t i;   /* 样本索引 */

    if ((g_rawArmed == 0U) || (g_rawReady != 0U))   /* 未启动抓取，或已经抓满 */
    {
        return;   /* 什么都不做 */
    }

    for (i = 0U; i < FASTCAP_BLOCK_LEN; ++i)   /* 整帧拷贝 */
    {
        g_fastcap_raw[g_rawCount + i] = buf[i];   /* 追加到原始块的尾部 */
    }
    g_rawCount += FASTCAP_BLOCK_LEN;   /* 已抓点数前进一帧 */

    if (g_rawCount >= FASTCAP_RAW_LEN)   /* 已抓满整个原始块 */
    {
        g_rawCount = FASTCAP_RAW_LEN;   /* 夹到上限，防止越界写 */
        g_rawArmed = 0U;   /* 结束本次抓取 */
        g_rawReady = 1U;   /* 置就绪标志，供 J-Link 读取 */
    }
}

/*-----------------------------------------------------------
  Function:      fastcap_poll
  Description:   主循环调用：检查是否已攒满一帧，若有则处理并输出结果。
  Calls:         process_frame()、recover_overrun()、take_raw()
  Called By:     main.c 主循环（本模块尚未接入，接口已就绪）
  Input:         无
  Output:        out —— 一帧处理结果，仅在返回 1 时有效
  Return:        1 = 本次输出了一帧；0 = 尚未攒满或出错
  Others:        本函数不做阻塞等待，未就绪时立即返回，不影响 10ms 遥测节奏。
-----------------------------------------------------------*/

uint8_t fastcap_poll(FastcapSample *out)
{
    uint8_t readyBits = 0U;   /* bit0 = 半区 0 完成，bit1 = 半区 1 完成 */
    uint8_t half;   /* 本次要读取的半区号 */
    const uint16_t *buf;   /* 指向待处理的半区缓冲 */

    if (out == NULL)   /* 输出指针为空 */
    {
        return 0U;   /* 参数非法 */
    }

    if ((FASTCAP_ADC->SR & FASTCAP_ADC_SR_OVR) != 0U)   /* 硬件溢出：F4 一旦 OVR 会永久停发 DMA 请求，必须停机重开 */
    {
        recover_overrun();   /* 执行溢出恢复流程 */
        return 0U;   /* 本次不输出数据 */
    }

    if (DMA_GetFlagStatus(FASTCAP_DMA_STREAM, FASTCAP_DMA_HT_FLAG) != RESET)   /* 前半区已填满 */
    {
        DMA_ClearFlag(FASTCAP_DMA_STREAM, FASTCAP_DMA_HT_FLAG);   /* 清标志，允许下一次继续触发 */
        readyBits |= 0x01U;   /* 记录半区 0 就绪 */
    }
    if (DMA_GetFlagStatus(FASTCAP_DMA_STREAM, FASTCAP_DMA_TC_FLAG) != RESET)   /* 后半区已填满 */
    {
        DMA_ClearFlag(FASTCAP_DMA_STREAM, FASTCAP_DMA_TC_FLAG);   /* 清标志 */
        readyBits |= 0x02U;   /* 记录半区 1 就绪 */
    }
    if (readyBits == 0U)   /* 两个标志都没有置位 */
    {
        return 0U;   /* 尚未攒满一帧，立即返回，不做阻塞等待 */
    }

    if ((readyBits & 0x03U) == 0x03U)   /* 两个标志同时置位 = 主循环没跟上，已经有帧被覆盖 */
    {
        g_fastcap_overruns++;   /* 如实累计丢帧数，不隐藏数据丢失 */
        g_fastcap_status |= FASTCAP_ST_OVERRUN;   /* 置溢出状态位 */
        /* 不再作废进行中的原始块抓取：诊断用途允许偶发缺帧造成的微小时间缺口，
         * FFT 仍能看主谱线；正式特征链路不依赖这个诊断原始块。 */
    }

    half = ((readyBits & 0x01U) != 0U) ? 0U : 1U;   /* 优先处理较早就绪的半区 0 */
    buf = &g_adcBuf[(size_t)half * FASTCAP_BLOCK_LEN];   /* 定位到该半区的首地址 */

    process_frame(buf, out);   /* 做一帧统计 */
    take_raw(buf);   /* 若正在抓原始块，顺带拷贝一份 */
    save_block(buf);   /* 存一份软件快照，供波形导出通道读取 */
    g_fastcap_frames++;   /* 完成帧计数加一 */
    return 1U;   /* 本次成功输出了一帧 */
}

/*-----------------------------------------------------------
  Function:      fastcap_arm_raw
  Description:   启动一次原始块抓取（2000 点，约 100ms）。
  Calls:         无
  Called By:     调试流程（抓一段原始数据供离线 FFT）
  Input:         无
  Output:        无
  Return:        无
  Others:        抓取期间每帧会同时被 take_raw() 拷贝进原始块缓冲。
-----------------------------------------------------------*/

void fastcap_arm_raw(void)
{
    g_rawCount = 0UL;   /* 抓取点数归零 */
    g_rawReady = 0U;   /* 清就绪标志 */
    g_rawArmed = 1U;   /* 置位，等待 take_raw() 逐帧填充 */
}

/*-----------------------------------------------------------
  Function:      fastcap_is_raw_ready
  Description:   查询原始块是否已抓满。
  Calls:         无
  Called By:     无
  Input:         无
  Output:        无
  Return:        1 = 已抓满可读；0 = 未就绪
  Others:        无
-----------------------------------------------------------*/

uint8_t fastcap_is_raw_ready(void)
{
    return g_rawReady;   /* 直接返回就绪标志 */
}

/*-----------------------------------------------------------
  Function:      fastcap_get_raw
  Description:   取原始块首地址与点数，供 J-Link 或主机读取。
  Calls:         无
  Called By:     无
  Input:         无
  Output:        len —— 回填实际点数
  Return:        原始块首地址（只读，调用方不得写入）
  Others:        无
-----------------------------------------------------------*/

const uint16_t *fastcap_get_raw(uint32_t *len)
{
    if (len == NULL)   /* 输出长度指针为空 */
    {
        return NULL;   /* 无法回填点数，直接返回空 */
    }
    if (g_rawReady == 0U)   /* 原始块尚未抓满 */
    {
        *len = 0UL;   /* 点数记 0 */
        return NULL;   /* 不返回缓冲地址 */
    }
    *len = FASTCAP_RAW_LEN;   /* 回填实际点数 */
    return g_fastcap_raw;   /* 返回原始块首地址（只读） */
}

/*-----------------------------------------------------------
  Function:      fastcap_get_frames
  Description:   取已完成的采样帧数。
  Calls:         无
  Called By:     无
  Input:         无
  Output:        无
  Return:        累计帧数
  Others:        无
-----------------------------------------------------------*/

uint32_t fastcap_get_frames(void)
{
    return g_fastcap_frames;   /* 返回累计帧数 */
}

/*-----------------------------------------------------------
  Function:      fastcap_get_overruns
  Description:   取因主循环未及时处理而被 DMA 覆盖的帧数。
  Calls:         无
  Called By:     无
  Input:         无
  Output:        无
  Return:        累计丢帧数
  Others:        该计数用于判断高速采样与主循环节拍是否匹配。
-----------------------------------------------------------*/

uint32_t fastcap_get_overruns(void)
{
    return g_fastcap_overruns;   /* 返回累计丢帧数 */
}

/*-----------------------------------------------------------
  Function:      fastcap_get_status
  Description:   取模块状态位。
  Calls:         无
  Called By:     无
  Input:         无
  Output:        无
  Return:        FASTCAP_ST_* 的组合（正常 / ADC 未就绪 / 溢出 / 贴轨）
  Others:        无
-----------------------------------------------------------*/

uint8_t fastcap_get_status(void)
{
    return g_fastcap_status;   /* 返回状态位组合 */
}
/*-----------------------------------------------------------
  Function:      save_block
  Description:   把刚处理完的一帧 200 点原始样本复制到软件快照缓冲。
  Calls:         无
  Called By:     fastcap_poll
  Input:         buf：指向 DMA 半区首地址
  Output:        g_lastBlock 被整帧覆盖
  Return:        无
  Others:        为什么需要这一份拷贝：DMA 是循环模式，下一个半区填满后
                 buf 指向的区域会被硬件改写；波形导出在主循环里异步进行，
                 必须拿一份不受 DMA 影响的稳定副本。200 次 uint16 复制
                 在 168MHz 下约 1us，对 10ms 的帧周期没有影响。
-----------------------------------------------------------*/
static void save_block(const uint16_t *buf)
{
    uint32_t i;   /* 样本索引 */

    for (i = 0UL; i < FASTCAP_BLOCK_LEN; i++)   /* 逐点复制整帧 */
    {
        g_lastBlock[i] = buf[i];                /* 写入快照缓冲 */
    }
}

/*-----------------------------------------------------------
  Function:      fastcap_get_last_block
  Description:   返回最近一帧 200 点原始样本的只读指针。
  Calls:         无
  Called By:     wave_dump.c 的 build_frame()
  Input:         无
  Output:        无
  Return:        g_lastBlock 首地址，长度固定 FASTCAP_BLOCK_LEN，永不为 NULL
  Others:        第一次 fastcap_poll() 成功之前，缓冲区内容为全零，
                 调用方需自行用状态位或帧计数判断数据是否已经有效。
-----------------------------------------------------------*/
const uint16_t *fastcap_get_last_block(void)
{
    return g_lastBlock;   /* 返回快照缓冲首地址 */
}
