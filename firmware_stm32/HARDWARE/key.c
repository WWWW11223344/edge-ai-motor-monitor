/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    key.c
  Author:      汪贤云
  Version:     1.0
  Date:        2026-09-21
  Description: 按键驱动。三颗按键 KEY0=PE4、KEY1=PE3、KEY2=PE2，均为低电平有效；
               PA0(WK_UP) 走独立电路，不参与按键逻辑。
               按键有效电平在上电头 200ms 内自动学习（不同批次板子接法可能相反），
               因此两种接法都能自动适配，换板子无需改代码。
               消抖与边沿检测放在 TIM7 的 1ms 中断中完成（每 5ms 采样、连续 4 次确认），
               主循环只通过 Key_Scan() 取事件，不存在全局锁。
  Function List:
    1. Key_Init()      初始化按键引脚
    2. Key_Tick1ms()   1ms 中断内的采样与消抖
    3. Key_Scan()      主循环取按键事件
  History:
    <author>   <time>      <version>   <desc>
    汪贤云     2026-10-16    1.0         创建文件并按华为规范补充注释
***********************************************************/

#include "stm32f4xx.h"
#include "key.h"        /* 按键驱动对外接口声明 */

/* ==========================================================================
 * 按键驱动（KEY0 = PE4 / KEY1 = PE3 / KEY2 = PE2）
 * --------------------------------------------------------------------------
 * 【关键】按键的"有效电平"不写死，上电时自己学。
 *
 *   为什么？因为不同批次的板子接线可能相反：
 *     接法 A：按键一端接 3V3、另一端接引脚，板上有下拉电阻
 *             -> 静止读 0，按下读 1（高电平有效）
 *     接法 B：按键一端接 GND、另一端接引脚，引脚内部/外部上拉
 *             -> 静止读 1，按下读 0（低电平有效）
 *
 *   神舟号 V2.0 这块板实测是 **接法 B（低电平有效）**：
 *   引脚配了内部上拉（约 40k）之后静止读到 1，按下才被拉到 GND 变成 0。
 *   J-Link 远程读 RAM 里学到的 s_idle 得到 {1,1,1}，可以直接证实。
 *   （旧版 key.c 用的是「全局限锁 + 主循环里读键」的写法：
 *    锁一旦合上就再也解不开，三颗键会一起变哑。
 *    现在改成在 1ms 中断里消抖、只往主循环发事件，不存在锁。）
 *
 *   所以这里在上电头 200ms 把三根线的电平记成"静止电平" s_idle，
 *   以后凡是 != s_idle 就算按下。两种接法都自动适配，换板子也不用改代码。
 *
 * 【为什么扫描要放进 1ms 中断】
 *   主循环每 200ms 才转一圈（要刷 OLED）。若在主循环里读按键，一次按下的
 *   持续时间必须长过 200ms 才可能被"撞"上，短按会整段漏掉。
 *   放进中断后：每 5ms 采一次，连续 4 次（20ms）相同才认，主循环只取事件。
 *   每颗键各管自己，没有全局锁，不存在"把别的键锁死"的可能。
 *
 * 注意：PA0（KEY_UP）**不参与按键逻辑**，只做输入 + 内部下拉。
 *       原理图上 PA0 走的是 WK_UP 网络，和 KEY0/1/2 不是同一套电路。
 * ========================================================================== */

#define KEY_SCAN_MS     5U      /* 每 5ms 采一次按键 */
#define KEY_STABLE_CNT  4U      /* 连续 4 次相同才认 -> 20ms 消抖 */
#define KEY_LEARN_CNT   40U     /* 上电头 40 x 5ms = 200ms 用来学静止电平 */

static volatile KeyEvent s_pending = KEY_EVENT_NONE;   /* 中断填，主循环取 */

/* 下面三个数组故意不加 static：留在符号表里，
 * 调试时可以直接用 J-Link 读它们判断按键到底认没认到。 */
uint8_t s_idle[3] = { 0U, 0U, 0U };    /* 学到的静止电平 */
uint8_t s_raw[3]  = { 0U, 0U, 0U };    /* 最近一次的原始电平 */
uint8_t s_down[3] = { 0U, 0U, 0U };    /* 0=松开 1=按下（已消抖确认） */
uint16_t s_hits[3] = { 0U, 0U, 0U };   /* 各键累计按下次数（远程诊断用） */

static uint8_t s_cnt[3]  = { 0U, 0U, 0U };   /* 连续相同的次数 */
static uint8_t s_ready   = 0U;               /* 1 = 静止电平已学完 */

/*-----------------------------------------------------------
  Function:      Key_Init
  Description:   初始化按键引脚：PE2/PE3/PE4 配置为输入并开启内部上拉；
                 同时把不参与逻辑的 PA0 配置为下拉输入，避免悬空。
  Calls:         RCC_AHB1PeriphClockCmd()、GPIO_Init()
  Called By:     main.c 上电初始化（必须排在 Tim7_Init() 之前）
  Input:         无
  Output:        无
  Return:        无
  Others:        选内部上拉是因为两种典型接法都能分辨：按下接地的板子静止读 1、按下读 0。
-----------------------------------------------------------*/

void Key_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;    /* GPIO 引脚配置结构体 */

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  /* 开 GPIOA 时钟（PA0 用） */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);  /* 开 GPIOE 时钟（PE2/PE3/PE4 用） */

    /* PA0（KEY_UP）：保留初始化，内部下拉，纯粹为了不让它悬空；
     * 现在不参与任何判断。 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN; /* 输入模式；PA0 本身不参与按键判断，仅避免悬空 */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  /* 翻转速度（输入模式下属占位） */
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;     /* 内部下拉，只为不让 PA0 悬空 */
    GPIO_Init(GPIOA, &GPIO_InitStructure);              /* 写入 GPIOA 配置 */

    /* 三颗要用的键：一律配内部上拉。
     * 上拉是"两种接法都能分辨"的唯一选择：
     *   接法 B（按下接地）  -> 静止 1、按下 0，能分辨；
     *   接法 A（按下接 3V3）-> 外部下拉更强，静止 0、按下 1，也能分辨。
     * 若改配下拉，接法 B 的按下和静止都读 0，就彻底分辨不出来了。 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN; /* 输入模式 */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  /* 翻转速度（占位） */
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;     /* 内部上拉：两种按键接法都能分辨 */
    GPIO_Init(GPIOE, &GPIO_InitStructure);              /* 写入 GPIOE 配置 */
}

/* -------------------------------------------------------------------------
 * 由 TIM7 的 1ms 中断调用
 * ------------------------------------------------------------------------- */
/*-----------------------------------------------------------
  Function:      Key_Tick1ms
  Description:   1ms 中断内的按键服务：每 5ms 采样一次电平，连续 4 次（20ms）一致才确认状态变化，
                 仅在按下瞬间产生一次事件。
  Calls:         内部读取 KEY0/KEY1/KEY2 宏，不调用其它函数
  Called By:     stm32f4xx_it.c 中的 TIM7_IRQHandler
  Input:         无
  Output:        无
  Return:        无
  Others:        1) 上电头 200ms 用于学习三根线的静止电平；
                                  2) 中断内只做采样、消抖与置事件，不做任何耗时操作；
                                  3) 每颗按键各自计数，互不影响。
-----------------------------------------------------------*/

void Key_Tick1ms(void)
{
    static uint16_t tick  = 0U;      /* 毫秒分频计数（每 5 个 1ms 采一次） */
    static uint8_t  learn = 0U;      /* 学习阶段的采样次数 */
    uint8_t i;    /* 三颗键的循环下标 */

    tick++;            /* 1ms 计一下 */
    if (tick < KEY_SCAN_MS)                   /* 未满 5ms 直接返回 */
    {
        return;                 /* 还没到采样时刻 */
    }
    tick = 0U;           /* 分频计数清零 */

    s_raw[0] = (KEY0 != 0U) ? 1U : 0U;  /* 采 KEY0 电平，归一成 0/1 */
    s_raw[1] = (KEY1 != 0U) ? 1U : 0U;  /* 采 KEY1 电平，归一成 0/1 */
    s_raw[2] = (KEY2 != 0U) ? 1U : 0U;  /* 采 KEY2 电平，归一成 0/1 */

    /* ---- 阶段一：上电头 200ms 学静止电平（这期间上电别按着键） ---- */
    if (s_ready == 0U)
    {
        if (learn == 0U)         /* 第一次采样：把当前电平记为静止电平 */
        {
            for (i = 0U; i < 3U; i++)            /* 三颗键一起学 */
            {
                s_idle[i] = s_raw[i];                /* 记下静止电平 */
                s_down[i] = 0U;                /* 初值视为松开 */
            }
        }
        learn++;        /* 学习计数加一 */
        if (learn >= KEY_LEARN_CNT)         /* 学满 40 次 = 200ms 即结束 */
        {
            s_ready = 1U;            /* 标记学习完成 */
        }
        return;        /* 学习阶段不产生按键事件 */
    }

    /* ---- 阶段二：正常扫描 ---- */
    for (i = 0U; i < 3U; i++)
    {
        uint8_t down = (s_raw[i] != s_idle[i]) ? 1U : 0U;        /* 与静止电平不同即视为按下 */

        if (down == s_down[i])         /* 状态未变：清计数重新等 */
        {
            s_cnt[i] = 0U;              /* 状态没变，重新计数 */
            continue;
        }

        s_cnt[i]++;        /* 变化次数累加 */
        if (s_cnt[i] < KEY_STABLE_CNT)         /* 连续 4 次（20ms）才算稳定 */
        {
            continue;                   /* 还没稳定够，继续观察 */
        }

        s_down[i] = down;        /* 确认状态切换 */
        s_cnt[i]  = 0U;        /* 消抖计数归零 */

        /* 只在"按下"那一瞬间报一次事件；必须松开才会有下一次 */
        if (down != 0U)
        {
            s_hits[i]++;                /* 累计次数，方便远程确认键认没认到 */
            if (s_pending == KEY_EVENT_NONE)
            {
                s_pending = (KeyEvent)((uint8_t)KEY_EVENT_0 + i);                /* 事件号 = KEY_EVENT_0 + 下标，正好对应 KEY0/1/2 */
            }
        }
    }
}

/* -------------------------------------------------------------------------
 * 主循环调用：把中断里记下的事件取走（取一次就清空）
 * ------------------------------------------------------------------------- */
/*-----------------------------------------------------------
  Function:      Key_Scan
  Description:   主循环取按键事件：返回一次事件后自动清空，没有新事件时返回 KEY_EVENT_NONE。
  Calls:         无
  Called By:     main.c 主循环
  Input:         无
  Output:        无
  Return:        KeyEvent 枚举（NONE / 0 / 1 / 2）
  Others:        事件由 Key_Tick1ms() 在中断中置位，本函数只负责取走，不加锁。
-----------------------------------------------------------*/

KeyEvent Key_Scan(void)
{
    KeyEvent ev = s_pending;    /* 取走中断里置的事件 */

    if (ev != KEY_EVENT_NONE)     /* 确实是有效事件 */
    {
        s_pending = KEY_EVENT_NONE;        /* 取一次就清空，保证不重复上报 */
    }

    return ev;    /* 返回给主循环 */
}
