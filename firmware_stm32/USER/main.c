/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    main.c
  Author:      汪贤云
  Version:     1.4
  Date:        2026-09-14
  Description: 边缘 AI 电机状态监控系统的程序入口。
               上电按固定顺序初始化各外设后进入主循环，每轮完成：
               扫按键（切档/急停）-> 10 ms 采转速/电流 ->
               500 ms 读母线电压 -> 50 ms 刷一片 OLED 区域 ->
               每轮发一帧 M2 遥测给 D1 mini。
  Function List:
    1. LED_AlarmIndicate()  把 AI 报警状态映射到板载 LED0（PF9，低电平点亮）
    2. main()               上电初始化 + 1 ms 周期主循环
  History:
    <author>   <time>      <version>   <desc>
    汪贤云     2026-09-14    1.0         创建文件（转速检测 + 电机自转测试）
    汪贤云     2026-10-05    1.1         改分片刷新，加电流/振动/电压上屏
    汪贤云     2026-10-16    1.2         按《华为编程规范》补充中文逐行注释
    汪贤云     2026-10-19    1.3         移除 MPU6050 振动采集，改用电流作为唯一监控变量
    汪贤云     2026-10-19    1.4         接入转速单变量特征 R，第 4~5 页改显示 R
***********************************************************/
#include "stm32f4xx.h"    /* CMSIS 内核 + StdPeriph 全部外设寄存器声明（必须第一个包含） */
#include "led.h"          /* 板载 LED0（PF9）初始化与开关 */
#include "oled.h"         /* SSD1306 OLED 底层驱动（PB8/PB9 软件 I2C） */
#include "ui.h"           /* 屏幕排版层：把 UI_Data 渲染成 4 行文字 */
#include "Systick.h"      /* 微秒/毫秒级阻塞延时基准 */
#include "encoder.h"      /* TIM4 编码器模式测速（PB6/PB7） */
#include "timer.h"        /* TIM6/TIM7 定时中断 */
#include "pwm.h"          /* 电机 PWM 初始化与占空比设置（TIM9，PE5/PE6） */
#include "key.h"          /* 三键扫描（PE2/PE3/PE4） */
#include "vbat.h"         /* 母线电压采集（PA4，板载 1/11 分压） */
#include "current.h"      /* ACS712 电流采集与零点校准（PA5） */
#include "feature_r.h"    /* 转速单变量特征 R = rpm / 母线电压（32 点窗口） */
#include "telemetry.h"    /* 向 D1 mini 发送 M2 二进制遥测帧（USART3） */
#include "ai_monitor.h"   /* 板上 NanoEdge AI 学习/检测状态机 */
#include "fastcap.h"
#include "wave_dump.h"   /* 20 kHz 电流波形的上位机导出通道（USART1） */      /* 20 kHz ADC2+DMA 电流前端（并行取证用） */

/* ==========================================================================
 * 第 4 步 · 转速检测（编码器 -> TIM4 编码器模式 -> OLED）
 *         + 电机自转测试（按键切档，省得用手拧输出轴）
 *         + 母线电压上屏（电池电量看得见）
 * --------------------------------------------------------------------------
 * 接线：
 *      OLED    GND -> GND     VDD -> 3V3     SCK -> PB8     SDA -> PB9   （接线表）
 *      编码器  GND -> GND     VCC -> 3V3     A相 -> PB6     B相 -> PB7
 *              （PB6 = 扩展排针 P8 第 50 脚，PB7 = P8 第 49 脚，相邻两脚）
 *      电机    AIN1 -> PE5    AIN2 -> PE6    PWMA -> 3V3    STBY -> 3V3
 *              （控制信号走 D153C 驱动板 H1「Input_IO」排针；
 *                PWMA 和 STBY 都是硬件直接接 3V3，所以软件不用管使能，
 *                只要给出 PWM，电机就会转 —— 这条已由 7_PWM_Motor 验证过）
 *      按键    KEY0 -> PE4    KEY1 -> PE3    KEY2 -> PE2
 *      电压    D153C H2 第 5 脚 ADC -> PA4（P11 第 5 脚）
 *              （板载 10k + 1k 分压，读出来乘 11 就是电池电压。
 *                只补这 1 根线，不用买任何东西。详见 vbat.h）
 *
 * 为什么要加「电机自转」这一小段？
 *   用手拧输出轴测转速有两个毛病：一是转不快（MG310 输出轴就一小截杆子，
 *   捏着快转既费劲又手疼），二是手抖导致读数忽高忽低，很难判断「到底准不准」。
 *   让电机用固定占空比自己转，转速就稳稳停在一个值上，
 *   再拿去和 MG310 的规格（7.4V 时空载 500±65 rpm）一比，标定对不对立刻见分晓。
 *
 * 按键怎么用（上电默认 0 档 = 停转，不会一上电就狂转）：
 *      KEY0 (PE4)   立即停转 —— 急停，出任何状况先按它
 *      KEY1 (PE3)   档位 +1
 *      KEY2 (PE2)   档位 -1
 *      屏底条会显示当前档位，形如 L1:58%（L档位号:占空比%）
 *      固定档位表：0档 0%(停) / 1档 58%
 *      58% 是板端 AI 的训练/检测档位（现场约 143~146rpm）。
 *
 * 母线电压怎么上屏？
 *   小字号那一行的左边原来是假的相似度 Sim x.xx，现在换成真实的
 *      Bat xx.x V      （屏幕显示格式示例）
 *   （第 5 步接上 ACS712 之后，相似度会挪回屏上；现在留着假值不如留着真值。）
 *
 * 屏上预期 + 判据：
 *      档位   占空比    屏上 Spd
 *      0 档     0%    0 rpm / STOP
 *      1 档    58%    低速基线（现场约 143~146 rpm，AI 训练/检测档）
 *
 *   第一层判据（先看这条）：**升一档，Spd 必须跟着升。**
 *   单调上升就说明 编码器 -> TIM4 -> TIM7 -> 屏 这条链路是通的。
 *
 *   第二层判据（绝对值准不准）：先看屏上 Bat 那一格，再对表 ——
 *      转速约等于 67.6 rpm/V 乘 母线电压 乘 占空比 （MG310 空载 500rpm @ 7.4V）
 *   例：母线 12.6V、占空比 58% -> 约 494 rpm；
 *       母线  5.6V、占空比 30% -> 约 114 rpm。
 *   光看一个绝对数就下结论是不可靠的；母线电压不知道，就等于没标定。
 *
 *   58% 是唯一的固定测试/训练档位。
 *
 * 数据是怎么从编码器走到屏上的：
 *      编码器 A/B 两相方波
 *              |  TIM4 硬件四倍频计数（纯硬件，不占 CPU）
 *              |  TIM7 每 1ms 中断一次，攒够 100ms 读一次计数差、算成转速
 *              v  Encoder_GetRpm()   （调用链示意）
 *      填进 UI_Data.speed_rpm --> UI_Update() 画到屏上
 *
 * 屏上现在的 4 行（振动行已随 MPU6050 一并移除，第 4~5 页空出）：
 *      Spd     xxx rpm       <- 真实数据！编码器测出来的转速
 *      Cur     x.xx  A       <- 电流（ACS712）；模块未接时显示 --.--
 *      Bat xx.x V  ST:XXX    <- 真实数据！母线电压（原来这里是假的相似度）
 *      SIM  L2:xx%  T xxxxxxs  |   <- 档位/扫描结果 + 运行秒数（真实）+ 心跳
 * ========================================================================== */

#define REFRESH_MS      50U         /* 分片刷屏周期：每次只发送一个区域 */
#define SENSOR_UPDATE_MS 10U        /* 原始电流和遥测采样周期 */
#define VBAT_UPDATE_MS  500U        /* 母线电压读取较慢，独立低频更新 */
#define DEMO_MARK       1U          /* 恒为 1：告诉上位机本帧含 SIM 标记；有效性另由 valid 位标 */

/* ---- 电机测试档位：占空比百分数 ----
 * TIM9 的 ARR = 100-1 = 99，所以 TIM_SetCompare 的值正好就是百分数。
 * 2026-10-14：只保留 0 档（停）与 1 档（58%），自动扫描已删除。 */
static const uint8_t DUTY_LEVEL[2] = { 0U, 58U };   /* 0 档 = 停转，1 档 = 58%（板端 AI 训练/检测档） */
#define LEVEL_MAX       1U          /* 档位上限：只有 0 与 1 */

/* 2026-10-14：最近一帧 20 kHz 采样统计。刻意不加 static，
 * 供 J-Link 按符号名读，确认 fastcap 前端在上电后真的在推进。 */
FastcapSample g_fastcap_last = {0};    /* 最近一帧统计，供 J-Link 按符号名读 */

/*-----------------------------------------------------------
  Function:      LED_AlarmIndicate
  Description:   把 AI 报警状态映射到板载 LED0（PF9）。
                 LED0 为低电平点亮：报警时拉低点亮，否则拉高熄灭。
  Calls:         GPIO_ResetBits()、GPIO_SetBits()
  Called By:     main()（每轮循环按 d.state 调用一次）
  Input:         alarm —— 0 = 正常（灭），非 0 = 报警（亮）
  Output:        改写 GPIOF 的 ODR 第 9 位
  Return:        无
  Others:        PF9 由 LED_Init() 配成推挽输出，此处只改电平，
-----------------------------------------------------------*/
static void LED_AlarmIndicate(uint8_t alarm)
{
    if (alarm != 0U)                      /* 报警：点亮 */
    {
        GPIO_ResetBits(GPIOF, GPIO_Pin_9);   /* PF9 拉低 = 点亮（低电平有效） */
    }
    else                      /* 正常：熄灭 */
    {
        GPIO_SetBits(GPIOF, GPIO_Pin_9);     /* PF9 拉高 = 熄灭 */
    }
}                                             /* LED_AlarmIndicate 函数结束 */

/*-----------------------------------------------------------
  Function:      main
  Description:   程序入口。先按固定顺序初始化各外设（顺序敏感：
                 按键要先于 1 ms 时基初始化，否则测不到正确的静止电平），
                 再进入 1 ms 周期的主循环，周期性完成按键处理、
                 传感器采样、AI 状态判决、遥测发送与 OLED 分片刷新。
  Calls:         SysTick_Init()、LED_Init()、LED_AlarmIndicate()、
                 UI_Init()、Key_Init()、Encoder_Init()、Vbat_Init()、
                 Tim7_Init()、PWM_Motor_Config()、Motor_Config()、
                 Current_Init()、Current_Calibrate()、fastcap_init()、
                 telemetry_init()、ai_monitor_init()（本轮注释掉，见下方说明）
  Called By:     C 运行时启动代码（复位向量调用，永不返回）
  Input:         无
  Output:        无（经全局状态、屏幕与串口对外产生影响）
  Return:        标准要求 int，但主循环不退出，实际永不返回
  Others:        1) 主循环周期由末尾 Delay_ms(1U) 决定，约 1 ms；
                 2) 所有「是否到期」判断都用无符号减法，天然抗回绕。
-----------------------------------------------------------*/
int main(void)
{
    UI_Data d = {0};                /* 一帧界面数据，先全部清零再逐步填充 */
    uint8_t  level = 0U;            /* 当前档位，上电 0 档 = 停转 */

    SysTick_Init();                    /* 建立延时基准（HCLK/8 = 21 MHz） */
    LED_Init();                        /* 配置 PF9 为推挽输出 */
    LED_AlarmIndicate(0U);             /* 上电先让 LED 熄灭（正常态） */

    /* OLED 上电后需要 >=100ms 稳定时间 */
    Delay_ms(200U);

    /* 初始化屏幕并把固定标签（Spd/Cur/Vib/Bat/ST:）画好 */
    UI_Init();                      /* 画好固定标签，准备逐区刷新 */

    /* ---- 顺序很重要：先把按键配好，再启动 1ms 时基 ----
     * 按键扫描跑在 TIM7 中断里，而上电后头 200ms 它要「学」三根线的静止电平。
     * 如果 TIM7 先启动，学习时引脚还没配置（复位默认是浮空输入），
     * 学到的「静止电平」就是错的，之后所有按键判断都会反过来。 */
    Key_Init();                             /* PE2/PE3/PE4（PA0 只做下拉，不参与）*/
    Encoder_Init();                         /* PB6/PB7 -> TIM4 编码器模式 */
    Vbat_Init();                            /* PA4 -> ADC1_IN4，母线电压（1/11） */

    Tim7_Init();                            /* TIM7 每 1ms：读编码器 + 扫按键 */

    /* ---- 第 4 步补充：电机驱动 ---- */
    PWM_Motor_Config();                     /* PE5/PE6 输出到 TIM9 的 CH1/CH2 */
    Motor_Config(DUTY_LEVEL[level], 1U);    /* 上电先停转，等人按键 */

    /* ---- 第 5 步：电流零点校准（电机停转时做，约 330ms） ---- */
    Current_Init();
    Current_Calibrate(); /* 上电先做电流零点校准（64 次平均） */

    /* ---- 第 6 步：20 kHz 电流采样前端（ADC2+DMA，相位锁到 TIM9 ON 窗中部） ----
     * 必须位于 Vbat_Init()（ADC 公共预分频）与 PWM_Motor_Config()（TIM9 运行）之后。 */
    fastcap_init();
    WaveDump_Init();                 /* 波形导出通道：USART1(PA9) 115200，须在 fastcap_init() 之后 */

    telemetry_init();               /* 初始化 USART3 遥测发送通道 */
    /* 2026-10-19：换用单变量 R 的 32 点窗模型库（Library ID
     * 6ad6053cc9f44b85284837b0，模型族 ZSM，输入 1 轴 x 32 点）。
     * 振动传感器已移除，模型输入不再是 19 维，而是 feature_r 窗口里的
     * 32 个 R 点；旧版「validMask 必须全有效」的门禁随 19 维一起删掉了，
     * 现在的有效判据在 feature_r 内部（转速 >= 20 rpm 且电压通路有效）。
     * 前置条件：下面主循环 10 ms 块里已经在调 feature_r_tick()，
     * 并且每攒进一个新点就触发一次 ai_monitor_tick()。 */
    ai_monitor_init();               /* 初始化板上 AI，成功后直接进入学习阶段 */

    {   /* 开一层局部作用域：下面三个「上次时刻」只在主循环内可见 */
        uint32_t lastSensorMs = Encoder_GetMs();       /* 上次 10 ms 传感器采样的时刻 */
        uint32_t lastVbatMs = lastSensorMs - VBAT_UPDATE_MS; /* 减去一个周期，保证首轮立刻读一次电压 */
        uint32_t lastUiMs = lastSensorMs - REFRESH_MS;       /* 同理，保证首轮立刻刷一次屏 */

    while (1)                                 /* 主循环：永不退出 */
    {
        /* ============ 按键切档 ============
         * KEY0(PE4) = 立即停转（急停，出任何状况先按它）
         * KEY1(PE3) = 档位 +1
         * KEY2(PE2) = 档位 -1
         *
         * 为什么改成"每颗键各管一件事"、而不是"任意键加一档"？
         * 原来那种写法靠 Key_Scan() 里的全局 key_locked 保证"一次按下只加一档"，
         * 而那个锁的解锁条件是"四颗键全部松开"；KEY_UP 接的 PA0 走 WK_UP 网络、
         * 和另外三颗键不是一套电路，只要 PA0 一直读到高，锁就永远解不开，
         * 表现就是"档位卡死不动、按什么键都没反应"。
         * 现在消抖和边沿检测全在 1ms 中断里做（key.c 的 Key_Tick1ms），
         * PA0 完全不参与，主循环只取事件，没有锁可以卡。 */
        {                                 /* 按键区：独立作用域，key 用完即弃 */
            KeyEvent key = Key_Scan();                                     /* 取一次按键事件（无事件返回 NONE） */

            if (key == KEY_EVENT_0)                        /* KEY0：急停 */
            {
                level = 0U;                             /* 急停：立刻停转 */
                Motor_Config(DUTY_LEVEL[0], 1U);                                                    /* 立即停转 */
            }
            else if (key == KEY_EVENT_1)                        /* KEY1：档位 +1 */
            {
                if (level < LEVEL_MAX)                                       /* 未到上限才允许加 */
                {
                    level++;                                    /* 档位加一 */
                }
                Motor_Config(DUTY_LEVEL[level], 1U);                                    /* 按新档位重设 PWM */
                if (level == 1U)                                       /* 恰好进到 58% 档 */
                {
                    /* 进入 58% 档时抓一次 100ms@20kHz 原始块（2000 点），
                     * 供 J-Link 读 g_fastcap_raw 后做 FFT；期间若有整帧丢失则作废。 */
                    fastcap_arm_raw();
                }
            }
            else if (key == KEY_EVENT_2)                        /* KEY2：档位 -1 */
            {
                /* level 只会从 1~2 往下落到 0，所以这里索引永远 <= 1，安全 */
                if (level > 0U)                                       /* 未到下限才允许减 */
                {
                    level--;                                    /* 档位减一（下限 0） */
                }
                Motor_Config(DUTY_LEVEL[level], 1U);                                    /* 按新档位重设 PWM */
            }
        }                                 /* 按键处理结束 */

        d.demo  = DEMO_MARK;                  /* 标记本帧含 SIM 字段 */
        d.level = level;                  /* 本帧档位 */

        d.duty = DUTY_LEVEL[level]; /* 底条显示当前占空比 */

        {                                 /* 采样区：独立作用域，隔离本轮临时变量 */
            uint32_t nowMs = Encoder_GetMs();                        /* 取一次毫秒时基 */
            uint8_t sampledThisLoop = 0U;                /* 本轮是否采到新的传感器快照 */
            uint8_t vbatUpdated = 0U;                /* 本轮是否更新了母线电压 */

            /* 2026-10-07：先把较慢的电压 I/O 做完，再给后面的快照打时间戳，
             * 这样它的延时不会挤占相邻两次遥测帧之间的发送间隔。 */
            if ((uint32_t)(nowMs - lastVbatMs) >= VBAT_UPDATE_MS)  /* 到达 500 ms 周期则读一次母线电压 */
            {
                lastVbatMs = nowMs;                    /* 记下本次时刻 */
                vbatUpdated = 1U;                /* 标记本轮已做慢速 I/O */
                d.vbat_v = Vbat_GetVolt();                     /* 母线电压（V） */
                d.vbat_valid = Vbat_IsValid();                     /* 电压有效性（分压通路是否在位） */
                nowMs = Encoder_GetMs();                       /* 慢速 I/O 会耗时，重新取一次时基 */
            }

            /* 10 ms 路径：一次电流 ADC 转换 + 一次转速读取。 */
            if ((uint32_t)(nowMs - lastSensorMs) >= SENSOR_UPDATE_MS)  /* 到达 10 ms 周期则采一次传感器 */
            {
                lastSensorMs = nowMs;                        /* 记下本次时刻 */
                sampledThisLoop = 1U;                /* 标记本轮已产生新快照 */
                d.speed_rpm = Encoder_GetRpm();                        /* 转速（rpm） */
                d.speed_valid = (d.duty == 0U || d.speed_rpm >= 20.0f) ? 1U : 0U; /* 停转时速度视为有效；转动时需 >= 20 rpm 才算有效 */
                /* 电流只作辅助观测量：送 OLED 与遥测，不参与 AI 判定。 */
                /* AI 的唯一输入是 feature_r 产出的 R = 转速 / 母线电压。 */
                d.current_a = Current_GetAmp();                        /* 电流（A） */
                d.current_valid = Current_IsValid();                        /* 电流有效性 */
                /* 2026-10-19：停机通知。转速低于 20 rpm 时每 10 ms 报一次，
                 * 连续 0.5 s 停机就重学基线 —— 换负载/换工况后不用断电就能重新对齐。
                 * 注意不能放在 feature_r_tick() 里面判断：停机时它直接返回 0，
                 * ai_monitor_tick() 根本不会被调用。 */
                if (d.speed_rpm < 20.0f)  /* 电机没转 */
                {
                    ai_monitor_notify_stopped();  /* 累计停机拍数 */
                }
                /* 2026-10-19：转速单变量方案 —— 按 100 ms 节拍把
                 * R = 转速 / 母线电压 攒进 32 点窗口（窗长 3.2 s）。
                 * 停转或电压通路失效时模块内部会自己丢窗，
                 * 不会把停机期间的数据混进正在检测的窗口里。 */
                if (feature_r_tick(d.speed_rpm, d.vbat_v, d.vbat_valid, nowMs) != 0U)
                {   /* 这一拍真的攒进了一个新点（大约每 100 ms 一次） */
                    ai_monitor_tick();   /* 窗满 32 点就喂一次模型，窗不满直接返回 */
                }
                d.r_ratio = feature_r_current();                        /* 当前 R（rpm/V），供屏显 */
                d.r_valid = feature_r_current_valid();                  /* R 是否处于有效运行区间 */
                d.r_fill  = feature_r_fill();                           /* 窗口已攒点数（0~32） */
                /* 2026-10-14：推进 20 kHz 前端。只更新 g_fastcap_last /
                 * g_fastcap_raw / 内部计数器，不改 d.current_a，
                 * 等原始块 FFT 验证后再切换 AI 的电流来源。 */
                if (fastcap_poll(&g_fastcap_last) != 0U)                /* 推进 20 kHz 前端，返回是否凑满一整帧 */
                {
                    WaveDump_Poll(d.duty, &g_fastcap_last);                /* 波形导出：每 100 ms 发一帧 200 点，停转不发 */
                }
                if ((d.duty == 0U) && (d.speed_rpm < 20.0f))                /* 停转且速度近零：适合做零点跟踪 */
                {
                    Current_TrackZero();                        /* 停机零点缓慢跟踪（alpha = 0.02） */
                }
            }                             /* 10 ms 传感器采样结束 */

            d.uptime_s = nowMs / 1000U;                        /* 运行秒数 = 毫秒时基 / 1000 */
        

        /* 2026-10-13：把板上 NanoEdge AI 的结果接进显示状态。
         * RUN  = 遥测有效且电机在转；
         * ALARM= 检测阶段相似度连续低于冻结阈值；
         * 任一传感器读数无效时，WARN 优先于 STOP/RUN。 */
        {                                 /* AI 状态判决区 */
            AiMonitorPhase aiPhase = ai_monitor_get_phase();                        /* 取当前 AI 阶段 */

            d.ai_phase = (aiPhase == AI_MONITOR_LEARNING) ? 1U :                         /* 把内部阶段枚举映射成界面用的 0/1/2 */
                         ((aiPhase == AI_MONITOR_DETECTING) ? 2U : 0U);                         /* 0=空闲 1=学习中 2=检测中 */
            d.similarity = (ai_monitor_has_result() != 0U) ?                         /* 有结果就用真实相似度 */
                           (float)ai_monitor_get_similarity() : 0.0f;                           /* 否则填 0 */
            d.ai_valid = ai_monitor_has_result();                                /* 屏上据此决定显示数值还是 --- */

            if ((d.speed_valid == 0U) || (d.vbat_valid == 0U) ||                /* 任一物理量无效 -> WARN 优先 */
                (d.current_valid == 0U))                                       /* 与上一行的条件互为或关系 */
            {
                d.state = UI_ST_WARN;                            /* 告警态：数据不可信 */
            }
            else if (d.speed_rpm < 20.0f)                /* 传感器都有效但电机没转 */
            {
                d.state = UI_ST_STOP;                            /* 停机态 */
            }
            else
            {
                d.state = UI_ST_RUN;                            /* 正常运行态 */
                /* 2026-10-14：ALARM 改由 ai_monitor 内部的事件级规则产生
                 * （连续多个窗口相似度偏低），不再看单个采样点低于阈值。
                 * 原因：正常运行时相似度本身就会下探，固定阈值会误报。 */
                if ((aiPhase == AI_MONITOR_DETECTING) &&                /* 检测阶段且 AI 报了事件级报警 */
                    (ai_monitor_is_alarm() != 0U))                /* 与上一行的条件互为与关系 */
                {
                    d.state = UI_ST_ALARM;                            /* 升级为报警态 */
                }
            }
        }                                 /* AI 状态判决结束 */

            /* 复用同一份快照；无效字段在 M2 帧里仍会被 valid 位屏蔽 */
            LED_AlarmIndicate((d.state == UI_ST_ALARM) ? 1U : 0U);            /* 报警则点灯，正常则熄灭 */
            telemetry_poll(&d, nowMs);            /* 发一帧 M2 遥测（内部按节奏发送） */
            /* 2026-10-07：只有拿到新快照、且本轮没做慢速电压读取时，
             * 才开一个 OLED 区域刷屏；不要把电压检测延时和 OLED I/O 挤在同一周期。 */
            if ((sampledThisLoop != 0U) && (vbatUpdated == 0U) &&            /* 本轮有新快照、且本轮没做慢速电压读取、且到刷屏周期 */
                ((uint32_t)(nowMs - lastUiMs) >= REFRESH_MS))            /* 与上一行的条件互为与关系 */
            {
                lastUiMs = nowMs;                /* 记下本次刷屏时刻 */
                UI_Update(&d);                /* 只刷一个区域，保证每帧耗时可控 */
            }
        }                             /* 本轮 采样/遥测/刷屏 结束 */

        Delay_ms(1U);        /* 主循环节拍：约 1 ms 一轮 */
    }                                         /* while (1) 主循环结束 */
    }                                         /* 局部时间戳作用域结束 */
}                                             /* main 函数结束 */
