/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    current.c
  Author:      汪贤云
  Version:     1.2
  Date:        2026-10-19
  Description: ACS712 电流检测驱动，传感器输出接 PA5 / ADC1_IN5。
               电流路径：电池正极 -> ACS712 的 IP+/IP- -> D153C 的 VIN，
               输出电压经 ADC 采样后按 I = (Vout - 零点) / 0.185 换算为安培。
               零点在上电时校准一次，并在电机确认停转时缓慢跟踪温漂。
               原始采样值先过 8 点滑动平均，用于抑制 PWM 纹波造成的零点附近符号翻转。
                【模块定位：只作辅助观测量，不参与 AI 判定】
                ------------------------------------------------------
                本模块产出的电流值只用于以下四处，不进入模型：
                  1) OLED 显示，供现场直接判读；
                  2) 遥测帧上报，保持数据完整；
                  3) 传感器健康判据：读取失败时界面强制显示 WARN；
                  4) 排障：电流长期约 0 说明功率回路未导通。
                AI 的唯一输入是 feature_r 模块产出的 R = 转速 / 母线电压。
                电流不参与判定，原因是电流没有除掉母线电压与占空比的影响，
                换电池、换档位后阈值都要重新标定；而 R 已经把电压除掉了。
                板上两批实测：R 的正常与异常两簇完全不重叠
                （正常 13.23~13.53，异常 10.79~11.07）；
                电流的两簇仍有约 16 mA 的重叠区（0.221~0.237 A），
                故本模块只观测、不判决。
  Function List:
    1. Current_Init()            初始化 PA5 为模拟输入
    2. Current_Calibrate()       停机校准零点电压
    3. Current_GetAmp()          取得当前电流(A)
    4. Current_TrackZero()       停机时跟踪零点温漂
    5. Current_IsValid()         查询数据是否可信
  History:
    <author>   <time>      <version>   <desc>
    汪贤云     2026-09-23    1.0         创建文件
    汪贤云     2026-10-16    1.1         按《华为编程规范》补充中文注释
    汪贤云     2026-10-19    1.2         明确模块定位：只作辅助观测量，不进入 AI 判定
***********************************************************/
#include "current.h"        /* 本模块接口声明 */
#include "vbat.h"           /* 复用 ADC1_ReadChannel() / ADC1_LastReadOk() */
#include "pwm.h"            /* 校准前要先确保电机停转 */
#include "Systick.h"        /* Delay_ms() 需要 */

/* 【滑动平均窗口 = 8 个样本】
 * 为什么要平均：PWM 以 20kHz 斩波，电流本身带纹波；
 * 再加上 ADC 每次要切换通道（PA4 电压 / PA5 电流共用 ADC1），
 * 单点采样可能正好落在纹波的谷底 —— 于是算出「负电流」，
 * OLED 上就会闪一个假的负号。平均掉就好了。
 * 代价是滞后：窗口 8、采样 10ms 一次，最坏引入约 (8-1)*10ms = 70ms 的延迟。
 * 70ms 对「看电机状态」完全够用，但别拿它去做高速闭环控制。 */
#define CURRENT_FILTER_WINDOW 8U

/* 【零点跟踪系数 = 0.02】
 * 这是一个一阶低通（指数平滑）的系数：
 *   新零点 = 旧零点 + 0.02 * (当前测量值 - 旧零点)
 * 系数越小越稳、越慢。0.02 配 10ms 周期，时间常数约 1/0.02 = 50 个周期 = 0.5s，
 * 也就是约半秒把温漂吃掉，同时不会去追 ADC 的随机噪声。
 * 【只在电机确实停转时才调用】（main.c 判断 duty==0 且转速<20）
 * 否则会把「真实电流」当成零点漂移抹掉，那就测不出东西了。 */
#define CURRENT_ZERO_TRACK_ALPHA  0.02f

/* 【采样相位抖动：PWM 纹波为什么必须这样处理】
 * 2026-10-21 定位到的真问题：主循环的传感器节拍是 10 ms，TIM9 的 PWM
 * 周期是 50 us（2 MHz / 100），10 ms 正好是 200 个 PWM 周期；而时基
 * nowMs 的刻度是 1 ms，也是 20 个 PWM 周期。两边的比值永远是整数，
 * 于是每一次电流采样都落在纹波的同一个相位上 ——
 * 8 点滑动平均平均的是同一个瞬时值，纹波一点都没被抵消。
 * 现场证据：停机零点 2.59 V、带载读数 2.52 V，算出来是 -0.37 A；
 * 而当天早些时候同一工况（58%、140 rpm）是 +0.10 A。
 * 因为停机时没有 PWM、没有纹波，零点一直是准的，所以问题只在带载时出现；
 * 又因为换用更重的 ICM 推理改变了主循环耗时，采样相位跟着变，
 * 读数才会在这一次固件升级后凭空跳了 68 mV。
 * 解法：让采样相位在 0..7 之间轮转，每档相差 1050 个内核周期
 * （168 MHz 下 = 6.25 us），8 个样本正好铺满一个 50 us 的 PWM 周期，
 * 8 点滑动平均就变成整周期平均，纹波被真正消掉。
 * 用 DWT 周期计数器计时，不占用定时器，也不需要改 ADC 配置。
 * 注意：零点校准是在停转状态下做的，没有纹波，所以 Current_Calibrate()
 * 里不加抖动，避免无谓地拉长上电校准时间。 */
#define CURRENT_DITHER_PHASES       8U
#define CURRENT_DITHER_STEP_CYCLES  1050U

/* ==========================================================================
 * ACS712 电流检测（PA5 / ADC1_IN5）
 * --------------------------------------------------------------------------
 * PA4 与 PA5 共用 ADC1：每个读取者先把自己要用的通道选好，再软件触发一次转换。
 * 没接 ACS712 时 PA5 是悬空的，读到的随机值不能当电流用。
 * 所以校准只在「转换成功且零点电压接近理论 2.5V」时才认可传感器已经接上。
 * ========================================================================== */

/* 【本文件的关键状态变量】
 *   g_i_zero        当前零点电压(V)。上电由 Current_Calibrate() 定，之后会被
 *                   Current_TrackZero() 缓慢修正。全部换算都以它为基准。
 *   g_i_valid       0 = 电流数据不可信（未校准 / ADC 出错 / 零点超范围）
 *                   1 = 传感器在、最近一次读数可用。main.c 靠它决定显示还是显示 --
 *   g_i_fail_reason 失败原因码：0=正常，1=ADC 转换超时，2=零点电压超出合理范围
 *                   （这两个是现场排查最常用的线索）
 *   g_i_adc_sr / g_i_adc_cr2 / g_i_adc_sqr3   （ADC 现场寄存器快照）
 *                   ADC 出错那一刻抄下来的三个寄存器现场（状态寄存器、
 *                   控制寄存器2、通道选择寄存器），用来事后判断是「没使能」
 *                   还是「通道选错」还是「转换没完成」
 *   g_i_raw_history[8] + g_i_raw_index + g_i_raw_count  （滑动平均环形缓冲）
 *                   8 点滑动平均用的环形缓冲：index 是下一个写的位置，
 *                   count 是已经攒到的样本数（上电初期不足 8 个）
 *   g_i_raw_avg     平均后的原始 ADC 码（还没换成电压），供 Get/TrackZero 复用
 * 【为什么这些缓冲都放在文件级 static】中断/主循环都会碰它们，
 * 放全局既省栈也保证地址固定，方便用调试器直接读。
 * 【重要】PA4(母线电压) 和 PA5(电流) 共用 ADC1，谁用谁先选通道 —— 见 vbat.c */
static float g_i_zero = 2.5f;  /* 当前零点电压（V），缺省按 ACS712 理论值 2.5V */
static uint8_t g_i_valid = 0U;  /* 电流数据是否可信 */
static volatile uint8_t g_i_fail_reason = 0U; /* 失败原因：0=正常 1=ADC 超时 2=零点越界 */
static volatile uint32_t g_i_adc_sr = 0U;  /* ADC 出错时的 SR 现场 */
static volatile uint32_t g_i_adc_cr2 = 0U;  /* ADC 出错时的 CR2 现场 */
static volatile uint32_t g_i_adc_sqr3 = 0U;  /* ADC 出错时的 SQR3 现场（通道号） */
static uint16_t g_i_raw_history[CURRENT_FILTER_WINDOW] = {0U};  /* 滑动平均用的环形缓冲 */
static uint8_t g_i_raw_count = 0U;  /* 已收集的样本数（上电初期不足 8） */
static uint8_t g_i_raw_index = 0U;  /* 下一个写入位置 */
static uint16_t g_i_raw_avg = 0U;  /* 平均后的原始 ADC 码 */
static uint8_t  g_i_dither = 0U;  /* 采样相位档位 0..7，轮转铺满一个 PWM 周期 */

/*-----------------------------------------------------------
  Function:      Current_ConfigPin
  Description:   按传入的模式与上下拉配置 PA5 引脚，供初始化复用。
  Calls:         GPIO_Init()
  Called By:     Current_Init()
  Input:         mode —— GPIO 工作模式（本工程传 GPIO_Mode_AN 模拟输入）
                                  pupd —— 内部上下拉（本工程传 GPIO_PuPd_NOPULL）
  Output:        无
  Return:        无
  Others:        结构体必须先清零，否则栈上残留值会被写入寄存器。
-----------------------------------------------------------*/
static void Current_ConfigPin(uint32_t mode, uint32_t pupd)
{
    GPIO_InitTypeDef g = {0};  /* GPIO 配置结构体（清零后赋值） */

    g.GPIO_Pin = GPIO_Pin_5;  /* 操作 PA5 */
    g.GPIO_Mode = (GPIOMode_TypeDef)mode;  /* 工作模式由调用者指定 */
    g.GPIO_OType = GPIO_OType_PP;  /* 推挽（仅输出模式有效） */
    g.GPIO_PuPd = (GPIOPuPd_TypeDef)pupd;  /* 上下拉由调用者指定 */
    g.GPIO_Speed = GPIO_Speed_50MHz;  /* 中速档 */
    GPIO_Init(GPIOA, &g);  /* 写入 GPIOA 配置 */
}

/*-----------------------------------------------------------
  Function:      Current_EnableCycleCounter
  Description:   打开 Cortex-M4 的 DWT 周期计数器，供采样相位抖动计时。
  Calls:         无
  Called By:     Current_Init()
  Input:         无
  Output:        无
  Return:        无
  Others:        DWT 是内核自带的调试单元，启用后 CYCCNT 按内核时钟 168 MHz
                 自增；这里只拿它做几十微秒级的短延时，不动任何外设。
-----------------------------------------------------------*/
static void Current_EnableCycleCounter(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  /* 打开跟踪单元电源 */
    DWT->CYCCNT = 0U;  /* 计数器清零 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;  /* 启动周期计数器 */
}

/*-----------------------------------------------------------
  Function:      Current_WaitCycles
  Description:   忙等指定的内核周期数（168 MHz 下 168 个周期约 1 us）。
  Calls:         无
  Called By:     Current_UpdateFiltered()
  Input:         cycles  需要等待的周期数
  Output:        无
  Return:        无
  Others:        用无符号减法比较，计数器回绕时也不会死等。
-----------------------------------------------------------*/
static void Current_WaitCycles(uint32_t cycles)
{
    uint32_t start = DWT->CYCCNT;  /* 记下起始计数 */

    while ((uint32_t)(DWT->CYCCNT - start) < cycles)  /* 等到够周期数为止 */
    {
        /* 空循环，靠 DWT 周期计数器计时 */
    }
}

/*-----------------------------------------------------------
  Function:      Current_Init
  Description:   初始化电流检测引脚：打开 GPIOA 时钟，将 PA5 配置为模拟输入，
                 并清空有效标志与失败原因。
  Calls:         RCC_AHB1PeriphClockCmd()、Current_ConfigPin()
  Called By:     main.c 初始化部分（只调用一次）
  Input:         无
  Output:        无
  Return:        无
  Others:        ADC1 的时钟与通用配置已在 vbat.c 中完成，本函数不重复配置。
-----------------------------------------------------------*/
void Current_Init(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  /* 开 GPIOA 时钟 */
    Current_ConfigPin(GPIO_Mode_AN, GPIO_PuPd_NOPULL);  /* PA5 配成模拟输入，并关掉内部上下拉 */
    Current_EnableCycleCounter();  /* 启动 DWT 周期计数器，供采样相位抖动计时 */
    g_i_valid = 0U;  /* 未校准前数据不可信 */
    g_i_fail_reason = 0U;  /* 清空失败原因 */
}

/*-----------------------------------------------------------
  Function:      Current_ReadRaw
  Description:   读取 PA5(ADC1_IN5) 的一次原始转换结果。
  Calls:         ADC1_ReadChannel()
  Called By:     Current_Calibrate()、Current_UpdateFiltered()
  Input:         无
  Output:        无
  Return:        原始 ADC 码（0~4095）
  Others:        返回 0 不代表失败，必须再用 ADC1_LastReadOk() 判断本次转换是否有效。
-----------------------------------------------------------*/
static uint16_t Current_ReadRaw(void)
{
    return ADC1_ReadChannel(ADC_Channel_5);  /* 读 PA5（电流通道） */
}

/*-----------------------------------------------------------
  Function:      Current_Calibrate
  Description:   停机状态下校准 ACS712 零点：先停转并等待 200ms 使电流归零，
                 再采样 64 次求平均并折算成零点电压，最后做 2.10~2.90V 合理性检查。
  Calls:         Motor_Config()、Delay_ms()、Current_ReadRaw()、ADC1_LastReadOk()
  Called By:     main.c 上电初始化阶段（只调用一次）
  Input:         无
  Output:        无
  Return:        校准后的零点电压(V)；失败时返回上一次的零点值
  Others:        1) 单次采样最多重试 3 次，全部失败则记录 ADC 寄存器现场并置 valid=0；
                                  2) 零点超出 2.10~2.90V 说明传感器未接或供电异常，同样置 valid=0；
                                  3) 校准成功会同时清空滑动平均缓冲。
-----------------------------------------------------------*/
float Current_Calibrate(void)
{
    uint32_t sum = 0U;  /* 64 次采样累加和 */
    uint8_t i;  /* 采样循环下标 */

    Motor_Config(0U, 1U);  /* 先确保电机停转（零点校准的前提） */
    Delay_ms(200U);  /* 等电流真正归零 */

    /* 采样 64 次（ACS712_ADC_AVG），每次之间等 2ms —— 平均能压掉随机噪声，
     * 间隔能让 ADC 采样保持电路充分充电（连续猛读反而会互相干扰）。 */
    for (i = 0U; i < ACS712_ADC_AVG; i++)  /* 共采 64 次求平均 */
    {
        uint8_t retry;  /* 单次采样的重试计数 */
        uint8_t sample_ok = 0U;  /* 本次采样是否成功 */
        uint16_t raw = 0U;  /* 本次原始码 */

        /* 单次采样最多重试 3 次：ADC 偶尔会有一次转换超时（尤其刚上电、
         * 和 OLED/MPU 的软件 I2C 抢总线的时候），重试比直接判失败更实际。 */
        for (retry = 0U; retry < 3U; retry++)  /* 最多重试 3 次 */
        {
            raw = Current_ReadRaw();  /* 读一次 */
            if (ADC1_LastReadOk() != 0U)  /* 转换成功 */
            {
                sample_ok = 1U;  /* 标记成功 */
                break;  /* 不必再重试 */
            }
            Delay_ms(1U);  /* 失败后稍等再试 */
        }

        /* 3 次都没成功：把 ADC 的三个关键寄存器现场抄下来再退出。
         * 这是给「上板调试」留的证据 —— 事后用调试器读 g_i_adc_sr / cr2 / sqr3，
         * 就能判断是 ADON 没打开、通道号选错、还是 EOC 一直不置位，
         * 比只看一个「失败」标志有用得多。 */
        if (sample_ok == 0U)  /* 3 次都没成功 */
        {
            g_i_adc_sr = ADC1->SR;  /* 抄下 SR 现场 */
            g_i_adc_cr2 = ADC1->CR2;  /* 抄下 CR2 现场 */
            g_i_adc_sqr3 = ADC1->SQR3;  /* 抄下 SQR3 现场 */
            g_i_fail_reason = 1U;  /* 失败原因：ADC 转换超时 */
            g_i_valid = 0U;  /* 数据置为不可信 */
            return g_i_zero;  /* 沿用上一次的零点值 */
        }

        sum += raw;  /* 累加本次结果 */
        Delay_ms(2U);  /* 两次采样之间留出保持电路充电时间 */
    }

    /* 平均码 -> 电压：电压 = 平均码 / 4095 * 3.3V
     * 依据：12 位 ADC 满量程 4095 对应参考电压 3.3V（VDDA）。
     * 此时算出来的就是「零电流时的传感器输出电压」，即新的零点。 */
    g_i_zero = ((float)sum / (float)ACS712_ADC_AVG) / ACS712_ADC_FULL * ACS712_VREF;  /* 平均码 -> 零点电压 */
    /* 【零点合理性检查】合理范围 2.10V ~ 2.90V（围绕理论值 2.5V 留了余量）。
     * 落在范围外通常意味着：传感器没插好 / 没供 5V / 接线接到别的引脚上
     * （PA5 悬空时会读到随机的漂移值）。
     * 这时宁可不给数据（valid=0，界面显示 --），也不能拿一个错的零点去算电流 ——
     * 错的零点会让所有电流值整体偏移，屏显与遥测的读数全部失真。 */
    if ((g_i_zero < ACS712_ZERO_MIN) || (g_i_zero > ACS712_ZERO_MAX))  /* 零点超出 2.10~2.90V 合理范围 */
    {
        g_i_fail_reason = 2U;  /* 失败原因：零点越界（多为未接线） */
        g_i_valid = 0U;  /* 数据置为不可信 */
        return g_i_zero;  /* 返回越界的零点供现场排查 */
    }

    g_i_fail_reason = 0U;  /* 校准成功：清失败原因 */
    g_i_valid = 1U;  /* 数据可信 */
    g_i_raw_count = 0U;  /* 清空滑动平均缓冲（样本数） */
    g_i_raw_index = 0U;  /* 清空滑动平均缓冲（写指针） */
    return g_i_zero;  /* 返回零点电压 */
}

/*-----------------------------------------------------------
  Function:      Current_UpdateFiltered
  Description:   向 8 点滑动平均缓冲写入最新样本，并把平均值写入 g_i_raw_avg。
                 在原始 ADC 码上求平均，可保留零点附近的符号信息。
  Calls:         Current_ReadRaw()、ADC1_LastReadOk()
  Called By:     Current_GetAmp()
  Input:         无
  Output:        通过全局变量 g_i_raw_avg 输出平均后的原始 ADC 码
  Return:        1 = 采样成功且平均值已更新；0 = 采样失败
  Others:        样本数未满 8 时按已收集的样本数求平均，避免前几帧被 0 拉低。
-----------------------------------------------------------*/
static uint8_t Current_UpdateFiltered(void)
{
    uint16_t raw;  /* 本次原始码 */
    uint32_t sum = 0U;  /* 缓冲内样本求和 */
    uint8_t count;  /* 本次参与平均的样本数 */
    uint8_t i;  /* 循环下标 */

    /* 采样前按相位抖动等一小段：让相邻样本落在 PWM 周期的不同位置（见文件头注释） */
    Current_WaitCycles((uint32_t)g_i_dither * CURRENT_DITHER_STEP_CYCLES);
    g_i_dither++;  /* 相位前进一档 */
    if (g_i_dither >= CURRENT_DITHER_PHASES)  /* 走完一个完整 PWM 周期 */
    {
        g_i_dither = 0U;  /* 回到起点，开始下一轮 */
    }

    raw = Current_ReadRaw();  /* 读一次 PA5 */
    if (ADC1_LastReadOk() == 0U)  /* 转换失败 */
    {
        return 0U;  /* 不更新平均值 */
    }

    /* 【写环形缓冲 + 求平均】三步：
     *   ① 新样本写进 index 位置，index 前移并回绕（模 8）
     *   ② count 在没满 8 之前逐次 +1 —— 所以上电头几十毫秒用的是
     *      「已经收集到的样本数」而不是硬凑 8 个，避免拿一堆 0 去平均
     *      （那样前几帧会显示成 0A，看着像传感器坏了）
     *   ③ 对 [0, count) 求和再除以 count
     * 【为什么可以直接从 0 遍历而不管环形顺序】这里只求和，加法与顺序无关，
     * 所以不需要按环形顺序绕一圈。 */
    g_i_raw_history[g_i_raw_index] = raw;  /* 新样本写入环形缓冲 */
    g_i_raw_index = (uint8_t)((g_i_raw_index + 1U) % CURRENT_FILTER_WINDOW);  /* 写指针前移并回绕（模 8） */
    if (g_i_raw_count < CURRENT_FILTER_WINDOW)  /* 样本还没攒满 8 个 */
    {
        g_i_raw_count = (uint8_t)(g_i_raw_count + 1U);  /* 有效样本数加一 */
    }

    count = g_i_raw_count;  /* 取本次实际参与平均的个数 */
    for (i = 0U; i < count; i++)  /* 只对有效样本求和 */
    {
        sum = (uint32_t)(sum + g_i_raw_history[i]);  /* 累加 */
    }

    g_i_raw_avg = (uint16_t)(sum / (uint32_t)count);  /* 整数平均后写回 */
    return 1U;  /* 更新成功 */
}

/*-----------------------------------------------------------
  Function:      Current_GetAmp
  Description:   返回当前电机电流(A)。内部先更新滑动平均，再按
                 I = (Vout - 零点) / 0.185 换算。
  Calls:         Current_UpdateFiltered()
  Called By:     main.c 主循环（每 10ms 一次）
  Input:         无
  Output:        无
  Return:        电流值(A)，可正可负；数据不可信时返回 0.0f
  Others:        采样失败会把 g_i_valid 清零（此后界面显示 --），需重新校准才恢复；
-----------------------------------------------------------*/
float Current_GetAmp(void)
{
    float v;  /* 平均码折算出的电压 */

    /* 采样期间不要反复切换 PA5 的内部上下拉：ACS712 是有源驱动的模拟源，
     * 判定「数据可信」的可靠依据是「转换成功」加上「零点已校准」。 */
    if (g_i_valid == 0U)  /* 未校准或已失效 */
    {
        return 0.0f;  /* 直接返回 0，不发数据 */
    }

    if (Current_UpdateFiltered() == 0U)  /* 本次采样失败 */
    {
        g_i_valid = 0U;  /* 连数据也一并置为不可信 */
        return 0.0f;  /* 返回 0 */
    }

    v = (float)g_i_raw_avg / ACS712_ADC_FULL * ACS712_VREF;  /* 平均码 -> 电压 */
    /* 最终换算：电流 = (实测电压 - 零点电压) / 0.185 V/A
     * 结果可正可负：正=电机在耗电，负=电流方向相反（或零点漂了一点点，
     * 这正是上面要做 8 点平均 + 停机跟踪零点的原因）。 */
    return (v - g_i_zero) / ACS712_SENSITIVITY;  /* (Vout - 零点) / 0.185 = 电流(A) */
}

/*-----------------------------------------------------------
  Function:      Current_TrackZero
  Description:   停机状态下用一阶低通修正零点：zero += 0.02 x (v - zero)，
                 用于抵消 ACS712 的零点温漂（可达 VCC 的 1.5%，约 200mA）。
  Calls:         无
  Called By:     main.c 主循环中「电机确认停转（duty==0 且转速<20）」时调用
  Input:         无
  Output:        修改全局零点 g_i_zero
  Return:        无
  Others:        时间常数约 0.5s（1/0.02 x 10ms）；当前值超出 2.10~2.90V 时不跟踪，
-----------------------------------------------------------*/
void Current_TrackZero(void)
{
    float v;  /* 平均码折算出的电压 */

    if (g_i_valid == 0U)  /* 数据不可信时不跟踪 */
    {
        return;  /* 直接返回 */
    }

    v = (float)g_i_raw_avg / ACS712_ADC_FULL * ACS712_VREF;  /* 平均码 -> 电压 */
    if ((v < ACS712_ZERO_MIN) || (v > ACS712_ZERO_MAX))  /* 越界值不参与跟踪 */
    {
        return;  /* 直接返回 */
    }

    g_i_zero += CURRENT_ZERO_TRACK_ALPHA * (v - g_i_zero);  /* 一阶低通：向当前值缓慢靠拢 */
}

/*-----------------------------------------------------------
  Function:      Current_IsValid
  Description:   查询电流数据当前是否可信。
  Calls:         无
  Called By:     main.c（决定显示数值还是 --）、telemetry.c（决定是否置有效位）
  Input:         无
  Output:        无
  Return:        1 = 传感器存在且最近一次读数可用；0 = 不可信
  Others:        无
-----------------------------------------------------------*/
uint8_t Current_IsValid(void)
{
    return g_i_valid;  /* 返回有效性标志 */
}
