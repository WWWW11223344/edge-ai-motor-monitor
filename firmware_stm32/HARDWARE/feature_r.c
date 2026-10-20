/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    feature_r.c
  Author:      汪贤云
  Version:     1.0
  Date:        2026-10-19
  Description: 转速单变量特征 R 的实现。
               核心只有一个除法：R = 转速(rpm) / 母线电压(V)。
               模块的作用是把 R 按 100 ms 节拍攒成 32 点定长序列，
               交给板上 AI 做「正常分布」的学习与相似度比对。
  Function List:
    1. feature_r_reset()          清空窗口
    2. feature_r_tick()           阶梯推点（失效时丢窗）
    3. feature_r_get()            取出 32 点序列
    4. feature_r_current()        取最近一次 R
    5. feature_r_current_valid()  取最近一次 R 是否有效
    6. feature_r_fill()           取窗口已攒点数
  History:
    <author>   <time>      <version>   <desc>
    汪贤云     2026-10-19    1.0         创建文件，转速单变量方案
***********************************************************/

#include "feature_r.h"  /* 本模块接口与窗口长度定义 */

/* ==========================================================================
 * 为什么是 R = rpm / vbat，而不是直接用 rpm
 * --------------------------------------------------------------------------
 * 实测 9 批数据：
 *   正常 6 批 R 落在 13.23 ~ 13.53（窗内标准差 0.051 ~ 0.064）；
 *   异常 3 批 R 落在 10.79 ~ 11.07。
 *   拿 12.0 当阈值：正常 2029 窗 0.00% 误报，异常 1113 窗 100% 检出，
 *   低电量批（9.85 V）也是 0% 误报。可用阈值区间 11.6 ~ 12.2。
 *
 * 如果不除以电压：
 *   裸转速 147.2 -> 121.8（只掉 17.2%），而 3S 锂电从满电 12.6 V
 *   掉到 11.3 V 本身就会让转速掉 11%。两者在裸转速上完全重叠，
 *   低电量时必然误报。除以电压之后电量变化被约掉，
 *   剩下的才是电机本身的状态。
 * ========================================================================== */

/* 窗口缓冲：32 个 float，按时间顺序存放（下标 0 最旧、末尾最新）。
 * 顺序不能反 —— NanoEdge Studio 训练时的样本顺序就是「先旧后新」。 */
static float    s_win[FEATURE_R_WINDOW];

static uint8_t  s_fill = 0U;        /* 已攒点数，满窗后恒等于 FEATURE_R_WINDOW */
static float    s_lastR = 0.0f;     /* 最近一次算出的 R，供屏显 */
static uint8_t  s_lastRValid = 0U;  /* 最近一次 R 是否有效（0 = 电机没转或电压通路坏） */
static uint32_t s_lastPushMs = 0U;  /* 上一次推点的毫秒时基 */

/*-----------------------------------------------------------
  Function:      feature_r_reset
  Description:   清空窗口与已攒点数，使窗口回到「未满」状态。
  Calls:         无
  Called By:     feature_r_tick()（状态离开有效运行区间时）、上电初始化
  Input:         无
  Output:        无
  Return:        无
  Others:        不重置 s_lastPushMs，避免清窗后立刻补推一堆重复点；
                 也不清 s_lastR，屏上数值不会跳成 0 看着像坏了。
-----------------------------------------------------------*/

void feature_r_reset(void)
{
    uint8_t i;  /* 循环下标 */

    for (i = 0U; i < FEATURE_R_WINDOW; i++)  /* 逐个清零窗口 */
    {
        s_win[i] = 0.0f;
    }
    s_fill = 0U;  /* 窗口回到未满状态 */
}

/*-----------------------------------------------------------
  Function:      feature_r_tick
  Description:   每 10 ms 由主循环调一次。先判当前是否处于有效运行状态，
                 再按 100 ms 节拍计算 R 并推入窗口。
  Calls:         feature_r_reset()
  Called By:     main.c 主循环 10 ms 传感器采样块
  Input:         rpm        当前转速（转/分），取自 Encoder_GetRpm()
                 vbat_v     当前母线电压（V），取自 Vbat_GetVolt()
                 vbat_valid 电压通路是否有效，取自 Vbat_IsValid()
                 now_ms     毫秒时基，取自 Encoder_GetMs()
  Output:        无（内部窗口与屏显值被更新）
  Return:        1 = 本次真的推入了一个新点；0 = 没到节拍或被拒收
  Others:        时间判断用无符号减法，天然抗回绕。
-----------------------------------------------------------*/

uint8_t feature_r_tick(float rpm, float vbat_v, uint8_t vbat_valid,
                       uint32_t now_ms)
{
    float r;  /* 本次算出的 R */

    /* ---- 第一步：判当前是否处于「有效运行状态」 ----
     * 停转、分压通路失效、电压过低时，R 没有物理意义
     * （分母不可信或分子为 0）。这时不能只是「不推点」，
     * 还必须丢掉已经攒下的点 —— 否则重新转起来时，
     * 窗口里混着停机前的数据，模型算出的相似度就是错的。 */
    if ((vbat_valid == 0U) || (vbat_v < FEATURE_R_VBAT_MIN) ||  /* 电压不可用 */
        (rpm < FEATURE_R_RPM_MIN))  /* 或电机没转起来 */
    {
        if (s_fill != 0U)  /* 窗口里还有旧数据 */
        {
            feature_r_reset();  /* 状态离开有效区间：丢窗 */
        }
        s_lastRValid = 0U;  /* 屏上改显示无效标记 */
        s_lastPushMs = now_ms;  /* 重新对齐节拍，避免恢复后补推 */
        return 0U;
    }

    /* ---- 第二步：算 R 并更新屏显用的瞬时值 ---- */
    r = rpm / vbat_v;   /* 转速归一化：把电量影响除掉 */
    s_lastR = r;
    s_lastRValid = 1U;

    /* ---- 第三步：按 100 ms 节拍推点 ---- */
    if ((uint32_t)(now_ms - s_lastPushMs) < FEATURE_R_PUSH_MS)  /* 还没到节拍 */
    {
        return 0U;
    }
    s_lastPushMs = now_ms;  /* 对齐到当前时刻，不累加、不补推 */

    if (s_fill < FEATURE_R_WINDOW)  /* 窗口还没满 */
    {
        s_win[s_fill] = r;  /* 直接追加到末尾，不用挪位 */
        s_fill++;           /* 已攒点数加一 */
    }
    else  /* 窗口已满：整体左移一位，丢掉最旧点 */
    {
        uint8_t i;  /* 循环下标 */

        for (i = 0U; (i + 1U) < FEATURE_R_WINDOW; i++)
        {
            s_win[i] = s_win[i + 1U];
        }
        s_win[FEATURE_R_WINDOW - 1U] = r;  /* 新点落到末尾 */
    }
    return 1U;
}

/*-----------------------------------------------------------
  Function:      feature_r_get
  Description:   把窗口内的 32 点 R 序列按时间顺序拷贝出去。
  Calls:         无
  Called By:     ai_monitor 层（接新模型后）；上位机调试时也可调用
  Input:         capacity —— output 能容纳的元素个数（不是字节）
  Output:        output —— 32 个 float，下标 0 最旧、下标 31 最新
  Return:        1 = 窗口已满且已写出；0 = 参数不合法或窗口未满
  Others:        窗口未满时不交半窗数据（它们前面还没有历史，
                 形状与训练时不一致），且失败时不动 output。
-----------------------------------------------------------*/

uint8_t feature_r_get(float *output, size_t capacity)
{
    uint8_t i;  /* 循环下标 */

    if ((output == 0) || (capacity < (size_t)FEATURE_R_WINDOW))  /* 参数校验 */
    {
        return 0U;
    }
    if (s_fill < FEATURE_R_WINDOW)  /* 窗口未满 */
    {
        return 0U;
    }
    for (i = 0U; i < FEATURE_R_WINDOW; i++)  /* 按时间顺序拷贝 */
    {
        output[i] = s_win[i];
    }
    return 1U;
}

/*-----------------------------------------------------------
  Function:      feature_r_current
  Description:   取最近一次算出的 R，供屏显使用。
  Calls:         无
  Called By:     main.c（填 UI_Data.r_ratio）
  Input:         无
  Output:        无
  Return:        最近一次 R（单位 rpm/V）；尚未有过有效值时为 0
  Others:        与 feature_r_current_valid() 配合使用：后者为 0 时本值不可信。
-----------------------------------------------------------*/

float feature_r_current(void)
{
    return s_lastR;
}

/*-----------------------------------------------------------
  Function:      feature_r_current_valid
  Description:   告诉调用方当前 R 是否处于可信的运行区间。
  Calls:         无
  Called By:     main.c（填 UI_Data.r_valid）
  Input:         无
  Output:        无
  Return:        1 = 可信；0 = 电机没转或电压通路失效
  Others:        无
-----------------------------------------------------------*/

uint8_t feature_r_current_valid(void)
{
    return s_lastRValid;
}

/*-----------------------------------------------------------
  Function:      feature_r_fill
  Description:   取窗口已攒点数，用于判断还要多久才能送 AI。
  Calls:         无
  Called By:     main.c（填 UI_Data.r_fill）；调试时看窗口进度
  Input:         无
  Output:        无
  Return:        0 ~ FEATURE_R_WINDOW
  Others:        无
-----------------------------------------------------------*/

uint8_t feature_r_fill(void)
{
    return s_fill;
}
