/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    feature_r.c
  Author:      汪贤云
  Version:     1.0
  Date:        2026-10-19
  Description: 转速单变量特征 R 的实现。
               核心是两级归一化（缺一不可）：
               R = 转速(rpm) / 母线电压(V) x (基准占空比 - 死区) / (当前占空比 - 死区)
               第一级除母线电压，除掉电量差异；第二级乘膝点比，除掉档位差异。
               模块的作用是把 R 按 100 ms 节拍攒成 32 点定长序列，
               交给板上 AI 做「正常分布」的学习与相似度比对。
               另维护一路滚动峰峰值 dR（窗内 rpm/V 的绝对跳变幅度），
               供固件做显式阈值判决，与模型相似度并联。
  Function List:
    1. feature_r_reset()          清空窗口
    2. feature_r_tick()           阶梯推点（失效时丢窗）
    3. feature_r_get()            取出 32 点序列
    4. feature_r_current()        取最近一次 R
    5. feature_r_current_valid()  取最近一次 R 是否有效
    6. feature_r_fill()           取窗口已攒点数
    7. feature_r_ripple_abs()     取窗内 rpm/V 绝对峰峰值（跳变幅度 dR）
  History:
    <author>   <time>      <version>   <desc>
    汪贤云     2026-10-19    1.0         创建文件，转速单变量方案
    汪贤云     2026-10-21    1.1         改膝点归一化(除 duty-50 而非 duty)；
                                           58%/64% 双档；新增 dR 绝对峰峰值
    汪贤云     2026-10-22    1.2         订正顶部注释与实现不符之处
    汪贤云     2026-10-22    1.3         新增起动爬升期不推点：起动期转速快速上升，
                                           峰峰值高达 9.5 会把模型相似度打到 0，
                                           实测起动后 +4.05~+4.54 s 误报 0.5 s；
                                           只拦正向上升(门限 2%)，不拦下降(捏轴照常检出)
***********************************************************/

#include "feature_r.h"  /* 本模块接口与窗口长度定义 */

/* ==========================================================================
 * 为什么是 R = rpm / vbat，而不是直接用 rpm
 * --------------------------------------------------------------------------
 * 实测 9 批数据（【单变量方案期】58% 单档、电池 11.8~12.4 V）：
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
 *
 * 为什么还要再乘一个档位比（膝点归一化）：
 *   空载下 rpm 近似正比于「占空比 x 母线电压」，上面只除了母线电压，
 *   所以 R 仍然正比于占空比。乘 (基准档位 - 死区) / (当前档位 - 死区)
 *   就能把任意档位折算回基准档，模型在基准档学一次即可跨档位使用。
 *
 *   【为什么分母是 (duty - 50) 而不是 duty】本工程实测标定曲线为
 *     rpm = 17.5 x (duty - 50)，死区正好在 50%：
 *       58% -> 140 rpm、64% -> 245 rpm（实测 245~262，58% 以上略上翘）、
 *       66% -> 280 rpm。
 *   即 rpm/vbat 对 (duty - 50) 才是线性的，而不是对 duty。
 *   若按 (REF / duty) 归一，等于假设 R 正比于 duty，档位越高偏得越远。
 *   按老式 (58 / duty) 实测三档残差达 61%，64% 档必然误报；
 *   改成膝点归一化后残差收到 6.3%，仍大于模型 ±1% 带宽，
 *   所以跨档并用必须用两档数据重训模型（68 号 ICM 即如此训练）。
 *   真正跨档通用、不需按档位标定系数的是 dR 判据（跨档只差 0.035 rpm/V）。
 *
 *   【为什么归一化后仍是两位数】58% 档 R = 11.79、64% 档 R = 11.79
 *   （两档完全一致，正是膝点归一化的目的）；电池放到 10.7 V 时 R = 13.08。
 *   所以屏显 R 必须给两位整数位，OLED 上 int_len 传 1 会把十位整位丢掉
 *   （11.79 显示成 "1.7"），见 ui.c 中 2026-10-22 的修正说明。
 *
 *   低占空比段存在启动死区与强非线性，R 没有物理意义，所以
 *   duty <= FEATURE_R_DUTY_KNEE（50%）时直接判无效并丢窗，别喂给模型。
 * ========================================================================== */

/* 窗口缓冲：32 个 float，按时间顺序存放（下标 0 最旧、末尾最新）。
 * 顺序不能反 —— NanoEdge Studio 训练时的样本顺序就是「先旧后新」。 */
static float    s_win[FEATURE_R_WINDOW];

static uint8_t  s_fill = 0U;        /* 已攒点数，满窗后恒等于 FEATURE_R_WINDOW */
static float    s_lastR = 0.0f;     /* 最近一次算出的 R，供屏显 */
static uint8_t  s_lastRValid = 0U;  /* 最近一次 R 是否有效（0 = 电机没转或电压通路坏） */
static uint32_t s_lastPushMs = 0U;  /* 上一次推点的毫秒时基 */
/* ---- 2026-10-21 新增：跳变幅度统计 ----
 * 绝对 R 值随档位变（54% 档约 6.3，68% 档约 21.8），所以绝对阈值不可跨档位用；
 * 但「窗口内 R 的相对峰峰值」只反映负载在窗口里有没有被反复捏放，
 * 与档位无关，这才是导师要的那个判据。 */
static float    s_ripple = 0.0f;       /* 窗口相对峰峰值(%)，跳变幅度 */
static uint8_t  s_rippleValid = 0U;    /* 1 = 窗口已满，幅度可用 */
static float    s_winDuty = 0.0f;      /* 本窗推点时的占空比(%)，用于把 R 还原成 rpm/V */
static float    s_rippleAbs = 0.0f;    /* 本窗 rpm/V 的绝对峰峰值，与档位无关 */

/* ---- 2026-10-21 新增：喂 AI 的滚动峰峰值窗口 ----
 * 为什么不直接用 s_win：新库是在 3.2 s 滚动峰峰值上训练的，
 * 峰峰值不会被库内部的平均池化抹平。
 * q = rpm / vbat 在原始空间计算，不做膝点归一化：
 * 峰峰值与转速不成正比（实测 58% 档 0.103、64% 档 0.120），
 * 乘上膝点比反而会把 64% 档小掉 0.571 倍，差距变大。 */
static float    s_qring[FEATURE_R_RIPPLE_SPAN];  /* 最近 8 个 q = rpm/vbat 的环 */
static uint8_t  s_qidx = 0U;        /* 环的写入位置 */
static uint8_t  s_qcnt = 0U;        /* 环内有效个数，满 8 才能算峰峰值 */
static float    s_rwin[FEATURE_R_WINDOW];        /* 峰峰值窗口，直接喂模型 */
static uint8_t  s_rfill = 0U;       /* 峰峰值窗口已填个数 */

/* 上一拍的转速（2026-10-22 新增）：用于识别起动爬升期。
 * 判据是「相对上一拍的正向上升率」，只拦上升不拦下降，
 * 免得把捏轴这种真正要检的下降异常也屏蔽掉。
 * 0 表示「没有上一拍可比」，此时不做上升率判定。 */
static float    s_prevRpm = 0.0f;


/*-----------------------------------------------------------
  Function:      feature_r_calc_ripple
  Description:   重算窗口内 R 的相对峰峰值 = (max - min) / mean x 100
  Calls:         无
  Called By:     feature_r_tick()
  Input:         无（直接读 s_win / s_fill）
  Output:        写 s_ripple / s_rippleValid
  Return:        无
  Others:        窗口没满时不置有效位，避免把攒窗阶段当异常。
-----------------------------------------------------------*/
static void feature_r_calc_ripple(void)
{
    float    vmin;      /* 窗口最小值 */
    float    vmax;      /* 窗口最大值 */
    float    vsum = 0.0f;  /* 窗口累加和 */
    float    vavg;      /* 窗口均值 */
    uint8_t  i;         /* 循环下标 */

    if (s_fill < FEATURE_R_WINDOW)   /* 窗口没满，先不给结论 */
    {
        s_rippleValid = 0U;
        return;
    }
    vmin = s_win[0];
    vmax = s_win[0];
    for (i = 0U; i < FEATURE_R_WINDOW; i++)
    {
        if (s_win[i] < vmin)
        {
            vmin = s_win[i];
        }
        if (s_win[i] > vmax)
        {
            vmax = s_win[i];
        }
        vsum += s_win[i];
    }
    vavg = vsum / (float)FEATURE_R_WINDOW;   /* 窗口均值 */
    if (vavg > 0.0f)
    {
        s_ripple = ((vmax - vmin) / vavg) * 100.0f;   /* 相对峰峰值(%) */
        /* 绝对峰峰值：先按占空比还原成 rpm/V（不带档位归一化）。
         * 为什么用绝对值不用百分比：捏的力恒定时负载力矩恒定，
         * 转速降 d(omega) = dT x Ra / Ke^2 与工作点无关，
         * 实测 58% 档跌 30 rpm、64% 档跌 25 rpm，绝对值几乎相同，
         * 而相对降幅是 21% 与 10%，差了一倍 —— 所以百分比不能跨档位用。 */
        s_rippleAbs = ((vmax - vmin) * (s_winDuty - FEATURE_R_DUTY_KNEE)) /
                      (FEATURE_R_DUTY_REF - FEATURE_R_DUTY_KNEE);

        s_rippleValid = 1U;
    }
    else                        /* 均值异常，判为不可用 */
    {
        s_ripple = 0.0f;
        s_rippleValid = 0U;
    }
}


/*-----------------------------------------------------------
  Function:      feature_r_push_ripple
  Description:   把当前 q = rpm/vbat 压入 8 点环，环满后算出峰峰值
                 并压入 32 点滚动窗口。
  Calls:         无
  Called By:     feature_r_tick()，每 100 ms 调一次
  Input:         rpm / vbat_v 当前转速与母线电压
  Output:        写 s_qring / s_rwin
  Return:        无
  Others:        需要 8 + 32 - 1 = 39 次采样才出第一个完整窗口，
                 与离线 export_ripple.py 「先在整段上算 r，再滑窗」
                 完全一致，保证固件喂进去的形状和训练时相同。
-----------------------------------------------------------*/
static void feature_r_push_ripple(float rpm, float vbat_v)
{
    float   qmin;   /* 环内最小值 */
    float   qmax;   /* 环内最大值 */
    float   ptp;    /* 当前点的滚动峰峰值 */
    uint8_t k;      /* 环遍历下标 */
    uint8_t i;      /* 窗口搬移下标 */

    s_qring[s_qidx] = rpm / vbat_v;   /* 入环 */
    s_qidx++;
    if (s_qidx >= FEATURE_R_RIPPLE_SPAN)
    {
        s_qidx = 0U;                  /* 环回绕 */
    }
    if (s_qcnt < FEATURE_R_RIPPLE_SPAN)
    {
        s_qcnt++;                     /* 还没满就只能数个数 */
    }
    if (s_qcnt < FEATURE_R_RIPPLE_SPAN)
    {
        return;                       /* 环没满，还算不出峰峰值 */
    }

    qmin = s_qring[0];
    qmax = s_qring[0];
    for (k = 1U; k < FEATURE_R_RIPPLE_SPAN; k++)
    {
        if (s_qring[k] < qmin)
        {
            qmin = s_qring[k];
        }
        if (s_qring[k] > qmax)
        {
            qmax = s_qring[k];
        }
    }
    ptp = qmax - qmin;

    if (s_rfill < FEATURE_R_WINDOW)   /* 窗口还没满：直接追加 */
    {
        s_rwin[s_rfill] = ptp;
        s_rfill++;
    }
    else                              /* 满了：整体前移一位，新点落尾 */
    {
        for (i = 0U; (i + 1U) < FEATURE_R_WINDOW; i++)
        {
            s_rwin[i] = s_rwin[i + 1U];
        }
        s_rwin[FEATURE_R_WINDOW - 1U] = ptp;
    }
}

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
                  峰峰值那条链（8 点小环 + 32 点窗口）也必须一起清，
                  否则停机再起转时窗口里混着停机前的峰峰值。
-----------------------------------------------------------*/

void feature_r_reset(void)
{
    uint8_t i;  /* 循环下标 */

    for (i = 0U; i < FEATURE_R_WINDOW; i++)  /* 逐个清零 R 电平窗口 */
    {
        s_win[i] = 0.0f;
    }
    s_fill = 0U;  /* 窗口回到未满状态 */
    s_ripple = 0.0f;      /* 跳变幅度清零 */
    s_rippleValid = 0U;   /* 幅度判为不可用 */

    for (i = 0U; i < FEATURE_R_RIPPLE_SPAN; i++)  /* 清零峰峰值小环 */
    {
        s_qring[i] = 0.0f;
    }
    s_qidx = 0U;    /* 环写指针归零 */
    s_qcnt = 0U;    /* 环内有效点数归零 */
    for (i = 0U; i < FEATURE_R_WINDOW; i++)  /* 清零峰峰值窗口 */
    {
        s_rwin[i] = 0.0f;
    }
    s_rfill = 0U;   /* 峰峰值窗口回到未满 */
    /* 【2026-10-22】不动 s_prevRpm：清窗只是丢掉已攒的点，
     * 上一拍转速这个「参照系」要继续留着，
     * 否则停机再起转的第一拍会因为没有参照而被当成爬升期。
     * 真正需要清它的是「彻底停机」那条路径（见 feature_r_tick 第一步）。 */
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
                 duty_pct   当前占空比百分数（0~100），取自 DUTY_LEVEL[level]
                 now_ms     毫秒时基，取自 Encoder_GetMs()
  Output:        无（内部窗口与屏显值被更新）
  Return:        1 = 本次真的推入了一个新点；0 = 没到节拍或被拒收
  Others:        时间判断用无符号减法，天然抗回绕。
-----------------------------------------------------------*/

uint8_t feature_r_tick(float rpm, float vbat_v, uint8_t vbat_valid,
                       float duty_pct, uint32_t now_ms)
{
    float r;  /* 本次算出的 R */

    /* ---- 第一步：判当前是否处于「有效运行状态」 ----
     * 停转、分压通路失效、电压过低时，R 没有物理意义
     * （分母不可信或分子为 0）。这时不能只是「不推点」，
     * 还必须丢掉已经攒下的点 —— 否则重新转起来时，
     * 窗口里混着停机前的数据，模型算出的相似度就是错的。 */
    if ((vbat_valid == 0U) || (vbat_v < FEATURE_R_VBAT_MIN) ||  /* 电压不可用 */
        (rpm < FEATURE_R_RPM_MIN) ||                            /* 或电机没转起来 */
        (duty_pct < FEATURE_R_DUTY_MIN) ||                      /* 低于下限 */
        (duty_pct <= FEATURE_R_DUTY_KNEE))                       /* 落在死区，R 无意义 */
    {
        if (s_fill != 0U)  /* 窗口里还有旧数据 */
        {
            feature_r_reset();  /* 状态离开有效区间：丢窗 */
        }
        s_lastRValid = 0U;  /* 屏上改显示无效标记 */
        s_lastPushMs = now_ms;  /* 重新对齐节拍，避免恢复后补推 */
        s_prevRpm = 0.0f;  /* 清上一拍转速，下一次起转从零算上升率 */
        return 0U;
    }

    /* ---- 第一步之二：起动爬升期不推点（2026-10-22 新增）----
     * 【要解决的问题】电机从停机起转时转速是「爬」上去的，
     * 这段爬升期的 q = rpm/vbat 峰峰值高达 9.5，
     * 进了 32 点窗口就把模型相似度打到 0 ——
     * 2026-10-22 实测起动后 +4.05 ~ +4.54 s 会误报一次 0.5 s。
     * 答辩现场「按 K1 就报警」是致命的。
     *
     * 【为什么只拦正向】起动是转速快速**上升**；负载突变（捏轴）是**下降**，
     * 后者正是要检出来的异常。用「变化率绝对值」会把捏轴一起屏蔽掉。
     *
     * 【为什么用相对上升率而不是绝对转速差】
     * 空载与带载的稳态转速能差几十 rpm，绝对值分不出「爬升」和「负载变化」；
     * 相对上升率与工作点无关，只看「在不在快速往上走」。
     *
     * 【爬升期也要丢窗而不是只不推点】起动时窗口里可能还留着上一次
     * 工况的稳态点，混着爬升点一样会污染模型，所以一并清掉。 */
    if ((s_prevRpm > 0.0f) &&
        (((rpm - s_prevRpm) / s_prevRpm) * 100.0f > FEATURE_R_START_RISE_PCT))
    {
        if (s_fill != 0U)  /* 窗口里还有旧数据 */
        {
            feature_r_reset();  /* 爬升期：丢窗，保证窗口里只有稳态样本 */
        }
        s_lastRValid = 0U;         /* 爬升期的 R 没有参考价值，屏上不显示 */
        s_lastPushMs = now_ms;     /* 重新对齐节拍 */
        s_prevRpm = rpm;           /* 记住本拍转速，下一拍继续判是否还在爬 */
        return 0U;
    }
    s_prevRpm = rpm;  /* 记下本拍转速，供下一拍算上升率 */

    /* ---- 第二步：算 R 并更新屏显用的瞬时值 ---- */
    /* 两步归一化，缺一不可：
     *   除母线电压 -> 把「电量」除掉（低电量时转速下降不再算异常）
     *   乘膝点比 -> 把「占空比」除掉（换档位时 R 不再整体平移）。
 *                因为空载转速对占空比是线性的：rpm/vbat = 1.72 x (duty - 50)，
 *                所以除的不是 duty 而是 (duty - 50)。
     * 折算后 R 才只反映电机自身的负载状态。 */
    r = (rpm / vbat_v) * ((FEATURE_R_DUTY_REF - FEATURE_R_DUTY_KNEE) /
                                (duty_pct - FEATURE_R_DUTY_KNEE));   /* 电量 + 档位双重归一化 */
    s_lastR = r;
    s_lastRValid = 1U;

    /* ---- 第三步：按 100 ms 节拍推点 ---- */
    if ((uint32_t)(now_ms - s_lastPushMs) < FEATURE_R_PUSH_MS)  /* 还没到节拍 */
    {
        return 0U;
    }
    s_lastPushMs = now_ms;  /* 对齐到当前时刻，不累加、不补推 */
    s_winDuty = duty_pct;   /* 记下本窗的占空比，供绝对峰峰值还原 rpm/V */
    feature_r_push_ripple(rpm, vbat_v);  /* 同一个 100 ms 节拍推峰峰值 */


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
    feature_r_calc_ripple();   /* 每推进一个点就重算一次跳变幅度 */

    return 1U;
}

/*-----------------------------------------------------------
  Function:      feature_r_get
  Description:   把窗口内的 32 点滚动峰峰值按时间顺序拷贝出去。
  Calls:         无
  Called By:     ai_monitor 层（接新模型后）；上位机调试时也可调用
  Input:         capacity —— output 能容纳的元素个数（不是字节）
  Output:        output —— 32 个 float，下标 0 最旧、下标 31 最新
  Return:        1 = 窗口已满且已写出；0 = 参数不合法或窗口未满
  Others:        本窗口内存的是滚动峰峰值，需要 8 + 32 - 1 = 39 次
                 100 ms 采样（约 3.9 s）才出第一个完整窗口，
                 与训练数据的构造方式一致。
                 窗口未满时不交半窗数据，且失败时不动 output。
-----------------------------------------------------------*/

uint8_t feature_r_get(float *output, size_t capacity)
{
    uint8_t i;  /* 循环下标 */

    if ((output == 0) || (capacity < (size_t)FEATURE_R_WINDOW))  /* 参数校验 */
    {
        return 0U;
    }
    if (s_rfill < FEATURE_R_WINDOW)  /* 窗口未满 */
    {
        return 0U;
    }
    for (i = 0U; i < FEATURE_R_WINDOW; i++)  /* 按时间顺序拷贝 */
    {
        output[i] = s_rwin[i];
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
  Description:   取峰峰值窗口已攒点数，用于判断还要多久才能送 AI。
  Calls:         无
  Called By:     main.c（填 UI_Data.r_fill）；调试时看窗口进度
  Input:         无
  Output:        无
  Return:        0 ~ FEATURE_R_WINDOW
  Others:        无
-----------------------------------------------------------*/

uint8_t feature_r_fill(void)
{
    return s_rfill;
}


/*-----------------------------------------------------------
  Function:      feature_r_ripple
  Description:   取窗口内 R 的相对峰峰值，即「跳变幅度」。
  Calls:         无
  Called By:     main.c 的跳变判据
  Input:         无
  Output:        无
  Return:        幅度(%)。窗口未满时返回 0，需配合 valid 使用。
  Others:        与绝对 R 值无关，因此换档位后同样可用 —— 这是它的价值所在。
-----------------------------------------------------------*/
float feature_r_ripple(void)
{
    return s_ripple;
}

/*-----------------------------------------------------------
  Function:      feature_r_ripple_valid
  Description:   询问跳变幅度是否已经可用（窗口是否攒满）。
  Calls:         无
  Called By:     main.c 的跳变判据
  Input:         无
  Output:        无
  Return:        1 = 可用；0 = 窗口没满
  Others:        无
-----------------------------------------------------------*/
uint8_t feature_r_ripple_valid(void)
{
    return s_rippleValid;
}


/*-----------------------------------------------------------
  Function:      feature_r_ripple_abs
  Description:   取窗口内 rpm/V 的绝对峰峰值，即「绝对跳变幅度」。
  Calls:         无
  Called By:     main.c 的跳变判据（主判据）
  Input:         无
  Output:        无
  Return:        幅度，单位 rpm/V。窗口未满时返回 0。
  Others:        这个量与档位无关：恒定捏力下转速降是绝对量，
                 实测 58% 档 2.53、64% 档 2.11，稳定运行只有 0.3~0.6。
                 所以它才是能跨档位通用的那个判据。
-----------------------------------------------------------*/
float feature_r_ripple_abs(void)
{
    return s_rippleAbs;
}
