/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    ai_monitor.c
  Author:      汪贤云
  Version:     3.1
  Date:        2026-10-22
  Description: 板上 AI 模块：现场学习与异常检测
               （NanoEdge AI Studio Library ID 6ad8b466f4c74310b9bc146f / 模型族 ICM）。
               数据流：main.c 主循环每 100 ms 推一个 q = 转速 / 母线电压，
               feature_r 把它压成 8 点滚动峰峰值后调一次本模块；
               峰峰值序列攒满 32 点（约 3.9 s）后整窗喂给 ICM 模型；
               库自带预训练知识（learning_min_lines = 368 已在 PC 端消化），
               上电 init(true) 后先跑一段「启动现场自适应学习」，
               把本机工况样本增量补进知识库（屏上 LRN），学满转 DET。
  Function List:
    1. ai_monitor_init()             初始化模型库
    2. ai_monitor_tick()             每个 R 点调一次（核心）
    3. ai_monitor_set_enabled()      采集/部署模式切换
    4. 一组查询接口                  阶段、相似度、有效性、报警、学习进度
  History:
    <author>   <time>      <version>   <desc>
    汪贤云     2026-10-16    1.0         创建文件并按华为规范补充注释
    汪贤云     2026-10-19    2.0         换用单变量 R 的 32 点窗模型库（ZSM），
                                                去掉 64 帧窗口、19 维特征、四组拼接的旧流程，
                                                ai_monitor_feed_frame() 改为 ai_monitor_tick()
    汪贤云     2026-10-21    3.0         换用 68 号 ICM 库，输入改为 q 的 8 点滚动峰峰值
                                                （dR 序列）；改走上电即检测 + 增量补学路线
                                                取代「学满 64 窗再转检测」；阈值与判据不变
    汪贤云     2026-10-22    3.1         新增「启动现场自适应学习阶段」：上电先补学
                                                AI_MONITOR_SITE_LEARN_WINDOWS 窗再转检测，
                                                屏上 LRN -> DET 可见；补学带相似度门限
***********************************************************/
/* 【模型与输入】
 * 输入 1 个样本 x 32 点 = 32 个 float，这 32 点是 feature_r 算出来的
 * 「q = 转速 / 母线电压」的 8 点滚动峰峰值（跨度 0.8 s，见 feature_r.h）。
 * 模型族是 ICM（增量聚类），artifacts/icm_preprocessing_config.json 里写的是
 * absolute -> extract_features(95 分位 / 峰值 / 25 分位) -> interleaved，
 * 即先从 32 点里取三个序统计量，再做聚类距离判定。
 * 所以本文件不参与任何特征计算，只负责「取窗 + 喂模型 + 判报警」。
 * 【为什么换成峰峰值】上一代喂的是 R 电平本身，而电平会被库里的池化平均掉，
 * 捏轴只占窗口一小段时几乎看不出变化（同批数据实测只检出 58.9%）；
 * 改喂滚动峰峰值后同样数据检出 87.0%，重训后离线基准 92.0%。 */

#include "ai_monitor.h"        /* 本模块接口与常量声明 */
/* 【引用的头文件】
 * NanoEdgeAI.h  是模型库的接口与输入尺寸宏（NEAI_INPUT_*）；
 * feature_r.h   提供 R 窗口（FEATURE_R_WINDOW）与 feature_r_get()，
 *               它已经由 ai_monitor.h 引入，这里再写一次是为了让读代码的人
 *               不用跳回去查「底下到底依赖了什么」（头文件有防重复包含保护）。 */
#include "NanoEdgeAI.h"        /* 模型库接口 */
#include "feature_r.h"         /* R 窗口与取窗接口 */

/* 一个 NanoEdge 输入缓冲要装多少个 float：32 点 x 1 轴 = 32 个。
 * 新库的输入就是「一个 32 点的时序窗」，不再拆成多个样本。 */
#define AI_MONITOR_NEAI_IN_LEN  \
    ((size_t)NEAI_INPUT_SIGNAL_LENGTH * (size_t)NEAI_INPUT_AXIS_NUMBER)  /* 32 x 1 = 32 个 float */

/* 【编译期保险：用一个「长度为 -1 的数组」来卡尺寸】
 * 两边必须一样长：一边是模型库声明的输入长度，
 * 另一边是 feature_r 窗口长度。不一致就译不过，
 * 而不是等烧进板子、运行时越界读内存（那种错误极难查）。
 * 这是嵌入式里很常用的技巧：把只在运行期暴露的错误提前到编译期。 */
typedef char ai_monitor_win_len_check[  /* 编译期尺寸守卫：数组长度为 1 表示通过 */
    ((AI_MONITOR_NEAI_IN_LEN == (size_t)FEATURE_R_WINDOW) ? 1 : -1)];  /* 必须等于 32，否则此处直接报错 */

static float g_neai_in[AI_MONITOR_NEAI_IN_LEN];  /* 模型输入缓冲（32 个 float） */
static uint32_t g_learned;  /* 已完成的 learn 调用次数 */
static uint32_t g_detectCount;  /* 自上次重学以来的检测次数（仅相似度达标时累计） */
static AiMonitorPhase g_phase = AI_MONITOR_OFF;  /* 当前阶段 */
static uint8_t g_similarity;  /* 最近一次相似度（0~100） */
static uint8_t g_result_valid;  /* 最近一次检测结果是否有效 */
static uint8_t g_initialized;  /* 模型库是否初始化成功 */
static uint8_t g_alarm;  /* 事件级报警状态 */
static uint8_t g_lowRun;  /* 连续低于阈值的窗数 */
static uint8_t g_highRun;  /* 连续达标的窗数 */
static uint8_t g_enabled = 1U;  /* 1 = 启用板上 AI，0 = 只采集不跑模型 */
static uint8_t g_winEmpty;  /* 连续拿不到满窗的节拍数（判断电机是否停过） */
static uint8_t g_needLearn;  /* 1 = 模型库明确要求继续学，0 = 知识已够可以检测 */
static uint16_t g_learnLeft;  /* 本轮还要学多少个窗（库要求学习时用它兜底防死循环） */
static uint16_t g_siteDone;  /* 启动现场自适应学习已补学的窗数（0 ~ AI_MONITOR_SITE_LEARN_WINDOWS） */
static uint16_t g_siteSeen;  /* 补学阶段一共看过的满窗数（不管有没有补成功，用于超时兜底） */
static uint8_t g_siteActive;  /* 1 = 还处于启动现场自适应学习阶段，0 = 已转入检测 */

/*-----------------------------------------------------------
  Function:      ai_monitor_init
  Description:   初始化 NanoEdge 模型库，并把本模块的内部状态全部复位。
  Calls:         neai_anomalydetection_init()
  Called By:     main.c 上电初始化段，只调一次
  Input:         无
  Output:        无
  Return:        无（是否成功请用 ai_monitor_get_phase() 查）
  Others:        1) 先按 true 初始化，直接用库自带的预训练知识，
                    这样上电就能检测，不必先现场学 368 个窗；
                  2) 万一库里没有预训练知识，init(true) 会失败，
                    这时退回 init(false) 从零学，并置 g_needLearn；
                  3) 成功且不需要学则进入「启动现场自适应学习阶段」：
                    边检测边用 learn() 增量补学本机工况样本，补满
                    AI_MONITOR_SITE_LEARN_WINDOWS 窗后转入纯检测阶段。
                    【为什么要有这一段】原来上电直接进检测，屏上从开机起
                    就是 DET，看不到「学习」这个过程。补学阶段屏上显示 LRN，
                    补学量通过 ai_monitor_get_site_progress() 报给屏显。
-----------------------------------------------------------*/
void ai_monitor_init(void)
{
    enum neai_state state = neai_anomalydetection_init(true);  /* 先用库内预训练知识 */

    g_needLearn = 0U;  /* 默认认为知识已够 */
    g_learnLeft = 0U;  /* 学习配额清零 */
    g_siteDone = 0U;  /* 现场补学计数归零 */
    g_siteSeen = 0U;  /* 补学阶段看过的窗数归零 */
    g_siteActive = 0U;  /* 默认不在现场补学阶段 */

    if (state != NEAI_OK)  /* 库里没有预训练知识，退回从零学 */
    {
        state = neai_anomalydetection_init(false);  /* 清空知识，从头学 */
        g_needLearn = 1U;  /* 标记需要现场学习 */
        g_learnLeft = (uint16_t)AI_MONITOR_LEARN_MAX;  /* 给足上限 */
    }

    g_learned = 0U;  /* 学习次数归零 */
    g_detectCount = 0U;  /* 检测计数归零 */
    g_similarity = 0U;  /* 相似度归零 */
    g_result_valid = 0U;  /* 结果置为无效 */
    g_alarm = 0U;  /* 清报警 */
    g_winEmpty = 0U;  /* 空窗计数归零 */
    g_lowRun = 0U;  /* 清低分计数 */
    g_highRun = 0U;  /* 清高分计数 */
    g_initialized = (state == NEAI_OK) ? 1U : 0U;  /* 记下初始化是否成功 */
    if (g_initialized == 0U)  /* 初始化失败 */
    {
        g_phase = AI_MONITOR_ERROR;  /* 报错阶段 */
    }
    else if (g_needLearn != 0U)  /* 初始化成功但要从零学 */
    {
        g_phase = AI_MONITOR_LEARNING;  /* 进入学习阶段 */
    }
    else  /* 初始化成功且已有预训练知识：进入启动现场自适应学习阶段 */
    {
        g_siteActive = 1U;  /* 打开现场补学阶段，屏上先显示 LRN */
        g_phase = AI_MONITOR_LEARNING;  /* 阶段显示为学习中 */
    }
}

/*-----------------------------------------------------------
  Function:      ai_monitor_set_enabled
  Description:   打开或关闭板上 AI。关掉时只采集不跑模型，
                 适用于数据采集（避免模型抢占主循环时间）。
  Calls:         无
  Called By:     main.c（如需切换时）
  Input:         enabled  非 0 = 开启，0 = 关闭
  Output:        无
  Return:        无
  Others:        关闭不改变已有学习结果，重新打开后继续累积。
-----------------------------------------------------------*/
void ai_monitor_set_enabled(uint8_t enabled)
{
    g_enabled = (enabled != 0U) ? 1U : 0U;  /* 归一化成 0/1 再保存 */
}

/*-----------------------------------------------------------
  Function:      ai_monitor_get_phase
  Description:   查询当前 AI 阶段（未启用 / 学习中 / 检测中 / 错误）。
  Calls:         无
  Called By:     telemetry.c、main.c
  Input:         无
  Output:        无
  Return:        AiMonitorPhase 枚举值
  Others:        无
-----------------------------------------------------------*/
AiMonitorPhase ai_monitor_get_phase(void)
{
    return g_phase;  /* 返回当前阶段 */
}

/*-----------------------------------------------------------
  Function:      ai_monitor_get_similarity
  Description:   查询最近一次检测得到的相似度。
  Calls:         无
  Called By:     main.c、调试取用
  Input:         无
  Output:        无
  Return:        相似度 0~100（尚未检测时为 0）
  Others:        无
-----------------------------------------------------------*/
uint8_t ai_monitor_get_similarity(void)
{
    return g_similarity;  /* 返回相似度 */
}

/*-----------------------------------------------------------
  Function:      ai_monitor_has_result
  Description:   查询最近一次检测结果是否有效。
  Calls:         无
  Called By:     main.c
  Input:         无
  Output:        无
  Return:        1 = 有效，0 = 无效
  Others:        学习阶段、窗口未满、模型报错时都为 0。
-----------------------------------------------------------*/
uint8_t ai_monitor_has_result(void)
{
    return g_result_valid;  /* 返回结果有效性 */
}

/*-----------------------------------------------------------
  Function:      ai_monitor_get_fill
  Description:   查询学习进度（已完成的 learn 次数）。
  Calls:         无
  Called By:     调试取用
  Input:         无
  Output:        无
  Return:        0 ~ 255（实际学习次数可能超过 255，超出就封顶）
  Others:        ICM 库的最少学习次数是 368，已经超过 uint8 上限，
                  所以这里必须封顶，否则转回 uint8 会截断成错的值。
-----------------------------------------------------------*/
uint8_t ai_monitor_get_fill(void)
{
    if (g_learned > 255U)  /* 超过 uint8 能表示的范围 */
    {
        return 255U;  /* 封顶 */
    }
    return (uint8_t)g_learned;  /* 未溢出时直接报进度 */
}

/*-----------------------------------------------------------
  Function:      ai_monitor_get_site_progress
  Description:   查询「启动现场自适应学习阶段」的进度，供屏显画 "n/m"。
  Calls:         无
  Called By:     main.c（填 UI_Data 的 learn_done / learn_total）
  Input:         done   —— 输出参数，已补学的窗数
                 total  —— 输出参数，目标窗数（0 表示当前不在该阶段）
  Output:        修改 *done 与 *total
  Return:        无
  Others:        两个指针都必须非空，函数内部已判空，不会崩。
-----------------------------------------------------------*/
void ai_monitor_get_site_progress(uint8_t *done, uint8_t *total)
{
    if (done != NULL)  /* 判空，避免调用方传 NULL 时崩 */
    {
        if (g_siteActive != 0U)  /* 还在补学阶段 */
        {
            *done = (uint8_t)((g_siteDone > 255U) ? 255U : g_siteDone);  /* 已补学窗数 */
        }
        else  /* 已转入检测阶段 */
        {
            *done = 0U;  /* 没有进行中的补学 */
        }
    }
    if (total != NULL)  /* 判空 */
    {
        if (g_siteActive != 0U)  /* 还在补学阶段才报目标窗数 */
        {
            *total = (uint8_t)AI_MONITOR_SITE_LEARN_WINDOWS;  /* 目标窗数 */
        }
        else  /* 已转入检测阶段：报 0，屏上据此把进度格清掉 */
        {
            *total = 0U;  /* 无进度 */
        }
    }
}

/*-----------------------------------------------------------
  Function:      ai_monitor_is_alarm
  Description:   查询当前是否处于报警状态。
  Calls:         无
  Called By:     telemetry.c（写入遥测帧的 AI 报警位）、main.c
  Input:         无
  Output:        无
  Return:        1 = 报警中，0 = 正常
  Others:        事件级状态会保持，直到连续 2 个窗达标才解除。
-----------------------------------------------------------*/
uint8_t ai_monitor_is_alarm(void)
{
    return g_alarm;  /* 返回报警状态 */
}

/*-----------------------------------------------------------
  Function:      ai_monitor_notify_stopped
  Description:   主循环发现电机停转时调用，用来在停机后清掉报警与计数。
  Calls:         无
  Called By:     main.c 主循环 10 ms 块（d.speed_rpm < 20 rpm 时每拍一次）
  Input:         无
  Output:        修改 g_winEmpty、g_alarm、g_lowRun、g_highRun、g_phase
  Return:        无
  Others:        1) 连续 AI_MONITOR_STOP_TICKS 拍（0.5 s）停机才动手，
                    避免转速瞬时跌破 20 rpm 就清一次；
                  2) 起转后窗口由 feature_r_tick() 的闸门自己重建，
                    这里只负责把报警和计数清干净；
                  3) 【2026-10-21 起不再重学基线】峰峰值特征档位/工况
                    不敏感，预训练知识可以直接沿用，停机不必丢知识。
-----------------------------------------------------------*/
void ai_monitor_notify_stopped(void)
{
    if ((g_initialized == 0U) || (g_enabled == 0U))  /* 没开 AI 就不用管 */
    {
        return;  /* 直接返回 */
    }
    if (g_winEmpty < 0xFFU)  /* 防溢出 */
    {
        ++g_winEmpty;  /* 停机拍数 +1 */
    }
    if (g_winEmpty >= AI_MONITOR_STOP_TICKS)  /* 连续 0.5 s 都停机 */
    {
        g_winEmpty = 0U;  /* 触发一次就归零，避免每拍重复清 */
        g_lowRun = 0U;  /* 清低分计数 */
        g_highRun = 0U;  /* 清高分计数 */
        g_alarm = 0U;  /* 停机期间不留报警 */
        g_detectCount = 0U;  /* 重新计检测次数 */
        g_result_valid = 0U;  /* 结果置为无效 */
        /* 【2026-10-22】停机不结束启动现场自适应学习阶段：
         * 停机只是暂时没有满窗，补学计数原样保留，
         * 重新起转后继续补。屏上阶段也跟着回到 LRN。 */
        if ((g_siteActive == 0U) && (g_needLearn == 0U))  /* 既不补学也不从零学 */
        {
            g_phase = AI_MONITOR_DETECTING;  /* 知识库保留，仍是检测阶段 */
        }
    }
}

/*-----------------------------------------------------------
  Function:      ai_monitor_switch_level
  Description:   切换电机档位（等于换工况）时调用：清空 R 窗口，
                  把本模块的报警与计数全部复位，用干净窗口重新判。
  Calls:         feature_r_reset()
  Called By:     main.c 按键处理（KEY1 加档、KEY2 减档之后）
  Input:         duty_pct  切换后的占空比百分数（0 表示切到停机档）
  Output:        无
  Return:        无
  Others:        1) 切到停机档（0）时不做复位：停机本身由
                      ai_monitor_notify_stopped() 的 0.5 s 逻辑接管；
                  2) 【2026-10-21 起不再重新 init 模型库】上一代喂的是 R 电平，
                      两档的 R 相差 16%，每档必须各学一份基线；这一代喂的是
                      滚动峰峰值，实测两档 0.834 与 0.836 只差 0.2%，档位不敏感，
                      一份预训练知识两档通用，切档只清窗不丢知识；
                  3) 清窗后约 3.9 s 才重新攒满 32 点，这段屏上相似度显示 ---。
-----------------------------------------------------------*/
void ai_monitor_switch_level(uint8_t duty_pct)
{
    if ((g_initialized == 0U) || (g_enabled == 0U))  /* 没开 AI 就不必清 */
    {
        return;  /* 直接返回 */
    }
    if (duty_pct == 0U)  /* 切到停机档：交给停机逻辑处理 */
    {
        g_winEmpty = 0U;  /* 停机计时从零开始 */
        return;  /* 直接返回 */
    }

    feature_r_reset();  /* 丢掉上一档的 32 点峰峰值窗 */

    g_detectCount = 0U;  /* 重新计检测次数 */
    g_lowRun = 0U;  /* 清低分计数 */
    g_highRun = 0U;  /* 清高分计数 */
    g_alarm = 0U;  /* 换工况后不保留上一档的报警 */
    g_result_valid = 0U;  /* 结果置为无效 */
    g_similarity = 0U;  /* 相似度归零 */
    g_winEmpty = 0U;  /* 空窗计数归零 */
    /* 【2026-10-22】切档不结束启动现场自适应学习阶段：
     * 峰峰值特征对档位不敏感（两档均值差 0.2%），
     * 补学进度原样保留，换档后继续补。 */
    if ((g_siteActive == 0U) && (g_needLearn == 0U))  /* 既不补学也不从零学 */
    {
        g_phase = AI_MONITOR_DETECTING;  /* 知识库保留，直接继续检测 */
    }
}

/*-----------------------------------------------------------
  Function:      ai_monitor_tick
  Description:   每产生一个新的 R 点调用一次。窗口未满 32 点直接返回；
                 窗满后按「学习阶段 / 检测阶段」两个分支处理。
  Calls:         feature_r_get()、neai_anomalydetection_learn()、
                 neai_anomalydetection_detect()
  Called By:     main.c 主循环 10 ms 块（feature_r_tick() 返回 1 时）
  Input:         无（数据从 feature_r 的窗口直接取，不再多一层拷贝）
  Output:        无
  Return:        无
  Others:        1) 只有窗满才喂模型，窗不满时清结果与报警，
                    防止拿半窗数据判异常；
                 2) 窗口由 feature_r 自己在停机/电压失效/转速过低时清空重建，
                    所以本函数不再重复做闸门判断；
                 3) 「要不要现场学习」由模型库的返回值决定，固件不写死窗数：
                    detect() 回 NEAI_LEARNING_IN_PROGRESS 就退回学习分支；
                 4) 报警与解除都要求连续 2 个窗（约 200 ms），抗单点抖动；
                 5) 周期重学习只在相似度高于硬故障门限时计数，避免把真异常学成正常；
                 6) 电机停过（连续 0.5 s 拿不到满窗）由 notify_stopped 清报警与计数。
-----------------------------------------------------------*/
void ai_monitor_tick(void)
{
    enum neai_state st;  /* 模型库返回值 */

    /* ---- 闸门一：未初始化或已关闭 AI，什么都不做 ---- */
    if ((g_initialized == 0U) || (g_enabled == 0U))  /* 没开就直接返回 */
    {
        return;  /* 直接返回 */
    }

    /* ---- 闸门二：窗口满了没有 ---- */
    if (feature_r_get(g_neai_in, AI_MONITOR_NEAI_IN_LEN) == 0U)  /* 没满 32 点 */
    {
        g_result_valid = 0U;  /* 没送模型就没结果 */
        g_alarm = 0U;  /* 没有连续稳态数据就不报警 */
        g_lowRun = 0U;  /* 清低分计数 */
        g_highRun = 0U;  /* 清高分计数 */
        return;  /* 等下一个点 */
    }

    g_winEmpty = 0U;  /* 拿到满窗，空窗计数归零 */

    /* ================= 学习分支 ================= */
    /* 只有两种情况会进来：库里没有预训练知识（上电就得从零学），
     * 或者上一次 detect() 明确回了 NEAI_LEARNING_IN_PROGRESS。
     * 带预训练知识时这条路根本不走，上电就能出检测结果。 */
    if (g_needLearn != 0U)  /* 库还没学够 */
    {
        if (g_learnLeft == 0U)  /* 这一轮配额已用完，但库还要 */
        {
            g_learnLeft = (uint16_t)AI_MONITOR_LEARN_MAX;  /* 再给一轮，学到底 */
        }
        --g_learnLeft;  /* 消耗一个学习配额 */
        st = neai_anomalydetection_learn(g_neai_in);  /* 喂一个窗进去学 */
        if ((st == NEAI_OK) ||  /* 返回成功 / */
            (st == NEAI_LEARNING_IN_PROGRESS))  /* 还在学 */
        {
            ++g_learned;  /* 学习次数 +1 */
            g_phase = AI_MONITOR_LEARNING;  /* 保持在学习阶段 */
        }
        else if (st == NEAI_LEARNING_DONE)  /* 达到最少学习次数 */
        {
            ++g_learned;  /* 学习次数 +1 */
            g_needLearn = 0U;  /* 清学习标志，下一拍开始检测 */
            g_learnLeft = 0U;  /* 配额清零 */
            g_phase = AI_MONITOR_DETECTING;  /* 转入检测阶段 */
        }
        else  /* 返回错误 */
        {
            g_phase = AI_MONITOR_ERROR;  /* 进错误阶段 */
        }
        g_result_valid = 0U;  /* 学习阶段不产生检测结果 */
        return;  /* 本次不检测 */
    }

    /* ================= 检测分支 ================= */
    /* 把 32 点峰峰值窗整个送进模型：模型内部先取 95 分位 / 峰值 / 25 分位
     * 三个序统计量，再做增量聚类算距离，输出 0~100 的相似度到 g_similarity。 */
    st = neai_anomalydetection_detect(g_neai_in, &g_similarity);  /* 调库的检测接口 */

    if (st == NEAI_LEARNING_IN_PROGRESS)  /* 库说知识还不够（最少学习次数未到） */
    {
        g_needLearn = 1U;  /* 退回学习分支，把知识补齐 */
        g_phase = AI_MONITOR_LEARNING;  /* 阶段显示为学习中 */
        g_result_valid = 0U;  /* 本次没有有效结果 */
        g_lowRun = 0U;  /* 清低分计数 */
        g_highRun = 0U;  /* 清高分计数 */
        return;  /* 本次不参与报警判定 */
    }

    g_result_valid = (st == NEAI_OK) ? 1U : 0U;  /* 记录本次结果是否有效 */

    if (st != NEAI_OK)  /* 检测报错 */
    {
        g_phase = AI_MONITOR_ERROR;  /* 进错误阶段 */
        g_lowRun = 0U;  /* 清低分计数 */
        g_highRun = 0U;  /* 清高分计数 */
        return;  /* 本次不参与报警判定 */
    }

    /* ---- 启动现场自适应学习阶段（2026-10-22 新增）----
     * 【放在检测之后】先把窗送去 detect() 拿到相似度，再决定要不要 learn()。
     * 这样整个补学阶段检测能力一直在线，报警随时能触发，
     * 不会出现「一上电就几十秒不报警」的空窗期。
     *
     * 【补学门限】只有相似度 >= AI_MONITOR_SITE_LEARN_MIN_SIM(32) 的窗才补学，
     * 也就是只把「模型自己判定为正常」的样本补进知识库。
     * 捏轴时相似度全程为 0，门限挡得死，异常不会被学成正常。
     *
     * 【为什么 learn() 不会破坏预训练知识】库头文件写明 learn() 在已初始化的
     * 模型上是 "complement the knowledge base"（增量补充），不是清空重建。
     * 2026-10-22 用板载同一个库（md5 一致）在 PC 仿真器实测：
     * 补学 8 / 32 / 64 窗正常数据后，检出 3056/3378、误报 4/2252，
     * 与只 init(true) 的基线逐窗完全一致。 */
    if (g_siteActive != 0U)  /* 还在启动现场自适应学习阶段 */
    {
        ++g_siteSeen;  /* 看过的窗数 +1（不管这一窗补没补成功） */
        if (g_similarity >= AI_MONITOR_SITE_LEARN_MIN_SIM)  /* 本窗判定为正常 */
        {
            (void)neai_anomalydetection_learn(g_neai_in);  /* 增量补学一个窗 */
            ++g_siteDone;  /* 补学计数 +1 */
            ++g_learned;  /* 总学习次数也 +1（ai_monitor_get_fill 报表用） */
            if (g_siteDone >= (uint16_t)AI_MONITOR_SITE_LEARN_WINDOWS)  /* 补够了 */
            {
                g_siteActive = 0U;  /* 退出补学阶段 */
                g_phase = AI_MONITOR_DETECTING;  /* 转入纯检测阶段，屏上 DET */
            }
            else  /* 还没补够 */
            {
                g_phase = AI_MONITOR_LEARNING;  /* 保持学习阶段，屏上 LRN + 进度 */
            }
        }
        else  /* 本窗异常：不补学，也不推进计数，等下一窗正常的再补 */
        {
            g_phase = AI_MONITOR_LEARNING;  /* 阶段仍是学习，屏上 LRN */
            /* 【超时兜底】一直遇不到正常窗时不能永远停在 LRN：
             * 预训练知识本来就在库里，检测能力不依赖这次补学，
             * 所以看够 AI_MONITOR_SITE_LEARN_MAX_WINDOWS 个窗就强制转检测。 */
            if (g_siteSeen >= (uint16_t)AI_MONITOR_SITE_LEARN_MAX_WINDOWS)  /* 到时限了 */
            {
                g_siteActive = 0U;  /* 退出补学阶段 */
                g_phase = AI_MONITOR_DETECTING;  /* 转入纯检测阶段，屏上 DET */
            }
        }
    }
    else  /* 已完成启动补学，或本来就不在补学阶段 */
    {
        g_phase = AI_MONITOR_DETECTING;  /* 阶段显示为检测中 */
    }

    if (g_similarity < AI_MONITOR_THRESHOLD)  /* 相似度低于阈值 = 疑似异常 */
    {
        g_highRun = 0U;  /* 断开「连续达标」 */
        if (g_lowRun < 0xFFU)  /* 防止计数溢出 */
        {
            ++g_lowRun;  /* 低分计数 +1 */
        }
        if (g_lowRun >= AI_MONITOR_ALARM_WINDOWS)  /* 连续 2 个窗都低 */
        {
            g_alarm = 1U;  /* 置报警 */
        }
    }
    else  /* 相似度达标 = 正常 */
    {
        g_lowRun = 0U;  /* 断开「连续低分」 */
        if (g_highRun < 0xFFU)  /* 防止计数溢出 */
        {
            ++g_highRun;  /* 高分计数 +1 */
        }
        if (g_highRun >= AI_MONITOR_CLEAR_WINDOWS)  /* 连续 2 个窗都达标 */
        {
            g_alarm = 0U;  /* 解除报警 */
        }
    }

    /* ---- 周期重学习：只要不是硬故障就照常累计 ---- */
    /* 原来是「只要报警就完全不累计」，本意是防止把异常学成正常，
     * 但板上实测会死锁：工况整体漂移（换负载后转速 150 变 216 rpm）时，
     * 相似度长期偏低、报警又不解除，重学习永远不触发，就再也回不来了。
     * 现在改成用相似度本身来挡：只有落在硬故障区间才拒绝重学。
     * 【2026-10-21 换 ICM 库后改为增量补学】预训练知识本身是好的，
     * 清空重学会把它丢掉，所以这里只 learn 一个窗、往知识库里补一点，
     * 而且补学期间不停检测（不像上一代那样整段回到 LEARNING）。 */
    if (g_similarity >= AI_MONITOR_RELEARN_MIN_SIM)  /* 不是硬故障 */
    {
        ++g_detectCount;  /* 检测计数 +1 */
        if (g_detectCount >= AI_MONITOR_RELEARN_DETECTS)  /* 达到重学间隔 */
        {
            g_detectCount = 0U;  /* 计数归零 */
            (void)neai_anomalydetection_learn(g_neai_in);  /* 补学一个窗，扩充知识库 */
        }
    }
}
