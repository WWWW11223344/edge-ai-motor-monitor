/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    ai_monitor.c
  Author:      汪贤云
  Version:     2.0
  Date:        2026-10-19
  Description: 板上 AI 模块：现场学习与异常检测
               （NanoEdge AI Studio Library ID 6ad6053cc9f44b85284837b0 / 模型族 ZSM）。
               数据流：main.c 主循环每 100 ms 产生一个 R 点
               （R = 转速 / 母线电压），每产生一个点调一次本模块；
               窗口满 32 点（约 3.2 s）后把整个窗直接喂给模型；
               学习阶段连续 64 个窗（约 6.4 s），之后进入检测，
               输出相似度（0~100），连续 2 个窗低于阈值即置报警。
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
***********************************************************/
/* 【新旧模型的区别】
 * 旧库：输入 4 个样本 x 19 轴 = 76 个 float，需要自己攒 64 帧算 19 维特征；
 * 新库：输入 1 个样本 x 32 点 = 32 个 float，特征提取已经做进模型里了。
 * 具体地说，模型包 artifacts/zsm_preprocessing_config.json 里写的是
 * 「均值池化，保留 5%」，即把 32 点压成 1 个均值，
 * 再对这个均值做 Z-score 判异常。
 * 所以本文件不再需要做任何特征计算，只负责「取窗 + 喂模型 + 判报警」。
 * 这也是为什么 ai_monitor.c 从 385 行变到不到 300 行：少了一整层特征。 */

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
static uint32_t g_detectCount;  /* 自上次重学以来的检测次数（仅未报警时累计） */
static AiMonitorPhase g_phase = AI_MONITOR_OFF;  /* 当前阶段 */
static uint8_t g_similarity;  /* 最近一次相似度（0~100） */
static uint8_t g_result_valid;  /* 最近一次检测结果是否有效 */
static uint8_t g_initialized;  /* 模型库是否初始化成功 */
static uint8_t g_alarm;  /* 事件级报警状态 */
static uint8_t g_lowRun;  /* 连续低于阈值的窗数 */
static uint8_t g_highRun;  /* 连续达标的窗数 */
static uint8_t g_enabled = 1U;  /* 1 = 启用板上 AI，0 = 只采集不跑模型 */
static uint8_t g_winEmpty;  /* 连续拿不到满窗的节拍数（判断电机是否停过） */

/*-----------------------------------------------------------
  Function:      ai_monitor_init
  Description:   初始化 NanoEdge 模型库，并把本模块的内部状态全部复位。
  Calls:         neai_anomalydetection_init()
  Called By:     main.c 上电初始化段，只调一次
  Input:         无
  Output:        无
  Return:        无（是否成功请用 ai_monitor_get_phase() 查）
  Others:        1) 参数 false 表示「不用预训练知识，从头学」，
                    这与编译时 use_knowledge = false 是一致的；
                 2) 初始化成功则直接进入 LEARNING，失败则进入 ERROR。
-----------------------------------------------------------*/
void ai_monitor_init(void)
{
    enum neai_state state = neai_anomalydetection_init(false);  /* 初始化模型库（false = 从头学） */

    g_learned = 0U;  /* 学习次数归零 */
    g_detectCount = 0U;  /* 检测计数归零 */
    g_similarity = 0U;  /* 相似度归零 */
    g_result_valid = 0U;  /* 结果置为无效 */
    g_alarm = 0U;  /* 清报警 */
    g_winEmpty = 0U;  /* 空窗计数归零 */
    g_lowRun = 0U;  /* 清低分计数 */
    g_highRun = 0U;  /* 清高分计数 */
    g_initialized = (state == NEAI_OK) ? 1U : 0U;  /* 记下初始化是否成功 */
    g_phase = (g_initialized != 0U) ? AI_MONITOR_LEARNING : AI_MONITOR_ERROR;  /* 成功就进入学习，失败则报错 */
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
  Return:        0 ~ AI_MONITOR_LEARN_WINDOWS
  Others:        超过上限时封顶在 AI_MONITOR_LEARN_WINDOWS，避免转回 uint8 时溢出。
-----------------------------------------------------------*/
uint8_t ai_monitor_get_fill(void)
{
    if (g_learned >= AI_MONITOR_LEARN_WINDOWS)  /* 已学完 */
    {
        return (uint8_t)AI_MONITOR_LEARN_WINDOWS;  /* 封顶 */
    }
    return (uint8_t)g_learned;  /* 未学完时直接报进度 */
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
  Description:   主循环发现电机停转时调用，用来在停机后重学基线。
  Calls:         无
  Called By:     main.c 主循环 10 ms 块（d.speed_rpm < 20 rpm 时每拍一次）
  Input:         无
  Output:        修改 g_winEmpty、g_learned、g_phase、g_detectCount
  Return:        无
  Others:        1) 连续 AI_MONITOR_STOP_TICKS 拍（0.5 s）停机才动手，
                    避免转速瞬时跌破 20 rpm 就触发一次无谓的重学；
                 2) 停机再起转等于换了工况（实测同一档位下转速会因负载
                    在 150 与 216 rpm 之间变化），旧基线不再代表当前状态；
                 3) 重新起转后由 ai_monitor_tick() 按新工况重学 64 个窗。
-----------------------------------------------------------*/
void ai_monitor_notify_stopped(void)
{
    if ((g_initialized == 0U) || (g_enabled == 0U))  /* 没开 AI 就不用管 */
    {
        return;  /* 直接返回 */
    }
    if (g_learned == 0U)  /* 已经处于待学习状态，没什么可清的 */
    {
        g_winEmpty = 0U;  /* 计数归零 */
        return;  /* 直接返回 */
    }
    if (g_winEmpty < 0xFFU)  /* 防溢出 */
    {
        ++g_winEmpty;  /* 停机拍数 +1 */
    }
    if (g_winEmpty >= AI_MONITOR_STOP_TICKS)  /* 连续 0.5 s 都停机 */
    {
        g_learned = 0U;  /* 学习次数归零，回到学习分支 */
        g_phase = AI_MONITOR_LEARNING;  /* 阶段回到学习 */
        g_detectCount = 0U;  /* 重学后重新计检测次数 */
        g_lowRun = 0U;  /* 清低分计数 */
        g_highRun = 0U;  /* 清高分计数 */
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
                 3) 报警与解除都要求连续 2 个窗（约 200 ms），抗单点抖动；
                 4) 周期重学习只在相似度高于硬故障门限时计数，避免把真异常学成正常；
                 5) 电机停过（连续 0.5 s 拿不到满窗）会重学基线，换工况后能自己恢复。
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
    if (g_learned < AI_MONITOR_LEARN_WINDOWS)  /* 还没学够 64 个窗 */
    {
        st = neai_anomalydetection_learn(g_neai_in);  /* 喂一个窗进去学 */
        if ((st == NEAI_OK) ||  /* 返回成功 / */
            (st == NEAI_LEARNING_IN_PROGRESS) ||  /* 还在学 / */
            (st == NEAI_LEARNING_DONE))  /* 已达到最小学习次数 */
        {
            ++g_learned;  /* 学习次数 +1 */
            g_phase = (g_learned >= AI_MONITOR_LEARN_WINDOWS) ?  /* 学够了就转检测 */
                      AI_MONITOR_DETECTING : AI_MONITOR_LEARNING;  /* 否则继续学 */
        }
        else  /* 返回错误 */
        {
            g_phase = AI_MONITOR_ERROR;  /* 进错误阶段 */
        }
        g_result_valid = 0U;  /* 学习阶段不产生检测结果 */
        return;  /* 本次不检测 */
    }

    /* ================= 检测分支 ================= */
    /* 把 32 点窗整个送进模型，模型内部会先做均值池化再算 Z-score，
     * 输出一个 0~100 的相似度到 g_similarity。 */
    st = neai_anomalydetection_detect(g_neai_in, &g_similarity);  /* 调库的检测接口 */
    g_phase = (st == NEAI_OK) ? AI_MONITOR_DETECTING : AI_MONITOR_ERROR;  /* 更新阶段 */
    g_result_valid = (st == NEAI_OK) ? 1U : 0U;  /* 记录本次结果是否有效 */

    if (st != NEAI_OK)  /* 检测报错 */
    {
        g_lowRun = 0U;  /* 清低分计数 */
        g_highRun = 0U;  /* 清高分计数 */
        return;  /* 本次不参与报警判定 */
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
    /* 原来这里是「只要报警就完全不累计」，本意是防止把异常学成正常，
     * 但板上实测会死锁：工况整体漂移（换负载后转速 150 变 216 rpm）时，
     * 相似度长期偏低、报警又不解除，重学习永远不触发，就再也回不来了。
     * 现在改成用相似度本身来挡：只有落在硬故障区间才拒绝重学。 */
    if (g_similarity >= AI_MONITOR_RELEARN_MIN_SIM)  /* 不是硬故障 */
    {
        ++g_detectCount;  /* 检测计数 +1 */
        if (g_detectCount >= AI_MONITOR_RELEARN_DETECTS)  /* 达到重学间隔 */
        {
            g_detectCount = 0U;  /* 计数归零 */
            g_learned = 0U;  /* 学习次数归零，回到学习分支 */
            g_lowRun = 0U;  /* 清低分计数 */
            g_highRun = 0U;  /* 清高分计数 */
            g_result_valid = 0U;  /* 结果置为无效 */
            g_phase = AI_MONITOR_LEARNING;  /* 阶段回到学习 */
        }
    }
}
