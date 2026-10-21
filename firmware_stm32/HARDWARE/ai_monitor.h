/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    ai_monitor.h
  Author:      汪贤云
  Version:     2.0
  Date:        2026-10-19
  Description: 板上 AI 模块的对外接口与常量声明。
               数据流：feature_r（R = 转速 / 母线电压，32 点窗）
               -> NanoEdge learn/detect -> 相似度（0~100）
               -> 连续 2 个窗低于阈值即置报警。
               本文件里的阈值、学习次数都是框架里的可调参数，
               修改前请先看对应注释里的取值依据。
  Function List:
    1. ai_monitor_init()             初始化模型库
    2. ai_monitor_tick()             每个 R 点调一次（核心）
    3. ai_monitor_set_enabled()      采集/部署模式切换
    4. 一组查询接口                  阶段、相似度、有效性、报警、填充度
  History:
    <author>   <time>      <version>   <desc>
    汪贤云     2026-10-16    1.0         创建文件并按华为规范补充注释
    汪贤云     2026-10-19    2.0         换用单变量 R 的 32 点窗模型库（ZSM），
                                                ai_monitor_feed_frame() 改为 ai_monitor_tick()，
                                                不再依赖 M2Frame 与 19 维特征
***********************************************************/
/* 【为何从 19 维改成单变量】
 * 旧版把 64 帧原始量算成 19 维幅值统计（均值/最大/最小/RMS 等），
 * 而这 19 维全是幅值类统计，它们互相高度相关，所以单看
 * 「是否异常」它与一个多维阈值等价，并未真正用上模型的优势。
 * 现在只留一个物理量：R = 转速 / 母线电压（rpm/V）。
 * R 把「电池电量下降带来的整体降速」除掉了，只留下
 * 「负载/堵转引起的异常」，两者才不会互相遮盖。 */
#ifndef AI_MONITOR_H
#define AI_MONITOR_H

#include <stdint.h>        /* 定长整数类型 */
#include "feature_r.h"     /* 需要 FEATURE_R_WINDOW：模型输入就是这一个窗 */

#ifdef __cplusplus  /* C++ 下按 C 方式链接 */
extern "C" {        /* 开启 C 链接区 */
#endif               /* __cplusplus 条件伪编译 */

/* 异常判定阈值：相似度低于它就当做异常。
 * 【离线标定】用 libneai 官方仿真器在 9 批真实 R 数据上跑（脚本
 * output/阈值校准_20261019/calib2.py，学习 64 窗）：
 *   正常最低 60，独立第 6 批最低 91，捏轴异常最高 20；
 *   该结论下 40~60 全都是 0% 误报 / 100% 检出，所以先取了 48。
 * 【上板后下调到 32】2026-10-19 板上连续采 45 点（约 32 秒）实测：
 *   正常运行时相似度 min 44 / p05 64 / 中位 76 / max 100，标准差 9.8；
 *   板上抖动明显比离线大（整台机器在振，3.2 s 窗均值的方差更大），
 *   45 个点里就有 1 次跌破 48 并真的触发了报警 —— 这是误报；
 *   而捏轴时相似度全程为 0，只在过渡点上出现过 16。
 *   取 32 = 正常下限 44 与异常上限 16 之间的中点略偏保守侧，
 *   两侧余量 12 分与 16 分，实测 45 个点 0 次跌破。
 * 报警仍要求连续 2 窗，单点抖动不会误触发。 */
#define AI_MONITOR_THRESHOLD       32U

/* 现场学习的兜底上限窗数（只有模型库明确要求学习时才会用到）。
 * 【为什么不再是 64】上一代 ZSM 库最少 16 次就学够，64 是离线标定出来的
 * 最优值；这一代 ICM 库的 metadata.json 里写的是 learning_min_lines = 368，
 * 也就是至少要喂 368 个窗（约 36.8 s）库才会返回 NEAI_LEARNING_DONE。
 * 固件若还按 64 窗就转去检测，detect() 会一直返回 NEAI_LEARNING_IN_PROGRESS，
 * 结果是永远没有相似度、永远不报警 —— 这个坑必须避开。
 * 现在把「学多少个窗」的决定权交给模型库自己的返回值，本常量只作兜底：
 * 368 再留 32 个窗余量。
 * 【正常路径用不到它】库自带预训练知识（NanoEdgeAI.h 写明 pretrained
 * model is embedded in the library），上电 init(true) 之后直接就能检测。 */
#define AI_MONITOR_LEARN_MAX       400U

/* 周期重学习的间隔（以检测次数计）。
 * 现在每个 R 点（约 100 ms）就检一次，所以 6000 次约等于 10 分钟。
 * 上一代设 180 是因为当时一次检测要 640 ms；现在节拍快了 6 倍，
 * 若直接拿老值会变成每 18 秒重学一次，那就太频了。
 * 【2026-10-19 改动】原来写的是「只要报警就完全不累计」，本意是防止把异常
 * 学成正常，但板上实测会死锁：工况整体漂移（换负载后转速 150 变 216 rpm）
 * 让相似度长期偏低、报警又不解除，重学习永远不触发，就再也回不来了。
 * 现在改成用相似度本身来挡，见下面的硬故障门限。 */
#define AI_MONITOR_RELEARN_DETECTS 6000U

/* 硬故障门限：相似度低于它就认定是真异常，拒绝重学习。
 * 捏轴实测相似度全程为 0（只在过渡点出现过 16），正常侧最低 44；
 * 取 20 放在两者之间，既挡得住真故障，又不会挡住工况漂移。
 * 报警去抖仍然由 AI_MONITOR_ALARM_WINDOWS 负责，两者各管一件事。 */
#define AI_MONITOR_RELEARN_MIN_SIM  20U

/* 停机多久就重学基线（以 10 ms 节拍计，50 拍 = 0.5 s）。
 * 停机再起转相当于换了一个工况，旧基线不再代表当前状态，
 * 这样换负载后不用断电重启也能重新对齐。
 * 取 0.5 s：比启动时重新攒窗的 3.2 s 短得多，又足以避开转速瞬时跌破 20 rpm 的抖动。 */
#define AI_MONITOR_STOP_TICKS       50U

/* 报警与解除的去抖窗数。
 * 相似度偶尔跳一下不能就报警，要连续几个窗都低才算；
 * 现在一窗就是 100 ms，所以 2 窗约 200 ms 就能触发，响应很快。 */
#define AI_MONITOR_ALARM_WINDOWS   2U
#define AI_MONITOR_CLEAR_WINDOWS   2U

/* 四个阶段。该状态通过遥测帧的 aiState 字节发给 D1，
 * 手机页面上会直接显示「未启用 / 学习中 / 检测中 / 错误」。
 *   OFF       = 关闭 AI（采集模式，g_enabled = 0）
 *   LEARNING  = 库要求现场学习（库里没有预训练知识时；有则上电即可检测）
 *   DETECTING = 学习完成，正在持续检测
 *   ERROR     = 模型初始化失败或调用报错（需重启/查）
 * 枚举值必须与协议里的 M2_AI_* 一一对应，不要随意改数字。 */
typedef enum {
    AI_MONITOR_OFF = 0, /* 未启用（采集模式，g_enabled = 0） */
    AI_MONITOR_LEARNING = 1,    /* 现场学习（上电或周期重学） */
    AI_MONITOR_DETECTING = 2,    /* 学习完成，持续检测 */
    AI_MONITOR_ERROR = 3    /* 初始化失败或调用报错 */
} AiMonitorPhase;  /* AI 阶段，与协议里 M2_AI_* 一一对应 */

void ai_monitor_init(void);   /* 初始化模型库（main.c 上电调一次） */

/* 【对外接口速查】
 *   ai_monitor_init()            初始化模型库，main.c 上电只调一次
 *   ai_monitor_tick()            每到一个新 R 点调一次（feature_r_tick() 返回 1 时）
 *   ai_monitor_set_enabled(0/1)  关/开板上 AI，采集模式调 0 以减少主循环占用
 *   ai_monitor_get_phase()       取当前阶段，上传 D1 显示
 *   ai_monitor_get_similarity()  取最近一次相似度（目前仅供调试取用）
 *   ai_monitor_has_result()      最近一次检测结果是否有效
 *   ai_monitor_is_alarm()        是否处于报警态（遥测帧的 AI 报警位）
 *   ai_monitor_get_fill()        学习进度（0..255）
 *   ai_monitor_notify_stopped()  转速 < 20 rpm 时每 10 ms 调一次，0.5 s 后重学
 *   ai_monitor_switch_level()    切档后调一次，清窗 + 清模型 + 重学本档基线
 * 【约束】除 ai_monitor_init() 外均只在主循环上下文调用，不要在中断里调。 */
void ai_monitor_tick(void);

/* 停机通知：main.c 的 10 ms 块里，转速低于 20 rpm 时每次都调一次。
 * 连续 AI_MONITOR_STOP_TICKS 拍（0.5 s）都停机就把学习状态归零，
 * 重新起转时按当前工况重学基线。 */
void ai_monitor_notify_stopped(void);

/* 切档通知：main.c 按键处理里改完档位、重设 PWM 之后调用一次。
 * 作用是把上一档的峰峰值窗口和报警计数全部作废，用干净窗口重新判。
 * 参数是切换后的占空比百分数；传 0（停机档）时本函数只清停机计时，
 * 真正的清窗交给 feature_r_tick() 自己的闸门。
 * 【2026-10-21 起不再重学基线】上一代喂的是 R 电平，R 里带了占空比补偿，
 * 两档之间转速并不按占空比等比例变化，所以每一档都必须有自己的基线；
 * 这一代喂的是滚动峰峰值，实测两档 0.834 与 0.836 只差 0.2%，档位不敏感，
 * 一份预训练知识两档通用，切档只清窗、不重新初始化模型库。 */
void ai_monitor_switch_level(uint8_t duty_pct);
/* 采集/部署切换：0 = 只采集（不跑模型），1 = 开启学习/检测。 */
void ai_monitor_set_enabled(uint8_t enabled);
AiMonitorPhase ai_monitor_get_phase(void);
uint8_t ai_monitor_has_result(void);   /* 最近一次检测结果是否有效 */
uint8_t ai_monitor_get_similarity(void);  /* 最近一次相似度（0~100） */
uint8_t ai_monitor_is_alarm(void);   /* 是否处于报警态（遥测帧 AI 报警位） */
/* 学习进度：已完成的 learn 次数，0..255（实际可能更多，超出封顶）。 */
uint8_t ai_monitor_get_fill(void);

#ifdef __cplusplus  /* C++ 下按 C 方式链接 */
}  /* extern "C" 结束 */
#endif  /* __cplusplus 条件伪编译 */
#endif  /* AI_MONITOR_H 头文件结束 */
