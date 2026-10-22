/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    ui.c
  Author:      汪贤云
  Version:     1.4
  Date:        2026-10-22
  Description: 显示界面层实现。所有布局坐标都定义为宏，调整位置或字号只需修改宏定义。
               刷新策略：软件 I2C 写入是同步阻塞的，一次重画全部数值会长时间占用主循环，
               导致 10ms 遥测出现空洞；因此把界面拆成若干区域，每次只刷新一个区域，
               多次调用后仍可完整更新，但单次阻塞时间显著缩短。
  Function List:
    1. UI_Init()       初始化 OLED 并绘制固定标签
    2. UI_DrawFrame()  绘制固定标签
    3. UI_Update()     轮流刷新各个数值区域
  History:
    <author>   <time>      <version>   <desc>
    汪贤云     2026-10-16    1.0         创建文件并按华为规范补充注释
    汪贤云     2026-10-19    1.1         删除 Vib 行与区域 2，刷新区域由 5 个减为 4 个
    汪贤云     2026-10-19    1.2         新增第 4~5 页 R 行，刷新区域由 4 个增为 5 个
    汪贤云     2026-10-22    1.3         R 与 dR 改为同排同时显示，消除每 2 s 轮显闪烁；
                                                底条档位改显示 L1/L2，新增学习进度 n/m
    汪贤云     2026-10-22    1.4         按实物照片反馈重排版面：R 行只留 R 与单位（原来与 dR
                                                挤成一团互相压字）；dR 改小字号移到底条顶替 T 标签；
                                                学习进度移到电压行；状态字与相似度之间留出空隙
***********************************************************/

#include "ui.h"  /* 本模块接口与 UI_Data 定义 */
#include "oled.h"  /* OLED 底层绘制接口 */

/* ==========================================================================
 * 显示界面层的实现
 * --------------------------------------------------------------------------
 * 布局坐标都写成了宏，以后想挪位置、改字号，只改这几行就行。
 * ========================================================================== */

/* 字号：16 = 8x16（大），12 = 6x8（小） */
#define FONT_BIG        16U
#define FONT_SMALL      12U

/* 大字号行的页号（每个大字号字占 2 页） */
#define ROW_SPD         0U   /* 转速行 */
#define ROW_CUR         2U   /* 电流行 */
#define ROW_R           4U   /* 归一化转速比 R 行（原振动行空出来给它的） */

/* 小字号行的页号（每个小字号字占 1 页） */
#define ROW_SIM         6U   /* 电压/状态行 */
#define ROW_BAR         7U   /* 底条行 */

/* 大字号行的列位置 */
#define BIG_LABEL_X     0U      /* 标签："Spd" / "Cur" */
#define BIG_VALUE_X     40U     /* 数值 */
#define BIG_UNIT_X      96U     /* 单位："rpm" / "A" */

/* 【R 行版面（2026-10-22 第二次调整，按实物照片反馈）】
 * 第一版把 R、dR、单位三段一起塞进这一行，实物照片上四个数字全贴在一起、
 * 互相压字，根本分不清哪个是哪个。现在 dR 缩小后挪到底条，这一行只留
 * R 与单位，按与上面两行完全一致的节奏排布：
 *   标签 "R"：   0 ~  23 列  = "R  "（R 加两个空格，补成 3 字宽，与 Spd/Cur 对齐）
 *   数值：      40 ~  79 列  = "xx.xx"（2 位整数 + 小数点 + 2 位小数，5 个 8x16 字）
 *   单位：      88 ~ 127 列  = "rpm/V"（5 个 8x16 字，正好到屏尾）
 * R 给到两位小数有两个理由：
 *   1) 与 D1 网页上的 R 保持同一精度（那边也是两位小数）；
 *   2) 模型对 R 的容差只有 ±2%，一位小数看不出这点漂移。
 * 数值段固定 5 个字，无效态也是 5 个字，切换时不会留下残影。 */
#define R_LBL_X         0U      /* R 标签（画 "R  "，占 0~23 列） */
#define R_VAL_X         40U     /* R 数值 "xx.xx"（5 个大字号字 = 40 列，40~79） */
#define R_UNIT_X        88U     /* 单位 "rpm/V"（5 个大字号字 = 40 列，88~127） */

/* 小字号行的列位置
 * 第 6 页从左到右：母线电压 | 中间 5 格 | 状态字。
 * 中间那 5 格有两种内容，长度都是 5 个字，整格覆盖、互不留残影：
 *   学习阶段 "32/64" = 已补学的窗数 / 目标窗数
 *   检测阶段 "S:100" = AI 相似度（100 表示与正常样本完全一致）
 * 原来相似度数值 72~89、状态字 92~121，中间只隔 2px，实物照片上
 * 看着就是「S:100RUN」连成一团；现在状态字挪到 96~125，中间空出 6px。 */
#define SML_LABEL_X     0U      /* "Bat"（3 个字 = 18px） */
#define SML_VALUE_X     24U     /* xx.x（4 个字 = 24px，正好到 48） */
#define SML_BAT_U_X     48U     /* "V"（1 个字 = 6px，占到 54 为止） */
#define SML_SLOT_X      62U     /* 中间 5 格（62~91）：进度 "32/64" 或相似度 "S:100" */
#define SML_STATE_V_X   96U     /* 状态文字 5 个字 = 30px（96~125），不出屏 */

/* 【底条列位置（2026-10-22 第二次重排，按实物照片反馈）】
 * 底条从左到右：阶段标记 | 档位 | dR 跳变幅度 | 运行秒数 | 心跳。
 * 小字号一个字占 6 列，全屏 128 列 = 21 个字位。各段终点都算过，不重叠；
 * **不同语义的两段之间至少留 3~4 列空隙，同一语义组内部（标签+数值、
 * 数值+单位）才允许紧贴**，这样读起来才分得清哪里到哪里是一组：
 *   0~17   阶段标记 "LRN" / "DET" / "SIM"（3 个字 = 18 列）
 *   18~21  空隙 4 列
 *   22~39  档位 "L1" / "L2" / "OFF"（最多 3 个字 = 18 列）
 *   40~43  空隙 4 列
 *   44~55  dR 标签 "dR"（2 个字 = 12 列）
 *   56~79  dR 数值 "x.xx"（4 个字 = 24 列）—— 与标签同属一组，紧贴
 *   80~83  空隙 4 列
 *   84~113 运行秒数（5 位 = 30 列）
 *   114~119 s 单位 —— 与秒数同属一组，紧贴
 *   120~121 空隙 2 列
 *   122~127 心跳风车
 * 【为什么把 dR 挪到底条】实物照片上 R 行四个数字全贴死、互相压字。
 *   dR 缩小后放到底条，R 行就只剩一个 R 加单位，一眼能读；
 *   底条本来就是一行小字，塞一个 4 个字的数值刚好放得下。
 * 【学习进度为什么也不在底条】底条 21 个字位已经排满，
 *   进度挪到第 6 页电压行中间那 5 格（检测阶段那里显示的是相似度）。
 * 【秒数为什么从 6 位收成 5 位】腾出 6 列给各段之间的空隙。
 *   5 位够显示到 99999 秒（约 27.7 小时），覆盖任何一次演示。
 * 【为什么不再显示占空比】原来这里画的是 "L1:58%"，把内部参数直接暴露在屏上，
 *   答辩时观众看不懂 58 是什么；档位号 L1/L2 才是使用者真正关心的操作量。 */
#define BAR_SIM_X       0U      /* SIM / LRN / DET 标记 */
#define BAR_LV_X        22U     /* 档位显示（最多 3 个字） */
#define BAR_DR_L_X      44U     /* dR 标签（2 个字 = 12 列） */
#define BAR_DR_V_X      56U     /* dR 数值（4 个字 = 24 列） */
#define BAR_TIME_V_X    84U     /* 秒数数值（5 位 = 30 列） */
#define BAR_TIME_U_X    114U    /* s 单位 */
#define BAR_SPIN_X      122U    /* 心跳风车 */

/* 转速最多 5 位（0 ~ 99999 转/分） */
#define SPD_DIGITS      5U      /* 转速显示位数 */
#define TIME_DIGITS     5U      /* 运行秒数显示位数（5 位 = 99999 秒，约 27.7 小时） */
#define UI_REFRESH_REGIONS 5U  /* 刷新区域总数（振动行让给 R 行后多一个区域） */

/* 状态文字：统一 5 个字符宽，这样刷新时不会留下上一次的残影 */
static const char *STATE_TXT[4] =   /* 状态文字表，索引即 UI_ST_* */
{
    "STOP ",     /* 停机 */
    "RUN  ",     /* 正常运行 */
    "WARN ",     /* 传感器异常 */
    "ALARM"      /* AI 报警 */
};  /* STATE_TXT 状态文本表结束 */

/* 心跳小风车：屏幕上有个东西在转，就说明程序没卡死 */
static const char SPIN[4] = { '|', '/', '-', '\\' };  /* 四帧循环，看起来就在转 */

/* -------------------------------------------------------------------------
 * 固定标签：只在初始化和清屏之后画，普通刷新不碰它（省时间）
 * ------------------------------------------------------------------------- */
/*-----------------------------------------------------------
  Function:      UI_DrawFrame
  Description:   绘制界面上固定不变的标签文字（各行的名称、单位等）。
  Calls:         OLED_ShowString()
  Called By:     UI_Init()、清屏之后
  Input:         无
  Output:        无
  Return:        无
  Others:        普通刷新不再重画这些标签，以减少同步 I2C 写入的耗时。
-----------------------------------------------------------*/

void UI_DrawFrame(void)
{
    OLED_ShowString(BIG_LABEL_X, ROW_SPD, "Spd", FONT_BIG);  /* 转速标签 */
    OLED_ShowString(BIG_UNIT_X,  ROW_SPD, "rpm", FONT_BIG);  /* 转速单位 */

    OLED_ShowString(BIG_LABEL_X, ROW_CUR, "Cur", FONT_BIG);  /* 电流标签 */
    OLED_ShowString(BIG_UNIT_X,  ROW_CUR, "A",   FONT_BIG);  /* 电流单位 */

    /* 第 4~5 页：R 与单位。dR 已挪到底条，这里只剩标签与单位。
     * 标签画成 "R  "（3 个字宽），与上面两行的 "Spd" / "Cur" 对齐，
     * 三行的数值列、单位列起点完全一致，看上去是一套版式。 */
    OLED_ShowString(R_LBL_X,  ROW_R, "R  ",   FONT_BIG);  /* 归一化转速比标签 */
    OLED_ShowString(R_UNIT_X, ROW_R, "rpm/V", FONT_BIG);  /* 单位 */

    /* 第 6 页左边：母线电压（原来是假的相似度 "Sim"，已让位给真数据） */
    OLED_ShowString(SML_LABEL_X,   ROW_SIM, "Bat", FONT_SMALL);  /* 电压标签 */
    OLED_ShowString(SML_BAT_U_X,   ROW_SIM, "V",   FONT_SMALL);  /* 电压单位 */
    /* 第 6 页中间那 5 格不放固定标签：学习阶段是 "32/64"，
     * 检测阶段是 "S:100"，两种内容都是 5 个字，整格覆盖即可，不会有残影。 */

    /* 底条：dR 的标签是固定的，画一次即可，刷新时只改数值。 */
    OLED_ShowString(BAR_DR_L_X, ROW_BAR, "dR", FONT_SMALL);  /* 跳变幅度标签 */
}

/*-----------------------------------------------------------
  Function:      UI_Init
  Description:   初始化 OLED 并绘制一次完整界面框架。
  Calls:         OLED_Init()、OLED_Clear()、UI_DrawFrame()
  Called By:     main.c 上电初始化（只调用一次）
  Input:         无
  Output:        无
  Return:        无
  Others:        无
-----------------------------------------------------------*/

void UI_Init(void)
{
    OLED_Init();  /* 初始化 OLED（软件 I2C + SSD1306） */
    OLED_Clear();  /* 清屏 */
    UI_DrawFrame();  /* 画固定标签 */
}

/* -------------------------------------------------------------------------
 * 刷新数值
 *
 * 软件 I2C 的 OLED 写入是同步阻塞的。一次重画全部数值会长时间占用主循环，
 * 进而让 10 ms 遥测发送出现空洞。把一次更新拆成五个区域，每次只发送一个
 * 区域；五次调用后所有内容仍会完整更新，但单次阻塞时间显著缩短。
 * 2026-10-19：振动行移除后腾出的第 4~5 页改画归一化转速比 R，区域仍是五个。
 * 2026-10-07：直接按固定宽度覆盖写入。只擦掉可选符号多占的那一列，
 * 不做整行「先擦后写」—— 那样同步 I/O 会翻倍。
 * ------------------------------------------------------------------------- */
/*-----------------------------------------------------------
  Function:      UI_Update
  Description:   刷新数值区域。每次调用只更新一个区域，多次调用后覆盖全部区域，
                 以此把一次性的长阻塞拆成若干次短阻塞。
  Calls:         OLED_ShowNum()、OLED_ShowFloat()、OLED_ShowString()
  Called By:     main.c 主循环
  Input:         d —— 只读的数据快照（函数内部不修改它）
  Output:        无
  Return:        无
  Others:        数值为无效时按无效样式显示，避免把过期数据当成当前值。
-----------------------------------------------------------*/

void UI_Update(const UI_Data *d)
{
    static uint8_t spin = 0U;  /* 心跳风车相位 */
    static uint8_t refreshRegion = 0U;  /* 本轮要刷新的区域号 */
    uint8_t st;  /* 本次要显示的状态 */

    switch (refreshRegion)  /* 轮流刷新：每次调用只处理一个区域 */
    {
        case 0U:  /* 区域 0：转速 */
            /* ---- 转速（大字号） ---- */
            if (d->speed_valid != 0U)  /* 转速有效 */
            {
                OLED_ShowNum(BIG_VALUE_X, ROW_SPD, (uint32_t)d->speed_rpm,  /* 显示整数转速 */
                             SPD_DIGITS, FONT_BIG);  /* 最多 5 位、大字号 */
            }
            else  /* 转速无效 */
            {
                OLED_ShowString(BIG_VALUE_X, ROW_SPD, "--   ", FONT_BIG);  /* 显示 -- */
            }
            break;  /* 区域 0 结束 */

        case 1U:  /* 区域 1：电流 */
            /* ---- 电流（大字号，1 位整数 + 2 位小数，最大 9.99A） ---- */
            if (d->current_valid != 0U)  /* 电流有效 */
            {
                /* 正数文本比「负数/无效文本」少一列，先补空格消残影 */
                OLED_ShowChar(BIG_VALUE_X + 4U * 8U, ROW_CUR, ' ', FONT_BIG);  /* 先覆盖掉上一帧多占的那一列，消残影 */
                OLED_ShowFloat(BIG_VALUE_X, ROW_CUR, d->current_a,  /* 1 位整数 + 2 位小数 */
                               1U, 2U, FONT_BIG);  /* 形如 x.xx */
            }
            else  /* 电流无效 */
            {
                OLED_ShowString(BIG_VALUE_X, ROW_CUR, "--.--", FONT_BIG);  /* 显示 --.-- */
            }
            break;  /* 区域 1 结束 */

        case 2U:  /* 区域 2：归一化转速比 R */
            /* ---- 2026-10-22 第二次调整：这一行只留 R 与单位 ----
             * 第一版把 R 与 dR 同排显示，实物照片上四个数字全贴在一起、
             * 互相压字，谁也分不清。现在 dR 缩小挪到底条，这一行按上面
             * 两行同样的节奏排：标签靠左、数值在第 40 列、单位靠右。
             *
             * R = 转速 / 母线电压，是绝对工作点，换档位整体平移。
             *
             * 【为什么给到两位小数】
             * 1) 与 D1 网页上的 R 保持同一精度（那边也是两位小数）；
             * 2) 模型对 R 的容差只有 ±2%，一位小数看不出这点漂移，
             *    两位小数才看得出 R 稳不稳。
             *
             * 【宽度为什么要固定 5 个字】
             * 正常态 "13.50"（2 位整数 + 小数点 + 2 位小数）正好 5 个字，
             * 无效态 "--.--" 也是 5 个字，切换时不会留下残影。
             * 数值只占 1 位整数时会显示成 " 9.87"，前导位是空格，右对齐。 */
            if (d->r_valid != 0U)  /* R 有效，电机在转且电压通路正常 */
            {
                OLED_ShowFloat(R_VAL_X, ROW_R,
                               (d->r_ratio > 99.99f) ? 99.99f : d->r_ratio,  /* 字段只有 2 位整数，先钳位 */
                               2U, 2U, FONT_BIG);  /* 输出 xx.xx */
            }
            else  /* R 无效 */
            {
                OLED_ShowString(R_VAL_X, ROW_R, "--.--", FONT_BIG);  /* 显示 --.--，与正常态等长 */
            }
            break;  /* 结束 2 区域 */

        case 3U:  /* 区域 3：母线电压 + 状态 */
            /* ---- 母线电压和状态（小字号，数值/无效标记均为四列） ---- */
            if (d->vbat_valid != 0U)  /* 电压有效 */
            {
                OLED_ShowFloat(SML_VALUE_X, ROW_SIM, d->vbat_v,  /* 2 位整数 + 1 位小数 */
                               2U, 1U, FONT_SMALL);  /* 形如 xx.x */
            }
            else  /* 电压无效 */
            {
                OLED_ShowString(SML_VALUE_X, ROW_SIM, "--.-", FONT_SMALL);  /* 显示 --.- */
            }

            /* ---- 中间 5 格：学习阶段显示进度，检测阶段显示相似度 ----
             * 两种内容长度都是 5 个字，整格覆盖、互不留残影：
             *   学习阶段 "32/64" = 已补学的窗数 / 目标窗数
             *   检测阶段 "S:100" = AI 相似度（100 表示与正常样本完全一致）
             * 学习阶段显示进度，是因为那时相似度还没成型，
             * 而「还差多少窗才能开始检测」才是当下最该让人看到的事。 */
            if ((d->ai_phase == 1U) && (d->learn_total > 0U))  /* 学习阶段且有进度 */
            {
                char    lp[6];  /* 5 个可见字 + 串终止符 */
                uint8_t done = d->learn_done;  /* 已补学的窗数 */
                uint8_t total = d->learn_total;  /* 目标窗数 */

                if (done > 99U)  /* 显示上限保护 */
                {
                    done = 99U;  /* 夹到 99 */
                }
                if (done > total)  /* 不可能发生，但夹一下免得显示成 m+1 */
                {
                    done = total;  /* 夹到目标值 */
                }
                if (total > 99U)  /* 目标窗数上限保护 */
                {
                    total = 99U;  /* 夹到 99 */
                }

                lp[0] = (char)('0' + (done / 10U));  /* 已完成数的十位 */
                lp[1] = (char)('0' + (done % 10U));  /* 已完成数的个位 */
                lp[2] = '/';  /* 分隔符 */
                lp[3] = (char)('0' + (total / 10U));  /* 目标数的十位 */
                lp[4] = (char)('0' + (total % 10U));  /* 目标数的个位 */
                lp[5] = 0;  /* 字符串终止 */
                OLED_ShowString(SML_SLOT_X, ROW_SIM, lp, FONT_SMALL);  /* 显示 32/64 */
            }
            else  /* 检测阶段（或没有进度数据）：显示相似度 */
            {
                char     sim[6];  /* "S:" + 3 个可见字 + 串终止符 */
                uint32_t v;  /* 取整后的相似度 0 ~ 100 */

                sim[0] = 'S';  /* 相似度标签首字符 */
                sim[1] = ':';  /* 相似度标签冒号 */

                /* 只有「检测阶段 + 模型给了有效结果」才显示数值。
                 * 学习阶段、窗口刚被丢掉、模型报错时显示 --- ，
                 * 免得把「还没有结果」显示成「相似度 0」而误导。 */
                if ((d->ai_valid != 0U) && (d->ai_phase == 2U))  /* 检测阶段且有结果 */
                {
                    v = (uint32_t)(d->similarity + 0.5f);  /* 四舍五入到整数，屏上只显示整数 */
                    if (v > 100U)  /* 库理论上不会超过 100，这里防越界 */
                    {
                        v = 100U;
                    }
                    if (v >= 100U)  /* 三位数原样显示 */
                    {
                        sim[2] = '1';
                        sim[3] = '0';
                        sim[4] = '0';
                    }
                    else if (v >= 10U)  /* 两位数：前面补一个空格，右对齐 */
                    {
                        sim[2] = ' ';
                        sim[3] = (char)('0' + (v / 10U));
                        sim[4] = (char)('0' + (v % 10U));
                    }
                    else  /* 一位数：前面补两个空格 */
                    {
                        sim[2] = ' ';
                        sim[3] = ' ';
                        sim[4] = (char)('0' + v);
                    }
                }
                else  /* 没有有效结果 */
                {
                    sim[2] = '-';
                    sim[3] = '-';
                    sim[4] = '-';
                }
                sim[5] = '\0';
                OLED_ShowString(SML_SLOT_X, ROW_SIM, sim, FONT_SMALL);  /* 显示 S:100 */
            }

            st = d->state;  /* 先取主循环判定的状态 */
            /* 任一传感器无效时，WARN 优先于其它状态判断 */
            if ((d->speed_valid == 0U) || (d->current_valid == 0U) ||  /* 任一传感器无效 */
                (d->vbat_valid == 0U))  /* 与上一行构成或关系 */
            {
                st = (uint8_t)UI_ST_WARN;  /* 强制显示 WARN */
            }
            if (st > (uint8_t)UI_ST_ALARM)  /* 索引越界保护 */
            {
                st = (uint8_t)UI_ST_ALARM;  /* 夹到最大值 */
            }
            OLED_ShowString(SML_STATE_V_X, ROW_SIM, STATE_TXT[st], FONT_SMALL);  /* 显示状态文字 */
            break;  /* 区域 3 结束 */

        default:  /* 区域 4（default）：底条 */
            /* ---- 底条：AI 阶段/模拟标记 + 档位 + 学习进度 + 运行秒数 + 心跳 ---- */
            if (d->ai_phase == 1U)  /* 板上 AI 正在学习 */
            {
                OLED_ShowString(BAR_SIM_X, ROW_BAR, "LRN", FONT_SMALL);  /* 显示 LRN */
            }
            else if (d->ai_phase == 2U)  /* 板上 AI 正在检测 */
            {
                OLED_ShowString(BAR_SIM_X, ROW_BAR, "DET", FONT_SMALL);  /* 显示 DET */
            }
            else if (d->demo != 0U)  /* 纯采集/仿真模式 */
            {
                OLED_ShowString(BAR_SIM_X, ROW_BAR, "SIM", FONT_SMALL);  /* 显示 SIM */
            }
            else  /* 其它情况 */
            {
                OLED_ShowString(BAR_SIM_X, ROW_BAR, "   ", FONT_SMALL);  /* 清掉该字段 */
            }

            /* 当前电机档位：只显示档位号 L1 / L2，停机档显示 OFF。
             * 【2026-10-22 改法】原来这里画的是 "L1:58%"，把内部占空比
             * 直接暴露在屏上，答辩时观众看不懂 58 是什么。
             * 现在只显示使用者真正关心的操作量（几档），
             * 占空比属于内部标定参数，屏上不再显示，需要时看 D1 网页。
             * 三种状态都是 3 个字宽，切换时不会留残影。 */
            {  /* 局部作用域：档位字符串缓冲 */
                char    lv[5];  /* 最多 4 字 + 串终止符 */

                if (d->level == UI_LEVEL_SCAN)  /* 自动扫描档（历史功能，保留显示） */
                {
                    OLED_ShowString(BAR_LV_X, ROW_BAR, "MIN", FONT_SMALL);  /* 显示 MIN */
                }
                else if (d->level == 0U)  /* 停机档 */
                {
                    OLED_ShowString(BAR_LV_X, ROW_BAR, "OFF", FONT_SMALL);  /* 显示 OFF */
                }
                else  /* 正常档位 1 / 2 */
                {
                    lv[0] = 'L';  /* 前缀 L */
                    lv[1] = (char)('0' + (d->level % 10U));  /* 档位数字 */
                    lv[2] = ' ';  /* 补一个空格，凑满 3 字宽 */
                    lv[3] = 0;  /* 字符串终止 */
                    OLED_ShowString(BAR_LV_X, ROW_BAR, lv, FONT_SMALL);  /* 显示 L1 / L2 */
                }
            }  /* 档位显示结束 */

            /* ---- dR 跳变幅度（小字号，4 个字）----
             * dR = 窗内 rpm/V 的绝对峰峰值，也就是捏一下轴时转速的掉幅。
             * 稳定运行只有 0.3~0.6，捏轴时顶到 2.1~2.5，阈值 1.0。
             * 它不随档位平移，是真正的判定量，所以常驻底条，
             * 不用切页就能看到「现在离报警还有多远」。
             * 无效态画 "----"（同样 4 个字），切换时不留残影。 */
            if (d->ripple_valid != 0U)  /* 窗口已满，幅度可用 */
            {
                OLED_ShowFloat(BAR_DR_V_X, ROW_BAR,
                               (d->ripple_abs > 9.99f) ? 9.99f : d->ripple_abs,  /* 字段只有 1 位整数，先钳位 */
                               1U, 2U, FONT_SMALL);  /* 输出 x.xx */
            }
            else  /* 窗口没满，先不给数 */
            {
                OLED_ShowString(BAR_DR_V_X, ROW_BAR, "----", FONT_SMALL);  /* 显示 ----，与正常态等长 */
            }

            OLED_ShowNum(BAR_TIME_V_X, ROW_BAR, d->uptime_s,  /* 运行秒数 */
                         TIME_DIGITS, FONT_SMALL);  /* 6 位、小字号 */
            OLED_ShowString(BAR_TIME_U_X, ROW_BAR, "s", FONT_SMALL);  /* s 单位 */
            OLED_ShowChar(BAR_SPIN_X, ROW_BAR, SPIN[spin], FONT_SMALL);  /* 画心跳风车 */
            spin = (uint8_t)((spin + 1U) % 4U);  /* 相位前移，形成动画 */
            break;  /* 区域 4 结束 */
    }

    refreshRegion = (uint8_t)((refreshRegion + 1U) % UI_REFRESH_REGIONS);  /* 指向下一个区域（循环） */
}
