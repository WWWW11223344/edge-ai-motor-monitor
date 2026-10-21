/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    ui.c
  Author:      汪贤云
  Version:     1.2
  Date:        2026-09-22
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
#define R_UNIT_X        80U     /* R 行的单位 "rpm/V" 有 5 个字，从 80 起才不会出屏 */

/* 小字号行的列位置 */
#define SML_LABEL_X     0U      /* "Bat"（3 个字 = 18px） */
#define SML_VALUE_X     24U     /* xx.x（4 个字 = 24px，正好到 48） */
#define SML_BAT_U_X     48U     /* "V"（1 个字 = 6px，占到 54 为止） */
#define SML_SIM_L_X     60U     /* "S:" —— AI 相似度标签（2 个字 = 12px）。54~59 空出一格，与 "V" 之间留一个空格 */
#define SML_SIM_V_X     72U     /* 相似度数值 3 个字右对齐（72~89），0~100 都放得下 */
#define SML_STATE_V_X   92U     /* 状态文字 5 个字 = 30px（92~121），不出屏 */

#define BAR_SIM_X       0U      /* SIM / LRN / DET 标记 */
#define BAR_LV_X        24U     /* 档位显示（SIM 和 T 之间正好 6 个字） */
#define BAR_TIME_L_X    60U    /* T 标签 */
#define BAR_TIME_V_X    66U    /* 秒数数值 */
#define BAR_TIME_U_X    102U   /* s 单位 */
#define BAR_SPIN_X      120U   /* 心跳风车 */

/* 转速最多 5 位（0 ~ 99999 转/分） */
#define SPD_DIGITS      5U      /* 转速显示位数 */
#define TIME_DIGITS     6U      /* 运行秒数显示位数 */
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

    OLED_ShowString(BIG_LABEL_X, ROW_R, "R",     FONT_BIG);  /* 归一化转速比标签 */
    OLED_ShowString(R_UNIT_X,    ROW_R, "rpm/V", FONT_BIG);  /* 归一化转速比单位 */

    /* 第 6 页左边：母线电压（原来是假的相似度 "Sim"，已让位给真数据） */
    OLED_ShowString(SML_LABEL_X,   ROW_SIM, "Bat", FONT_SMALL);  /* 电压标签 */
    OLED_ShowString(SML_BAT_U_X,   ROW_SIM, "V",   FONT_SMALL);  /* 电压单位 */
    OLED_ShowString(SML_SIM_L_X,   ROW_SIM, "S:",  FONT_SMALL);  /* AI 相似度标签 */
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

        case 2U:  /* 区域 2：第一级转速比 R / 跳变幅度 dR 轮显 */
            /* ---- 2026-10-21：这一行每 2 s 在 R 与 dR 之间切换 ----
             * R  = 转速 / 母线电压，是绝对工作点，换档位整体平移；
             * dR = 窗内 rpm/V 的绝对峰峰值，也就是跳变幅度。捏一下停一下
             *      会把它顶到 2.1~2.5，稳定运行只有 0.3~0.6，
             *      而且它不随档位平移，是真正的判定量（阈值 1.0）。
             * 两者单位都是 rpm/V，所以右边的单位标签固定不用重画；
             * 标签在 "R " 与 "dR" 之间切换，退回时多画一个空格擦掉 d。 */
            if (((d->uptime_s / 2U) & 1U) != 0U)  /* 奇数 2 s 段：显示跳变幅度 */
            {
                OLED_ShowString(BIG_LABEL_X, ROW_R, "dR", FONT_BIG);  /* 标签 dR */
                if (d->ripple_valid != 0U)  /* 窗口已满，幅度可用 */
                {
                    OLED_ShowFloat(BIG_VALUE_X, ROW_R,
                                   (d->ripple_abs > 9.99f) ? 9.99f : d->ripple_abs,  /* 字段只有 1 位整数，先钳位 */
                                   1U, 2U, FONT_BIG);  /* 输出 x.xx */
                }
                else  /* 窗口没满，先不给数 */
                {
                    OLED_ShowString(BIG_VALUE_X, ROW_R, "--.--", FONT_BIG);  /* 显示 --.-- */
                }
            }
            else  /* 偶数 2 s 段：显示 R */
            {
                OLED_ShowString(BIG_LABEL_X, ROW_R, "R ", FONT_BIG);  /* 标签 R，多一个空格擦掉上次的 d */
                if (d->r_valid != 0U)  /* R 有效，电机在转且电压通路正常 */
                {
                    OLED_ShowFloat(BIG_VALUE_X, ROW_R, d->r_ratio, 1U, 1U, FONT_BIG);  /* 输出 xx.x */
                }
                else  /* R 无效 */
                {
                    OLED_ShowString(BIG_VALUE_X, ROW_R, "--.-", FONT_BIG);  /* 显示 --.- */
                }
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

            /* ---- AI 相似度：固定 3 个字右对齐，0~100 都放得下 ---- */
            /* 只有「检测阶段 + 模型给了有效结果」才显示数值。
             * 学习阶段、窗口刚被丢掉、模型报错时显示 --- ，
             * 免得把「还没有结果」显示成「相似度 0」而误导。 */
            if ((d->ai_valid != 0U) && (d->ai_phase == 2U))  /* 检测阶段且有结果 */
            {
                char     sim[4];  /* 3 个可见字 + 串终止符 */
                uint32_t v;  /* 取整后的相似度 0 ~ 100 */
                v = (uint32_t)(d->similarity + 0.5f);  /* 四舍五入到整数，屏上只显示整数 */
                if (v > 100U)  /* 库理论上不会超过 100，这里防越界 */
                {
                    v = 100U;
                }
                if (v >= 100U)  /* 三位数原样显示 */
                {
                    sim[0] = '1';
                    sim[1] = '0';
                    sim[2] = '0';
                }
                else if (v >= 10U)  /* 两位数：前面补一个空格，右对齐 */
                {
                    sim[0] = ' ';
                    sim[1] = (char)('0' + (v / 10U));
                    sim[2] = (char)('0' + (v % 10U));
                }
                else  /* 一位数：前面补两个空格 */
                {
                    sim[0] = ' ';
                    sim[1] = ' ';
                    sim[2] = (char)('0' + v);
                }
                sim[3] = '\0';
                OLED_ShowString(SML_SIM_V_X, ROW_SIM, sim, FONT_SMALL);  /* 显示相似度 */
            }
            else  /* 没有有效结果 */
            {
                OLED_ShowString(SML_SIM_V_X, ROW_SIM, "---", FONT_SMALL);  /* 显示 --- */
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
            /* ---- 底条：AI 阶段/模拟标记 + 档位 + 运行秒数 + 心跳 ---- */
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

            /* 当前电机档位：填在 SIM 和 T 中间那 6 个字的位置。 */
            {  /* 局部作用域：档位字符串缓冲 */
                char    lv[7];  /* 最多 6 字 + 串终止符 */
                uint8_t duty = d->duty;  /* 本帧占空比 */

                if (duty > 99U)  /* 显示上限保护 */
                {
                    duty = 99U;  /* 夹到 99 */
                }

                if (d->level == UI_LEVEL_SCAN)  /* 自动扫描档（历史功能，保留显示） */
                {
                    if (duty == 0U)  /* 扫描未找到可用档位 */
                    {
                        OLED_ShowString(BAR_LV_X, ROW_BAR, "NOGO  ", FONT_SMALL);  /* 显示 NOGO */
                    }
                    else  /* 扫描到可用档位 */
                    {
                        lv[0] = 'M';  /* 前缀 M */
                        lv[1] = 'I';  /* I */
                        lv[2] = 'N';  /* N */
                        lv[3] = (char)('0' + (duty / 10U));  /* 占空比十位 */
                        lv[4] = (char)('0' + (duty % 10U));  /* 占空比个位 */
                        lv[5] = '%';  /* 百分号 */
                        lv[6] = 0;  /* 字符串终止 */
                        OLED_ShowString(BAR_LV_X, ROW_BAR, lv, FONT_SMALL);  /* 显示 MINxx% */
                    }
                }
                else  /* 普通固定档位 */
                {
                    lv[0] = 'L';  /* 前缀 L */
                    lv[1] = (char)('0' + (d->level % 10U));  /* 档位数字 */
                    lv[2] = ':';  /* 冒号 */
                    lv[3] = (char)('0' + (duty / 10U));  /* 占空比十位 */
                    lv[4] = (char)('0' + (duty % 10U));  /* 占空比个位 */
                    lv[5] = '%';  /* 百分号 */
                    lv[6] = 0;  /* 字符串终止 */
                    OLED_ShowString(BAR_LV_X, ROW_BAR, lv, FONT_SMALL);  /* 显示 Lx:xx% */
                }
            }  /* 档位显示结束 */

            OLED_ShowString(BAR_TIME_L_X, ROW_BAR, "T", FONT_SMALL);  /* T 标签 */
            OLED_ShowNum(BAR_TIME_V_X, ROW_BAR, d->uptime_s,  /* 运行秒数 */
                         TIME_DIGITS, FONT_SMALL);  /* 6 位、小字号 */
            OLED_ShowString(BAR_TIME_U_X, ROW_BAR, "s", FONT_SMALL);  /* s 单位 */
            OLED_ShowChar(BAR_SPIN_X, ROW_BAR, SPIN[spin], FONT_SMALL);  /* 画心跳风车 */
            spin = (uint8_t)((spin + 1U) % 4U);  /* 相位前移，形成动画 */
            break;  /* 区域 4 结束 */
    }

    refreshRegion = (uint8_t)((refreshRegion + 1U) % UI_REFRESH_REGIONS);  /* 指向下一个区域（循环） */
}
