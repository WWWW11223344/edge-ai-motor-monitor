/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    key.h
  Author:      汪贤云
  Version:     1.0
  Date:        2026-09-21
  Description: 按键驱动对外接口。按键共三颗：KEY0 = PE4、KEY1 = PE3、KEY2 = PE2，
               均为低电平有效；PA0(WK_UP) 目前不参与按键逻辑。
               按键的有效电平在上电头 200ms 内自动学习（不同批次板子接法可能相反），
               因此业务代码不得写死判断 0 或 1，应使用 Key_Scan() 返回的事件。
  Function List:
    1. Key_Init()      初始化按键引脚
    2. Key_Tick1ms()   1ms 中断内做采样与消抖（由 TIM7 调用）
    3. Key_Scan()      主循环取按键事件
  History:
    <author>   <time>      <version>   <desc>
    汪贤云     2026-09-21    1.0         创建文件
***********************************************************/
#ifndef _KEY_H_
#define _KEY_H_

#include "stm32f4xx.h"  /* 引脚读取需要 GPIO 寄存器定义 */

/* 【按键事件类型】
 *   KEY_EVENT_NONE = 0  没有新事件（主循环每轮都会读到这个，正常）
 *   KEY_EVENT_UP        备用（PA0/WK_UP 现在不参与，保留位置）
 *   KEY_EVENT_0/1/2     第 0/1/2 颗键被按下的瞬间
 * 枚举值是连号的：驱动内部用 (KEY_EVENT_0 + i) 由按键下标直接算出事件，
 * 所以顺序不能随便改。 */
typedef enum
{
	KEY_EVENT_NONE = 0U,	/* 无事件（默认值） */
	KEY_EVENT_UP,		/* 保留：PA0 不参与按键逻辑 */
	KEY_EVENT_0,		/* KEY0 按下（PE4，急停） */
	KEY_EVENT_1,		/* KEY1 按下（PE3，档位 +1） */
	KEY_EVENT_2		/* KEY2 按下（PE2，档位 -1） */
} KeyEvent;  /* 按键事件类型 */

/* 【对外接口】
 *   Key_Init()      上电调一次：配 PE2/PE3/PE4 为输入 + 内部上拉
 *   Key_Tick1ms()   由 TIM7 的 1ms 中断调用：每 5ms 采一次，连续 4 次相同才认（20ms 消抖）
 *   Key_Scan()      主循环调用：取走一个事件（取完自动清空），没有事件返回 KEY_EVENT_NONE
 * 【调用顺序要求】Key_Init() 必须排在 Tim7_Init() 之前，
 * 否则中断一开就可能去读还没配置好的引脚。 */
void Key_Init(void);
void Key_Tick1ms(void);
/* 【直接读电平的宏】返回 1 = 高电平，0 = 低电平。
 * 注意这里读到的是「静止电平」还是「按下」取决于具体接法：
 * 本板（接法 B，按下接地）是：松开读 1、按下读 0。
 * 所以业务代码不要写死判断 0/1，用 Key_Scan() 的事件最省心。 */
KeyEvent Key_Scan(void);

#define KEY_UP 	GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)  /* PA0（不参与逻辑） */
#define KEY0 	GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4)  /* PE4 急停 */
#define KEY1 	GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)  /* PE3 档位 +1 */
#define KEY2 	GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2)  /* PE2 档位 -1 */

#endif  /* __KEY_H 头文件结束 */
