/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    timer.h
  Author:      汪贤云
  Version:     1.0
  Date:        2026-09-21
  Description: TIM6、TIM7 基本定时器接口。
               TIM6 每 1 秒中断一次，TIM7 每 1 毫秒中断一次。
               两者的中断服务函数均在 stm32f4xx_it.c 中实现。
  Function List:
    1. Tim6_Init()  开启 1 秒定时中断
    2. Tim7_Init()  开启 1 毫秒定时中断（系统心跳）
  History:
    <author>   <time>      <version>   <desc>
    汪贤云     2026-09-21    1.0         创建文件
    汪贤云     2026-10-16    1.1         按《华为编程规范》补充中文注释
    汪贤云     2026-10-16    1.1         按《华为编程规范》补充中文注释
    汪贤云     2026-10-16    1.1         按《华为编程规范》补充中文注释
***********************************************************/
#ifndef _TIMER_H_
#define _TIMER_H_

void Tim6_Init(void);       /* TIM6：1 秒更新中断 */
void Tim7_Init(void);       /* TIM7：1 毫秒更新中断 */

#endif  /* __TIMER_H 头文件结束 */
