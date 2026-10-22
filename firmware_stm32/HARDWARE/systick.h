/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    systick.h
  Author:      汪贤云
  Version:     1.0
  Date:        2026-09-21
  Description: 阻塞式延时接口。基于 Cortex-M4 内核 SysTick，不占用外设定时器。
               阻塞式延时，无法进行任何输入输出，只能用于上电初始化和传感器等待。
  Function List:
    1. SysTick_Init()  建立延时基准（必须先调用）
    2. Delay_us()      微秒级延时
    3. Delay_ms()      毫秒级延时
  History:
    <author>   <time>      <version>   <desc>
    汪贤云     2026-09-21    1.0         创建文件
    汪贤云     2026-10-16    1.1         按《华为编程规范》补充中文注释
    汪贤云     2026-10-16    1.1         按《华为编程规范》补充中文注释
    汪贤云     2026-10-16    1.1         按《华为编程规范》补充中文注释
***********************************************************/
#ifndef __SYSTICK_H
#define __SYSTICK_H
#include "stm32f4xx.h"    /* 标准外设库声明 */

void SysTick_Init(void);       /* 建立延时基准（只调用一次） */
void Delay_ms(uint32_t nTime); /* 毫秒级阻塞延时 */



#endif  /* __SYSTICK_H 头文件结束 */
