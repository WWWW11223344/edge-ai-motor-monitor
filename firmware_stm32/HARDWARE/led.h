/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    led.h
  Author:      汪贤云
  Version:     1.0
  Date:        2026-09-21
  Description: LED 驱动接口。板载 LED0 接在 PF9，低电平点亮。
               使用前必须先调用 LED_Init() 完成引脚配置。
  Function List:
    1. LED_Init()  初始化 LED0 使用的引脚
  History:
    <author>   <time>      <version>   <desc>
    汪贤云     2026-09-21    1.0         创建文件
    汪贤云     2026-10-16    1.1         按《华为编程规范》补充中文注释
    汪贤云     2026-10-16    1.1         按《华为编程规范》补充中文注释
    汪贤云     2026-10-16    1.1         按《华为编程规范》补充中文注释
***********************************************************/
#ifndef _LED_H_
#define _LED_H_

void LED_Init(void);       /* 板载 LED0（PF9）初始化 */

#endif  /* __LED_H 头文件结束 */
