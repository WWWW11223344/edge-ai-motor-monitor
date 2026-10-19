/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    led.c
  Author:      汪贤云
  Version:     1.0
  Date:        2026-09-21
  Description: 板载 LED0（PF9）的初始化。LED0 为低电平点亮。
               PF9 与本工程 pwm.c 中的 PWM_LED_Init() 复用同一引脚，
               两者不可同时生效：后调用的配置会覆盖先调用的。
  Function List:
    1. LED_Init()  配置 PF9 为推挽输出
  History:
    <author>   <time>      <version>   <desc>
    汪贤云     2026-09-21    1.0         创建文件
***********************************************************/
#include "led.h"
#include "stm32f4xx.h"

//PF9--》LED0--》低亮
/*-----------------------------------------------------------
  Function:      LED_Init
  Description:   把 PF9 配置为推挽输出，用于驱动板载 LED0。
                 内部上拉使引脚在上电高阻期间保持高电平，LED 不会闪一下。
  Calls:         无
  Called By:     main.c 初始化部分（只调用一次）
  Input:         无
  Output:        无
  Return:        无
  Others:        本函数只做引脚配置，不点亮 LED；点灯由 main.c 中的 GPIO 操作完成。
-----------------------------------------------------------*/
void LED_Init(void)
{
	
	//定义GPIO外设结构体变量
	GPIO_InitTypeDef  GPIO_InitStructure;
	//开关--》时钟开关、时钟使能
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
 
   //结构体成员的配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;//指定引脚号
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出工作模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//引脚高速响应
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉模式
	
	//初始化端口
    GPIO_Init(GPIOF, &GPIO_InitStructure);
	
	
}
