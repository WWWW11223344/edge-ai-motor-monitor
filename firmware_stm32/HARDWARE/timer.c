/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    timer.c
  Author:      汪贤云
  Version:     1.0
  Date:        2026-09-21
  Description: TIM6、TIM7 两个基本定时器的中断配置。
               TIM6 每 1 秒产生一次中断，用于慢速节拍与运行统计；
               TIM7 每 1 毫秒产生一次中断，作为系统心跳，
               按键扫描、各路软件计时、遥测 10ms 节拍都以它为准。
               时钟计算：APB1 = 42MHz，APB1 分频不为 1 时定时器时钟为总线 2 倍，
               即 TIM6/TIM7 时钟 = 84MHz；84MHz/8400 = 10kHz，
               故 TIM6 数 10000 次得 1s，TIM7 数 10 次得 1ms。
  Function List:
    1. Tim6_Init()  配置 TIM6 为 1 秒更新中断
    2. Tim7_Init()  配置 TIM7 为 1 毫秒更新中断
  History:
    <author>   <time>      <version>   <desc>
    汪贤云     2026-09-21    1.0         创建文件
    汪贤云     2026-10-16    1.1         按《华为编程规范》补充中文注释
***********************************************************/
#include "timer.h"
#include "stm32f4xx.h"


//TIM6---》1S中断
/*-----------------------------------------------------------
  Function:      Tim6_Init
  Description:   配置 TIM6：PSC = 8400-1、ARR = 10000-1，产生 1 秒定时中断；
                 NVIC 抢占优先级 0、子优先级 0。
                 中断服务函数位于 stm32f4xx_it.c 的 TIM6_DAC_IRQHandler。
  Calls:         无
  Called By:     main.c 初始化部分（只调用一次）
  Input:         无
  Output:        无
  Return:        无
  Others:        本函数只完成配置与启动，不包含任何中断处理逻辑。
-----------------------------------------------------------*/
void Tim6_Init(void)
{

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;    /* TIM6 时基结构体 */
	NVIC_InitTypeDef NVIC_InitStructure;	/* NVIC 中断优先级结构体 */
	
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);	/* 使能 TIM6 时钟（TIM6 挂在 APB1） */

	/* APB1 总线 42 MHz，APB1 分频不为 1 时定时器时钟取总线 2 倍 = 84 MHz。
	 * 84 MHz / 8400 = 10 kHz，即一个计数 = 0.1 ms；
	 * 数 10000 个数 = 1 s，就是 TIM6 的 1 秒周期。 */
    TIM_TimeBaseStructure.TIM_Prescaler = 8400-1;//PSC
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//计数模式
    TIM_TimeBaseStructure.TIM_Period = 10000-1;//ARR
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;    /* 时钟分割：不使用数字滤波，固定填 0 */
 
	//初始化配置
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn ;//选择中断编号
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;    /* 抢占优先级 0（最高） */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;    /* 子优先级 0 */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;    /* 使能该中断通道 */
    NVIC_Init(&NVIC_InitStructure);     /* 写入 NVIC，正式打开中断 */
	
	//开启定时器中断--》更新中断
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
	
	//启动定时器6
	TIM_Cmd(TIM6,ENABLE);
	


	
	
}

//TIM7---》1ms中断（给编码器测速当采样基准）
/*-----------------------------------------------------------
  Function:      Tim7_Init
  Description:   配置 TIM7：PSC = 8400-1、ARR = 10-1，产生 1 毫秒定时中断；
                 NVIC 抢占优先级 1、子优先级 1（低于 TIM6）。
                 中断服务函数位于 stm32f4xx_it.c 的 TIM7_IRQHandler。
  Calls:         无
  Called By:     main.c 初始化部分（只调用一次）
  Input:         无
  Output:        无
  Return:        无
  Others:        必须在 Key_Init() 之后调用：中断一开启就会去读按键引脚。
-----------------------------------------------------------*/
void Tim7_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;	/* TIM7 时基结构体 */
	NVIC_InitTypeDef NVIC_InitStructure;	/* NVIC 中断优先级结构体 */

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);	/* 使能 TIM7 时钟（TIM7 挂在 APB1） */

	/* 同样地，TIM7 计数时钟 = 84 MHz，PSC 取 8400-1 得 10 kHz，
	 * 一个计数 = 0.1 ms；数 10 个数 = 1 ms，就是 1 毫秒心跳。 */
	TIM_TimeBaseStructure.TIM_Prescaler = 8400-1;//PSC
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//计数模式
	TIM_TimeBaseStructure.TIM_Period = 10-1;//ARR
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;	/* 时钟分割：不使用数字滤波，固定填 0 */

	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);	/* 写入 TIM7 时基配置 */

	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;//选择中断编号
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	/* 抢占优先级 1（低于 TIM6） */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	/* 子优先级 1 */
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	/* 使能该中断通道 */
	NVIC_Init(&NVIC_InitStructure);	/* 写入 NVIC，正式打开中断 */

	//开启定时器中断--》更新中断
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);

	//启动定时器7
	TIM_Cmd(TIM7, ENABLE);
}
