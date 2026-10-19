/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    systick.c
  Author:      汪贤云
  Version:     1.0
  Date:        2026-09-21
  Description: 用 Cortex-M4 内核自带的 SysTick 计数器实现阻塞式延时，不占用外设定时器。
               时间基准：时钟源取 HCLK/8 = 168MHz/8 = 21MHz，即 21 个计数为 1us、
               21000 个计数为 1ms。
               SysTick 只有 24 位（最大 0xFFFFFF），单次最长延时 798ms，
               因此 Delay_ms() 内部把长延时拆成若干 798ms 的段再加余数。
  Function List:
    1. SysTick_Init()  建立延时基准，算出每微秒/每毫秒的计数值
    2. Delay_us()      微秒级阻塞延时
    3. Delay_ms()      毫秒级阻塞延时（自动分段，支持任意毫秒数）
  History:
    <author>   <time>      <version>   <desc>
    汪贤云     2026-09-21    1.0         创建文件
    汪贤云     2026-10-16    1.1         按《华为编程规范》补充中文注释
***********************************************************/
#include "SysTick.h"
#include "stm32f4xx.h" 

static uint16_t delay_us = 0;  /* 1 微秒对应的 SysTick 计数（= HCLK/8 的兆赫数 21） */
static uint16_t delay_ms = 0;  /* 1 毫秒对应的 SysTick 计数（= delay_us 乘 1000 = 21000） */

/*-----------------------------------------------------------
  Function:      SysTick_Init
  Description:   初始化 SysTick 延时基准。时钟源设为 HCLK/8（21MHz），
                 并据此算出每微秒、每毫秒需要计数多少个。
  Calls:         无
  Called By:     main.c 初始化部分（只调用一次）
  Input:         无
  Output:        无
  Return:        无
  Others:        必须在 Delay_us()/Delay_ms() 之前调用；否则换算常数为 0，延时长度会变成 0。
-----------------------------------------------------------*/
void SysTick_Init(void)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);   /* 时钟源取 HCLK/8 = 168/8 = 21 MHz */
	delay_us = 168/8;   /* 21 个计数 = 1 微秒 */
	delay_ms = delay_us * 1000;   /* 21000 个计数 = 1 毫秒 */
}


/*-----------------------------------------------------------
  Function:      Delay_us
  Description:   微秒级阻塞延时。向 LOAD 装载计数值后启动计数器，
                 轮询 CTRL 寄存器的 COUNTFLAG 位等待计数结束。
  Calls:         无
  Called By:     需要微秒级等待的驱动（OLED / MPU6050 的软件 I2C 时序）
  Input:         nTime —— 延时微秒数，需满足 nTime * 21 <= 0xFFFFFF（约 79 万）
  Output:        无
  Return:        无
  Others:        阻塞式等待，期间 CPU 无法执行其他任务。
-----------------------------------------------------------*/
void Delay_us(uint32_t nTime)
{

	
	uint32_t temp;	/* 暂存 CTRL 寄存器值 */
	
	SysTick->LOAD = nTime * delay_us ;   //重装载寄存器，最大计数值
	SysTick->VAL = 0x00	;				//清空计数值
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk ; // 最低位为1时使能，开启倒数；core_cm4.h的642行
	
	do	/* 轮询等待计数到 0 */
	{
		temp=SysTick->CTRL;//先读出来状态值
		//先判断第一位是否使能；检测CTRL的第16位如果是1（移位后是65536），则倒数计数结束
	}while((temp&0x01)&& !(temp & (1<<16)));
	
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk ; // 关闭计数器
	SysTick->VAL = 0x00	;	//清空计数值
	
}


/*-----------------------------------------------------------
  Function:      Delay_xms
  Description:   毫秒级延时的内部实现，单次最多延时 798ms。
                 外部接口 Delay_ms() 负责把长延时拆成不超过 798ms 的分段后调用本函数。
  Calls:         无
  Called By:     Delay_ms()
  Input:         nTime —— 延时毫秒数，由调用者保证不超过 798
  Output:        无
  Return:        无
  Others:        static 内部函数，外部不可直接调用。
-----------------------------------------------------------*/
static void Delay_xms(uint32_t nTime)
{

	uint32_t temp;	/* 暂存 CTRL 寄存器值 */
	
	
	SysTick->LOAD = nTime * delay_ms ;   //重装载寄存器，最大计数值
	SysTick->VAL = 0x00	;				//清空计数值
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk ; // 开启倒数
	
	do	/* 轮询等待计数到 0 */
	{
		temp=SysTick->CTRL;		/* 读一次状态（读 CTRL 会清 COUNTFLAG） */
		
	}while((temp&0x01)&& !(temp & (1<<16))); //等待倒数到0为止
	
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk ; // 关闭计数器
	SysTick->VAL = 0x00	;	//清空计数值
	
}


/*-----------------------------------------------------------
  Function:      Delay_ms
  Description:   毫秒级阻塞延时。因 SysTick 为 24 位计数器，单段最长 798ms，
                 本函数先按 798ms 拆成 repeat 段，再补一段余数。
  Calls:         Delay_xms()
  Called By:     上电初始化、传感器稳定等待、按键学习期等
  Input:         nTime —— 延时毫秒数（任意大小，内部自动分段）
  Output:        无
  Return:        无
  Others:        阻塞式；不要在数据采集或 AI 运行期间使用长延时。
-----------------------------------------------------------*/
void Delay_ms(uint32_t nTime)
{
	uint32_t repeat = nTime / 798U;//重复调用次数--》取商
	uint32_t remain = nTime % 798U;//取余
	while(repeat)	/* 先按整段 798 ms 循环 */
	{
		Delay_xms(798);		/* 单段最长 798 ms（24 位计数器上限） */
		repeat--;		/* 剩余整段数减一 */
	}
	
	if(remain)	/* 有余数再补一段 */
		Delay_xms(remain);		/* 补足剩余毫秒数 */
	//例如2000ms--》2个798余若干
}

