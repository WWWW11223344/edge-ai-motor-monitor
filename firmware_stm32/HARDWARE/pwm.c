#include "pwm.h"        /* 本模块对外接口声明 */
#include "stm32f4xx.h"   /* StdPeriph 外设寄存器定义 */

/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    pwm.c
  Author:      汪贤云
  Version:     1.0
  Date:        2026-09-21
  Description: PWM 驱动，提供 LED 调光与电机调速两路独立输出。
               LED：PF9 / TIM14_CH1，500Hz，占空比 40%（仅调光用）。
               电机：PE5、PE6 / TIM9_CH1、CH2，20kHz（高于人耳听觉上限，避免啸叫），
               驱动方式为双 PWM 输入，一路给占空比另一路给 0 即可调速并确定转向。
               PA8 接驱动板 STBY 使能脚，必须由固件拉高，否则驱动板处于待机、电机不转。
               参数推导：TIM14 时钟 84MHz(PSC=84-1) 得 1MHz、ARR=2000 得 500Hz；
               TIM9 时钟 168MHz(PSC=84-1) 得 2MHz、ARR=100 得 20kHz。
  Function List:
    1. PWM_LED_Init()      LED 调光 PWM 初始化
    2. PWM_Motor_Config()  电机 PWM 初始化（含 STBY 使能）
    3. Motor_Config()      设置转速与方向
  History:
    <author>   <time>      <version>   <desc>
    汪贤云     2026-09-21    1.0         创建文件
    汪贤云     2026-10-16    1.1         按《华为编程规范》补充中文注释
***********************************************************/


/*-----------------------------------------------------------
  Function:      PWM_LED_Init
  Description:   配置 TIM14_CH1 在 PF9 输出 500Hz PWM 用于 LED 调光，
                 初始占空比 40%（CCR = 800，ARR+1 = 2000）。
  Calls:         GPIO_Init()、GPIO_PinAFConfig()、TIM_TimeBaseInit()、TIM_OC1Init()、TIM_OC1PreloadConfig()、TIM_Cmd()
  Called By:     main.c（需要 LED 调光时调用）
  Input:         无
  Output:        无
  Return:        无
  Others:        与 led.c 的 LED_Init() 复用同一引脚（PF9），两者不可同时生效。
-----------------------------------------------------------*/
void PWM_LED_Init(void)
{
	/* 三个配置结构体：GPIO 引脚 / 定时器输出比较(OC=Output Compare) / 定时器时基
	 * 一定要写 = {0} 清零：不清零的话，栈上残留的垃圾值会被当成参数写进寄存器
	 * （本工程 TIM9 就被这条坑过一次，症状是电机完全不转，详见下面
	 *   PWM_Motor_Config() 里 TIM_ClockDivision 那段说明） */
	GPIO_InitTypeDef  GPIO_InitStructure = {0};       /* GPIO 引脚配置结构体，必须清零（防栈垃圾写进寄存器） */
	TIM_OCInitTypeDef  TIM_OCInitStructure = {0};     /* 输出比较 OC 配置结构体，必须清零 */
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = {0}; /* 时基配置结构体，必须清零 */
	
   	/* 开时钟：STM32 的外设默认全部断电，用之前必须先「通电」
   	 * GPIOF 挂在 AHB1 总线上，TIM14 挂在 APB1 总线上
   	 * 忘了开时钟 = 后面所有配置都写进「空气」，引脚毫无反应 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE); /* 使能 TIM14 时钟（LED 呼吸灯用） */

   //结构体成员的配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;          //要配哪根引脚：PF9
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       //AF=复用功能，把引脚交给定时器管，而不是当普通 IO
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     //推挽输出：能主动输出强高/强低电平
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //引脚翻转速度，输出 PWM 用最高档
	
	//初始化端口：把上面配好的参数真正写进 GPIOF 的寄存器
    GPIO_Init(GPIOF, &GPIO_InitStructure);
	
	//引脚复用：告诉芯片「PF9 这根脚的复用功能选 TIM14」
	//一个引脚能当好几种外设用（叫 AF0~AF15），不指定的话定时器信号到不了引脚
	 GPIO_PinAFConfig(GPIOF, GPIO_PinSource9, GPIO_AF_TIM14);
	
	/* 定时器时钟 84 MHz 经 PSC=84-1 分频得 1 MHz，即每 1 us 数一个数；
	 * ARR=2000-1 时数满 2000 个数 = 2 ms，对应 500 Hz。 */
	TIM_TimeBaseStructure.TIM_Prescaler = 84-1;//PSC：84MHz / 84 = 1MHz，即每 1us 数一个数
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//计数模式：从 0 往上数到 ARR 再回 0
    TIM_TimeBaseStructure.TIM_Period =2000-1 ;//ARR：数满 2000 个数是一个周期 -> 2ms -> 500Hz
 
	//初始化配置：把时基参数写进 TIM14
    TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);
	
   	/* 输出比较通道配置（决定引脚上的波形长什么样） */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             //PWM 模式1：CNT < CCR 时输出有效电平
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //打开这个通道的输出，否则引脚不输出波形
    TIM_OCInitStructure.TIM_Pulse = 800;//CCR 比较值：800/2000 = 40% 占空比
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //有效电平=高电平（极性不反）
 
    TIM_OC1Init(TIM14, &TIM_OCInitStructure);//把通道1的配置写进 TIM14
 
	//将ccr1放入「影子寄存器」中，后续随时可改变
	//使能预装载后，改 CCR 不会立刻生效，而是等当前周期结束再换新值，
	//这样调亮度时波形不会出现毛刺（半个周期的怪波形）
    TIM_OC1PreloadConfig(TIM14, TIM_OCPreload_Enable);
	TIM_Cmd(TIM14, ENABLE);//最后一步：启动定时器，引脚上才真正开始出波形

	
	
}


//PE5、PE6--->TIM9-ch1、ch2---》Ain1、Ain2
/*-----------------------------------------------------------
  Function:      PWM_Motor_Config
  Description:   配置 TIM9_CH1/CH2 在 PE5/PE6 输出 20kHz PWM 驱动电机，
                 并把 PA8(STBY) 拉高使能驱动板；初始占空比为 0。
  Calls:         GPIO_Init()、GPIO_PinAFConfig()、TIM_TimeBaseInit()、TIM_OC1Init()、TIM_OC1PreloadConfig()、TIM_Cmd()
  Called By:     main.c 初始化部分（只调用一次）
  Input:         无
  Output:        无
  Return:        无
  Others:        1) TIM9 时钟 168MHz，PSC=84 得 2MHz，ARR=100 得 20kHz，故 CCR 数值即占空比百分数；
                                  2) 必须显式给 TIM_ClockDivision 赋值，否则栈上残留值会把 CR1 的
                                     UDIS/OPM/ARPE 误置位，导致 ARR 永不生效、各档位都不转。
-----------------------------------------------------------*/
void PWM_Motor_Config(void)
{
	//定义GPIO外设结构体变量
	GPIO_InitTypeDef  GPIO_InitStructure = {0};       /* GPIO 引脚配置结构体，必须清零 */
	TIM_OCInitTypeDef  TIM_OCInitStructure = {0};     /* 输出比较 OC 配置结构体，必须清零 */
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = {0}; /* 时基配置结构体，必须清零 */
	
   	/* 开时钟：GPIOE（PE5/PE6）、GPIOA（PA8 使能脚）在 AHB1，TIM9 在 APB2 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE); /* 使能 TIM9 时钟（两路电机 PWM） */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);   /* PA8 = STBY 使能脚 */
 
   //结构体成员的配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;//一次配两根脚：PE5 和 PE6（用 | 把位或起来）
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能，交给 TIM9
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //高速档
	
	//初始化端口
    GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	//分别指定两根引脚的复用功能：PE5=TIM9_CH1，PE6=TIM9_CH2
	 GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);
	 GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_TIM9); /* PE6 切到 TIM9 通道 2（BIN2） */

	/* ---- STBY（TB6612 使能脚）接到 PA8 ----
	 * D153C 手册：STBY=1 正常工作，STBY=0 待机（输出高阻）。
	 * 硬件上 STBY 是飞线到单片机的 PA8，不是常接 3.3V，
	 * 所以必须由固件把 PA8 拉高，驱动芯片才会输出。
	 * 漏掉这一步，STBY 悬空 => 驱动板待机 => 电机不转或极弱。 */
	{
		//这里单独用一个局部结构体，配 PA8 为普通推挽输出
		GPIO_InitTypeDef GPIO_InitSTBY = {0};
		GPIO_InitSTBY.GPIO_Pin   = GPIO_Pin_8; /* STBY 接在 PA8 */
		GPIO_InitSTBY.GPIO_Mode  = GPIO_Mode_OUT;            //普通输出，不接定时器
		GPIO_InitSTBY.GPIO_OType = GPIO_OType_PP;   /* 推挽输出 */
		GPIO_InitSTBY.GPIO_Speed = GPIO_Speed_100MHz;   /* 高速档 */
		GPIO_InitSTBY.GPIO_PuPd  = GPIO_PuPd_NOPULL;         //不用内部上下拉，外部电路决定
		GPIO_Init(GPIOA, &GPIO_InitSTBY);        /* 写入 GPIOA 配置 */
	}
	GPIO_SetBits(GPIOA, GPIO_Pin_8);   /* STBY = 1，使能驱动板 */
	
	/* APB2 总线 84 MHz，APB2 分频不为 1 时定时器时钟取总线 2 倍 = 168 MHz；
	 * PSC=84-1 分频得 2 MHz，ARR=100-1 时数满 100 个数 = 50 us，对应 20 kHz。 */
	TIM_TimeBaseStructure.TIM_Prescaler = 84-1;//PSC：168MHz/84 = 2MHz，每 0.5us 数一个数
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//计数模式
    TIM_TimeBaseStructure.TIM_Period =100-1 ;//ARR：100 个数 = 50us -> 20kHz；CCR 数值刚好等于占空比百分数
	/* 【根治】TIM_ClockDivision 必须显式赋值！
	   StdPeriph 的 TIM_TimeBaseInit 先清掉 CR1 的 bit8~9，然后把
	   TIM_ClockDivision 这个 16 位字段整段 OR 进 CR1。本结构体从未
	   给该字段赋值，栈上残留值（实测 0x038A）会把 CR1 的
	   UDIS(bit1) / OPM(bit3) 置 1：
	     UDIS=1 -> 更新事件被关闭；而垃圾值 bit7=1 又顺手把 ARPE 打开了，
   于是 ARR 预装载值 99 只能靠更新事件才生效 -> 永远加载不上
	     -> 计数器按复位值 0xFFFF 自由跑（TIM9 时钟 2MHz，周期 50us 变 33ms）
	     -> 实际占空比不到 0.1% -> 电机切到哪一档都不转。
	   实测症状：CR1=0x38B，SR 的 UIF 一直为 0，
	   CNT 读到 15965 / 63493（远超 ARR=99）。
	   同样写法的 encoder.c 显式赋了 TIM_CKD_DIV1，所以编码器一直正常。*/
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//数字滤波用的时钟分频：不滤波，写 DIV1 明确表意
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;//高级定时器才用的重复计数，TIM9 用不到，写 0 保证干净
	TIM9->CR1 = 0;   /* 保险：把 CR1 里可能残留的 UDIS/OPM/CKD 位彻底清掉 */
 
	//初始化配置：把时基参数写进 TIM9
    TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);
	
   	/* 配置两个输出比较通道，参数共用一个结构体（两个通道都用 PWM 模式1） */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             //PWM 模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //打开输出
    TIM_OCInitStructure.TIM_Pulse = 0;//CCR 初始值 0：上电先占空比 0%，电机不转（安全）
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //极性高：CNT<CCR 时输出高
 
    TIM_OC1Init(TIM9, &TIM_OCInitStructure);//通道1 -> PE5 -> BIN1
	TIM_OC2Init(TIM9, &TIM_OCInitStructure);//通道2 -> PE6 -> BIN2
 

	//将结构体值写入寄存器
	//使能预装载：以后改 CCR 会在本周期结束时才生效，波形不会撕裂
    TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable); /* 通道 2 同样开启预装载 */
	TIM_Cmd(TIM9, ENABLE);//启动 TIM9，PE5/PE6 开始输出波形（此时占空比还是 0）
	
}

//电机控制函数
/*-----------------------------------------------------------
  Function:      Motor_Config
  Description:   设置电机转速与方向：把 speed 写入需要导通的一路 CCR，另一路写 0。
  Calls:         TIM_SetCompare1()、TIM_SetCompare2()
  Called By:     main.c（上电、按键换档、急停）
  Input:         speed —— 占空比百分数（0~100，本工程上限 58）
                                  dir   —— 1 = 正转（CH1 输出）；其它值 = 反转（CH2 输出）
  Output:        无
  Return:        无
  Others:        speed 上限由调用方把关：母线 12.6V x 58% = 7.3V 已接近电机额定 7.4V。
-----------------------------------------------------------*/
void Motor_Config(uint16_t speed,uint16_t dir)
{
	if(dir == 1)   /* dir = 1：正转 */
	{
		//正转：CH1 出 PWM，CH2 拉 0
		TIM_SetCompare1(TIM9,speed);//改 CCR1 -> 其实就是改 PE5 上的占空比
		TIM_SetCompare2(TIM9,0); /* 反转通道占空比清零 */
	}
	else   /* 其它值：反转 */
	{
		//翻转：CH2 出 PWM，CH1 拉 0
		TIM_SetCompare1(TIM9,0);
		TIM_SetCompare2(TIM9,speed); /* CH2 输出本次速度（反转） */
	}
	
}
