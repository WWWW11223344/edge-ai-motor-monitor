/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    oled.c
  Author:      汪贤云
  Version:     1.0
  Date:        2026-09-21
  Description: 0.96 寸 I2C OLED 驱动实现（STM32F407 标准外设库，软件 I2C）。
               I2C 为开漏总线：输出 1 表示释放（由上拉电阻拉高），输出 0 表示主动拉低。
               SSD1306 每次通信的格式为：从机地址 0x78 -> 控制字节 -> 数据；
               控制字节 0x00 表示后续为命令，0x40 表示后续为显示数据。
               显存 128x64 分 8 页，每页 8 行像素，按「页 + 列」定位，一列占一个字节（低位在上）。
               为避免长时间阻塞主循环，整页数据合并为一次 I2C 事务发送。
  Function List:
    1. OLED_Init()           初始化屏幕
    2. OLED_Clear()          清屏
    3. OLED_SetPos()         设置写入位置
    4. OLED_ShowChar()       显示单个字符
    5. OLED_ShowString()     显示字符串
    6. OLED_ShowNum()        显示无符号整数
    7. OLED_ShowFloat()      显示浮点数
    8. 一组软件 I2C 底层函数
  History:
    <author>   <time>      <version>   <desc>
    汪贤云     2026-10-16    1.0         创建文件并按华为规范补充注释
***********************************************************/

#include "oled.h"
#include "font.h" /* 内置字模表：ASCII 8x16 与中文 16x16 */

/* ==========================================================================
 * 0.96 寸 I2C OLED 驱动 —— STM32F407 标准外设库版（软件 I2C）
 * --------------------------------------------------------------------------
 * 原理：
 *   I2C 是"两根线"的串行总线，SCL 是时钟、SDA 是数据，两者都是开漏结构，
 *   只能把线拉低，拉高靠上拉电阻。所以代码里"输出 1"= 释放总线（靠电阻拉高），
 *   "输出 0"= 主动拉低。这就是为什么 GPIO 要配成 OType_OD（开漏）。
 *
 *   SSD1306 的数据格式：
 *     每次通信先发 从机地址(0x78) -> 应答 -> 控制字节 -> 应答 -> 数据 -> 应答
 *     控制字节 0x00 = 后面跟的是"命令"
 *     控制字节 0x40 = 后面跟的是"显示数据"
 *
 *   显存 128x64，分成 8 个"页"(page)，每页 8 行像素；写入时按"页 + 列"
 *   定位，一列一个字节（低 bit 在上），所以 8x16 的字符要跨两页写。
 * ========================================================================== */

/* -------------------------------------------------------------------------
 * 一、软件 I2C 底层
 * ------------------------------------------------------------------------- */

/* 微秒级延时：168MHz 下这个空循环约 2~3us，对应 I2C 速率约 100~150kHz。
 * 必须要有！不加延时的话 SCL 翻转是纳秒级，OLED 根本跟不上。
 * volatile 是为了防止编译器把这个"没用的循环"优化掉。 */
/*-----------------------------------------------------------
  Function:      I2C_Delay
  Description:   软件 I2C 的位延时，168MHz 下空循环约 2~3us，对应总线速率约 100~150kHz。
  Calls:         无
  Called By:     所有软件 I2C 时序函数
  Input:         无
  Output:        无
  Return:        无
  Others:        必须有：不加延时时 SCL 翻转是纳秒级，OLED 根本跟不上；volatile 防止空循环被优化掉。
-----------------------------------------------------------*/

static void I2C_Delay(void)
{
    volatile uint16_t i = 16;   /* 循环计数初值：16 次空转约 2~3us */
    while (i--)   /* 递减到 0 结束，构成忙等待延时 */
    {
    }
}

#define OLED_SCL_H()      GPIO_SetBits(OLED_GPIO_PORT, OLED_SCL_PIN)   /* 时钟线拉高（释放，由上拉电阻拉高） */
#define OLED_SCL_L()      GPIO_ResetBits(OLED_GPIO_PORT, OLED_SCL_PIN)   /* 时钟线拉低 */
#define OLED_SDA_H()      GPIO_SetBits(OLED_GPIO_PORT, OLED_SDA_PIN)   /* 数据线拉高（释放） */
#define OLED_SDA_L()      GPIO_ResetBits(OLED_GPIO_PORT, OLED_SDA_PIN)   /* 数据线拉低 */
#define OLED_SDA_READ()   GPIO_ReadInputDataBit(OLED_GPIO_PORT, OLED_SDA_PIN)   /* 读数据线电平，用于判断从机应答 */

/*-----------------------------------------------------------
  Function:      OLED_GPIO_Init
  Description:   把 PB8(SCL) 与 PB9(SDA) 配置为开漏输出并释放为高电平。
  Calls:         RCC_AHB1PeriphClockCmd()、GPIO_Init()、GPIO_SetBits()
  Called By:     OLED_Init()
  Input:         无
  Output:        无
  Return:        无
  Others:        开漏是 I2C 的正确接法；板上已有 4.7k 上拉电阻，无需再开内部上拉。
-----------------------------------------------------------*/

static void OLED_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;   /* GPIO 初始化结构体 */

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);   /* 打开 GPIOB 时钟（PB8=SCL、PB9=SDA） */

    GPIO_InitStructure.GPIO_Pin   = OLED_SCL_PIN | OLED_SDA_PIN;   /* 同时配置 SCL 与 SDA 两根线 */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;   /* 通用输出：软件 I2C 需要由程序翻转电平 */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;   /* 开漏：I2C 的标准接法 */
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;    /* 内部弱上拉，双保险 */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(OLED_GPIO_PORT, &GPIO_InitStructure); /* 按上面的参数初始化 SCL(PB8) 与 SDA(PB9) */

    OLED_SCL_H();   /* 空闲态：时钟线释放为高 */
    OLED_SDA_H();   /* 空闲态：数据线释放为高 */
}

/* 起始信号：SCL 高电平期间，SDA 由高变低 */
/*-----------------------------------------------------------
  Function:      I2C_Start
  Description:   产生 I2C 起始信号：SCL 为高电平期间，SDA 由高变低。
  Calls:         无
  Called By:     OLED_WrCmds()、OLED_WrDataBuffer()、I2C_WriteByte() 的调用方
  Input:         无
  Output:        无
  Return:        无
  Others:        起始信号之后引脚保持拉低状态，供后续字节传输使用。
-----------------------------------------------------------*/

static void I2C_Start(void)
{
    OLED_SDA_H();   /* SDA 先置 1 */
    OLED_SCL_H();   /* SCL 拉高：此期间 SDA 变化即构成起始条件 */
    I2C_Delay();   /* 保持时序间隔 */
    OLED_SDA_L();   /* SDA 由高变低 = 起始信号 */
    I2C_Delay();   /* 保持时序间隔 */
    OLED_SCL_L();   /* SCL 拉低，准备发送字节 */
    I2C_Delay();   /* 保持时序间隔 */
}

/* 停止信号：SCL 高电平期间，SDA 由低变高 */
/*-----------------------------------------------------------
  Function:      I2C_Stop
  Description:   产生 I2C 停止信号：SCL 为高电平期间，SDA 由低变高。
  Calls:         无
  Called By:     OLED_WrCmds()、OLED_WrDataBuffer()
  Input:         无
  Output:        无
  Return:        无
  Others:        无
-----------------------------------------------------------*/

static void I2C_Stop(void)
{
    OLED_SDA_L();   /* 先确保 SDA 为低 */
    OLED_SCL_H();   /* SCL 拉高：此期间 SDA 变化即构成停止条件 */
    I2C_Delay();   /* 保持时序间隔 */
    OLED_SDA_H();   /* SDA 由低变高 = 停止信号 */
    I2C_Delay();   /* 保持时序间隔 */
}

/* 写一个字节，返回 1 表示从机给了应答（ACK）。高位先发。 */
/*-----------------------------------------------------------
  Function:      I2C_WriteByte
  Description:   写一个字节（高位先发），并在第 9 个时钟读取从机应答。
  Calls:         无
  Called By:     OLED_WrCmds()、OLED_WrDataBuffer()
  Input:         dat —— 待写入的字节
  Output:        无
  Return:        1 = 从机返回 ACK；0 = 未应答
  Others:        第 9 个时钟主机释放 SDA，通过判断 SDA 是否被从机拉低来识别 ACK。
-----------------------------------------------------------*/

static uint8_t I2C_WriteByte(uint8_t dat)
{
    uint8_t i;   /* 位计数 0~7 */
    uint8_t ack;   /* 从机应答标志 */

    for (i = 0; i < 8; i++)   /* 逐位发送 8 个 bit，高位先出 */
    {
        if (dat & 0x80)   /* 取当前最高位判断发 1 还是 0 */
        {
            OLED_SDA_H();   /* 该位为 1：释放 SDA（靠上拉拉高） */
        }
        else   /* 该位为 0 */
        {
            OLED_SDA_L();   /* 该位为 0：主动拉低 SDA */
        }
        dat <<= 1;   /* 左移一位，把下一位顶到最高位 */

        I2C_Delay();   /* 数据建立后等待稳定 */
        OLED_SCL_H();          /* SCL 拉高期间数据必须保持稳定 */
        I2C_Delay();
        OLED_SCL_L(); /* 拉低时钟，一个位周期结束 */
        I2C_Delay(); /* 时钟拉低后保持一段时间，满足最小低电平宽度 */
    }

    /* 第 9 个时钟：主机释放 SDA，读从机拉低与否来判断应答 */
    OLED_SDA_H();
    I2C_Delay(); /* 释放 SDA 后等电平稳定，再给第 9 个时钟 */
    OLED_SCL_H(); /* 给出第 9 个时钟脉冲，让从机拉低 SDA 表示应答 */
    I2C_Delay();   /* 等电平稳定后再采样应答 */
    ack = (OLED_SDA_READ() == Bit_RESET) ? 1U : 0U;   /* 被拉低 = 有应答 */
    OLED_SCL_L();
    I2C_Delay(); /* 拉低时钟后保持，准备下一位 */

    return ack;   /* 返回应答结果：1 = 有应答 */
}

/* 在同一个 I2C 事务里连续发送多条命令。 */
/*-----------------------------------------------------------
  Function:      OLED_WrCmds
  Description:   在同一个 I2C 事务内连续发送多条命令，减少事务开销。
  Calls:         无
  Called By:     OLED_Init()
  Input:         cmd —— 命令数组首地址；len —— 命令条数
  Output:        无
  Return:        无
  Others:        无
-----------------------------------------------------------*/

static void OLED_WrCmds(const uint8_t *cmd, uint8_t len)
{
    uint8_t i;   /* 命令索引 */

    I2C_Start();   /* 起始信号 */
    I2C_WriteByte(OLED_I2C_ADDR);   /* 发送从机地址 0x78 */
    I2C_WriteByte(0x00);   /* 控制字节 0x00 = 后续为命令 */
    for (i = 0U; i < len; i++)   /* 逐条发送命令 */
    {
        I2C_WriteByte(cmd[i]);   /* 发送第 i 条命令 */
    }
    I2C_Stop();   /* 停止信号，结束本次事务 */
}

/* 在同一个 I2C 事务里连续发送一段显示数据。 */
/*-----------------------------------------------------------
  Function:      OLED_WrDataBuffer
  Description:   在同一个 I2C 事务内连续发送一段显示数据。
  Calls:         无
  Called By:     OLED_Clear()、OLED_ShowChar()、OLED_ShowString() 等
  Input:         data —— 数据首地址；len —— 字节数
  Output:        无
  Return:        无
  Others:        整页合并发送比逐字节发送快很多，可显著缩短对主循环的阻塞。
-----------------------------------------------------------*/

static void OLED_WrDataBuffer(const uint8_t *data, uint16_t len)
{
    uint16_t i;   /* 数据索引 */

    I2C_Start();   /* 起始信号 */
    I2C_WriteByte(OLED_I2C_ADDR);   /* 发送从机地址 0x78 */
    I2C_WriteByte(0x40);   /* 控制字节 0x40 = 后续为显示数据 */
    for (i = 0U; i < len; i++)   /* 逐字节发送 */
    {
        I2C_WriteByte(data[i]);   /* 发送第 i 个字节 */
    }
    I2C_Stop();   /* 停止信号，结束本次事务 */
}

/* -------------------------------------------------------------------------
 * 二、OLED 基础操作
 * ------------------------------------------------------------------------- */

/* 初始化。注意：调用前最好先 Delay_ms(200) 等屏上电稳定。 */
/*-----------------------------------------------------------
  Function:      OLED_Init
  Description:   初始化 OLED：配置引脚并连续发送 SSD1306/SSD1315 的上电配置命令表。
  Calls:         OLED_GPIO_Init()、OLED_WrCmds()
  Called By:     main.c 上电初始化（只调用一次）
  Input:         无
  Output:        无
  Return:        无
  Others:        调用前建议先延时 200ms 等待屏幕供电稳定。
-----------------------------------------------------------*/

void OLED_Init(void)
{
    /* 标准 SSD1306/SSD1315 上电配置表（27 条命令，一次连续发完） */
    static const uint8_t init_cmd[] = {
        0xAE,           /* 关显示 */
        0x00, 0x10,     /* 列地址低 4 位 / 高 4 位 = 0 */
        0x40,           /* 显示起始行 = 0 */
        0xB0,           /* 页地址 = 0 */
        0x81, 0xFF,     /* 对比度 = 255 */
        0xA1,           /* 左右方向：列 127 映射到 SEG0（正常朝向） */
        0xA6,           /* 正常显示（不是反白） */
        0xA8, 0x3F,     /* 多路复用比 = 1/64（64 行全用） */
        0xC8,           /* 上下方向：COM 反向扫描（正常朝向） */
        0xD3, 0x00,     /* 显示偏移 = 0 */
        0xD5, 0x80,     /* 时钟分频 / 振荡频率 */
        0xD9, 0xF1,     /* 预充电周期 */
        0xDA, 0x12,     /* COM 引脚硬件配置 */
        0xDB, 0x30,     /* VCOMH 电压 */
        0x8D, 0x14,     /* 使能内部电荷泵 —— 屏不亮十有八九是漏了这条 */
        0xAF            /* 开显示 */
    };
    uint8_t i;   /* 命令索引 */

    OLED_GPIO_Init();   /* 先把 PB8/PB9 配成开漏输出 */

    I2C_Start();   /* 起始信号 */
    I2C_WriteByte(OLED_I2C_ADDR);   /* 发送从机地址 0x78 */
    I2C_WriteByte(0x00);                          /* 整段都是命令 */
    for (i = 0; i < (uint8_t)sizeof(init_cmd); i++)   /* 逐条发送配置命令 */
    {
        I2C_WriteByte(init_cmd[i]);   /* 发送第 i 条命令 */
    }
    I2C_Stop();   /* 停止信号，初始化命令序列发送完毕 */
}

/* 设置写入位置：x = 列 0~127，y = 页 0~7（每页 8 行像素） */
/*-----------------------------------------------------------
  Function:      OLED_SetPos
  Description:   设置下一次写入的起始位置。
  Calls:         无
  Called By:     OLED_ShowChar()、OLED_ShowString() 等
  Input:         x —— 列坐标 0~127；y —— 页坐标 0~7（每页 8 行像素）
  Output:        无
  Return:        无
  Others:        无
-----------------------------------------------------------*/

void OLED_SetPos(uint8_t x, uint8_t y)
{
    uint8_t cmd[3];   /* 三条定位命令的缓冲区 */

    cmd[0] = (uint8_t)(0xB0U + y);   /* 页地址命令：0xB0 + 页号 */
    cmd[1] = (uint8_t)(((x & 0xF0U) >> 4) | 0x10U);   /* 列地址高 4 位（低半字节在前加 0x10） */
    cmd[2] = (uint8_t)(x & 0x0FU);   /* 列地址低 4 位 */
    OLED_WrCmds(cmd, 3U);   /* 一次事务发完三条定位命令 */
}

/* 清屏：8 页 x 128 列全写 0。整页一次事务发完，比一字节一事务快很多 */
/*-----------------------------------------------------------
  Function:      OLED_Clear
  Description:   清空整屏显存，把 8 页 x 128 列全部写 0。
  Calls:         OLED_SetPos()、OLED_WrDataBuffer()
  Called By:     UI_Init()、main.c
  Input:         无
  Output:        无
  Return:        无
  Others:        整页一次性发送，比一字节一个事务快很多。
-----------------------------------------------------------*/

void OLED_Clear(void)
{
    uint8_t page;   /* 页索引 0~7 */
    static const uint8_t blank[128] = {0};   /* 128 列全 0 的一行，static 放静态区省栈 */

    for (page = 0; page < 8; page++)   /* 逐页清空 */
    {
        OLED_SetPos(0, page);   /* 定位到该页首列 */

        OLED_WrDataBuffer(blank, (uint16_t)sizeof(blank));   /* 整页一次刷完，比逐字节快很多 */
    }
}

/* -------------------------------------------------------------------------
 * 三、字符 / 字符串 / 数字显示
 * ------------------------------------------------------------------------- */

/* 在 (x, y) 显示一个字符。size = 16 用 8x16 字库，其他值用 6x8 字库。 */
/*-----------------------------------------------------------
  Function:      OLED_ShowChar
  Description:   在指定位置显示一个字符。
  Calls:         OLED_SetPos()、OLED_WrDataBuffer()
  Called By:     OLED_ShowString()、OLED_ShowNum()、OLED_ShowFloat()
  Input:         x / y —— 起始坐标；chr —— 待显示字符；size —— 字号（16 或其它）
  Output:        无
  Return:        无
  Others:        size 为 16 时使用 8x16 字库（跨两页），其它值使用 6x8 字库。
-----------------------------------------------------------*/

void OLED_ShowChar(uint8_t x, uint8_t y, char chr, uint8_t size)
{
    char text[2];   /* 单字符转成字符串的缓冲 */

    text[0] = chr;   /* 第 0 位放待显示字符 */
    text[1] = '\0';   /* 第 1 位补字符串结束符 */
    OLED_ShowString(x, y, text, size);   /* 复用字符串显示函数 */
}

/* 显示字符串。一行放不下会自动折到下一行（按 size 决定每行几个字） */
/*-----------------------------------------------------------
  Function:      OLED_ShowString
  Description:   在指定位置显示字符串，一行放不下时自动折到下一行。
  Calls:         OLED_ShowChar()
  Called By:     UI_DrawFrame()、UI_Update()
  Input:         x / y —— 起始坐标；str —— 字符串；size —— 字号
  Output:        无
  Return:        无
  Others:        每行可容纳的字符数由字号决定（8x16 为 16 个、6x8 为 21 个）。
-----------------------------------------------------------*/

void OLED_ShowString(uint8_t x, uint8_t y, const char *str, uint8_t size)
{
    uint8_t step = (size == 16U) ? 8U : 6U;   /* 单字符横向占 8 列（8x16）或 6 列（6x8） */
    uint8_t page_step = (size == 16U) ? 2U : 1U;   /* 单字符纵向占 2 页（8x16）或 1 页（6x8） */
    uint8_t top[128];   /* 上半部分（或 6x8 全部）字模缓冲 */
    uint8_t bottom[128];   /* 下半部分字模缓冲，只有 8x16 用得到 */

    while (*str != '\0')   /* 一次循环处理「本行放得下的一段」 */
    {
        uint8_t max_chars;   /* 当前行从 x 起还能放几个字符 */
        uint8_t count = 0U;   /* 本段实际字符数 */
        uint8_t i;   /* 字符索引 */

        max_chars = (uint8_t)((128U - x) / step);   /* 剩余列数除以字宽 = 本行容量 */
        if (max_chars == 0U)   /* 本行已放不下任何字符 */
        {
            x = 0U;   /* 回到首列 */
            y = (uint8_t)(y + page_step);   /* 换到下一行 */
            continue;   /* 重新计算新一行的容量 */
        }

        while ((str[count] != '\0') && (count < max_chars))   /* 数出本段字符个数（不越行、不越界） */
        {
            count++;   /* 逐个累加 */
        }

        for (i = 0U; i < count; i++)   /* 逐字符取出字模 */
        {
            uint8_t c = (uint8_t)(str[i] - ' ');   /* 字符相对空格的字库索引 */
            uint8_t j;   /* 字模列索引 */

            if (size != 16U)   /* 非 16 号：走 6x8 小字库 */
            {
                if (c > 91U)   /* 索引越界（小字库只到 91） */
                {
                    c = 0U;   /* 退回空格字模，避免越界读 */
                }
                for (j = 0U; j < 6U; j++)   /* 6x8 字模共 6 列 */
                {
                    top[(uint16_t)i * 6U + j] = F6x8[c][j];   /* 取 6x8 字模写入上缓冲 */
                }
            }
            else   /* 16 号：走 8x16 大字库 */
            {
                for (j = 0U; j < 8U; j++)   /* 8x16 每列 8 位 */
                {
                    top[(uint16_t)i * 8U + j] = F8X16[c * 16U + j];   /* 前 8 字节 = 上半页 */
                    bottom[(uint16_t)i * 8U + j] = F8X16[c * 16U + j + 8U];   /* 后 8 字节 = 下半页 */
                }
            }
        }

        OLED_SetPos(x, y);   /* 定位到本段起点 */
        OLED_WrDataBuffer(top, (uint16_t)count * step);   /* 写上半部分 */
        if (size == 16U)   /* 大字才需要写下半页 */
        {
            OLED_SetPos(x, (uint8_t)(y + 1U));   /* 定位到下一页 */
            OLED_WrDataBuffer(bottom, (uint16_t)count * step);   /* 写下半部分 */
        }

        x = (uint8_t)(x + (uint8_t)(count * step));   /* 列坐标右移本段宽度 */
        str += count;   /* 指针后移，继续处理剩余字符 */
    }
}

/* 显示无符号整数。len = 总位数，前面的 0 会显示成空格；size = 字号 */
/*-----------------------------------------------------------
  Function:      OLED_ShowNum
  Description:   显示无符号整数，按固定位数右对齐，前导零显示为空格。
  Calls:         OLED_ShowChar()
  Called By:     UI_Update()
  Input:         x / y —— 起始坐标；num —— 待显示数值；len —— 总位数；size —— 字号
  Output:        无
  Return:        无
  Others:        固定宽度可避免刷新时残留上一位数字的残影。
-----------------------------------------------------------*/

void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size)
{
    char text[32];   /* 数字转成的字符串缓冲 */
    uint8_t t;   /* 位序号，0 = 最高位 */
    uint8_t pos = 0U;   /* 输出写指针 */
    uint8_t enshow = 0U;   /* 是否已遇到第一个非零位 */
    uint32_t p;   /* 当前位的权值（10 的幂） */

    if (len > 31U)   /* 位数上限保护（缓冲区 32 字节） */
    {
        len = 31U;   /* 超出则截到 31 位 */
    }

    for (t = 0U; t < len; t++)   /* 从最高位开始逐位取数 */
    {
        uint8_t temp;   /* 该位的数字 0~9 */
        uint8_t k = (uint8_t)(len - t - 1U);   /* 该位后面还有几位 */

        p = 1U;   /* 权值从 1 开始 */
        while (k-- > 0U)   /* 累乘得到 10^k */
        {
            p *= 10U;   /* p = 10^k */
        }
        temp = (uint8_t)((num / p) % 10U);   /* 取出该位数字 */

        if ((enshow == 0U) && (t < (uint8_t)(len - 1U)) && (temp == 0U))   /* 前导零且不是个位 */
        {
            text[pos++] = ' ';   /* 用空格占位，保持右对齐 */
        }
        else   /* 已是有效数字位 */
        {
            enshow = 1U;   /* 标记已进入有效数字区 */
            text[pos++] = (char)('0' + temp);   /* 转成 ASCII 存入缓冲 */
        }
    }
    text[pos] = '\0';   /* 补字符串结束符 */
    OLED_ShowString(x, y, text, size);   /* 交给字符串显示函数 */
}

/* 显示浮点数（内部全用整数运算 + 四舍五入，不用 %f，省 Flash 也更准）
 * int_len = 整数位数，dec_len = 小数位数，size = 字号 */
/*-----------------------------------------------------------
  Function:      OLED_ShowFloat
  Description:   显示浮点数，内部全部使用整数运算与四舍五入（不依赖 %f 格式化）。
  Calls:         OLED_ShowNum()、OLED_ShowChar()
  Called By:     UI_Update()
  Input:         x / y —— 起始坐标；num —— 待显示数值；int_len —— 整数位数；dec_len —— 小数位数；size —— 字号
  Output:        无
  Return:        无
  Others:        不用库里的浮点格式化可显著节省 Flash，同时避免精度显示问题。
-----------------------------------------------------------*/

void OLED_ShowFloat(uint8_t x, uint8_t y, float num, uint8_t int_len, uint8_t dec_len, uint8_t size)
{
    char     text[40];   /* 输出字符串缓冲 */
    uint8_t  pos = 0U;   /* 输出写指针 */
    uint8_t  i;   /* 循环变量 */
    uint32_t divisor = 1U;   /* 10^dec_len，用于把小数放大成整数 */
    uint32_t frac;   /* 小数部分 */
    uint32_t d;   /* 小数位权值 */
    int32_t  v;   /* 放大取整后的数值 */

    for (i = 0; i < dec_len; i++)   /* 求 10^dec_len */
    {
        divisor *= 10U;                        /* 累乘得到 divisor = 10^dec_len */
    }

    /* 放大成整数并四舍五入 */
    v = (int32_t)(num * (float)divisor);
    v += (int32_t)((num >= 0.0f) ? 0.5f : -0.5f); /* 四舍五入到最近整数 */

    if (v < 0)   /* 负数先输出符号 */
    {
        text[pos++] = '-';   /* 写入负号 */
        v = -v;   /* 取正值继续处理 */
    }

    /* 整数部分：前导零同样显示成空格，与 OLED_ShowNum 行为一致。 */
    {
        uint32_t integer = (uint32_t)(v / (int32_t)divisor); /* 取出整数部分 */
        uint8_t enshow = 0U; /* 首次出现有效数字后置 1，用于把前导零显示成空格 */

        for (i = 0U; i < int_len; i++)   /* 逐位输出整数部分 */
        {
            uint8_t k = (uint8_t)(int_len - i - 1U);   /* 该位后面还有几位 */
            uint32_t p = 1U;   /* 权值初值 */
            uint8_t digit;   /* 该位数字 0~9 */

            while (k-- > 0U)   /* 求 10^k */
            {
                p *= 10U;   /* p = 10^k */
            }
            digit = (uint8_t)((integer / p) % 10U);   /* 取出该位数字 */
            if ((enshow == 0U) && (i < (uint8_t)(int_len - 1U)) && (digit == 0U))   /* 前导零用空格占位 */
            {
                text[pos++] = ' ';   /* 空格占位，保持右对齐 */
            }
            else   /* 已是有效数字位 */
            {
                enshow = 1U;   /* 标记已出现有效数字 */
                text[pos++] = (char)('0' + digit);   /* 转 ASCII 存入缓冲 */
            }
        }

        /* 小数点与各位小数。 */
        if (dec_len > 0U)
        {
            text[pos++] = '.'; /* 补上小数点，写入位置后移一位 */

            frac = (uint32_t)v % divisor;   /* 取出小数部分 */
            d = divisor / 10U;   /* 最高小数位的权值 */
            while (d > 0U)   /* 从高到低输出各位小数 */
            {
                text[pos++] = (char)('0' + (char)((frac / d) % 10U));   /* 逐位取数并转成 ASCII */
                d /= 10U;   /* 权值降低一位 */
            }
        }
    }

    text[pos] = '\0';   /* 补字符串结束符 */
    OLED_ShowString(x, y, text, size);   /* 交给字符串显示函数 */
}
