/************************************************************
  Copyright (C), 2026, 汪贤云
  FileName:    oled.h
  Author:      汪贤云
  Version:     1.0
  Date:        2026-09-21
  Description: 0.96 寸 I2C OLED 驱动接口（SSD1315/SSD1306 兼容，128x64）。
               采用软件 I2C：SCL = PB8、SDA = PB9，板上已焊 4.7k 上拉，不占用硬件 I2C 外设。
               从机地址 0x78（7 位地址 0x3C 左移一位）。
               显存按 8 页组织，每页 8 行像素；8x16 字号的字符需跨两页写入。
  Function List:
    1. OLED_Init()        初始化屏幕
    2. OLED_Clear()       清屏
    3. OLED_SetPos()      设置写入位置
    4. OLED_ShowChar()    显示单个字符
    5. OLED_ShowString()  显示字符串
    6. OLED_ShowNum()     显示无符号整数
    7. OLED_ShowFloat()   显示浮点数
  History:
    <author>   <time>      <version>   <desc>
    汪贤云     2026-10-16    1.0         创建文件并按华为规范补充注释
***********************************************************/

#ifndef __OLED_H
#define __OLED_H  /* 防止重复包含 */

#include "stm32f4xx.h"  /* 标准外设库声明 */

/* ==========================================================================
 * 0.96 寸 4 针 I2C OLED 驱动（SSD1315 / SSD1306 兼容，128x64）
 * --------------------------------------------------------------------------
 * 本驱动用「软件 I2C」（模拟 I2C）实现，不占用硬件 I2C1 外设。
 *
 *   SCL  ->  PB8（时钟线）
 *   SDA  ->  PB9（数据线）
 *
 * 这两个脚就是开发板原理图上的 IIC 总线，板上已经焊了 4.7k 上拉电阻到
 * 3.3V（R50 / R46），所以软件 I2C 也能跑得很稳。
 *
 * 从机地址 0x78 = 7 位地址 0x3C 左移一位。
 *
 * 接线（务必对着屏上丝印一针对一针，不同批次针序不一样）：
 *   GND -> GND      VDD -> 3V3      SCK -> PB8      SDA -> PB9     （四线接线表）
 * ========================================================================== */

#define OLED_GPIO_PORT      GPIOB
#define OLED_SCL_PIN        GPIO_Pin_8         /* PB8 = I2C 时钟 */
#define OLED_SDA_PIN        GPIO_Pin_9         /* PB9 = I2C 数据 */
#define OLED_I2C_ADDR       0x78               /* 从机地址 0x3C 左移一位（末尾补读写位） */

void OLED_Init(void);  /* 初始化 OLED（调用前建议先延时 200ms 等屏上电稳定） */
void OLED_Clear(void);  /* 清空整屏显存 */
void OLED_SetPos(uint8_t x, uint8_t y);  /* 设置写入位置：x = 列 0~127，y = 页 0~7 */
void OLED_ShowChar(uint8_t x, uint8_t y, char chr, uint8_t size);  /* 在指定位置显示一个字符，size=16 用 8x16 字库，其它用 6x8 */
void OLED_ShowString(uint8_t x, uint8_t y, const char *str, uint8_t size);  /* 显示字符串，一行放不下会自动折行 */
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);  /* 显示无符号整数，len 为总位数（前导 0 显示为空格） */
void OLED_ShowFloat(uint8_t x, uint8_t y, float num, uint8_t int_len, uint8_t dec_len, uint8_t size);  /* 显示浮点数，int_len 为整数位数、dec_len 为小数位数 */

#endif
