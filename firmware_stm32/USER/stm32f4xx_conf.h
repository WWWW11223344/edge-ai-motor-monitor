/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_conf.h  
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    04-August-2014
  * @brief   Library configuration file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_CONF_H
#define __STM32F4xx_CONF_H

/* Includes ------------------------------------------------------------------*/
/* Uncomment the line below to enable peripheral header file inclusion */
/* 【精简说明】本工程只用到 7 个 StdPeriph 外设，FWLIB\inc 与 FWLIB\src 也只保留了
 * 这 7 个：misc / adc / dma / gpio / rcc / tim / usart。
 * 其余外设的头文件已随精简一并删除，所以下面这些 include 必须注释掉，
 * 否则编译会报 cannot open source input file。
 * 以后要用到某个外设时：把对应 .h 放回 FWLIB\inc、把 .c 加进 Keil 工程，再把这一行取消注释。
 * 依据：USER 与 HARDWARE 下的源码没有任何一处调用这 19 个外设的库函数，
 * 注释掉之后完整重编，Code/RO/RW/ZI 四个尺寸与注释前完全一致。 */
#include "stm32f4xx_adc.h"                        /* 用到：ADC1/ADC2 采样 */
/* #include "stm32f4xx_crc.h" */  /* 未使用：CRC 计算单元（帧校验用软件 CRC16，不走硬件） */
/* #include "stm32f4xx_dbgmcu.h" */  /* 未使用：调试冻结寄存器（J-Link 直接读内存，不用它） */
#include "stm32f4xx_dma.h"                        /* 用到：ADC2 + DMA2 的 20 kHz 采集前端 */
/* #include "stm32f4xx_exti.h" */  /* 未使用：外部中断（按键改为 1 ms 定时器轮询） */
/* #include "stm32f4xx_flash.h" */  /* 未使用：内部 Flash 编程（烧录由调试器完成） */
#include "stm32f4xx_gpio.h"                       /* 用到：全部数字/复用引脚 */
/* #include "stm32f4xx_i2c.h" */  /* 未使用：硬件 I2C（OLED 走 PB8/PB9 寄存器直写） */
/* #include "stm32f4xx_iwdg.h" */  /* 未使用：独立看门狗 */
/* #include "stm32f4xx_pwr.h" */  /* 未使用：低功耗控制 */
#include "stm32f4xx_rcc.h"                        /* 用到：时钟树与定时器/串口时钟使能 */
/* #include "stm32f4xx_rtc.h" */  /* 未使用：实时时钟 */
/* #include "stm32f4xx_sdio.h" */  /* 未使用：SD 卡接口 */
/* #include "stm32f4xx_spi.h" */  /* 未使用：SPI 接口 */
/* #include "stm32f4xx_syscfg.h" */  /* 未使用：系统配置控制器 */
#include "stm32f4xx_tim.h"                        /* 用到：TIM4 编码器、TIM7 时基、TIM9 PWM */
#include "stm32f4xx_usart.h"                      /* 用到：USART1 调试口、USART3 遥测 */
/* #include "stm32f4xx_wwdg.h" */  /* 未使用：窗口看门狗 */
#include "misc.h" /* High level functions for NVIC and SysTick (add-on to CMSIS functions) */

/* 以下三个分支对应本工程不使用的型号，预处理不会进入，里面引用的头文件
 * 也没有随精简保留，所以保持原样未动。换型号时要连头文件一起补回来。 */
#if defined (STM32F429_439xx)
#include "stm32f4xx_cryp.h"
#include "stm32f4xx_hash.h"
#include "stm32f4xx_rng.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_dac.h"
#include "stm32f4xx_dcmi.h"
#include "stm32f4xx_dma2d.h"
#include "stm32f4xx_fmc.h"
#include "stm32f4xx_ltdc.h"
#include "stm32f4xx_sai.h"
#endif /* STM32F429_439xx */

#if defined (STM32F427_437xx)
#include "stm32f4xx_cryp.h"
#include "stm32f4xx_hash.h"
#include "stm32f4xx_rng.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_dac.h"
#include "stm32f4xx_dcmi.h"
#include "stm32f4xx_dma2d.h"
#include "stm32f4xx_fmc.h"
#include "stm32f4xx_sai.h"
#endif /* STM32F427_437xx */

/* STM32F40_41xxx（本工程实际型号）分支：这 7 个外设全部未使用，头文件也未随精简保留。 */
#if defined (STM32F40_41xxx)
/* #include "stm32f4xx_cryp.h" */  /* 未使用：加解密 */
/* #include "stm32f4xx_hash.h" */  /* 未使用：哈希 */
/* #include "stm32f4xx_rng.h" */  /* 未使用：随机数 */
/* #include "stm32f4xx_can.h" */  /* 未使用：CAN 总线 */
/* #include "stm32f4xx_dac.h" */  /* 未使用：DAC */
/* #include "stm32f4xx_dcmi.h" */  /* 未使用：摄像头接口 */
/* #include "stm32f4xx_fsmc.h" */  /* 未使用：外部存储控制器（未扩展 SRAM、未启用以太网） */
#endif /* STM32F40_41xxx */

#if defined (STM32F411xE)
#include "stm32f4xx_flash_ramfunc.h"
#endif /* STM32F411xE */
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* If an external clock source is used, then the value of the following define 
   should be set to the value of the external clock source, else, if no external 
   clock is used, keep this define commented */
/*#define I2S_EXTERNAL_CLOCK_VAL   12288000 */ /* Value of the external clock in Hz */


/* Uncomment the line below to expanse the "assert_param" macro in the 
   Standard Peripheral Library drivers code */
/* #define USE_FULL_ASSERT    1 */

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT

/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function
  *   which reports the name of the source file and the source
  *   line number of the call that failed. 
  *   If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

#endif /* __STM32F4xx_CONF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
