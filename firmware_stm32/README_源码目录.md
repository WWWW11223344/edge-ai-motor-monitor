# STM32 固件目录说明

本目录是 Keil MDK 5（MDK-ARM Standard 5.14，ARMCC 5.05）可直接打开的完整工程，
对应板上实测通过的最终版本。

## 目录内容

| 目录 | 内容 |
|---|---|
| `USER/` | 入口 `main.c`、中断 `stm32f4xx_it.c`、系统时钟 `system_stm32f4xx.c`、Keil 工程 `STM32F407_Template.uvprojx` |
| `HARDWARE/` | 自写板级驱动，共 16 组 `.c/.h`（见下表） |
| `NEAI/` | NanoEdge AI 静态库 `libneai.a` + 头文件 `NanoEdgeAI.h`（result_id 44 / ZSM） |
| `CORE/` | CMSIS 内核头文件与启动文件 `startup_stm32f40_41xxx.s` |
| `FWLIB/` | StdPeriph V1.4，只保留实际用到的 7 个外设（misc / adc / dma / gpio / rcc / tim / usart） |
| `OBJ_firmware/` | 最终编译产物：`.hex`、`.map`、烧录脚本 `burn_r64_20261019.jlink` |

## 源文件职责

| 文件 | 职责 |
|---|---|
| `ai_monitor.c/h` | AI 状态机：学习 / 检测 / 报警 / 停机重学，包裹 NanoEdge 库调用 |
| `feature_r.c/h` | 特征量 R 的滑动窗口，每 100 ms 推一个点，32 点成窗 |
| `encoder.c/h` | TIM4 编码器接口，四倍频，1040 counts/圈 |
| `pwm.c/h` | TIM9 双路 PWM（20 kHz），档位与占空比 |
| `current.c/h` | ACS712 电流采样，上电零点校准 |
| `vbat.c/h` | 母线电压采样，板载 10k+1k 分压 1/11 |
| `telemetry.c/h` | 42 字节遥测帧组包，USART3 以 10 Hz 推送 |
| `m2_protocol.c/h` | M2 帧格式定义与 CRC16（与 D1 mini 侧同一份实现） |
| `ui.c/h`、`oled.c/h`、`font.h` | OLED 页面排版与 SSD1306 驱动 |
| `key.c/h`、`timer.c/h` | 按键 1 ms 中断消抖、TIM7 时基 |
| `led.c/h`、`systick.c/h` | 报警灯 PF9、微秒/毫秒延时 |
| `fastcap.c/h`、`wave_dump.c/h` | 高速波形抓取与串口导出，仅调试用，不参与 AI 判定 |

## 编译与烧录

- **编译**：Keil 打开 `USER/STM32F407_Template.uvprojx` 直接 Build；命令行可用
  `UV4.exe -b USER\STM32F407_Template.uvprojx -j0 -o build.log`。
  最终构建尺寸见 `USER/build_r64.log`：`Code=18200 RO-data=3480 RW-data=216 ZI-data=14216`。
- **烧录**：可直接用 `OBJ_firmware/STM32F407_Template.hex`（2026-10-19 上板实测版）。
- **历史产物**：`OBJ_firmware/` 里的 `.hex` 与 `.map` 是同一次构建的产物，两者的
  Grand Totals 一致；源码改动后再编译，`.map` 里的符号地址会整体平移，调试读内存前需重新取地址。

## 已移除的文件（不在本目录）

以下内容在最终版本中已删除，若在别处看到请以本目录为准：

- **MPU6050 相关**：`mpu6050.c/h`、`mpuiic.c/h`。加速度已不参与判定，遥测帧中
  `value[1..3]` 保留字段但恒不写入，`validMask` 的 bit1~bit3 恒为 0。
- **19 维特征相关**：`feature19.c/h`，已由 `feature_r.c/h` 取代。
- **页面字符串**：`monitor_page.h` 从不在 STM32 工程内，它属于 D1 mini 固件
  （`../firmware_d1mini/monitor_page.h`）。主控只发二进制帧，不承载 HTTP。
- **编译中间件**：`OBJ/*.o`、`Listings/`、`*.uvguix.*`、`JLinkLog.txt` 等不入库。
