# STM32 固件目录说明（2026-10-21 定版）

本目录是 Keil MDK 5（MDK-ARM Standard 5.14，ARMCC 5.05）可直接打开的完整工程，
对应板上实测通过的最终版本。

## 目录内容

| 目录 | 内容 |
|---|---|
| `USER/` | 入口 `main.c`、中断 `stm32f4xx_it.c`、系统时钟 `system_stm32f4xx.c`、Keil 工程 `STM32F407_Template.uvprojx` |
| `HARDWARE/` | 自写板级驱动，共 16 组 `.c/.h`（见下表） |
| `NEAI/` | NanoEdge AI 静态库 `libneai.a` + 头文件 `NanoEdgeAI.h`（result_id 68 / **ICM 分类库**） |
| `CORE/` | CMSIS 内核头文件与启动文件 `startup_stm32f40_41xxx.s` |
| `FWLIB/` | StdPeriph V1.4，只保留实际用到的 7 个外设（misc / adc / dma / gpio / rcc / tim / usart） |
| `OBJ_firmware/` | 最终编译产物：`STM32F407_Template.hex`、同名 `.map`、烧录脚本 `burn_final_20261021.jlink` |
| `OBJ/` | Keil 的编译输出目录（空目录，Build 时自动填充） |

## 源文件职责

| 文件 | 职责 |
|---|---|
| `ai_monitor.c/h` | AI 状态机：预训练知识直接进检测 / 报警 / 停机后只清报警不重学，包裹 NanoEdge 库调用 |
| `feature_r.c/h` | 三路特征：`R_new`（膝点归一化水平值，32 点喂 AI）、`dR`（窗内滚动峰峰值，显式阈值判决）、`g_ripplePct`（相对百分比，仅供 Watch 观察） |
| `encoder.c/h` | TIM4 编码器接口，四倍频，1040 counts/圈 |
| `pwm.c/h` | TIM9 双路 PWM（20 kHz），档位与占空比（L1 = 58%，L2 = 64%） |
| `current.c/h` | ACS712 电流采样，上电零点校准 + **DWT 采样相位抖动**（消除 PWM 纹波与采样节拍混叠） |
| `vbat.c/h` | 母线电压采样，板载 10k+1k 分压 1/11 |
| `telemetry.c/h` | 42 字节遥测帧组包，USART3 以 10 Hz 推送 |
| `m2_protocol.c/h` | M2 帧格式定义与 CRC16（与 D1 mini 侧同一份实现） |
| `ui.c/h`、`oled.c/h`、`font.h` | OLED 页面排版与 SSD1306 驱动 |
| `key.c/h`、`timer.c/h` | 按键 1 ms 中断消抖、TIM7 时基 |
| `led.c/h`、`systick.c/h` | 报警灯 PF9、微秒/毫秒延时 |
| `fastcap.c/h`、`wave_dump.c/h` | 高速波形抓取与串口导出，仅调试用，不参与 AI 判定 |

## 源码编码与行尾

| 项目 | 约定 | 原因 |
|---|---|---|
| 文件编码 | **UTF-8（无 BOM）** | Keil MDK 5.14 的 uVision 内置「Encode in UTF-8 without signature」选项，UTF-8 中文注释在编辑器里正常显示；同时 GitHub 网页按 UTF-8 渲染，源码在网页上不会乱码 |
| 行尾 | **原样保留**（部分文件 CRLF、部分 LF） | 与实际编译验证过的工程逐字节一致；`.gitattributes` 已对 `*.c` `*.h` `*.s` `*.ino` `*.cpp` 声明 `-text`，保证克隆下来行尾不被 `core.autocrlf` 改写 |
| 编译验证 | ARMCC 5.05 对 UTF-8 源文件编译结果与 GBK 版**逐字节相同** | `Code=22296 RO-data=6548 RW-data=264 ZI-data=17992`，0 Error 0 Warning，hex md5 `d5483850d3cb58adc392afc50b4f6399` |

> 早期版本本仓库的 `.c/.h` 用的是 GBK 编码。ARMCC 5 虽能正常编译，但 GitHub 网页
> 按 UTF-8 渲染 GBK 字节，`main.c` 会出现 7613 个替换字符（乱码）。因此统一改为
> UTF-8 无 BOM。
>
> **两次产物变化的区分（别混淆）**：
>
> | 变更 | hex md5 | 说明 |
> | --- | --- | --- |
> | GBK → UTF-8（纯编码转换） | `eb7e23174ca6873ddb9aa1962eccae4e` | **产物逐字节未变**，A/B 编译实测确认 |
> | 2026-10-22 修 OLED 显示缺陷 | `d5483850d3cb58adc392afc50b4f6399` | Code 22288 → 22296，**当前有效版本** |
>
> 即：编码转换本身不影响机器码；现在 hex 与 10-22 之前的不同，是因为 OLED 显示
> 缺陷修复（`ui.c` 的 R 行整数位由 1 位改 2 位、dR 无效态由 5 字符改 4 字符）
> 引起的，与编码无关。

## 编译与烧录

- **编译**：Keil 打开 `USER/STM32F407_Template.uvprojx` 直接 Build；命令行用
  `UV4.exe -b USER\STM32F407_Template.uvprojx -j0 -o 日志绝对路径`
  （`-o` 只给相对路径时不会生成日志文件）。
  当前构建尺寸：`Code=22296 RO-data=6548 RW-data=264 ZI-data=17992`，0 Error 0 Warning，
  完整日志见 `USER/build_final_20261021.log`。
- **烧录**：直接用 `OBJ_firmware/STM32F407_Template.hex`
  （md5 `d5483850d3cb58adc392afc50b4f6399`，与上面这组尺寸是同一次构建的产物）。
  配套脚本 `OBJ_firmware/burn_final_20261021.jlink` 用的是**相对路径**，
  需 `cd` 到 `OBJ_firmware/` 目录再执行：
  `E:\Keil_v5\ARM\Segger\JLink.exe -CommanderScript burn_final_20261021.jlink`。
  脚本要点：
  - 第一行必须是 `device STM32F407ZG`，**不能省**。省掉后 erase 会被静默拒绝，
    而 loadbin 照样打印 O.K.，结果 flash 一字未变（假成功）；
  - 已包含烧前关闭 ADC2 / DMA2 的三条 `w4`，不关会在 RAM check 处失败；
  - 若报 `Programming failed`，多半是上一轮 J-Link 会话残留，`h` 之后补一次 `r` 即过。
- **尺寸证据**：`OBJ_firmware/STM32F407_Template.map` 是完整的 armlink listing
  （含 Image Symbol Table / Global Symbols / Memory Map / Image component sizes 四段），
  `Total RO Size = 28844 (28.17 kB)`、`Total RW Size = 18256 (17.83 kB)`，
  与 hex 同一次链接产出。源码改动后再编译，`.map` 里的符号地址会整体平移，
  用 J-Link 读内部变量前需重新取地址。

## 已移除的文件（不在本目录）

以下内容在最终版本中已删除，若在别处看到请以本目录为准：

- **MPU6050 相关**：`mpu6050.c/h`、`mpuiic.c/h`。加速度已不参与判定，遥测帧中
  `value[1..3]` 保留字段但恒不写入，`validMask` 的 bit1~bit3 恒为 0。
- **19 维特征相关**：`feature19.c/h`，已由 `feature_r.c/h` 取代。
- **页面字符串**：`monitor_page.h` 从不在 STM32 工程内，它属于 D1 mini 固件
  （`../02_D1mini网页监视器/monitor_page.h`）。主控只发二进制帧，不承载 HTTP。
- **编译中间件**：`OBJ/*.o`、`Listings/`、`*.uvguix.*`、`JLinkLog.txt` 等不入库。
- **`keil_keil.bat`**：原例程自带的清理脚本，会 `del *.map /s`，**会把
  `OBJ_firmware/STM32F407_Template.map` 一起删掉**，故已移出本包。
