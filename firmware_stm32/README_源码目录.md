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
| 行尾 | **原样保留**（部分文件 CRLF、部分 LF） | 与实际编译验证过的工程逐字节一致。`.gitattributes` 对全部文本类扩展名（`*.c` `*.h` `*.s` `*.ino` `*.cpp` `*.hex` `*.map` `*.log` `*.ini` `*.jlink` `*.uvprojx` `*.uvoptx` `*.md` `*.json` `*.csv` `*.svg` `*.txt`）声明 `-text`，二进制声明 `binary`，克隆下来行尾不会被 `core.autocrlf` 改写 |
| 行尾入库验证 | 仓库内 121 个已跟踪文件，远程 blob 与工作区**逐字节一致**（0 处差异） | 2026-10-22 实测：`.gitattributes` 的规则对「添加前已入库」的文件不生效，已对 19 个曾被 `core.autocrlf` 归一化成 LF 的文件（FWLIB 14 个 + `.map` + `.ini` + 3 个 `.svg`）执行 `git rm --cached` + `git add` 重新入库 |
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

## 10-22 固件更新（OLED 显示修正）—— 已烧录并实测验证

**这一版改了 OLED 上 R 行的显示。已于 2026-10-22 烧录到板上并实测通过：
R 显示为两位数（`12.2` / `12.8`），不再是旧的 `1.7`。**

> 如果你的板子上还是旧固件，R 会显示成 `1.7` 这种丢十位的值，需重新烧录
> `OBJ_firmware/STM32F407_Template.hex`。烧录后的逐字节回读校验与板上实测见
> `05_文档/烧录记录_20261022.md`。

| 缺陷 | 现象 | 成因 | 修法 |
| --- | --- | --- | --- |
| R 行丢十位 | R 实测 11.79 却显示 `1.7`；13.08 显示 `3.0` | `OLED_ShowFloat` 整数部分按位取模输出（`((integer/p)%10)`），而调用时 `int_len` 只给了 1 位 | `int_len` 由 `1U` 改 `2U` |
| R/dR 轮显残影 | dR 切回 R 时出现 `3.48-` 这类脏串 | R 正常态只有 3 字符、dR 有 4 字符，长度不等 | R 改 2 位整数后两态都是 4 字符，残影自动消失 |
| dR 无效态贴单位 | `--.--` 5 字符 = 40 px，终点 x=80 正好压住单位首列 | 无效态比正常态多 1 字符 | 改为 `----`（4 字符），四种组合统一 4 字符 |

改后 R 行与 dR 行的正常态/无效态**全部是 4 字符 = 32 px**，终点 x=72 < 单位起点
`R_UNIT_X`(80)，既不压单位，两态互切也无残留。

**产物变化**（实测编译，非推断）：

| 项目 | 10-21 版 | 10-22 版（当前） |
| --- | --- | --- |
| Code | 22288 | **22296**（+8 字节，来自 ARMCC 字符串池布局） |
| RO / RW / ZI | 6548 / 264 / 17992 | 6548 / 264 / 17992（不变） |
| hex 大小 | 81898 B | **81927 B** |
| hex md5 | `eb7e23174ca6873ddb9aa1962eccae4e` | **`d5483850d3cb58adc392afc50b4f6399`** |
| Total RO Size | 28836 (28.16 kB) | **28844 (28.17 kB)** |

> +8 字节的来源已用 A/B 对照编译定位：只把 dR 无效态字符串从 `--.--` 换成 `----`
> 就是这 8 字节（ARMCC 把 5 字符常量优化成单条 `str` 指令 + 立即数偏移，
> 4 字符反而需要多一张偏移表项），`int_len=2U` 本身**不占**额外代码。

**重新烧录**：

```bash
cd OBJ_firmware/
E:\Keil_v5\ARM\Segger\JLink.exe -CommanderScript burn_final_20261021.jlink
```

烧完按老规矩**独立回读逐字节比对**（别只信工具自报的 O.K.）。
确认板上跑的是新版：电机转起来后看 OLED 第 4~5 页的 R 行，应显示
`11.7`（或 `13.0`）这类**带十位**的两位数，而不是 `1.7`。

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
