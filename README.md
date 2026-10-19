# 基于边缘 AI 的电机智能状态监控 / Edge-AI Motor Condition Monitoring

基于 STM32F407ZGT6 + NanoEdge AI 的电机异常状态在线监测系统。单片机端实时计算特征量 **R = 转速 / 母线电压**，用 NanoEdge AI（ZSM 库）在板上做异常检测，OLED 本地报警，同时通过 UART 把遥测数据发给 ESP8266（D1 mini），在浏览器网页上实时查看电机转速、电压、电流与 AI 状态。

> 本科毕业设计项目。全套硬件约 ¥17（不含主控板与电机），全程零焊接。

## 系统组成

```
[电机] MG310 直流减速电机
   |
[D153C 驱动] <-- PWM (TIM9, 20kHz, PE5/PE6)
   |
   |-- 母线电压采样 (PA4, 板载 10k+1k 分压 1/11)
   |-- 电流采样 (PA5, ADC1_IN5)
[STM32F407ZGT6]
   |-- 编码器输入 TIM4 (PB6/PB7, 四倍频, 1040 counts/圈)
   |-- 特征量 R = rpm / 母线电压, 32 点窗 x 100ms = 3.2s
   |-- NanoEdge AI 异常检测 (ZSM, RAM 56B / Flash 40B)
   |-- OLED (PB8/PB9) 状态显示 + 报警
   |-- USART3 (PB10, 57600 8N1) 10Hz 二进制遥测帧
   |
[D1 mini (ESP8266)]
   |-- 解析遥测帧
   |-- AP/STA 双模式, 内置 HTTP 服务器
   |-- 浏览器实时监控页 (http://192.168.4.1)
```

### 为什么选 R = rpm / 母线电压 作为唯一特征量

实测对比后排除的方案：

| 候选特征 | 结论 |
|---|---|
| 裸转速 | 负载变化 -17% 与低电量 -11% 区间重叠，无法区分 |
| 电流波形 | 20kHz PWM 波形在板上采不全，相关性仅 0.0106 |
| **R = rpm/Vbus** | 正常区间 13.23~13.53，捏轴异常 10.79~11.07，分离干净 |

## 目录结构

```
firmware_stm32/     STM32F407 固件 (Keil MDK 工程)
  USER/             main.c、中断、Keil 工程 (STM32F407_Template.uvprojx)
  HARDWARE/         板级驱动 (oled/pwm/encoder/ai_monitor/feature_r/...)
  NEAI/             NanoEdge AI 库 (libneai.a, result_id 44 ZSM)
  CORE/ FWLIB/      CMSIS + StdPeriph V1.4 (已裁剪到实际使用的外设)
  OBJ_firmware/     最终编译产物 (.hex)
firmware_d1mini/    D1 mini 网页监视器固件 (Arduino)
  d1mini_telemetry.ino   入口
  monitor_page.h         内嵌网页页面
  status_json.cpp        状态 JSON 接口
  wifi_config.h          WiFi 配置 (占位符, 需自行填写)
dataset/            NanoEdge AI Studio 训练数据集 (32 列, 无表头无标签)
  R32_regular_normal.csv       正常运转 1000 窗
  R32_abnormal_grip.csv        捏轴堵转异常 500 窗
  R32_test_normal_batch06.csv  独立验证 569 窗
docs/               硬件连接图 / 程序流程图 / ZSM 模型参数
```

## 实测效果

- 开机空转 6~9 s 学满 64 窗基线，之后进入检测模式
- 捏轴堵转：相似度 100 -> 68 -> 16 -> **0**，约 3~3.5 s 触发报警（PF9 LED）
- 松开恢复正常后自动解除报警
- 稳态复核：110 s 内 phase=2 / 相似度=100 / 无误报

## 复现步骤

### 1. STM32 固件

1. Keil MDK 5 打开 `firmware_stm32/USER/STM32F407_Template.uvprojx`
2. 编译烧录（或直接用 `OBJ_firmware/` 里的 hex）
3. 首次上电会自动进行电流零点校准（保持电机停转）

### 2. D1 mini 网页监视器

1. Arduino IDE 安装 ESP8266 支持
2. 打开 `firmware_d1mini/d1mini_telemetry.ino`
3. 在 `wifi_config.h` 里填入你的 WiFi 名称与密码（已脱敏）
4. 烧录后浏览器访问 D1 mini 的 IP（AP 模式固定为 http://192.168.4.1）

### 3. 训练自己的 NanoEdge AI 模型（可选）

数据集在 `dataset/`，每行 32 个数（一个 3.2s 特征窗的 R 序列），无表头无标签。
在 NanoEdge AI Studio 中新建工程：信号 32 点、1 维，**务必手动关闭 Detrend**
（开着会抹平 R 的均值差异），然后按正常/异常分别导入 CSV。

## 硬件清单

| 器件 | 用途 |
|---|---|
| STM32F407ZGT6 开发板（正点原子/精畅等） | 主控 |
| WHEELTEC MG310 减速直流电机 + 编码器 | 被监控对象 |
| D153C 双路驱动板 | 功率驱动（板载分压电阻直接可用） |
| D1 mini (ESP8266) | 网页监视器 |
| 0.96" OLED I2C | 本地显示 |
| 杜邦线若干 | 连接（全程零焊接） |

## License

MIT
