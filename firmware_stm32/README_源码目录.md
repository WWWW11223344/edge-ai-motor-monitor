# STM32 工程目录说明（2026-10-16 整理后）

## 正式源码（不要乱动，编译靠它们）

- `USER/`：入口 `main.c` + Keil 工程文件 `STM32F407_Template.uvprojx`。
- `HARDWARE/`：所有外设驱动，例如 `telemetry.c`、`m2_protocol.c`、`ai_monitor.c`、`ui.c`、`current.c`、`fastcap.c` 等。
- `NEAI/`：NanoEdge AI 库（`libneai.a` + `NanoEdgeAI.h`）。
- `CORE/`、`FWLIB/`：启动文件与标准外设库，一般不用改。
- `OBJ/`：Keil 编译输出（axf/hex 在这里生成），可随时清空。

## 编译与烧录

- `keil_keil.bat`：清理临时文件的辅助脚本。
- 命令行编译：`H:\毕设设计\output\telemetry_validation_20260927\build_stm32.ps1`
- 烧录（需电机停转）：`H:\毕设设计\output\telemetry_validation_20260927\deploy_stm32_duty_levels_20261010.ps1`

## 归档区 `_archive\`（只读历史，不参与编译）

- `backups\`：历次源码备份目录（`_bak_*`、`backup_*`、`validation_20260923` 等）。
- `deploys\`：历次部署产物目录（`DEPLOY_*`，含 hex/axf/读回校验记录）。
- `tools\`：散落的 J-Link 脚本、构建日志、临时 Python 脚本、UTF8 转换文件。
- `hardware_bak\`：`HARDWARE` 里清理出来的 `*.bak_*` 历史备份文件。
