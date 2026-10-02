# StarFlight构建与验证

依赖：ARM GNU 13.3.Rel1、CMake ≥3.21、Ninja、OpenOCD；VS Code 扩展 C/C++、CMake Tools、Cortex-Debug。`ARM_GCC_PATH` 可指向工具链根目录，相关 bin 目录加入 PATH。

```sh
cmake --preset debug
cmake --build --preset debug
cmake --preset release
cmake --build --preset release
```

输出为 `build/debug` 或 `build/release` 下的 `xingxuan_uav.elf/.hex/.bin/.map` 和 GCC `.su`。VS Code `Ctrl+Shift+B` 配置后构建；F5 使用 OpenOCD 调试。调试接口：ST-Link / SWD。

连接匹配的板卡与调试器后，可运行：

```sh
cmake --build build/debug --target flash
```

下载配置使用 `program ... verify reset exit`。ELF 由启动汇编复制 `.data`、清零 `.bss`；堆、栈和 `.bss` 不作为烧录数据。Flash/RAM 分区见链接脚本，`.su` 不替代 RTOS 实测栈余量。

## 软件验证与串口工具

```sh
python tools/validate.py --cc gcc
python tools/validate.py --preset debug
python tools/validate.py --preset release
python -m pip install -r tools/requirements.txt
python tools/serial_cli.py --port COM3 identify
python tools/serial_cli.py --port COM3 status
python tools/serial_cli.py --port COM3 diagnostics
python tools/serial_cli.py --port COM3 record capture.jsonl --seconds 10
python tools/replay_log.py capture.jsonl
```

使用实际串口，Linux 例如 `/dev/ttyUSB0`。录包文件必须不存在。回放用于帧与状态时间线检查，不是高频传感器解算或 HIL。主机测试使用本机 gcc；构建报告输出到 `build/reports`。CI 在 dev/master push 与 PR 上运行主机测试、Debug/Release 并保存固件/map/报告。

## 硬件验证

先在无桨台架验证八通道遥控标定、状态守卫、失联/旧姿态/急停、传感器量程、控制增益及混控方向、总线时序、任务栈和 Flash 掉电；完成这些检查后再独立验收飞行。CubeMX 重生成回归、六面标定、yaw 混控和定高仍待完善，见 TODO。

Debug/Release 软件构建及主机测试已有通过记录；探针烧录、实时任务水位和硬件行为仍待实际板卡验证。发布固件、map、报告应关联源码版本保存，构建目录不提交。

## 版本兼容与回滚

旧源码提交 `977d5b068c73dccd3e709bc8414d04f70d11acac` 可在独立 checkout 中检查；协议/参数格式按对应固件版本使用。功能在 dev 完成验收后合入 master，TM4C 仅同步适用的基础变更。不要以旧参数格式解释新版记录。
