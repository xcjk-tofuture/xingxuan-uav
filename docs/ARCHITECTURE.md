# StarFlight（StarFlight）架构

入口：`firmware/platform/stm32/startup_stm32f407xx.S: Reset_Handler` → `Core/Src/main.c: main` → `MX_FREERTOS_Init` → `firmware/app/startup.c: app_tasks_init` → `osKernelStart`。
`APP_RTOS_EXTERNAL_TASKS=1` 排除生成模板中的旧任务，实际创建配置以 `startup.c` 为准。

`Core/Drivers/Middlewares/.ioc` 保持 CubeMX 结构。手写代码位于 `firmware/app/services/algorithms/drivers/platform/boards/os`，分别负责业务与任务、服务、独立计算、设备、芯片适配、板级资源和必要同步。
CubeMX 开启 Keep User Code，重新生成后核查初始化桥接、DMA 回调、内核配置、时基和 CMake 源清单，完整构建后再上板。

传感器 → `Sensor_Data_Task_Proc` → 数据处理/独立姿态算法 → 姿态快照 → `Motor_Task_Proc` → 控制/PWM。SBUS 任务发布遥控快照，控制入口判定运行状态；显示和存储与控制任务分开。独立状态机、诊断及带校验双副本标定记录位于 `dev`，未纳入此基线。

## 任务与资源所有权

| 任务 | 优先级 | 配置栈 | 周期/等待 | 资源与失败策略 |
|---|---|---|---|---|
| Sensor | Realtime | 768 words | 1ms 基准、2ms读取惯性量、5ms解算、20ms磁场 | SPI2独占、算法上下文、标定与姿态快照；过期不补算积分 |
| Control | High | 512 words | 5ms固定周期 | 飞行状态、PID/混控；失联/姿态旧于100ms停止 |
| SBUS | AboveNormal | 256 words | 队列事件，100ms失联判定 | UART6 RX4×25，校准通道快照 |
| PC | Normal | 768 words | RX事件/5ms超时检查，遥测20–1000ms | UART1，RX4×100；仅查询姿态，不远程解锁 |
| OLED | Idle | 256 words | 100ms | UI/页面、屏幕；SPI1互斥保护每次CS事务 |
| Storage | Idle | 768 words | 写请求事件 | 队列2个按值副本；原标定存储；独占4K scratch，busy超时5s |
| Flow | Idle | 128 words | 队列事件，100ms旧数据失效 | RX4×14；ISR不解析，不计算浮点 |
| Key/RGB | Idle | 各128 words | 5ms按键/低速LED | 只提交标定与页面请求 |
| Log | Idle | 128 words | 队列事件/批量64字节 | UART3，256字节；TX≤20ms，满则丢日志 |

配置栈为 FreeRTOS 的 32 位项数，不是实测余量。厂商 SDK/内核保持原版；GCC 使用匹配内核端口。具体版本见 SOURCES.md。
