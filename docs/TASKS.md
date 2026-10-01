# 任务与资源所有权

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

STM32 CMSIS-RTOS v1 适配器直接把 stacksize 传给 FreeRTOS xTaskCreate，因此这里是32位 words；AR CMSIS-RTOS2 是 bytes。配置值不是实测高水位。STM32堆24KiB，TM4C堆16KiB；MSP与newlib堆另由链接脚本预留。具体余量须测 uxTaskGetStackHighWaterMark 和空闲堆。
