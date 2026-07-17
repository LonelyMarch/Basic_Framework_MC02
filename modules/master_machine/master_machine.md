# master_machine 上位机通信模块

## 1. 模块定位

`master_machine` 是最新版《ESP32-S3 HTTP 转 UART 通信协议文档》的 STM32 接收端实现。

本模块只负责以下工作：

1. 通过 `bsp/usart` 接收 ESP32-S3 发来的 UART 字节流。
2. 从拆包、粘包和带噪声的字节流中恢复完整帧。
3. 校验帧类型、长度、checksum 和 payload 字段。
4. 保存最新 Command、最新 Vision，并排队保存离散 Event。
5. 提供桥接端在线状态、数据新鲜度、急停锁存和错误统计。

本模块不负责底盘、云台、视觉决策、事件业务处理，也不会把 WASD 转换为速度指令。所有控制含义都由 application 层解释。

新协议只定义了 ESP32-S3 到 STM32 的帧，因此本模块没有 STM32 到 ESP32-S3 的发送接口。

## 2. 文件结构

```text
modules/master_machine/
├── master_protocol.h       协议常量、packed payload和校验接口
├── master_protocol.c       checksum及payload语义校验
├── master_machine.h        通信管理层公开接口
├── master_machine.c        USART注册、流解析、数据通道和状态统计
└── master_machine.md       本文档
```

原 `master_process.c/.h` 及其旧接口已经删除，不保留兼容层。

## 3. UART协议

串口由 CubeMX 配置为：

```text
921600 baud
8 data bits
No parity
1 stop bit
No flow control
```

完整帧：

```text
AA 55 | type | len_low | len_high | payload | checksum
```

`payload_length` 使用小端格式：

```c
uint16_t payload_length = len_low | (len_high << 8);
```

`checksum` 是以下所有字节累加后的低 8 位：

```text
type + len_low + len_high + payload全部字节
```

当前三种类型及固定 payload 长度：

|   type | payload                |     长度 |
|-------:|------------------------|-------:|
| `0x01` | `MasterCommandPayload` |  23 字节 |
| `0x02` | `MasterVisionPayload`  | 125 字节 |
| `0x03` | `MasterEventPayload`   |  15 字节 |

`master_protocol.c` 使用 `_Static_assert` 在编译期锁定这三个尺寸。所有 payload 均为 packed 布局，整数为小端，浮点数为
IEEE754 单精度。

## 4. 接收与解析路径

```text
UART DMA/IT + IDLE中断
        │
        ▼
bsp/usart把本次接收片段放入双缓冲帧队列
        │
        ▼
BSPServiceTask调用USARTProcess()
        │
        ▼
MasterMachineReceiveCallback()（任务上下文）
        │
        ▼
追加到512字节流缓存并扫描AA 55
        │
        ├── 未知type/错误长度/错误checksum：前移1字节重新同步
        ├── 帧不完整：保留残帧，等待下一次USART回调
        ├── payload语义错误：丢弃当前完整帧
        └── 合法帧：按type分发
```

因此本模块不创建独立 FreeRTOS 任务。解析发生在现有 BSP 服务任务上下文，不在 UART 中断中执行。

流解析支持：

- 一个 UART 回调中包含多帧，即粘包。
- 一帧被多个 UART IDLE 回调切开，即拆包。
- 帧前、帧间存在噪声字节。
- 错误帧之后重新查找下一处 `AA 55`。
- payload 内部恰好出现 `AA 55` 时，不会破坏一帧已经通过 checksum 的边界。

## 5. payload语义校验

除长度和 checksum 外，还会检查以下字段：

### 5.1 Command

- `ms` 必须为 `0~999`。
- `w/a/s/d` 只能为 `0` 或 `1`。
- `rotate` 只能为 `-1/0/1`。
- `dx/dy` 必须是有限浮点数，拒绝 NaN 和正负无穷。

### 5.2 Vision

- `ms` 必须为 `0~999`。
- `result` 必须为 `-1~8`。
- `order[i]` 必须为 `0~8` 或未知字符编号 `255`。
- `goal[i]` 必须为 `-1/0/1`。
- `coords/rvec/tvec` 的所有浮点数必须为有限值。

### 5.3 Event

- `ms` 必须为 `0~999`。
- `event` 必须为 `1~3`。
- `value` 必须为有限浮点数。

## 6. 帧ID处理

Command、Vision 和 Event 分别维护自己的 `last_id`，不会把不同类型的编号混在一起比较。

- 第一次收到某种类型时直接接受。
- 新 ID 等于上一 ID：记为重复帧并丢弃。
- 新 ID 正向递增：接受；差值大于 1 时累计估算丢帧数。
- 新 ID 位于反向半区：记为乱序帧并丢弃。
- 使用无符号差值，支持 `uint32_t` 从 `0xFFFFFFFF` 回绕到 `0`。

重复或乱序帧不会覆盖上层数据，但只要 checksum 和 payload 语义合法，仍会刷新 bridge online 时间，因为它能够证明 ESP32-S3 与
UART 链路仍在工作。

## 7. 三类数据的保存方式

### 7.1 Command：最新值通道

Command 是周期控制输入，只保留最新一帧。内部使用双缓冲，发布时先写非活动槽，再原子切换活动槽。

每个消费者使用独立 `MasterMachineCursor_s`：

```c
static MasterMachineCursor_s chassis_command_cursor = {0};
MasterCommandPayload command;

if (MasterMachineReadCommand(&command, &chassis_command_cursor) != 0U)
{
    /* chassis消费者首次读取或收到新一代Command。 */
}
```

另一个消费者应定义自己的 cursor，不能共用：

```c
static MasterMachineCursor_s gimbal_command_cursor = {0};
```

这样任何消费者读取数据都不会清除其他消费者的“有新数据”状态。

如果只需要当前值而不关心是否更新，可调用：

```c
MasterMachinePeekCommand(&command);
```

### 7.2 Vision：最新值通道

Vision 同样采用双缓冲和分消费者 generation cursor：

```c
static MasterMachineCursor_s vision_cursor = {0};
MasterVisionPayload vision;

if (MasterMachineReadVision(&vision, &vision_cursor) != 0U)
{
    /* 处理最新视觉数据。中间来不及处理的旧视觉帧会被更新值覆盖。 */
}
```

这符合视觉控制通常只关心最新结果的特性，避免旧帧在系统繁忙后堆积。

### 7.3 Event：固定FIFO

Event 是不可用“最新值覆盖”表达的离散动作，因此使用容量为 8 的 FIFO：

```c
MasterEventPayload event_payload;

while (MasterMachinePopEvent(&event_payload) != 0U)
{
    switch (event_payload.event)
    {
    case MASTER_PROTOCOL_EVENT_EMERGENCY_STOP:
        break;
    case MASTER_PROTOCOL_EVENT_START_MATCH:
        break;
    case MASTER_PROTOCOL_EVENT_SERVE:
        break;
    default:
        break;
    }
}
```

Event FIFO应由 application 层的单一事件分发者消费。如果多个业务模块需要同一个事件，应由该分发者再发送消息或设置各自状态，而不是让多个模块竞争
`PopEvent()`。

队列已满时保留尚未处理的旧事件，并丢弃新事件，同时增加 `event_queue_overflow_count`。

## 8. 紧急停机锁存

紧急停机不只依赖 Event FIFO。收到合法 `event = 0x01` 时会立即设置独立锁存：

```c
if (MasterMachineIsEmergencyStopLatched() != 0U)
{
    /* 上层进入安全停机状态。 */
}
```

即使 Event FIFO 已满，急停锁存仍然生效。

普通 Command、开始比赛事件或通信恢复都不会自动清除锁存。上层完成机械、电气和状态机复位后，必须显式调用：

```c
MasterMachineClearEmergencyStop();
```

## 9. 初始化

```c
#include "master_machine.h"

MasterMachine_Init_Config_s master_config = {
    .uart_handle = &huart1,
    .bridge_timeout_ms = 1000U,
    .command_timeout_ms = 200U,
    .vision_timeout_ms = 1000U,
};

if (MasterMachineInit(&master_config) == 0U)
{
    /* 初始化失败处理。 */
}
```

三个 timeout 填 `0` 时使用默认值。模块是单实例设计，同一固件中只允许初始化一次，因为一块 STM32 只对应一个 ESP32-S3 上位机桥接端，且
BSP USART 也是点对点独占注册。

## 10. 在线状态与新鲜度

三个状态相互独立：

```c
MasterMachineIsBridgeOnline(); /* 最近是否收到任意合法类型帧。 */
MasterMachineIsCommandFresh(); /* 最近接受的新Command是否超时。 */
MasterMachineIsVisionFresh();  /* 最近接受的新Vision是否超时。 */
```

例如 ESP32-S3 仍在发送 Vision，但控制客户端停止发送 Command 时：

```text
bridge online = true
vision fresh  = true
command fresh = false
```

上层应根据所使用的数据类型判断新鲜度，不能只用 bridge online 代替控制命令看门狗。

模块使用 `HAL_GetTick()` 记录本地接收时间，不使用 payload 中的 `sec/ms` 判断超时。payload 时间戳属于远端时钟，除非系统另行完成时钟同步，否则不能直接和
STM32 本地时间比较。

## 11. 统计与诊断

```c
MasterMachineStatistics_s statistics;

if (MasterMachineGetStatistics(&statistics) != 0U)
{
    /* 读取checksum错误、语义错误、重复、乱序、估算丢帧和USART错误等信息。 */
}
```

统计内容包括：

- 接收总字节数和合法帧数。
- 重同步丢弃的噪声字节数。
- 流缓存保护性溢出次数。
- 未知 type、长度错误、checksum 错误、payload 语义错误。
- Event FIFO 溢出次数。
- BSP USART 硬件/接收错误次数。
- Command、Vision、Event 各自的接受、重复、乱序和估算丢帧数。

这些计数只保存于内存，不在高频接收路径中打印日志，避免通信异常反过来阻塞实时任务。

## 12. application层迁移说明

本次重构明确删除以下旧内容：

- `master_process.h/.c`
- `VisionInit()`、`VisionSend()`、`VisionSetFlag()`、`VisionSetAltitude()`
- `Uart_Control_Init()`
- `MasterMachineFetchCommand/Vision/Event()` 的全局 updated 标志接口
- `Vision_Recv_s`、`Vision_Send_s`、`UART_Recv_s`
- 旧 RoboMaster 目标类型、敌方颜色、弹速和开火模式枚举
- 模块内部 WASD 映射、`rev_order` 和统一 `MasterControlData_s` 业务状态

因此 application 中原先包含 `master_process.h` 或使用上述类型/函数的文件需要后续改为包含 `master_machine.h`，并在各自业务层解释新协议
payload。当前重构按要求没有修改这些调用处。
