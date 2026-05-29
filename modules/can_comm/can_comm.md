# can_comm

<p align='right'>neozng1@hnu.edu.cn</p>

## 模块定位

`can_comm` 是基于 `bsp/can` 的跨板结构体通信模块，主要用于双板模式下在云台板和底盘板之间传递应用层数据。

`bsp/can` 只负责单帧 CAN 报文的注册、过滤、发送和接收回调；`can_comm` 在它之上增加了业务包封装，负责把一个结构体拆成多帧 CAN 报文发送，并在接收端重新拼成完整结构体。

当前工程仍使用 Classic CAN over FDCAN，单帧有效数据最大 8 字节。`can_comm` 单个业务包的有效数据长度上限由 `CAN_COMM_MAX_BUFFSIZE` 决定，当前为 60 字节。

## 使用场景

双板模式下，`robot_cmd` 和 `chassis` 通过 `can_comm` 交换数据：

- 云台板发送 `Chassis_Ctrl_Cmd_s`，接收 `Chassis_Upload_Data_s`。
- 底盘板接收 `Chassis_Ctrl_Cmd_s`，发送 `Chassis_Upload_Data_s`。

`can_comm` 适合固定长度、小数据量、周期性发送的结构体通信。不适合大文件、长数据流或需要可靠重传的大块数据传输。

## 协议格式

一个 CANComm 业务包格式如下：

| 字段 | 长度 | 说明 |
| --- | --- | --- |
| 帧头 | 1 字节 | 固定为 `'s'` |
| 包序号 | 1 字节 | 每发送一个业务包递增，用于观察整包跳变 |
| 数据长度 | 1 字节 | 有效数据长度，必须等于初始化时配置的接收长度 |
| 有效数据 | n 字节 | 调用者传入的结构体字节流 |
| CRC8 | 1 字节 | 覆盖包序号、数据长度和有效数据 |
| 帧尾 | 1 字节 | 固定为 `'e'` |

接收端只支持初始化时配置的固定长度业务包。若长度字节不匹配、CRC 错误、帧尾错误或半包超时，当前半包会被丢弃并等待下一包重新同步。

## 对齐要求

`can_comm` 按字节复制结构体内容，不理解结构体字段含义。因此跨板传输的结构体必须保证两端内存布局一致。

推荐对需要跨板传输的结构体使用 `#pragma pack(1)`：

```c
#pragma pack(1)
typedef struct
{
    uint8_t mode;
    float vx;
    float vy;
} ExamplePacket_t;
#pragma pack()
```

注意：`CANCommInstance` 本身是 MCU 内部运行期对象，不直接作为协议帧发送，因此不需要 `pack(1)`，保持默认对齐即可。

## 初始化

接口：

```c
CANCommInstance *CANCommInit(CANComm_Init_Config_s *comm_config);
```

初始化配置示例：

```c
CANComm_Init_Config_s comm_conf = {
    .can_config = {
        .can_handle = &hfdcan1,
        .tx_id = 0x312,
        .rx_id = 0x311,
    },
    .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    .recv_data_len = sizeof(Chassis_Upload_Data_s),
    .daemon_count = 10,
};

CANCommInstance *comm = CANCommInit(&comm_conf);
```

初始化时会完成以下工作：

- 从静态池中分配 `CANCommInstance`。
- 注册一个 `CANInstance` 到 `bsp/can`。
- 注册一个 `DaemonInstance` 用于在线检测。
- 预填充发送缓冲区中的帧头、长度和帧尾相关信息。

若参数非法、长度超过上限、实例数量超限、CAN 注册失败或 Daemon 注册失败，函数返回 `NULL`。

## 发送接口

接口：

```c
uint8_t CANCommSend(CANCommInstance *instance, const uint8_t *data);
```

示例：

```c
Chassis_Ctrl_Cmd_s cmd;
(void)CANCommSend(comm, (const uint8_t *)&cmd);
```

返回值：

- `1`：整包所有 CAN 子帧均已成功提交到 `bsp/can` 发送接口。
- `0`：参数错误，或任意一个 CAN 子帧提交失败。

发送流程：

1. 填入包序号、数据长度和有效数据。
2. 计算 CRC8。
3. 将完整业务包按 8 字节以内拆成多个 CAN 子帧。
4. 每个子帧调用 `CANSetDLC()` 设置 DLC，再调用 `CANTransmit()` 发送。

同一个 `CANCommInstance` 默认只允许由一个任务调用 `CANCommSend()`。当前模块没有为整包分片发送过程增加实例级互斥，原因是双板通信通常由固定应用任务周期发送，额外互斥会增加同步开销。若后续多个任务需要共用同一个实例发送，应在上层合并发送入口，或再为该实例增加发送保护。

## 接收接口

接口：

```c
uint8_t CANCommGet(CANCommInstance *instance, void *data);
```

示例：

```c
Chassis_Upload_Data_s feedback;
if (CANCommGet(comm, &feedback) != 0U)
{
    // feedback 是本次收到的新数据
}
```

返回值：

- `1`：本次读取到了新数据，数据已复制到调用者传入的缓存。
- `0`：没有新数据，或参数非法。

接收流程：

1. FDCAN 中断只读取硬件 FIFO，并把单帧数据投递到 `bsp/can` 内部队列。
2. `CANProcessTask` 在任务上下文取出单帧数据，并调用 `CANCommRxCallback()`。
3. `CANCommRxCallback()` 按协议拼包、校验长度、校验 CRC 和帧尾。
4. 收到合法整包后，复制有效数据到内部解包缓存，置位 `update_flag`，并调用 `DaemonReload()`。
5. 应用任务调用 `CANCommGet()` 时，若有新数据，则短暂锁住调度器并复制到调用者缓存，避免读写撕裂。

通信在线但本周期没有新数据时，应用层可以保留上一帧数据，避免偶发丢包造成控制量抖动。底盘板当前在通信离线时会强制进入零力状态。

## 在线检测

接口：

```c
uint8_t CANCommIsOnline(CANCommInstance *instance);
```

返回值：

- `1`：最近收到过合法整包，Daemon 仍在线。
- `0`：实例非法，或 Daemon 已超时。

离线时，`CANCommLostCallback()` 会重置当前半包接收状态，并打印一次离线告警。再次收到合法整包后，会允许下一次离线重新打印告警。

## 错误统计

`CANCommInstance` 内部保存了若干接收错误计数：

- `rx_len_error_count`：长度错误计数。
- `rx_crc_error_count`：CRC 错误计数。
- `rx_tail_error_count`：帧尾错误计数。
- `rx_seq_jump_count`：包序号跳变事件计数。
- `rx_timeout_count`：半包超时计数。

这些字段当前主要用于调试观察，没有额外封装读取接口。若后续需要在 UI、日志或上位机中长期监控，可以再补充轻量 getter。

## 注意事项

- 协议格式变更后，通信两端必须同时使用同一版 `can_comm`，否则无法互通。
- `CANCommSend()` 返回成功只表示所有子帧已成功提交发送，不表示对端一定收到完整业务包。
- 若总线干扰较多或业务包较长，可以考虑在 CubeMX 中为双板通信所在 FDCAN 开启 `Auto Retransmission`。
- 若双板通信和电机共用同一路 CAN，需要关注总线负载，避免多帧业务包挤占高频电机通信。
- 当前半包超时为 `CAN_COMM_RX_TIMEOUT_MS`，超过该时间仍未收齐整包会丢弃半包。
- 当前最大有效数据长度为 60 字节，如果增大该值，需要同时评估 CAN 总线负载和发送周期。
