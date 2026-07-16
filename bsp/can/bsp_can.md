# bsp_can

`bsp_can` 是 STM32H723 三路 FDCAN 的 BSP 封装。当前工程使用 Classic CAN over FDCAN,单帧数据长度最大 8 字节。

> 使用 CAN 总线时,总线上应只保留两个终端电阻。开发板、电调、电机可能都带终端电阻,需要按实际接线确认。

## 职责边界

- CubeMX 负责 FDCAN 时钟、引脚、FIFO、过滤器数量和中断配置。
- BSP 负责 CAN 实例注册、过滤器配置、三路 FDCAN 启动、发送互斥、接收队列和模块回调分发。
- 电机协议、双板通信协议等解析逻辑属于 modules 层,通过 `can_module_callback` 接收数据。

## 注册

```c
CAN_Init_Config_s conf = {
    .can_handle = &hfdcan2,
    .tx_id = 0x200,
    .rx_id = 0x201,
    .can_module_callback = MotorDecode,
    .id = motor,
};

CANInstance *can = CANRegister(&conf);
```

每个 `CANInstance` 保存发送 ID、接收 ID、8 字节发送缓存、8 字节接收缓存和模块回调。模块发送前先写 `instance->tx_buff`,再调用
`CANTransmit()`。

## 过滤器与FIFO

注册时 BSP 会根据 `rx_id` 为对应 FDCAN 配置标准 ID 过滤器。当前支持三路 FDCAN,每路维护独立的实例列表和过滤器索引。

CubeMX 中每路 `StdFiltersNbr` 必须不小于该路注册实例数量。若使用两个接收 FIFO,需要在 CubeMX 中同时给 FIFO0/FIFO1
分配元素,并开启对应中断。

## 接收流程

FDCAN 接收中断中只做轻量工作:

1. 循环读取当前 FIFO 中已有报文。
2. 根据 FDCAN 句柄和接收 ID 找到注册实例。
3. 将帧数据复制到 CAN 内部接收事件队列。
4. 唤醒高优先级 `CANProcessTask`。

`CANProcessTask` 在任务上下文中取出队列事件,更新对应实例的 `rx_buff/rx_len`,再调用模块注册的 `can_module_callback()`。

这种设计避免在 FDCAN 中断里直接解析电机协议,也避免高速 CAN 接收挤占通用 `BSPServiceTask`。

## 发送流程

```c
can->tx_buff[0] = value;
CANTransmit(can, 1.0f);
```

`CANTransmit()` 会先获取对应 FDCAN 总线的发送互斥锁,再等待 Tx FIFO Queue 有空位。若 FreeRTOS 已运行且超时时间达到毫秒级,等待过程中会
`osDelay(1)` 主动让出 CPU;亚毫秒等待仍使用 DWT 短忙等。

`timeout` 单位为 ms,不应大于调用任务自身周期。

## 接口

```c
CANInstance *CANRegister(CAN_Init_Config_s *config);
void CANSetDLC(CANInstance *instance, uint8_t length);
uint8_t CANTransmit(CANInstance *instance, float timeout);
void CANProcessTask(void *argument);
uint32_t CANGetDroppedRxEventCount(void);
uint32_t CANGetFifoLostCount(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo);
uint32_t CANGetHardwareLostCount(void);
uint32_t CANGetRxHalErrorCount(void);
uint32_t CANGetRxInvalidDlcCount(void);
```

## 错误统计

- `CANGetDroppedRxEventCount()`: CAN 内部接收事件队列满导致的软件丢帧。
- `CANGetFifoLostCount()`: 指定硬件 FIFO 的 FDCAN 丢帧计数。
- `CANGetRxHalErrorCount()`: 中断中 HAL 接收失败次数。
- `CANGetRxInvalidDlcCount()`: 收到非法 DLC 的次数。

中断上下文不输出日志,只更新计数。

## 注意事项

- 当前不是 CAN FD 大帧模式,`tx_buff/rx_buff` 固定 8 字节。
- 若后续启用 CAN FD/BRS,需要同步扩展 DLC 映射、缓存大小、CubeMX element size 和上层协议解析。
- CAN 注册失败、过滤器不足、FDCAN 启动失败属于初始化阶段严重错误。
