# bsp_usart

`bsp_usart` 封装 UART/USART 的注册、ReceiveToIdle 接收、任务上下文解析、阻塞/IT/DMA 发送和 DMA 内存域适配。

## 注册

```c
USART_Init_Config_s conf = {
    .usart_handle = &huart9,
    .recv_buff_size = 128,
    .module_callback = VisionDecode,
};

USARTInstance *usart = USARTRegister(&conf);
```

注册成功后 BSP 会从静态实例池中分配 `USARTInstance`,绑定 RX/TX DMA 缓冲区和接收帧缓存,然后自动启动接收。

## 接收流程

若串口配置了 RX DMA,使用 `HAL_UARTEx_ReceiveToIdle_DMA()`;否则使用 `HAL_UARTEx_ReceiveToIdle_IT()`。

DMA 接收启动后会关闭 half transfer 中断,避免同一帧在半满和 IDLE/满传输时被重复回调处理。

HAL 触发 `HAL_UARTEx_RxEventCallback()` 后,BSP 执行:

1. 根据 `UART_HandleTypeDef *` 找到实例。
2. 将本帧数据复制进 `BSPFrameQueue` 管理的接收帧缓存。
3. 立即重新启动 ReceiveToIdle 接收。
4. 唤醒 `BSPServiceTask`。

`USARTProcess()` 在任务上下文中遍历所有实例,取出待解析帧,更新 `instance->recv_buff` 和 `instance->recv_len`,再调用模块注册的
`module_callback()`。

## 发送流程

```c
HAL_StatusTypeDef USARTSend(USARTInstance *instance, uint8_t *buf, uint16_t len, USART_TRANSFER_DMA);
```

- 阻塞发送直接使用上层 buffer。
- IT/DMA 发送会先复制到实例内部 TX 缓冲区,避免上层局部变量提前失效。
- DMA 发送前会 clean D-Cache。
- 上一次异步发送未完成时返回 `HAL_BUSY`。

`USARTIsReady()` 可用于高频发送前判断串口是否空闲。

## DMA缓冲区

USART RX/TX DMA 缓冲区位于 `.dma_buffer` / `RAM_D2`,避免 DMA 访问 DTCM。接收帧缓存只由 CPU 访问,由 `BSPFrameQueue` 管理。

## 错误处理

`HAL_UART_ErrorCallback()` 中只递增实例 `error_count`,释放发送忙标志并尝试重新启动接收,不在中断中输出日志。上层可通过
`USARTGetErrorCount()` 查询错误次数。

## CubeMX要求

- DMA 接收串口需要配置 RX DMA。
- ReceiveToIdle 依赖 UART global interrupt 处理 IDLE 事件,对应 UART 全局中断必须开启。
- DMA/USART 中断优先级数值必须满足 FreeRTOS 可调用 API 的约束。

## 当前限制

- `module_callback` 当前没有参数,模块需要通过自己保存的 `USARTInstance *` 读取 `recv_buff/recv_len`。
- 当前没有 TX 队列,高频连续发送需要上层节流或后续扩展发送队列。
