# bsp usart

## 功能说明

`bsp_usart` 用于统一管理基于 UART/USART 的模块通信。每个使用串口的模块通过 `USARTRegister()` 注册一个 `USARTInstance`,注册时需要指定 UART 硬件句柄、单次接收缓冲区长度和协议解析回调函数。

当前 USART BSP 支持:

- 使用 `HAL_UARTEx_ReceiveToIdle_DMA()` 或 `HAL_UARTEx_ReceiveToIdle_IT()` 接收变长帧。
- 将接收回调中的数据保存到双解析缓冲区,再由 `USARTProcess()` 在任务上下文中执行模块解析回调。
- 使用阻塞、中断或 DMA 方式发送数据。
- 为 DMA 收发准备位于 `RAM_D2` 的内部缓冲区,避免 DMA 访问 DTCM。
- 对注册失败、参数错误、发送忙等情况通过返回值和日志提示。

## 容量配置

当前头文件中的容量宏为:

```c
#define DEVICE_USART_CNT 8
#define USART_RXBUFF_LIMIT 256
#define USART_TXBUFF_LIMIT 256
#define USART_PARSE_BUFF_CNT 2
```

`DEVICE_USART_CNT` 表示 BSP 最多管理的串口实例数量。

`USART_RXBUFF_LIMIT` 表示单次接收数据的最大长度。注册时的 `recv_buff_size` 必须大于 0 且不超过该值。

`USART_TXBUFF_LIMIT` 表示 IT/DMA 发送内部缓冲区的最大长度。阻塞发送直接使用上层传入的发送缓冲区,IT/DMA 发送会先复制到实例内部 `tx_dma_buff`。

`USART_PARSE_BUFF_CNT` 表示每个串口实例拥有的解析缓冲数量。当前为双缓冲,用于避免任务解析当前帧时被下一帧接收数据覆盖。

## 实例注册

使用 `USARTRegister()` 注册一个串口实例:

```c
USART_Init_Config_s conf = {
    .usart_handle = &huartx,
    .recv_buff_size = RECV_SIZE,
    .module_callback = ModuleRxCallback,
};

USARTInstance *instance = USARTRegister(&conf);
if (instance == NULL)
{
    LOGERROR("[module] USART register failed");
    return NULL;
}
```

注册成功后,`USARTRegister()` 会分配并初始化 `USARTInstance`,绑定内部 RX/TX DMA 缓冲区和解析缓冲区,随后自动调用 `USARTServiceInit()` 启动接收。

注册失败返回 `NULL`,常见原因包括:

- 配置指针为空。
- `usart_handle` 为空。
- `recv_buff_size` 为 0 或超过 `USART_RXBUFF_LIMIT`。
- 注册实例数量超过 `DEVICE_USART_CNT`。
- 同一个 `UART_HandleTypeDef` 重复注册。
- `malloc()` 申请实例内存失败。

上层模块必须检查 `USARTRegister()` 的返回值,避免注册失败后继续使用空指针。

## 接收流程

`USARTServiceInit()` 会根据 UART 是否配置了 RX DMA 选择接收方式:

- 若 `huart->hdmarx != NULL`,使用 `HAL_UARTEx_ReceiveToIdle_DMA()`。
- 若没有 RX DMA,使用 `HAL_UARTEx_ReceiveToIdle_IT()`。

DMA 接收启动后会关闭 half transfer 中断:

```c
__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
```

这样可以避免同一段数据在 DMA 半满、DMA 满传输和 IDLE 事件中被重复处理。当前逻辑只希望处理传输完成和 IDLE 两类事件。

当 HAL 触发 `HAL_UARTEx_RxEventCallback()` 时,BSP 不在中断上下文中直接解析协议,而是执行以下动作:

1. 根据 `UART_HandleTypeDef *` 找到对应的 `USARTInstance`。
2. 将本次接收数据从 `rx_dma_buff` 复制到当前写入解析缓冲。
3. 记录本帧长度到 `parse_len[]`。
4. 增加 `pending_frame_cnt`。
5. 立即重新启动接收。

如果两个解析缓冲都还没有被任务处理完,新的接收帧会被丢弃,同时 `dropped_frame_cnt` 加 1。

## 延后解析

`USARTProcess()` 必须在任务上下文中周期调用。当前工程暂时由 `application/robot_task.h` 中的 USART 任务调用。

`USARTProcess()` 会遍历所有已注册实例,处理每个实例等待解析的帧:

1. 取出当前读缓冲索引。
2. 将 `instance->recv_buff` 指向当前解析缓冲。
3. 将 `instance->recv_len` 更新为当前帧长度。
4. 调用模块注册的 `module_callback()`。
5. 清空已经处理的解析缓冲。
6. 更新读索引和待处理帧计数。

模块回调函数当前没有参数,因此模块需要通过自己保存的 `USARTInstance *` 访问接收数据:

```c
static USARTInstance *module_usart_instance;

static void ModuleRxCallback(void)
{
    uint8_t *rx = module_usart_instance->recv_buff;
    uint16_t len = module_usart_instance->recv_len;

    if (len < MODULE_MIN_FRAME_SIZE)
        return;

    /* 解析 rx 中的数据 */
}
```

## 发送流程

使用 `USARTSend()` 发送数据:

```c
HAL_StatusTypeDef status = USARTSend(instance, tx_buf, tx_len, USART_TRANSFER_DMA);
if (status == HAL_BUSY)
{
    /* 上一次异步发送尚未完成 */
}
```

发送模式由调用参数指定:

```c
typedef enum
{
    USART_TRANSFER_NONE = 0,
    USART_TRANSFER_BLOCKING,
    USART_TRANSFER_IT,
    USART_TRANSFER_DMA,
} USART_TRANSFER_MODE;
```

`USART_TRANSFER_BLOCKING` 使用 `HAL_UART_Transmit()`。函数返回时本次发送已经结束,随后立即释放 TX 占用标志。

`USART_TRANSFER_IT` 使用 `HAL_UART_Transmit_IT()`。发送前会先将上层数据复制到实例内部 `tx_dma_buff`,避免上层局部变量在发送完成前失效。

`USART_TRANSFER_DMA` 使用 `HAL_UART_Transmit_DMA()`。发送前同样先复制到位于 `RAM_D2` 的 `tx_dma_buff`,避免 DMA 访问 DTCM 中的普通局部变量、普通全局变量或堆内存。

`USARTSend()` 返回值含义:

- `HAL_OK`: 阻塞发送完成,或异步发送已经成功启动。
- `HAL_BUSY`: 当前串口上一次发送尚未完成。
- `HAL_ERROR`: 参数非法、发送长度超限、发送模式非法或 HAL 启动发送失败。

当前 BSP 没有实现发送队列。短时间连续调用 IT/DMA 发送时,上层应检查 `USARTSend()` 返回值,或先通过 `USARTIsReady()` 判断串口是否可发送。

## 发送完成与错误处理

BSP 通过 `tx_busy` 标志保护异步发送,防止上一帧还没发完时覆盖实例内部 TX 缓冲。

以下 HAL 回调会释放 `tx_busy`:

- `HAL_UART_TxCpltCallback()`
- `HAL_UART_AbortCpltCallback()`
- `HAL_UART_AbortTransmitCpltCallback()`
- `HAL_UART_ErrorCallback()`

`HAL_UART_ErrorCallback()` 还会重新调用 `USARTStartReceive()` 尝试恢复接收,并通过 `LOGWARNING` 记录错误。

`USARTIsReady()` 同时检查 `tx_busy` 和 HAL 的 `gState`,用于判断当前串口是否可以发起新的发送。

## DMA缓冲区

STM32H7 的 DMA1/DMA2 不能访问 DTCM。本工程默认部分普通数据可能位于 DTCM,因此 USART BSP 将 DMA 收发缓冲区放在 `.dma_buffer` 段:

```c
static uint8_t usart_rx_dma_buffer[DEVICE_USART_CNT][USART_RXBUFF_LIMIT]
    __attribute__((section(".dma_buffer"), aligned(32)));

static uint8_t usart_tx_dma_buffer[DEVICE_USART_CNT][USART_TXBUFF_LIMIT]
    __attribute__((section(".dma_buffer"), aligned(32)));
```

当前链接脚本将 `.dma_buffer` 映射到 `RAM_D2`。注册时会主动清空 RX/TX DMA 缓冲区,因为 `.dma_buffer` 是 `NOLOAD` 段,上电后内容不保证为 0。

解析缓冲 `usart_parse_buffer` 不直接交给 DMA,只由 CPU 访问,用于任务上下文中的协议解析。

## CubeMX配置要求

使用 DMA 接收的串口需要在 CubeMX 中配置 RX DMA,并开启对应 UART global interrupt。`HAL_UARTEx_ReceiveToIdle_DMA()` 需要依赖 UART 全局中断处理 IDLE 事件。

没有 RX DMA 的串口可以使用 `HAL_UARTEx_ReceiveToIdle_IT()`,但仍需要开启对应 UART global interrupt。

若使用 DMA 发送,需要为对应 UART 配置 TX DMA。若使用 IT 发送,需要开启对应 UART global interrupt。

## @TODO

@TODO: 当前 USART 延后解析已经通过 `USARTProcess()` 实现,但服务任务暂时仍由 `application/robot_task.h` 创建。后续应和 CAN/GPIO/USB/IIC/PWM/LOG 等 BSP 层中断延后处理问题一起统一设计,将 BSP 服务任务的创建与调度入口收回到 BSP 层或统一的 BSP 服务管理模块中。

@TODO: 当前模块接收回调 `usart_module_callback` 仍然没有参数。后续可以考虑改为 `module_callback(USARTInstance *instance, uint16_t recv_len)`,让模块显式获得触发回调的实例和本帧长度。

@TODO: 当前发送接口没有发送队列。后续如果存在高频连续发送场景,可以增加按实例管理的发送队列,由 TX complete 回调或 USART 服务任务串行取队列发送。
