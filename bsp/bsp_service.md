# bsp_service

`bsp_service` 是 BSP 层统一的轻量服务任务,用于把中断中的简单事件延后到任务上下文处理。

## 职责

- 提供 BSP 延后事件队列。
- 处理 GPIO EXTI 延后回调。
- 统一调用 `USARTProcess()`、`USBProcess()` 和 `BSPLogProcess()`。
- 为 USART/USB/log 这类自带内部缓冲区的模块提供任务唤醒入口。

CAN 接收不走 `BSPServiceTask`。高速 CAN 报文由独立的 `CANProcessTask` 处理,避免挤占通用 BSP 服务任务。

## 初始化

```c
BSPServiceInit();
```

`BSPServiceInit()` 在 `BSPInit()` 阶段创建 FreeRTOS 静态 Queue,不依赖 heap。

`BSPServiceTask` 由 `BSPTaskInit()` 创建。任务启动后会记录自身 task handle,供其他 BSP 模块唤醒。

## 事件投递

```c
uint8_t BSPServicePost(BSPServiceEventCallback callback, void *arg);
uint8_t BSPServicePostFromISR(BSPServiceEventCallback callback, void *arg);
void BSPServiceNotify(void);
```

`BSPServicePost()` 用于任务上下文投递回调。`BSPServicePostFromISR()` 用于 ISR 中投递回调。

USART、USB、log 这类模块已经有内部队列或帧缓存,不需要额外投递空事件,只调用 `BSPServiceNotify()` 唤醒服务任务即可。

## 处理流程

`BSPServiceProcess()` 的处理顺序为:

1. 先输出一轮日志队列。
2. 处理通用延后事件队列。
3. 调用 `USARTProcess()`。
4. 调用 `USBProcess()`。
5. 再输出一轮日志队列。

任务循环中使用任务通知阻塞等待,并保留较慢超时作为兜底,避免极端漏通知后数据长期滞留。

## 使用原则

- ISR 中只保存必要数据并投递事件,不要直接解析协议或执行耗时逻辑。
- 延后事件回调运行在任务上下文,可以执行轻量模块状态更新。
- 长时间计算、Flash 擦写、高频通信解析不应塞入通用 BSP 服务任务。
- 可通过 `BSPServiceGetDroppedEventCount()` 查看延后事件队列满导致的丢弃次数。
