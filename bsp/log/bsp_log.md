# bsp_log

`bsp_log` 是 BSP 日志接口。默认后端为 SEGGER RTT,调度器启动后日志进入内部队列,由 `BSPServiceTask` 统一输出。

## 接口

```c
LOGINFO("system init ok");
LOGWARNING("value:%d", value);
LOGERROR("error:%d", err);
LOG("raw text");
```

日志接口不支持浮点格式化输出。需要打印浮点值时,建议先换算成整数单位。

Debug 配置默认保留日志。Release、RelWithDebInfo、MinSizeRel 等配置可通过 `DISABLE_LOG_SYSTEM=1` 关闭
`LOGINFO/LOGWARNING/LOGERROR`。

## 队列化输出

FreeRTOS 调度器启动前还没有 `BSPServiceTask`,日志会直接输出到 RTT,方便查看早期初始化问题。

调度器启动后,日志流程为:

```text
LOG宏 -> 格式化到短buffer -> 投递到日志队列 -> BSPServiceTask统一输出RTT/USB
```

单条日志最大长度由 `BSP_LOG_LINE_SIZE` 控制。队列深度由 `BSP_LOG_QUEUE_SIZE` 控制。队列满时新日志会被丢弃,可通过
`BSPLogGetDroppedCount()` 查看。

## 中断约束

日志格式化和 RTT/USB 输出都不适合在 ISR 中执行。若 ISR 中误调用 LOG 宏,该条日志会被丢弃并计入丢弃计数。

中断中的错误应优先使用计数器记录,由任务上下文再统一查看或输出。

## 输出后端

默认只输出到 RTT:

```c
#define BSP_LOG_USE_USB 0U
```

若定义 `BSP_LOG_USE_USB=1`,日志会额外调用 `USBTransmit()` 发送到 USB CDC。但 USB 可能同时承担视觉、VOFA
或调试数据通道,开启前需要确认不会互相抢占。

## 调试工具

RTT 日志可通过 J-Link RTT Viewer、J-Scope、Ozone 或支持 RTT 的调试环境查看。

## 后续扩展

- 若 USB 日志后端长期启用,建议进一步处理 `log -> USBTransmit()` 与 USB 内部日志之间的潜在递归。
- 高频任务中仍应节制日志频率,队列化只降低输出阻塞,不消除格式化开销。
