# bsp_dwt

`bsp_dwt` 使用 Cortex-M7 内核的 `DWT->CYCCNT` 作为高精度时间基准,用于时间间隔测量、系统运行时间轴和短时间忙等待延时。

## 初始化

```c
DWT_Init(SystemCoreClock / 1000000U);
```

参数单位为 MHz。当前由 `BSPInit()` 使用 `SystemCoreClock` 自动换算,避免系统主频调整后忘记同步修改。

## 时间接口

```c
float DWT_GetDeltaT(uint32_t *cnt_last);
double DWT_GetDeltaT64(uint32_t *cnt_last);
float DWT_GetTimeline_s(void);
float DWT_GetTimeline_ms(void);
uint64_t DWT_GetTimeline_us(void);
```

`DWT_GetDeltaT()` 适合计算周期任务的 `dt`。`DWT_GetTimeline_us()` 使用 64 位时间轴,更适合长时间运行后的微秒时间戳。

## 64位时间轴

STM32H723 在 480MHz 下,32 位 `CYCCNT` 大约 8.95s 溢出一次。BSP 内部用溢出扩展维护 64 位 cycle
计数,并在极短临界区内完成读取和溢出判断,避免任务/中断并发读取时出现时间轴跳变。

TIM6 当前用于每 1s 调用一次 `DWT_SysTimeUpdate()`,防止系统长时间没有主动读取 DWT 时漏记 `CYCCNT` 溢出。

## 短延时

```c
DWT_Delay(0.0005f);
```

`DWT_Delay()` 是忙等待,适合初始化阶段、关中断场景和 us/ms 级短时序。FreeRTOS 任务中需要较长等待时,应优先使用 `osDelay()`
或状态机。

单次忙等待不能跨越过长时间。当前实现会限制过长延时并输出错误日志。

## 调试宏

```c
float dt;
TIME_ELAPSE(dt, {
    SomeFunction();
});
```

`TIME_ELAPSE()` 会计算代码块耗时并通过 BSP 日志输出,适合临时调试。高频路径中不建议长期保留该宏。

## 注意事项

- DWT 是内核计数器,不依赖外设定时器引脚。
- 该模块不负责 FreeRTOS tick,也不替代系统调度延时。
- `DWT_SysTimeUpdate()` 只维护时间轴,不应放入耗时逻辑。
