# bsp_dwt

DWT 是 Cortex-M 内核中的调试计数单元，本模块使用 `DWT->CYCCNT` 作为高精度时间基准，用于计算时间间隔、获取系统运行时间轴以及进行短时间忙等待延时。

## 初始化

使用前需要先初始化 DWT，传入 CPU 内核频率，单位为 MHz：

```c
DWT_Init(SystemCoreClock / 1000000U);
```

`DWT->CYCCNT` 按 CPU cycle 计数，不是按 RTOS tick 计数，因此 DWT 时间不依赖 FreeRTOS 调度。

## 常用功能

### 计算两次进入同一个函数的时间间隔

```c
static uint32_t cnt;
float deltaT;

deltaT = DWT_GetDeltaT(&cnt);
```

如果需要更高精度的返回值，可以使用：

```c
static uint32_t cnt;
double deltaT;

deltaT = DWT_GetDeltaT64(&cnt);
```

### 获取当前运行时间

```c
float time_s;
float time_ms;
uint64_t time_us;

time_s = DWT_GetTimeline_s();
time_ms = DWT_GetTimeline_ms();
time_us = DWT_GetTimeline_us();
```

`DWT_GetTimeline_s()` 和 `DWT_GetTimeline_ms()` 返回 `float`，适合调试和短时间计时；`DWT_GetTimeline_us()` 返回 `uint64_t`，适合需要整数微秒时间戳的场景。

### 计算执行某部分代码的耗时

```c
float start, dt;

start = DWT_GetTimeline_ms();

// some proc to go...
for (uint8_t i = 0; i < 10; i++)
{
    foo();
}

dt = DWT_GetTimeline_ms() - start;
```

也可以使用 `TIME_ELAPSE` 宏进行调试计时：

```c
static float my_func_dt;

TIME_ELAPSE(my_func_dt,
            Function1(vara);
            Function2(some, var);
            Function3(your, param);
);
```

### 短时间延时

```c
DWT_Delay(0.001f); // 延时 1ms
```

`DWT_Delay()` 是忙等待延时，适合初始化阶段、关中断场景以及 us/ms 级短时序等待。它不会让出 CPU，因此在 FreeRTOS 任务中进行较长延时时，应优先使用 `osDelay()` 或状态机。

当前实现会对非正数延时直接返回；如果传入过长延时，会打印错误日志并截断到安全上限，避免超过 `CYCCNT` 计数范围后卡死。

### 维护 DWT 长时间轴

```c
DWT_SysTimeUpdate();
```

`DWT_SysTimeUpdate()` 用于刷新 `CYCCNT` 溢出计数。如果系统长时间不调用任何 DWT 时间接口，需要周期性调用该函数，确保 64 位扩展时间轴能够正确记录 `CYCCNT` 溢出。
