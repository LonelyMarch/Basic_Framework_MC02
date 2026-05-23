# BSP 层说明

`bsp` 是当前工程中最靠近硬件的一层,位于 CubeMX/HAL 生成代码之上,向 `modules` 和 `application` 提供稳定的硬件访问接口。

开发板 PCB 上集成的功能模块,例如 BMI088、蜂鸣器、TFmini、蓝牙、裁判系统、视觉通信协议等,应放在 `modules` 层。BSP 层只封装 MCU 片上外设、外部存储访问能力和必要的 BSP 公共服务。

## 目录结构

- `bsp_init.c/.h`: BSP 初始化边界。`BSPInit()` 做调度器启动前基础初始化,`BSPTaskInit()` 创建 BSP 运行期资源和后台任务。
- `bsp_service.c/.h`: BSP 通用服务任务和延后事件队列。
- `bsp_frame_queue.c/.h`: BSP 内部固定帧队列,供 USART/USB 这类 ISR 生产帧、任务消费帧的模块复用。
- `adc`: ADC DMA 循环采样。
- `can`: 三路 FDCAN 的 Classic CAN 收发、过滤器和接收任务。
- `dwt`: DWT 高精度时间轴和短延时。
- `flash`: STM32H723 片上 Flash 用户区读写和异步擦写队列。
- `gpio`: GPIO 读写和 EXTI 延后回调。
- `iic`: I2C/IIC 总线互斥和阻塞/IT/DMA 同步事务。
- `log`: RTT 日志队列,可选 USB 后端。
- `pwm`: TIM PWM 输出、周期/占空比设置和 PWM DMA 回调。
- `qspi_flash`: 板载 W25Q64 外部 Flash 读写、擦除和内存映射。
- `spi`: SPI 从设备注册、片选控制、总线互斥和 DMA 中转缓冲。
- `usart`: UART/USART ReceiveToIdle 接收、任务解析和发送封装。
- `usb`: USB CDC 调试/VOFA 通道封装。

## 初始化边界

CubeMX 生成的 `MX_..._Init()` 仍由 `main.c` 调用,负责完成底层外设初始化。

`RobotInit()` 调用 `BSPInit()`。`BSPInit()` 只初始化必须在调度器启动前存在的基础资源:

```c
DWT_Init(SystemCoreClock / 1000000U);
BSPLogInit();
BSPServiceInit();
BSP_Flash_Init();
BSP_FlashAsyncInit();
HAL_TIM_Base_Start_IT(&htim6);
```

CAN、USART、SPI、IIC、PWM、GPIO、ADC 等实例型外设不在 `BSPInit()` 中统一注册,而由模块层按需调用 `XXXRegister()`。

FreeRTOS 内核初始化后,`RobotOSTaskInit()` 通过 `BSPTaskInit()` 创建 BSP 运行期资源:

- `SPIBusOsInit()`: 为已登记 SPI 总线创建 mutex/semaphore。
- `IICBusOsInit()`: 为已登记 I2C 总线创建 mutex/semaphore。
- `CANProcessTask`: 高优先级处理 CAN 接收队列。
- `BSPServiceTask`: 普通优先级处理 GPIO/USART/USB/log 等轻量延后事件。
- `BSP_FlashAsyncTask`: 低优先级处理片上 Flash 和 QSPI Flash 异步擦写。

## 设计原则

BSP 层采用 `XXXInstance + XXXRegister()` 的注册式结构。模块注册实例时传入 HAL 句柄、硬件参数、回调和用户指针;BSP 保存这些信息,后续操作围绕返回的实例指针进行。

注册型 BSP 控制结构体使用静态池,不依赖 `malloc()`。这些实例池默认位于 `.bss` / DTCM,适合 CPU 高频访问。

DMA 真正访问的数据缓冲必须位于 DMA 可访问内存域。当前 SPI/IIC/USART/ADC 的 DMA 缓冲区位于 `.dma_buffer` / `RAM_D2`,必要时由 BSP 内部 bounce buffer 转接上层普通内存。

`bsp_init`、`bsp_service`、`bsp_flash_async` 属于 BSP 组合/调度层,允许依赖多个 BSP 模块。普通外设模块应避免彼此直接依赖,除公共的 `bsp_log`、`bsp_service`、`bsp_frame_queue` 外,尽量只依赖自身 HAL 外设。

## FreeRTOS约定

- 中断中只保存必要数据、释放信号量、递增计数或投递延后事件。
- 中断中不做协议解析、长循环、阻塞等待、动态内存申请或复杂日志输出。
- USART/USB 接收使用 `BSPFrameQueue` 缓存完整帧,再由 `BSPServiceTask` 在任务上下文解析。
- CAN 接收频率高,使用独立 `CANProcessTask`,不压到通用 BSP 服务任务。
- SPI/IIC 这类总线型外设使用按总线管理的 mutex 串行访问。
- Flash/QSPI Flash 同步擦写不允许在 ISR 中调用,实时任务中优先使用异步接口或自行避开关键周期。

## 错误处理约定

初始化或注册阶段的硬件资源错误属于严重错误,例如实例数量超限、句柄为空、过滤器数量不足、外设启动失败等。此类错误通常先输出 `LOGERROR`,再返回 `NULL/HAL_ERROR` 或进入 `Error_Handler()`。

运行期接口错误不直接停机,例如发送忙、参数非法、等待超时、DMA 长度超限、队列满等。此类错误通过返回值表示失败,必要时输出 `LOGWARNING` 或 `LOGERROR`。

ISR 中不输出复杂日志。需要记录 ISR 错误时,优先维护错误计数器,由任务上下文查询。

## 解耦关系

当前大部分底层外设模块已经保持相对解耦:

- `ADC/CAN/SPI/IIC/PWM/GPIO/Flash/QSPI Flash` 主要依赖自身 HAL 和 `bsp_log`。
- `GPIO/USART/USB/log` 依赖 `bsp_service` 做任务唤醒或延后回调。
- `USART/USB` 共用 `BSPFrameQueue`,避免重复维护相似帧队列。
- `bsp_log` 默认只依赖 RTT;启用 USB 日志后会额外依赖 `bsp_usb`。

不建议把 `bsp_service` 进一步改成复杂插件式注册表。当前显式调用 `USARTProcess()`、`USBProcess()` 和 `BSPLogProcess()` 更直接,复杂度更低。

## 文档约定

每个 BSP 模块的 `.md` 文档应说明:

- 模块职责和边界。
- 注册/初始化方式。
- 主要接口和调用流程。
- FreeRTOS、中断、DMA、D-Cache 相关约束。
- CubeMX 配置要求和当前限制。

源码注释应优先解释“为什么这么做”和“上下文约束”,避免重复描述显而易见的赋值语句。
