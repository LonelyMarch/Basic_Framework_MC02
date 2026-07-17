# bsp_spi

`bsp_spi` 是 SPI 总线型外设 BSP。它以“一个 SPIInstance 对应一个 SPI 从设备”的方式封装片选、总线互斥、阻塞/IT/DMA 传输和 DMA
内存域适配。

## 注册

```c
SPI_Init_Config_s conf = {
    .spi_handle = &hspi2,
    .GPIOx = BMI088_CS_GPIO_Port,
    .cs_pin = BMI088_CS_Pin,
    .spi_work_mode = SPI_DMA_MODE,
    .callback = Bmi088RxDone,
    .id = imu,
};

SPIInstance *spi = SPIRegister(&conf);
```

注册阶段只记录从设备和片选信息,不会创建 RTOS 对象。`BSPTaskInit()` 会在 FreeRTOS 内核初始化后调用 `SPIBusOsInit()`,为已经登记的
SPI 总线创建 mutex 和完成信号量。

## 工作模式

- `SPI_BLOCK_MODE`: 调用 HAL 阻塞接口,适合初始化和短事务。
- `SPI_IT_MODE`: 调用 HAL 中断接口,随后等待完成信号量。
- `SPI_DMA_MODE`: 调用 HAL DMA 接口,随后等待完成信号量。

IT/DMA 模式对上层仍表现为同步事务:函数返回 `HAL_OK` 时,本次传输已经完成,片选已经释放。

若实例配置为 `SPI_IT_MODE` 或 `SPI_DMA_MODE`,但当前 FreeRTOS 调度器尚未运行,本次事务会自动降级为 `SPI_BLOCK_MODE`
。这样模块可以在 `RobotInit()` 阶段完成芯片初始化,进入任务后再按注册时配置使用 IT/DMA。

## 传输接口

```c
HAL_StatusTypeDef SPITransmit(SPIInstance *spi, uint8_t *tx, uint16_t len);
HAL_StatusTypeDef SPIRecv(SPIInstance *spi, uint8_t *rx, uint16_t len);
HAL_StatusTypeDef SPITransRecv(SPIInstance *spi, uint8_t *rx, uint8_t *tx, uint16_t len);
```

执行流程:

1. 检查参数和上下文。
2. 获取对应 SPI 总线 mutex。
3. 拉低片选。
4. 执行阻塞/IT/DMA 传输。
5. 等待传输完成或超时。
6. 拉高片选并释放 mutex。
7. 成功接收后在调用者任务上下文执行实例回调。

同步 SPI 接口不允许在中断上下文调用。若由 EXTI 触发读取,应在中断里通知任务,再由任务调用 SPI 接口。

## DMA缓冲区

STM32H7 的 DMA1/DMA2 不能访问 DTCM。`bsp_spi` 为每条 SPI 总线准备了位于 `.dma_buffer` / `RAM_D2` 的 TX/RX
中转缓冲区。上层可以传入普通局部数组、全局数组或堆内存。

单次 DMA 传输长度受 `SPI_DMA_BOUNCE_BUFFER_SIZE` 限制。大块屏幕刷新或 Flash 流式传输应另行设计分块 DMA 或专用缓冲。

## D-Cache

若开启 D-Cache,BSP 会在 DMA 发送前 clean TX 缓冲区,在 DMA 接收前后 invalidate RX 缓冲区。若 `.dma_buffer` 后续被 MPU 配置为
non-cacheable,可将 `SPI_USE_DMA_CACHE_MAINTENANCE` 设为 `0U`。

## CubeMX要求

- 使用 `SPI_DMA_MODE` 时,对应 SPI 必须配置 TX/RX DMA。
- 使用 `SPI_IT_MODE` 或 DMA 模式进行任务态传输时,需要开启 SPI/DMA 中断。
- 会调用 FreeRTOS API 的中断优先级数值必须 `>= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`。

## 注意事项

- 同一条 SPI 总线上的多个从设备由 BSP mutex 串行访问。
- 片选 GPIO 必须由 CubeMX 配置为普通输出。
- `SPIRegister()` 失败属于初始化阶段严重错误。
