# bsp_iic

`bsp_iic` 是 I2C/IIC 总线型外设 BSP。它以“一个 IICInstance 对应一个 I2C 从设备”的方式封装总线互斥、阻塞/IT/DMA 传输、寄存器访问和 DMA 内存域适配。

## 注册

```c
IIC_Init_Config_s conf = {
    .handle = &hi2c2,
    .dev_address = 0x68,
    .work_mode = IIC_DMA_MODE,
    .callback = SensorRxDone,
    .id = sensor,
};

IICInstance *iic = IICRegister(&conf);
```

`dev_address` 使用 7 位地址,不需要提前左移。注册阶段只登记总线资源,RTOS mutex/semaphore 由 `BSPTaskInit()` 中的 `IICBusOsInit()` 统一创建。

## 工作模式

- `IIC_BLOCK_MODE`: HAL 阻塞接口。
- `IIC_IT_MODE`: HAL 中断接口,随后等待完成信号量。
- `IIC_DMA_MODE`: HAL DMA 接口,随后等待完成信号量。

IT/DMA 模式对上层仍表现为同步事务:函数返回 `HAL_OK` 时,本次 I2C 事务已经结束。

## 接口

```c
HAL_StatusTypeDef IICTransmit(IICInstance *iic, uint8_t *data, uint16_t size, IIC_Seq_Mode_e mode);
HAL_StatusTypeDef IICReceive(IICInstance *iic, uint8_t *data, uint16_t size, IIC_Seq_Mode_e mode);
HAL_StatusTypeDef IICAccessMem(IICInstance *iic, uint16_t mem_addr, uint8_t *data,
                               uint16_t size, IIC_Mem_Mode_e mode, uint8_t mem8bit_flag);
```

`IICAccessMem()` 用于访问从设备寄存器或内部内存。`mem8bit_flag` 为 1 时使用 8 位寄存器地址,否则使用 16 位寄存器地址。

## 序列传输

`IIC_SEQ_HOLDON` 用于连续多段 I2C 事务期间保持 BSP 总线 mutex,只支持 IT/DMA 模式。每个 HOLDON 序列必须以 `IIC_SEQ_RELEASE` 结束,否则同一条 I2C 总线会一直被当前实例占用。

阻塞模式不支持 HOLDON。

## DMA缓冲区

STM32H7 的 DMA1/DMA2 不能访问 DTCM。`bsp_iic` 为每条 I2C 总线准备位于 `.dma_buffer` / `RAM_D2` 的中转缓冲区。上层可以传入普通栈、全局或堆内存。

若开启 D-Cache,BSP 会在 DMA 发送前 clean TX 缓冲,在 DMA 接收前后 invalidate RX 缓冲。若 `.dma_buffer` 后续被 MPU 配置为 non-cacheable,可将 `IIC_USE_DMA_CACHE_MAINTENANCE` 设为 `0U`。

## 回调语义

HAL 的 I2C 完成、错误和 Abort 回调只释放等待信号量,不做日志、不做协议解析。实例 `callback` 由调用 IIC API 的任务上下文触发。

## CubeMX要求

- DMA 模式需要配置对应 I2C 的 TX/RX DMA。
- IT/DMA 模式需要开启 I2C event/error interrupt。
- 中断优先级数值必须满足 FreeRTOS 可调用 API 的约束。

## 注意事项

- I2C 是总线型外设,同一路 I2C 上所有从设备共享 mutex。
- 不要在中断上下文直接调用同步 IIC 接口。
- 注册失败属于初始化阶段严重错误。
