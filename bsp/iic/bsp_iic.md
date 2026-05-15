# bsp iic

## 功能说明

`bsp_iic` 用于统一管理 I2C 从设备实例。每个从设备通过 `IICRegister()` 注册,注册时需要指定 I2C 硬件句柄、7 位从设备地址、工作模式、接收完成回调和用户标识。

使用时填写 7 位设备地址,不需要左移。BSP 内部会将地址左移 1 位后传给 HAL。

## 工作模式

`IIC_BLOCK_MODE` 使用 HAL 阻塞接口,函数返回时本次传输已经结束。

`IIC_IT_MODE` 使用 HAL 中断接口。BSP 会在发起传输后等待完成、错误或 Abort 回调,因此对上层仍表现为“函数返回时事务已结束”。

`IIC_DMA_MODE` 使用 HAL DMA 接口。CubeMX 必须为对应 I2C 配置 TX/RX DMA 和相关中断。STM32H7 使用 DMA 时,收发缓冲区必须位于 DMA 可访问的 RAM 区域,不要直接使用 DTCM 中的局部数组、全局数组或 `malloc()` 缓冲区。

## 总线互斥

同一条 I2C 总线上可以挂多个从设备实例,因此互斥锁按 `I2C_HandleTypeDef *handle` 共享,不是按单个从设备实例创建。

FreeRTOS 调度器运行后,同一条 I2C 总线上的访问会被 mutex 串行化。注册阶段只登记总线资源,RTOS 对象会在任务态第一次通信时创建。

## 序列传输

`IIC_SEQ_RELEASE` 表示本次传输结束后释放总线,这是默认用法。

`IIC_SEQ_HOLDON` 表示本次传输结束后继续保持总线占用,直到同一任务、同一实例再次使用 `IIC_SEQ_RELEASE` 完成最后一帧。它用于需要 repeated start 或多帧连续事务的场景。

阻塞模式不支持 `IIC_SEQ_HOLDON`。

## 回调约束

接收完成和内存读取完成会调用注册时传入的 `callback`。

在 `IIC_BLOCK_MODE` 下,回调在调用者上下文执行。在 `IIC_IT_MODE` 和 `IIC_DMA_MODE` 下,回调由 HAL 完成回调触发,属于中断上下文,不要在回调中阻塞等待、申请释放动态内存、调用普通 mutex 或执行耗时逻辑。

## 接口

```c
IICInstance *IICRegister(IIC_Init_Config_s *conf);
void IICSetMode(IICInstance *iic, IIC_Work_Mode_e mode);
HAL_StatusTypeDef IICTransmit(IICInstance *iic, uint8_t *data, uint16_t size, IIC_Seq_Mode_e mode);
HAL_StatusTypeDef IICReceive(IICInstance *iic, uint8_t *data, uint16_t size, IIC_Seq_Mode_e mode);
HAL_StatusTypeDef IICAccessMem(IICInstance *iic, uint16_t mem_addr, uint8_t *data, uint16_t size, IIC_Mem_Mode_e mode, uint8_t mem8bit_flag);
```

上述传输接口返回 `HAL_StatusTypeDef`。`HAL_OK` 表示完成,其他值表示失败、忙或超时。
