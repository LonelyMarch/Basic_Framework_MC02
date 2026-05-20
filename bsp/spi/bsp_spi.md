# bsp spi

## 功能说明

`bsp_spi` 用于统一管理 SPI 从设备实例。每个从设备通过 `SPIRegister()` 注册,注册时需要指定 SPI 硬件句柄、片选 GPIO、传输模式、接收完成回调和用户标识。

SPI 是总线型外设,同一条 SPI 硬件总线可以挂多个从设备。BSP 会负责拉低/拉高片选、按硬件 SPI 总线做互斥、等待中断/DMA 传输完成,并在失败时通过 `LOGERROR` 输出错误信息。

## 实例注册

使用 `SPIRegister()` 注册一个 SPI 从设备。注册成功返回 `SPIInstance *`,后续通过该实例访问对应从设备。

注册失败属于系统基础资源初始化失败。当前实现会先通过 `LOGERROR` 输出原因,然后停在死循环中,避免错误继续扩散到业务层。

注册失败的常见原因:

- 配置指针为空
- `spi_handle` 为空
- 片选 `GPIOx` 为空
- 注册实例数量超过 `SPI_MX_INSTANCE_CNT`
- 同一条 SPI 总线上的同一个片选引脚被重复注册
- `malloc()` 申请实例内存失败

`GPIOx` 和 `cs_pin` 使用 HAL 提供的 GPIO 宏,例如 `GPIOA`、`GPIO_PIN_4`。SPI 的 SCK/MISO/MOSI、片选 GPIO 输出模式、DMA 和中断等硬件配置仍由 CubeMX 生成代码完成。

## 容量配置

当前头文件中的容量宏为:

```c
#define SPI_BUS_CNT 3
#define MX_SPI_BUS_SLAVE_CNT 4
#define SPI_MX_INSTANCE_CNT (SPI_BUS_CNT * MX_SPI_BUS_SLAVE_CNT)
```

`SPI_BUS_CNT` 表示当前主控板最多按 3 条 SPI 硬件总线管理。

`MX_SPI_BUS_SLAVE_CNT` 表示每条 SPI 总线上预留的从设备数量。

`SPI_MX_INSTANCE_CNT` 是 BSP 层最多可注册的 SPI 从设备实例总数。

## 工作模式

`SPI_BLOCK_MODE` 使用 HAL 阻塞接口。该模式可以在 FreeRTOS 调度器启动前使用,适合初始化阶段、芯片 ID 检查、寄存器配置和短数据事务。

`SPI_IT_MODE` 使用 HAL 中断接口。BSP 在发起传输后等待完成信号量,因此对上层仍表现为同步接口:函数返回时本次 SPI 事务已经结束,片选也已经释放。

`SPI_DMA_MODE` 使用 HAL DMA 接口。BSP 同样会等待 DMA 完成信号量,函数返回时本次 SPI 事务已经结束。使用该模式时,CubeMX 必须为对应 SPI 配置 DMA 通道和相关中断。

传输模式在 `SPIRegister()` 的 `SPI_Init_Config_s.spi_work_mode` 中确定。注册完成后当前 BSP 不提供公共接口修改传输模式,避免运行期一个任务正在传输时另一个任务切换 `spi_work_mode`。

## FreeRTOS总线互斥

同一条 SPI 总线上的从设备共享 SCK/MISO/MOSI,同一时刻只能选中一个从设备。因此互斥锁按 `SPI_HandleTypeDef *handle` 共享,不是按单个 `SPIInstance` 创建。

FreeRTOS 调度器运行后,每次传输会按以下顺序执行:

1. 查找 SPI 硬件总线资源。
2. 获取该总线的 mutex。
3. 记录当前 `active_instance`。
4. 拉低片选。
5. 执行阻塞/中断/DMA 传输。
6. 拉高片选。
7. 清空 `active_instance` 并释放 mutex。

注册阶段只登记总线资源,不创建 RTOS 对象。mutex 和完成信号量会在调度器运行后第一次任务态通信时创建。当前懒创建逻辑适合本工程的初始化流程,但第一次通信如果被多个任务同时并发触发,理论上仍应避免。

同步 SPI 接口不允许在中断上下文调用。如果需要由 EXTI 或其他中断触发 SPI 读取,应在中断里通知任务,再由任务调用 `SPITransmit()`、`SPIRecv()` 或 `SPITransRecv()`。

## DMA缓冲区

STM32H7 的 DMA1/DMA2 不能访问 DTCM。为了避免上层把普通局部变量、普通 static 数组或 `malloc()` 得到的 DTCM 地址直接交给 DMA,`bsp_spi` 在 `.dma_buffer` 段中为每条 SPI 总线准备了内部 TX/RX 中转缓冲区。

当前工程的 `.dma_buffer` 段放在 `RAM_D2`,并按 32 字节对齐。SPI DMA 的数据路径如下:

- DMA 发送:先把上层 TX 数据复制到内部 TX 中转缓冲区,再启动 DMA。
- DMA 接收:DMA 先写入内部 RX 中转缓冲区,完成后 BSP 再复制回上层 RX 缓冲区。
- DMA 全双工收发:TX 和 RX 分别使用内部中转缓冲区。

单次 DMA 传输长度不能超过 `SPI_DMA_BOUNCE_BUFFER_SIZE`,当前为 `1024U`。如果后续用于屏幕、Flash 等大块 SPI DMA 传输,应设计更大的专用 DMA 缓冲区或分块传输策略。

## D-Cache一致性

Cortex-M7 打开 D-Cache 后,CPU 看到的数据和 DMA 直接访问 SRAM 的数据可能不一致。当前 BSP 默认启用 DMA cache 维护:

```c
#define SPI_USE_DMA_CACHE_MAINTENANCE 1U
```

运行时会检查 D-Cache 是否真的开启。若 CubeMX 中关闭 D-Cache,相关维护逻辑会自动跳过。

D-Cache 打开时,BSP 会执行以下处理:

- DMA 发送前 clean TX 缓冲区,确保 DMA 读到 CPU 刚写入的数据。
- DMA 接收前 invalidate RX 缓冲区,避免后续读到旧 cache。
- DMA 接收完成后再次 invalidate RX 缓冲区,确保 CPU 读到 DMA 写入的新数据。

如果后续通过 MPU 将 `.dma_buffer` 配置为 non-cacheable 区域,可以把 `SPI_USE_DMA_CACHE_MAINTENANCE` 改为 `0U`。

## 回调语义

HAL 的 `HAL_SPI_TxCpltCallback()`、`HAL_SPI_RxCpltCallback()`、`HAL_SPI_TxRxCpltCallback()`、`HAL_SPI_ErrorCallback()` 和 `HAL_SPI_AbortCpltCallback()` 只负责通知等待任务,不在中断里执行模块回调。

注册时传入的 `callback` 只会在 `SPIRecv()` 或 `SPITransRecv()` 成功后调用,并且运行在调用 SPI API 的任务上下文中,不是 HAL SPI 中断上下文。

`SPITransmit()` 成功后当前不会调用注册回调。

## CubeMX配置要求

使用 `SPI_BLOCK_MODE` 时,只需要 CubeMX 正确配置 SPI 外设和片选 GPIO。

使用 `SPI_IT_MODE` 时,需要打开对应 SPI global interrupt。

使用 `SPI_DMA_MODE` 时,需要为对应 SPI 配置匹配的 TX/RX DMA,并打开 SPI global interrupt 和 DMA stream interrupt。只发送的 SPI 至少需要 TX DMA,需要接收或全双工收发的 SPI 需要 RX DMA。

在 FreeRTOS 环境下,会调用 `osSemaphoreRelease()` 的 SPI/DMA 中断优先级必须满足 FreeRTOS 约束。当前工程 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` 为 `5`,因此 SPI 和 DMA 中断抢占优先级应设置为数值 `>= 5`。当前 `.ioc` 中 SPI1、SPI2 以及 DMA stream 的优先级为 `5`。

## API

```c
SPIInstance *SPIRegister(SPI_Init_Config_s *conf);
HAL_StatusTypeDef SPITransmit(SPIInstance *spi_ins, uint8_t *ptr_data, uint16_t len);
HAL_StatusTypeDef SPIRecv(SPIInstance *spi_ins, uint8_t *ptr_data, uint16_t len);
HAL_StatusTypeDef SPITransRecv(SPIInstance *spi_ins, uint8_t *ptr_data_rx, uint8_t *ptr_data_tx, uint16_t len);
```

上述传输接口返回 `HAL_StatusTypeDef`。`HAL_OK` 表示本次事务完成,其他值表示参数错误、HAL 错误、总线忙或等待超时。

## 使用示例

```c
static SPIInstance *imu_spi;

static void IMUSpiCallback(SPIInstance *spi)
{
    /* SPIRecv/SPITransRecv成功后,这里在任务上下文执行。 */
    (void)spi;
}

void IMUSpiInit(void)
{
    SPI_Init_Config_s spi_conf = {
        .spi_handle = &hspi2,
        .GPIOx = GPIOA,
        .cs_pin = GPIO_PIN_4,
        .spi_work_mode = SPI_DMA_MODE,
        .callback = IMUSpiCallback,
        .id = NULL,
    };

    imu_spi = SPIRegister(&spi_conf);
}

HAL_StatusTypeDef IMUReadReg(uint8_t reg, uint8_t *data)
{
    uint8_t tx[2] = {reg | 0x80U, 0x00U};
    uint8_t rx[2] = {0};
    HAL_StatusTypeDef status;

    status = SPITransRecv(imu_spi, rx, tx, sizeof(tx));
    if (status == HAL_OK)
    {
        *data = rx[1];
    }

    return status;
}
```

## 注意事项

- SPI 接口当前是同步语义。即使底层使用 IT/DMA,函数返回时本次事务也已经结束。
- 初始化阶段建议使用 `SPI_BLOCK_MODE`,不要在 FreeRTOS 调度器启动前使用 IT/DMA 模式。
- 不要在中断回调中直接调用同步 SPI 接口。
- DMA 模式单次长度超过 `SPI_DMA_BOUNCE_BUFFER_SIZE` 会返回错误。
- 上层应检查 `SPITransmit()`、`SPIRecv()`、`SPITransRecv()` 的返回值,不要在 SPI 失败后继续解析旧数据。
- `ptr_data_rx` 和 `ptr_data` 在函数返回前必须保持有效。当前实现返回时事务已完成,回调也已经执行完。

## @TODO

后续可以增加 SPI DMA 配置自检:当实例选择 `SPI_DMA_MODE` 但对应 `hspi->hdmatx` 或 `hspi->hdmarx` 为空时,通过 `LOGWARNING` 提示 CubeMX 配置不完整,并按需要退回到 IT 或阻塞模式。
