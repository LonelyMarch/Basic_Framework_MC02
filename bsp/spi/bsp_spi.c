#include "bsp_spi.h"
#include "bsp_log.h"
#include "cmsis_os2.h"
#include "memory.h"
#include "stdlib.h"

// HAL单次阻塞SPI事务超时时间,单位ms。BMI088等小包通信不应长时间阻塞高优先级任务。
#define SPI_HAL_TIMEOUT_MS 5U
// FreeRTOS等待SPI总线互斥锁或异步完成信号的超时时间,单位tick。
#define SPI_OS_TIMEOUT_TICK 100U
// SPI DMA内部中转缓冲区大小。当前BMI088单次传输只有几个字节,这里预留到1KB以兼顾后续小型SPI设备。
#define SPI_DMA_BOUNCE_BUFFER_SIZE 1024U

#define SPI_DCACHE_LINE_SIZE 32U

#if defined(__GNUC__)
#define SPI_DMA_BUFFER_ATTR __attribute__((section(".dma_buffer"), aligned(32)))
#else
#define SPI_DMA_BUFFER_ATTR
#endif

/**
 * @brief SPI硬件总线资源
 *
 * 一个SPI硬件总线上可以挂多个从设备,片选线不同但SCK/MISO/MOSI共享。
 * 因此互斥锁必须按SPI硬件handle共享,不能按SPIInstance单独创建。
 * IT/DMA模式在BSP内部等待完成信号量,保证函数返回时本次SPI事务已经结束。
 * DMA模式统一使用RAM_D2中的内部中转缓冲区,避免上层把DTCM中的局部变量/static变量直接交给DMA。
 */
typedef struct
{
    SPI_HandleTypeDef *handle;        // 该资源对应的SPI硬件句柄
    osMutexId_t mutex;                // 同一条SPI总线的互斥锁
    osSemaphoreId_t complete_sem;     // IT/DMA传输完成信号量
    SPIInstance *active_instance;     // 当前正在占用该总线的SPI从设备实例
    HAL_StatusTypeDef last_status;    // 最近一次异步传输结果
    uint8_t *dma_tx_buffer;            // RAM_D2中的DMA发送中转缓冲区
    uint8_t *dma_rx_buffer;            // RAM_D2中的DMA接收中转缓冲区
} SPIBusResource;

/* 所有的spi instance保存于此,用于callback时判断中断来源*/
static SPIInstance *spi_instance[SPI_MX_INSTANCE_CNT] = {NULL};
static SPIBusResource spi_bus[SPI_BUS_CNT] = {0};
static uint8_t idx = 0;     // 已注册的SPI从设备实例数量
static uint8_t bus_idx = 0; // 已登记的SPI硬件总线数量

// DMA1/DMA2无法访问DTCM,因此SPI DMA的内部中转缓冲区必须显式放到RAM_D2。
static uint8_t spi_dma_tx_buffer[SPI_BUS_CNT][SPI_DMA_BOUNCE_BUFFER_SIZE] SPI_DMA_BUFFER_ATTR;
static uint8_t spi_dma_rx_buffer[SPI_BUS_CNT][SPI_DMA_BOUNCE_BUFFER_SIZE] SPI_DMA_BUFFER_ATTR;

/**
 * @brief SPI注册阶段错误处理
 *
 * 注册阶段属于系统基础资源初始化。为了保持原框架“初始化失败立即停机”的语义,
 * 这里先打印错误原因,再进入死循环,避免错误继续扩散到后续业务代码。
 */
static void SPIRegisterErrorHandler(const char *error_msg)
{
    LOGERROR("[bsp_spi] SPI注册失败: %s", error_msg);
    while (1)
        ;
}

/**
 * @brief 判断当前是否处于中断上下文
 */
static uint8_t SPIIsInISR(void)
{
    return (__get_IPSR() != 0U);
}

/**
 * @brief 根据SPI硬件句柄查找总线资源
 *
 * @param handle SPI硬件句柄
 * @return 找到则返回对应资源,否则返回NULL
 */
static SPIBusResource *SPIFindBusByHandle(SPI_HandleTypeDef *handle)
{
    for (uint8_t i = 0; i < bus_idx; i++)
    {
        if (spi_bus[i].handle == handle)
        {
            return &spi_bus[i];
        }
    }

    return NULL;
}

/**
 * @brief 根据SPI句柄和片选引脚查找已注册实例
 *
 * 同一个SPI从设备不应被重复注册,否则两个实例会控制同一根CS线,
 * 上层看起来像两个设备,底层实际会访问同一个从机。
 */
static SPIInstance *SPIFindInstanceByChipSelect(SPI_HandleTypeDef *handle, GPIO_TypeDef *GPIOx, uint16_t cs_pin)
{
    for (uint8_t i = 0; i < idx; i++)
    {
        if (spi_instance[i] == NULL)
        {
            continue;
        }

        if (spi_instance[i]->spi_handle == handle &&
            spi_instance[i]->GPIOx == GPIOx &&
            spi_instance[i]->cs_pin == cs_pin)
        {
            return spi_instance[i];
        }
    }

    return NULL;
}

/**
 * @brief 获取或登记一条SPI硬件总线
 *
 * 这里只登记handle,不立即创建mutex/semaphore。当前工程中RobotInit()早于
 * osKernelInitialize()执行,过早创建RTOS对象可能失败,因此RTOS对象在任务态第一次通信时懒创建。
 *
 * @param handle SPI硬件句柄
 * @return 对应的总线资源
 */
static SPIBusResource *SPIGetOrCreateBus(SPI_HandleTypeDef *handle)
{
    SPIBusResource *bus = SPIFindBusByHandle(handle);
    uint8_t new_bus_idx;
    if (bus != NULL)
    {
        return bus;
    }

    if (bus_idx >= SPI_BUS_CNT)
    {
        SPIRegisterErrorHandler("超过最大SPI总线数量");
    }

    new_bus_idx = bus_idx;
    bus = &spi_bus[new_bus_idx];
    memset(bus, 0, sizeof(SPIBusResource));
    bus->handle = handle;
    bus->last_status = HAL_OK;
    bus->dma_tx_buffer = spi_dma_tx_buffer[new_bus_idx];
    bus->dma_rx_buffer = spi_dma_rx_buffer[new_bus_idx];
    bus_idx++;
    return bus;
}

/**
 * @brief 检查本次DMA传输长度是否能放入内部中转缓冲区
 *
 * SPI DMA使用RAM_D2中的内部缓冲区做中转,这样上层可以继续使用普通局部变量或static变量。
 * 如果后续要做屏幕刷图等大块SPI DMA,应单独设计更大的DMA缓冲区或分块传输策略。
 */
static HAL_StatusTypeDef SPICheckDmaLength(uint16_t len)
{
    if (len > SPI_DMA_BOUNCE_BUFFER_SIZE)
    {
        LOGERROR("[bsp_spi] SPI DMA传输长度超过内部缓冲区: len=%u, limit=%u",
                 (unsigned int)len,
                 (unsigned int)SPI_DMA_BOUNCE_BUFFER_SIZE);
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief 将地址范围扩展到D-Cache line边界
 *
 * Cortex-M7的D-Cache按cache line维护,STM32H7为32字节一行。
 * CMSIS的按地址清理/失效接口要求传入的起始地址和长度覆盖完整cache line,
 * 因此这里统一做向下/向上对齐,避免只维护到半行数据。
 */
static void SPIAlignDCacheRange(uintptr_t address, uint32_t size, uintptr_t *aligned_address, int32_t *aligned_size)
{
    uintptr_t start = address & ~((uintptr_t)SPI_DCACHE_LINE_SIZE - 1U);
    uintptr_t end = (address + size + SPI_DCACHE_LINE_SIZE - 1U) & ~((uintptr_t)SPI_DCACHE_LINE_SIZE - 1U);

    *aligned_address = start;
    *aligned_size = (int32_t)(end - start);
}

/**
 * @brief DMA读取内存前清理D-Cache
 *
 * CPU刚写入TX缓冲区时,数据可能还停留在D-Cache中。启动DMA发送前需要Clean,
 * 把cache里的新数据写回SRAM,确保DMA读到的是最新发送内容。
 */
static void SPICleanDCacheByAddr(const void *buffer, uint16_t len)
{
#if SPI_USE_DMA_CACHE_MAINTENANCE
    uintptr_t aligned_address;
    int32_t aligned_size;

    if (buffer == NULL || len == 0U || (SCB->CCR & SCB_CCR_DC_Msk) == 0U)
    {
        return;
    }

    SPIAlignDCacheRange((uintptr_t)buffer, len, &aligned_address, &aligned_size);
    SCB_CleanDCache_by_Addr((uint32_t *)aligned_address, aligned_size);
#else
    (void)buffer;
    (void)len;
#endif
}

/**
 * @brief DMA写入内存后失效D-Cache
 *
 * DMA接收完成后,SRAM中已经是新数据,但CPU可能仍持有旧cache副本。
 * 读取RX缓冲区前需要Invalidate,强制CPU从SRAM重新取回DMA写入的数据。
 */
static void SPIInvalidateDCacheByAddr(const void *buffer, uint16_t len)
{
#if SPI_USE_DMA_CACHE_MAINTENANCE
    uintptr_t aligned_address;
    int32_t aligned_size;

    if (buffer == NULL || len == 0U || (SCB->CCR & SCB_CCR_DC_Msk) == 0U)
    {
        return;
    }

    SPIAlignDCacheRange((uintptr_t)buffer, len, &aligned_address, &aligned_size);
    SCB_InvalidateDCache_by_Addr((uint32_t *)aligned_address, aligned_size);
#else
    (void)buffer;
    (void)len;
#endif
}

/**
 * @brief 确保总线的RTOS对象已经创建
 *
 * 本函数只允许在FreeRTOS调度器运行后调用。mutex使用优先级继承,
 * 降低低优先级任务占用SPI时对高优先级任务造成的优先级反转影响。
 */
static HAL_StatusTypeDef SPIEnsureBusOsObjects(SPIBusResource *bus)
{
    const osMutexAttr_t mutex_attr = {
        .name = "bsp_spi_mutex",
        .attr_bits = osMutexPrioInherit,
    };

    if (bus == NULL)
    {
        return HAL_ERROR;
    }

    if (osKernelGetState() != osKernelRunning)
    {
        return HAL_ERROR;
    }

    if (bus->mutex == NULL)
    {
        bus->mutex = osMutexNew(&mutex_attr);
        if (bus->mutex == NULL)
        {
            LOGERROR("[bsp_spi] SPI总线资源创建失败: 互斥锁为空");
            return HAL_ERROR;
        }
    }

    if (bus->complete_sem == NULL)
    {
        bus->complete_sem = osSemaphoreNew(1, 0, NULL);
        if (bus->complete_sem == NULL)
        {
            LOGERROR("[bsp_spi] SPI总线资源创建失败: 完成信号量为空");
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

/**
 * @brief 清空异步完成信号量
 *
 * 发起新的IT/DMA事务前要清掉可能遗留的完成信号,避免立刻误判为本次传输完成。
 */
static void SPIClearCompleteSem(SPIBusResource *bus)
{
    while (osSemaphoreAcquire(bus->complete_sem, 0) == osOK)
    {
    }
}

/**
 * @brief 获取SPI总线使用权
 *
 * RTOS启动前没有多任务并发,只允许阻塞HAL路径继续执行。RTOS启动后,
 * 每次事务必须先拿到对应硬件总线的mutex,避免同一条SPI总线上多个片选同时有效。
 */
static HAL_StatusTypeDef SPILockBus(SPIInstance *spi_ins, SPIBusResource **bus_out, uint8_t *need_unlock)
{
    SPIBusResource *bus;

    if (spi_ins == NULL || spi_ins->spi_handle == NULL || bus_out == NULL || need_unlock == NULL)
    {
        LOGERROR("[bsp_spi] SPI总线加锁失败: 参数为空");
        return HAL_ERROR;
    }

    *bus_out = NULL;
    *need_unlock = 0;

    if (SPIIsInISR())
    {
        LOGERROR("[bsp_spi] SPI总线加锁失败: 不允许在中断上下文调用同步SPI接口");
        return HAL_ERROR;
    }

    bus = SPIFindBusByHandle(spi_ins->spi_handle);
    if (bus == NULL)
    {
        LOGERROR("[bsp_spi] SPI总线加锁失败: 未找到总线资源");
        return HAL_ERROR;
    }
    *bus_out = bus;

    if (osKernelGetState() != osKernelRunning)
    {
        return HAL_OK;
    }

    if (SPIEnsureBusOsObjects(bus) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (osMutexAcquire(bus->mutex, SPI_OS_TIMEOUT_TICK) != osOK)
    {
        LOGERROR("[bsp_spi] SPI总线加锁失败: 等待互斥锁超时");
        return HAL_TIMEOUT;
    }

    *need_unlock = 1;
    return HAL_OK;
}

/**
 * @brief 释放SPI总线使用权
 */
static void SPIUnlockBus(SPIBusResource *bus, uint8_t need_unlock)
{
    if (bus == NULL)
    {
        return;
    }

    bus->active_instance = NULL;

    if (need_unlock && osKernelGetState() == osKernelRunning)
    {
        (void)osMutexRelease(bus->mutex);
    }
}

/**
 * @brief 拉低片选,开始一次SPI事务
 */
static void SPISelectDevice(SPIInstance *spi_ins, SPIBusResource *bus)
{
    bus->active_instance = spi_ins;
    HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_RESET);
    spi_ins->CS_State = HAL_GPIO_ReadPin(spi_ins->GPIOx, spi_ins->cs_pin);
}

/**
 * @brief 拉高片选,结束一次SPI事务
 */
static void SPIDeselectDevice(SPIInstance *spi_ins)
{
    HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_SET);
    spi_ins->CS_State = HAL_GPIO_ReadPin(spi_ins->GPIOx, spi_ins->cs_pin);
}

/**
 * @brief 统一记录SPI事务失败原因
 *
 * HAL_StatusTypeDef只能表示OK/ERROR/BUSY/TIMEOUT这类粗粒度状态,
 * hspi->ErrorCode能进一步指出是否发生OVR、MODF、DMA等底层错误。
 */
static void SPILogTransferError(const char *op, SPIInstance *spi_ins, HAL_StatusTypeDef status)
{
    uint32_t error_code = 0U;

    if (spi_ins != NULL && spi_ins->spi_handle != NULL)
    {
        error_code = spi_ins->spi_handle->ErrorCode;
    }

    LOGERROR("[bsp_spi] SPI%s失败: status=%u, error=0x%lx",
             op,
             (unsigned int)status,
             (unsigned long)error_code);
}

/**
 * @brief 等待IT/DMA异步传输完成
 *
 * 正常完成、错误中断、Abort完成都会释放complete_sem。若等待超时,
 * 主动发起Abort,尽量让HAL状态机回到可再次使用的状态。
 */
static HAL_StatusTypeDef SPIWaitAsyncDone(SPIInstance *spi_ins, SPIBusResource *bus)
{
    if (spi_ins == NULL || spi_ins->spi_handle == NULL || bus == NULL)
    {
        return HAL_ERROR;
    }

    if (osKernelGetState() != osKernelRunning)
    {
        LOGERROR("[bsp_spi] IT/DMA模式需要在FreeRTOS调度器启动后使用");
        return HAL_ERROR;
    }

    if (osSemaphoreAcquire(bus->complete_sem, SPI_OS_TIMEOUT_TICK) != osOK)
    {
        bus->last_status = HAL_TIMEOUT;
        (void)HAL_SPI_Abort(spi_ins->spi_handle);
        SPILogTransferError("异步等待", spi_ins, HAL_TIMEOUT);
        return HAL_TIMEOUT;
    }

    if (bus->last_status != HAL_OK)
    {
        SPILogTransferError("异步传输", spi_ins, bus->last_status);
    }

    return bus->last_status;
}

/**
 * @brief 从HAL回调中通知等待任务
 *
 * HAL的SPI完成/错误回调运行在中断上下文,这里不做日志、不阻塞,
 * 只记录结果并释放信号量。真正的片选释放和用户回调在任务上下文完成。
 */
static void SPINotifyDone(SPI_HandleTypeDef *hspi, HAL_StatusTypeDef status)
{
    SPIBusResource *bus = SPIFindBusByHandle(hspi);
    if (bus == NULL || bus->complete_sem == NULL || bus->active_instance == NULL)
    {
        return;
    }

    bus->last_status = status;
    (void)osSemaphoreRelease(bus->complete_sem);
}

/**
 * @brief 注册一个spi instance
 */
SPIInstance *SPIRegister(SPI_Init_Config_s *conf)
{
    SPIInstance *instance;

    if (conf == NULL)
    {
        SPIRegisterErrorHandler("配置指针为空");
    }

    if (conf->spi_handle == NULL)
    {
        SPIRegisterErrorHandler("SPI句柄为空");
    }

    if (conf->GPIOx == NULL)
    {
        SPIRegisterErrorHandler("片选GPIO为空");
    }

    if (idx >= SPI_MX_INSTANCE_CNT) // 超过最大SPI从设备实例数
    {
        SPIRegisterErrorHandler("超过最大从设备实例数量");
    }

    if (SPIFindInstanceByChipSelect(conf->spi_handle, conf->GPIOx, conf->cs_pin) != NULL)
    {
        SPIRegisterErrorHandler("同一SPI片选引脚重复注册");
    }

    (void)SPIGetOrCreateBus(conf->spi_handle);

    instance = (SPIInstance *)malloc(sizeof(SPIInstance));
    if (instance == NULL)
    {
        SPIRegisterErrorHandler("实例内存申请失败");
    }

    memset(instance, 0, sizeof(SPIInstance));

    instance->spi_handle = conf->spi_handle;
    instance->GPIOx = conf->GPIOx;
    instance->cs_pin = conf->cs_pin;
    instance->spi_work_mode = conf->spi_work_mode;
    instance->callback = conf->callback;
    instance->id = conf->id;
    instance->CS_State = HAL_GPIO_ReadPin(instance->GPIOx, instance->cs_pin);

    spi_instance[idx++] = instance;
    return instance;
}

HAL_StatusTypeDef SPITransmit(SPIInstance *spi_ins, uint8_t *ptr_data, uint16_t len)
{
    HAL_StatusTypeDef status;
    SPIBusResource *bus;
    uint8_t need_unlock;

    if (spi_ins == NULL || ptr_data == NULL || len == 0U)
    {
        LOGERROR("[bsp_spi] SPI发送失败: 参数非法");
        return HAL_ERROR;
    }

    status = SPILockBus(spi_ins, &bus, &need_unlock);
    if (status != HAL_OK)
    {
        return status;
    }

    SPISelectDevice(spi_ins, bus);

    switch (spi_ins->spi_work_mode)
    {
    case SPI_BLOCK_MODE:
        status = HAL_SPI_Transmit(spi_ins->spi_handle, ptr_data, len, SPI_HAL_TIMEOUT_MS);
        break;
    case SPI_IT_MODE:
        if (osKernelGetState() != osKernelRunning)
        {
            LOGERROR("[bsp_spi] SPI发送失败: IT模式需要在FreeRTOS调度器启动后使用");
            status = HAL_ERROR;
            break;
        }
        SPIClearCompleteSem(bus);
        bus->last_status = HAL_BUSY;
        status = HAL_SPI_Transmit_IT(spi_ins->spi_handle, ptr_data, len);
        if (status == HAL_OK)
        {
            status = SPIWaitAsyncDone(spi_ins, bus);
        }
        break;
    case SPI_DMA_MODE:
        if (osKernelGetState() != osKernelRunning)
        {
            LOGERROR("[bsp_spi] SPI发送失败: DMA模式需要在FreeRTOS调度器启动后使用");
            status = HAL_ERROR;
            break;
        }
        status = SPICheckDmaLength(len);
        if (status != HAL_OK)
        {
            break;
        }
        // 上层buffer可能位于DTCM,先复制到RAM_D2中的内部DMA缓冲区再启动DMA。
        memcpy(bus->dma_tx_buffer, ptr_data, len);
        SPICleanDCacheByAddr(bus->dma_tx_buffer, len);
        SPIClearCompleteSem(bus);
        bus->last_status = HAL_BUSY;
        status = HAL_SPI_Transmit_DMA(spi_ins->spi_handle, bus->dma_tx_buffer, len);
        if (status == HAL_OK)
        {
            status = SPIWaitAsyncDone(spi_ins, bus);
        }
        break;
    default:
        LOGERROR("[bsp_spi] SPI发送失败: 工作模式非法");
        status = HAL_ERROR;
        break;
    }

    SPIDeselectDevice(spi_ins);
    SPIUnlockBus(bus, need_unlock);
    if (status != HAL_OK)
    {
        SPILogTransferError("发送", spi_ins, status);
    }
    return status;
}

HAL_StatusTypeDef SPIRecv(SPIInstance *spi_ins, uint8_t *ptr_data, uint16_t len)
{
    HAL_StatusTypeDef status;
    SPIBusResource *bus;
    uint8_t need_unlock;

    if (spi_ins == NULL || ptr_data == NULL || len == 0U)
    {
        LOGERROR("[bsp_spi] SPI接收失败: 参数非法");
        return HAL_ERROR;
    }

    spi_ins->rx_size = len;
    spi_ins->rx_buffer = ptr_data;

    status = SPILockBus(spi_ins, &bus, &need_unlock);
    if (status != HAL_OK)
    {
        return status;
    }

    SPISelectDevice(spi_ins, bus);

    switch (spi_ins->spi_work_mode)
    {
    case SPI_BLOCK_MODE:
        status = HAL_SPI_Receive(spi_ins->spi_handle, ptr_data, len, SPI_HAL_TIMEOUT_MS);
        break;
    case SPI_IT_MODE:
        if (osKernelGetState() != osKernelRunning)
        {
            LOGERROR("[bsp_spi] SPI接收失败: IT模式需要在FreeRTOS调度器启动后使用");
            status = HAL_ERROR;
            break;
        }
        SPIClearCompleteSem(bus);
        bus->last_status = HAL_BUSY;
        status = HAL_SPI_Receive_IT(spi_ins->spi_handle, ptr_data, len);
        if (status == HAL_OK)
        {
            status = SPIWaitAsyncDone(spi_ins, bus);
        }
        break;
    case SPI_DMA_MODE:
        if (osKernelGetState() != osKernelRunning)
        {
            LOGERROR("[bsp_spi] SPI接收失败: DMA模式需要在FreeRTOS调度器启动后使用");
            status = HAL_ERROR;
            break;
        }
        status = SPICheckDmaLength(len);
        if (status != HAL_OK)
        {
            break;
        }
        SPIClearCompleteSem(bus);
        bus->last_status = HAL_BUSY;
        // DMA先写入RAM_D2内部缓冲区,完成后再复制回上层buffer。
        SPIInvalidateDCacheByAddr(bus->dma_rx_buffer, len);
        status = HAL_SPI_Receive_DMA(spi_ins->spi_handle, bus->dma_rx_buffer, len);
        if (status == HAL_OK)
        {
            status = SPIWaitAsyncDone(spi_ins, bus);
        }
        break;
    default:
        LOGERROR("[bsp_spi] SPI接收失败: 工作模式非法");
        status = HAL_ERROR;
        break;
    }

    if (status == HAL_OK && spi_ins->spi_work_mode == SPI_DMA_MODE)
    {
        SPIInvalidateDCacheByAddr(bus->dma_rx_buffer, len);
        // 复制完成前继续持有总线锁,避免其他任务抢占同一条SPI总线并覆盖内部DMA缓冲区。
        memcpy(ptr_data, bus->dma_rx_buffer, len);
    }

    SPIDeselectDevice(spi_ins);
    SPIUnlockBus(bus, need_unlock);

    if (status != HAL_OK)
    {
        SPILogTransferError("接收", spi_ins, status);
    }

    if (status == HAL_OK && spi_ins->callback != NULL)
    {
        spi_ins->callback(spi_ins);
    }

    return status;
}

HAL_StatusTypeDef SPITransRecv(SPIInstance *spi_ins, uint8_t *ptr_data_rx, uint8_t *ptr_data_tx, uint16_t len)
{
    HAL_StatusTypeDef status;
    SPIBusResource *bus;
    uint8_t need_unlock;

    if (spi_ins == NULL || ptr_data_rx == NULL || ptr_data_tx == NULL || len == 0U)
    {
        LOGERROR("[bsp_spi] SPI收发失败: 参数非法");
        return HAL_ERROR;
    }

    // 用于稍后回调使用,请保证ptr_data_rx在回调函数被调用之前仍然在作用域内,否则析构之后的行为是未定义的!!!
    spi_ins->rx_size = len;
    spi_ins->rx_buffer = ptr_data_rx;

    status = SPILockBus(spi_ins, &bus, &need_unlock);
    if (status != HAL_OK)
    {
        return status;
    }

    SPISelectDevice(spi_ins, bus);

    switch (spi_ins->spi_work_mode)
    {
    case SPI_BLOCK_MODE:
        status = HAL_SPI_TransmitReceive(spi_ins->spi_handle, ptr_data_tx, ptr_data_rx, len, SPI_HAL_TIMEOUT_MS);
        break;
    case SPI_IT_MODE:
        if (osKernelGetState() != osKernelRunning)
        {
            LOGERROR("[bsp_spi] SPI收发失败: IT模式需要在FreeRTOS调度器启动后使用");
            status = HAL_ERROR;
            break;
        }
        SPIClearCompleteSem(bus);
        bus->last_status = HAL_BUSY;
        status = HAL_SPI_TransmitReceive_IT(spi_ins->spi_handle, ptr_data_tx, ptr_data_rx, len);
        if (status == HAL_OK)
        {
            status = SPIWaitAsyncDone(spi_ins, bus);
        }
        break;
    case SPI_DMA_MODE:
        if (osKernelGetState() != osKernelRunning)
        {
            LOGERROR("[bsp_spi] SPI收发失败: DMA模式需要在FreeRTOS调度器启动后使用");
            status = HAL_ERROR;
            break;
        }
        status = SPICheckDmaLength(len);
        if (status != HAL_OK)
        {
            break;
        }
        // 全双工DMA同时需要TX/RX两个RAM_D2缓冲区,避免DMA直接访问DTCM。
        memcpy(bus->dma_tx_buffer, ptr_data_tx, len);
        SPICleanDCacheByAddr(bus->dma_tx_buffer, len);
        SPIInvalidateDCacheByAddr(bus->dma_rx_buffer, len);
        SPIClearCompleteSem(bus);
        bus->last_status = HAL_BUSY;
        status = HAL_SPI_TransmitReceive_DMA(spi_ins->spi_handle, bus->dma_tx_buffer, bus->dma_rx_buffer, len);
        if (status == HAL_OK)
        {
            status = SPIWaitAsyncDone(spi_ins, bus);
        }
        break;
    default:
        LOGERROR("[bsp_spi] SPI收发失败: 工作模式非法");
        status = HAL_ERROR;
        break;
    }

    if (status == HAL_OK && spi_ins->spi_work_mode == SPI_DMA_MODE)
    {
        SPIInvalidateDCacheByAddr(bus->dma_rx_buffer, len);
        // 复制完成前继续持有总线锁,避免其他任务抢占同一条SPI总线并覆盖内部DMA缓冲区。
        memcpy(ptr_data_rx, bus->dma_rx_buffer, len);
    }

    SPIDeselectDevice(spi_ins);
    SPIUnlockBus(bus, need_unlock);

    if (status != HAL_OK)
    {
        SPILogTransferError("收发", spi_ins, status);
    }

    if (status == HAL_OK && spi_ins->callback != NULL)
    {
        spi_ins->callback(spi_ins);
    }

    return status;
}

/**
 * @brief HAL SPI发送完成回调
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    SPINotifyDone(hspi, HAL_OK);
}

/**
 * @brief HAL SPI接收完成回调
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    SPINotifyDone(hspi, HAL_OK);
}

/**
 * @brief HAL SPI收发完成回调
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    SPINotifyDone(hspi, HAL_OK);
}

/**
 * @brief HAL SPI错误回调
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    SPINotifyDone(hspi, HAL_ERROR);
}

/**
 * @brief HAL SPI Abort完成回调
 */
void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi)
{
    SPINotifyDone(hspi, HAL_TIMEOUT);
}
