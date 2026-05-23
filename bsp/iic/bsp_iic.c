#include "bsp_iic.h"
#include "bsp_log.h"
#include "cmsis_os2.h"
#include "memory.h"

// HAL阻塞接口的超时时间,单位ms。
#define IIC_HAL_TIMEOUT_MS 100U
// FreeRTOS等待I2C总线互斥锁或异步完成信号的超时时间,单位tick。
#define IIC_OS_TIMEOUT_TICK 100U
// I2C DMA内部中转缓冲区大小。I2C常用于小包传感器通信,这里预留256字节。
#define IIC_DMA_BOUNCE_BUFFER_SIZE 256U
// STM32H7 D-Cache line大小为32字节。
#define IIC_DCACHE_LINE_SIZE 32U

#if defined(__GNUC__)
#define IIC_DMA_BUFFER_ATTR __attribute__((section(".dma_buffer"), aligned(32)))
#else
#define IIC_DMA_BUFFER_ATTR
#endif

/**
 * @brief I2C硬件总线资源
 *
 * 一个I2C硬件外设上可以挂多个从设备实例,因此互斥锁必须按handle共享,
 * 不能按IICInstance单独创建。IT/DMA模式发起传输后会立即返回到HAL,
 * 本BSP使用complete_sem等待HAL完成/错误/Abort回调,保证函数返回时本次事务已经结束。
 */
typedef struct
{
    I2C_HandleTypeDef *handle;        // 该资源对应的I2C硬件句柄
    osMutexId_t mutex;                // 同一条I2C总线的互斥锁
    osSemaphoreId_t complete_sem;     // IT/DMA传输完成信号量
    HAL_StatusTypeDef last_status;    // 最近一次异步传输结果
    IICInstance *hold_instance;       // HOLDON序列传输期间持有总线的实例
    osThreadId_t hold_thread;         // HOLDON序列传输期间持有总线的任务
    uint8_t *dma_tx_buffer;            // RAM_D2中的DMA发送中转缓冲区
    uint8_t *dma_rx_buffer;            // RAM_D2中的DMA接收中转缓冲区
    volatile uint8_t os_objects_ready; // mutex/semaphore是否已经完整创建
} IICBusResource;

static uint8_t idx = 0;     // 已注册的I2C从设备实例数量
static uint8_t bus_idx = 0; // 已登记的I2C硬件总线数量
static IICInstance iic_instance_pool[MX_IIC_SLAVE_CNT]; // I2C实例静态池,控制结构体放默认.bss/DTCM
static IICInstance *iic_instance[MX_IIC_SLAVE_CNT] = {NULL};
static IICBusResource iic_bus[IIC_DEVICE_CNT] = {0};

/*
 * STM32H7的DMA1/DMA2不能访问DTCM。I2C DMA统一使用RAM_D2中的内部中转缓冲,
 * 上层可以继续传入普通局部变量、普通static变量或malloc得到的缓冲区。
 */
static uint8_t iic_dma_tx_buffer[IIC_DEVICE_CNT][IIC_DMA_BOUNCE_BUFFER_SIZE] IIC_DMA_BUFFER_ATTR;
static uint8_t iic_dma_rx_buffer[IIC_DEVICE_CNT][IIC_DMA_BOUNCE_BUFFER_SIZE] IIC_DMA_BUFFER_ATTR;

/**
 * @brief 将地址范围扩展到D-Cache line边界
 */
static void IICAlignDCacheRange(uintptr_t address, uint32_t size, uintptr_t *aligned_address, int32_t *aligned_size)
{
    uintptr_t start = address & ~((uintptr_t)IIC_DCACHE_LINE_SIZE - 1U);
    uintptr_t end = (address + size + IIC_DCACHE_LINE_SIZE - 1U) & ~((uintptr_t)IIC_DCACHE_LINE_SIZE - 1U);

    *aligned_address = start;
    *aligned_size = (int32_t)(end - start);
}

/**
 * @brief DMA读取内存前清理D-Cache
 */
static void IICCleanDCacheByAddr(const void *buffer, uint16_t len)
{
#if IIC_USE_DMA_CACHE_MAINTENANCE
    uintptr_t aligned_address;
    int32_t aligned_size;

    if (buffer == NULL || len == 0U || (SCB->CCR & SCB_CCR_DC_Msk) == 0U)
    {
        return;
    }

    IICAlignDCacheRange((uintptr_t)buffer, len, &aligned_address, &aligned_size);
    SCB_CleanDCache_by_Addr((uint32_t *)aligned_address, aligned_size);
#else
    (void)buffer;
    (void)len;
#endif
}

/**
 * @brief DMA写入内存前后失效D-Cache
 */
static void IICInvalidateDCacheByAddr(const void *buffer, uint16_t len)
{
#if IIC_USE_DMA_CACHE_MAINTENANCE
    uintptr_t aligned_address;
    int32_t aligned_size;

    if (buffer == NULL || len == 0U || (SCB->CCR & SCB_CCR_DC_Msk) == 0U)
    {
        return;
    }

    IICAlignDCacheRange((uintptr_t)buffer, len, &aligned_address, &aligned_size);
    SCB_InvalidateDCache_by_Addr((uint32_t *)aligned_address, aligned_size);
#else
    (void)buffer;
    (void)len;
#endif
}

/**
 * @brief 检查本次DMA传输长度是否能放入内部中转缓冲区
 */
static HAL_StatusTypeDef IICCheckDmaLength(uint16_t size)
{
    if (size > IIC_DMA_BOUNCE_BUFFER_SIZE)
    {
        LOGERROR("[bsp_iic] IIC DMA传输长度超过内部缓冲区: len=%u, limit=%u",
                 (unsigned int)size,
                 (unsigned int)IIC_DMA_BOUNCE_BUFFER_SIZE);
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief 检查CubeMX是否给当前I2C句柄配置了所需DMA通道
 */
static HAL_StatusTypeDef IICCheckDmaReady(IICInstance *iic, uint8_t need_tx, uint8_t need_rx)
{
    if (iic == NULL || iic->handle == NULL)
    {
        return HAL_ERROR;
    }

    if (need_tx != 0U && iic->handle->hdmatx == NULL)
    {
        LOGERROR("[bsp_iic] IIC DMA发送失败: hdmatx为空,请在CubeMX中配置I2C TX DMA");
        return HAL_ERROR;
    }

    if (need_rx != 0U && iic->handle->hdmarx == NULL)
    {
        LOGERROR("[bsp_iic] IIC DMA接收失败: hdmarx为空,请在CubeMX中配置I2C RX DMA");
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief I2C注册阶段错误处理
 *
 * 注册阶段属于系统基础资源初始化。为了保持原框架“初始化失败立即停机”的语义,
 * 这里先打印错误原因,再进入Error_Handler(),避免错误继续扩散到后续业务代码。
 */
__attribute__((noreturn)) static void IICRegisterErrorHandler(const char *error_msg)
{
    LOGERROR("[bsp_iic] IIC注册失败: %s", error_msg);
    Error_Handler();
    __builtin_unreachable();
}

/**
 * @brief 根据I2C硬件句柄查找总线资源
 *
 * @param handle I2C硬件句柄
 * @return 找到则返回对应资源,否则返回NULL
 */
static IICBusResource *IICFindBusByHandle(I2C_HandleTypeDef *handle)
{
    for (uint8_t i = 0; i < bus_idx; i++)
    {
        if (iic_bus[i].handle == handle)
        {
            return &iic_bus[i];
        }
    }

    return NULL;
}

/**
 * @brief 获取或登记一条I2C硬件总线
 *
 * 这里只登记handle,不立即创建mutex/semaphore。RTOS对象由BSPTaskInit()在
 * osKernelInitialize()之后统一创建；若运行期动态注册新总线,首次访问时会兜底创建。
 *
 * @param handle I2C硬件句柄
 * @return 对应的总线资源
 */
static IICBusResource *IICGetOrCreateBus(I2C_HandleTypeDef *handle)
{
    IICBusResource *bus = IICFindBusByHandle(handle);
    uint8_t new_bus_idx;

    if (bus != NULL)
    {
        return bus;
    }

    if (bus_idx >= IIC_DEVICE_CNT)
    {
        IICRegisterErrorHandler("超过最大I2C总线数量");
    }

    new_bus_idx = bus_idx;
    bus = &iic_bus[new_bus_idx];
    memset(bus, 0, sizeof(IICBusResource));
    bus->handle = handle;
    bus->last_status = HAL_OK;
    bus->dma_tx_buffer = iic_dma_tx_buffer[new_bus_idx];
    bus->dma_rx_buffer = iic_dma_rx_buffer[new_bus_idx];
    memset(bus->dma_tx_buffer, 0, IIC_DMA_BOUNCE_BUFFER_SIZE);
    memset(bus->dma_rx_buffer, 0, IIC_DMA_BOUNCE_BUFFER_SIZE);
    bus_idx++;
    return bus;
}

/**
 * @brief 确保总线的RTOS对象已经创建
 *
 * 当前主要由IICBusOsInit()在osKernelInitialize()之后、任务启动前统一创建。
 * 若后续确实在任务运行期动态注册I2C总线,本函数也会在第一次访问时兜底创建。
 */
static HAL_StatusTypeDef IICEnsureBusOsObjects(IICBusResource *bus)
{
    const osMutexAttr_t mutex_attr = {
        .name = "bsp_iic_mutex",
        .attr_bits = osMutexRecursive | osMutexPrioInherit,
    };
    osKernelState_t kernel_state;
    int32_t kernel_lock = -1;
    HAL_StatusTypeDef status = HAL_OK;

    if (bus == NULL)
    {
        return HAL_ERROR;
    }

    if (bus->os_objects_ready != 0U)
    {
        return HAL_OK;
    }

    kernel_state = osKernelGetState();
    if (kernel_state == osKernelInactive || kernel_state == osKernelError)
    {
        LOGERROR("[bsp_iic] IIC总线资源创建失败: RTOS内核尚未初始化");
        return HAL_ERROR;
    }

    /*
     * 正常路径是在调度器启动前统一创建,没有并发。若运行期动态注册总线,
     * 这里短暂锁住内核调度,避免两个任务同时为同一总线创建RTOS对象。
     */
    if (kernel_state == osKernelRunning)
    {
        kernel_lock = osKernelLock();
        if (kernel_lock < 0)
        {
            LOGERROR("[bsp_iic] IIC总线资源创建失败: 内核锁定失败");
            return HAL_ERROR;
        }

        if (bus->os_objects_ready != 0U)
        {
            (void)osKernelRestoreLock(kernel_lock);
            return HAL_OK;
        }
    }

    do
    {
        if (bus->complete_sem == NULL)
        {
            bus->complete_sem = osSemaphoreNew(1, 0, NULL);
            if (bus->complete_sem == NULL)
            {
                LOGERROR("[bsp_iic] IIC总线资源创建失败: 完成信号量为空");
                status = HAL_ERROR;
                break;
            }
        }

        if (bus->mutex == NULL)
        {
            bus->mutex = osMutexNew(&mutex_attr);
            if (bus->mutex == NULL)
            {
                LOGERROR("[bsp_iic] IIC总线资源创建失败: 互斥锁为空");
                status = HAL_ERROR;
                break;
            }
        }

        bus->os_objects_ready = 1U;
    } while (0);

    if (kernel_lock >= 0)
    {
        (void)osKernelRestoreLock(kernel_lock);
    }

    return status;
}

HAL_StatusTypeDef IICBusOsInit(void)
{
    for (uint8_t i = 0; i < bus_idx; i++)
    {
        if (IICEnsureBusOsObjects(&iic_bus[i]) != HAL_OK)
        {
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
static void IICClearCompleteSem(IICBusResource *bus)
{
    if (bus == NULL || bus->complete_sem == NULL)
    {
        return;
    }

    while (osSemaphoreAcquire(bus->complete_sem, 0) == osOK)
    {
    }
}

/**
 * @brief 获取I2C总线使用权
 *
 * RELEASE模式会在本次事务结束后释放总线。HOLDON模式会把总线保持给当前任务和实例,
 * 直到同一任务、同一实例再次以IIC_SEQ_RELEASE完成最后一帧,从而避免重复起始条件
 * 或多帧序列传输中被其他I2C从设备插队。
 *
 * @param iic I2C从设备实例
 * @param seq_mode 序列传输模式
 * @param need_unlock 输出参数,表示本次事务结束后是否需要释放mutex
 */
static HAL_StatusTypeDef IICLockBus(IICInstance *iic, IIC_Seq_Mode_e seq_mode, uint8_t *need_unlock)
{
    IICBusResource *bus;
    osThreadId_t current_thread;

    if (iic == NULL || iic->handle == NULL || need_unlock == NULL)
    {
        LOGERROR("[bsp_iic] IIC总线加锁失败: 参数为空");
        return HAL_ERROR;
    }

    if (seq_mode != IIC_SEQ_RELEASE && seq_mode != IIC_SEQ_HOLDON)
    {
        LOGERROR("[bsp_iic] IIC总线加锁失败: 序列传输模式非法");
        return HAL_ERROR;
    }

    *need_unlock = 0;
    bus = IICFindBusByHandle(iic->handle);
    if (bus == NULL)
    {
        LOGERROR("[bsp_iic] IIC总线加锁失败: 未找到总线资源");
        return HAL_ERROR;
    }

    // RTOS启动前没有多任务并发,只允许阻塞HAL路径继续执行。
    if (osKernelGetState() != osKernelRunning)
    {
        return HAL_OK;
    }

    if (IICEnsureBusOsObjects(bus) != HAL_OK)
    {
        return HAL_ERROR;
    }

    current_thread = osThreadGetId();

    // HOLDON会在一次序列传输中持续占有总线,直到同一实例使用RELEASE结束。
    if (bus->hold_instance == iic && bus->hold_thread == current_thread)
    {
        *need_unlock = (seq_mode == IIC_SEQ_RELEASE);
        return HAL_OK;
    }

    if (osMutexAcquire(bus->mutex, IIC_OS_TIMEOUT_TICK) != osOK)
    {
        LOGERROR("[bsp_iic] IIC总线加锁失败: 等待互斥锁超时");
        return HAL_TIMEOUT;
    }

    if (seq_mode == IIC_SEQ_HOLDON)
    {
        bus->hold_instance = iic;
        bus->hold_thread = current_thread;
        *need_unlock = 0;
    }
    else
    {
        *need_unlock = 1;
    }

    return HAL_OK;
}

/**
 * @brief 根据事务结果释放I2C总线
 *
 * 如果HOLDON序列中间出错,必须强制释放总线,否则后续任务会一直等不到mutex。
 */
static void IICUnlockBus(IICInstance *iic, HAL_StatusTypeDef status, uint8_t need_unlock)
{
    IICBusResource *bus;

    if (iic == NULL || iic->handle == NULL || osKernelGetState() != osKernelRunning)
    {
        return;
    }

    bus = IICFindBusByHandle(iic->handle);
    if (bus == NULL)
    {
        return;
    }

    if (status != HAL_OK && bus->hold_instance == iic)
    {
        bus->hold_instance = NULL;
        bus->hold_thread = NULL;
        need_unlock = 1;
    }
    else if (need_unlock)
    {
        bus->hold_instance = NULL;
        bus->hold_thread = NULL;
    }

    if (need_unlock)
    {
        (void)osMutexRelease(bus->mutex);
    }
}

/**
 * @brief 等待IT/DMA异步传输完成
 *
 * 正常完成、错误中断、Abort完成都会释放complete_sem。若等待超时,
 * 主动发起Abort,尽量让HAL状态机回到可再次使用的状态。
 */
static HAL_StatusTypeDef IICWaitAsyncDone(IICInstance *iic)
{
    IICBusResource *bus;

    if (iic == NULL || iic->handle == NULL)
    {
        return HAL_ERROR;
    }

    if (osKernelGetState() != osKernelRunning)
    {
        LOGERROR("[bsp_iic] IT/DMA模式需要在FreeRTOS调度器启动后使用");
        return HAL_ERROR;
    }

    bus = IICFindBusByHandle(iic->handle);
    if (bus == NULL)
    {
        return HAL_ERROR;
    }

    if (osSemaphoreAcquire(bus->complete_sem, IIC_OS_TIMEOUT_TICK) != osOK)
    {
        bus->last_status = HAL_TIMEOUT;
        (void)HAL_I2C_Master_Abort_IT(iic->handle, iic->dev_address);
        (void)osSemaphoreAcquire(bus->complete_sem, IIC_OS_TIMEOUT_TICK);
        return HAL_TIMEOUT;
    }

    return bus->last_status;
}

/**
 * @brief 从HAL回调中通知等待任务
 *
 * HAL的I2C完成/错误回调运行在中断上下文,这里不做日志、不阻塞,
 * 只记录结果并释放信号量。
 */
static void IICNotifyDone(I2C_HandleTypeDef *hi2c, HAL_StatusTypeDef status)
{
    IICBusResource *bus = IICFindBusByHandle(hi2c);
    if (bus == NULL || bus->complete_sem == NULL)
    {
        return;
    }

    bus->last_status = status;
    (void)osSemaphoreRelease(bus->complete_sem);
}

/**
 * @brief 注册一个I2C从设备实例
 */
IICInstance *IICRegister(IIC_Init_Config_s *conf)
{
    IICInstance *instance;

    if (conf == NULL)
    {
        IICRegisterErrorHandler("配置指针为空");
    }

    if (conf->handle == NULL)
    {
        IICRegisterErrorHandler("I2C句柄为空");
    }

    if (idx >= MX_IIC_SLAVE_CNT) // 超过最大实例数
    {
        IICRegisterErrorHandler("超过最大从机实例数量");
    }

    (void)IICGetOrCreateBus(conf->handle);

    instance = &iic_instance_pool[idx];
    memset(instance, 0, sizeof(IICInstance));

    instance->dev_address = conf->dev_address << 1; // 地址左移一位,最低位为读写位
    instance->callback = conf->callback;
    instance->work_mode = conf->work_mode;
    instance->handle = conf->handle;
    instance->id = conf->id;

    iic_instance[idx++] = instance;
    return instance;
}

/**
 * @brief 发送I2C数据
 *
 * BLOCK模式会在HAL阻塞发送完成后返回。IT/DMA模式会发起异步传输,
 * 然后等待HAL完成/错误回调,因此本函数返回时本次事务已经结束。
 */
HAL_StatusTypeDef IICTransmit(IICInstance *iic, uint8_t *data, uint16_t size, IIC_Seq_Mode_e seq_mode)
{
    HAL_StatusTypeDef status;
    uint8_t need_unlock = 0;
    IICBusResource *bus;

    if (iic == NULL || data == NULL || size == 0)
    {
        LOGERROR("[bsp_iic] IIC发送失败: 参数非法");
        return HAL_ERROR;
    }

    if (iic->work_mode == IIC_BLOCK_MODE && seq_mode != IIC_SEQ_RELEASE)
    {
        LOGERROR("[bsp_iic] IIC发送失败: 阻塞模式不支持HOLDON");
        return HAL_ERROR;
    }

    if (iic->work_mode != IIC_BLOCK_MODE && osKernelGetState() != osKernelRunning)
    {
        LOGERROR("[bsp_iic] IIC发送失败: IT/DMA模式需要在FreeRTOS调度器启动后使用");
        return HAL_ERROR;
    }

    status = IICLockBus(iic, seq_mode, &need_unlock);
    if (status != HAL_OK)
    {
        return status;
    }

    bus = IICFindBusByHandle(iic->handle);

    switch (iic->work_mode)
    {
    case IIC_BLOCK_MODE:
        status = HAL_I2C_Master_Transmit(iic->handle, iic->dev_address, data, size, IIC_HAL_TIMEOUT_MS);
        break;
    case IIC_IT_MODE:
        if (bus == NULL)
        {
            status = HAL_ERROR;
            break;
        }
        IICClearCompleteSem(bus);
        bus->last_status = HAL_BUSY;
        status = HAL_I2C_Master_Seq_Transmit_IT(iic->handle, iic->dev_address, data, size,
                                                seq_mode == IIC_SEQ_RELEASE ? I2C_OTHER_AND_LAST_FRAME : I2C_OTHER_FRAME);
        if (status == HAL_OK)
        {
            status = IICWaitAsyncDone(iic);
        }
        break;
    case IIC_DMA_MODE:
        if (bus == NULL)
        {
            status = HAL_ERROR;
            break;
        }
        if (IICCheckDmaReady(iic, 1U, 0U) != HAL_OK || IICCheckDmaLength(size) != HAL_OK)
        {
            status = HAL_ERROR;
            break;
        }

        /*
         * 上层发送buffer可能位于DTCM,而DMA1/DMA2不能访问DTCM。
         * 因此先复制到RAM_D2中的内部中转缓冲区,再启动I2C DMA发送。
         */
        memcpy(bus->dma_tx_buffer, data, size);
        IICCleanDCacheByAddr(bus->dma_tx_buffer, size);
        IICClearCompleteSem(bus);
        bus->last_status = HAL_BUSY;
        status = HAL_I2C_Master_Seq_Transmit_DMA(iic->handle, iic->dev_address, bus->dma_tx_buffer, size,
                                                 seq_mode == IIC_SEQ_RELEASE ? I2C_OTHER_AND_LAST_FRAME : I2C_OTHER_FRAME);
        if (status == HAL_OK)
        {
            status = IICWaitAsyncDone(iic);
        }
        break;
    default:
        LOGERROR("[bsp_iic] IIC发送失败: 工作模式非法");
        status = HAL_ERROR;
        break;
    }

    IICUnlockBus(iic, status, need_unlock);
    return status;
}

/**
 * @brief 接收I2C数据
 *
 * 接收成功后会调用注册时传入的callback。BLOCK/IT/DMA模式均在调用者任务上下文
 * 执行callback,HAL完成中断只负责释放完成信号量。
 */
HAL_StatusTypeDef IICReceive(IICInstance *iic, uint8_t *data, uint16_t size, IIC_Seq_Mode_e seq_mode)
{
    HAL_StatusTypeDef status;
    uint8_t need_unlock = 0;
    IICBusResource *bus;

    if (iic == NULL || data == NULL || size == 0)
    {
        LOGERROR("[bsp_iic] IIC接收失败: 参数非法");
        return HAL_ERROR;
    }

    if (iic->work_mode == IIC_BLOCK_MODE && seq_mode != IIC_SEQ_RELEASE)
    {
        LOGERROR("[bsp_iic] IIC接收失败: 阻塞模式不支持HOLDON");
        return HAL_ERROR;
    }

    if (iic->work_mode != IIC_BLOCK_MODE && osKernelGetState() != osKernelRunning)
    {
        LOGERROR("[bsp_iic] IIC接收失败: IT/DMA模式需要在FreeRTOS调度器启动后使用");
        return HAL_ERROR;
    }

    iic->rx_buffer = data;
    iic->rx_len = size;

    status = IICLockBus(iic, seq_mode, &need_unlock);
    if (status != HAL_OK)
    {
        return status;
    }

    bus = IICFindBusByHandle(iic->handle);

    switch (iic->work_mode)
    {
    case IIC_BLOCK_MODE:
        status = HAL_I2C_Master_Receive(iic->handle, iic->dev_address, data, size, IIC_HAL_TIMEOUT_MS);
        break;
    case IIC_IT_MODE:
        if (bus == NULL)
        {
            status = HAL_ERROR;
            break;
        }
        IICClearCompleteSem(bus);
        bus->last_status = HAL_BUSY;
        status = HAL_I2C_Master_Seq_Receive_IT(iic->handle, iic->dev_address, data, size,
                                               seq_mode == IIC_SEQ_RELEASE ? I2C_OTHER_AND_LAST_FRAME : I2C_OTHER_FRAME);
        if (status == HAL_OK)
        {
            status = IICWaitAsyncDone(iic);
        }
        break;
    case IIC_DMA_MODE:
        if (bus == NULL)
        {
            status = HAL_ERROR;
            break;
        }
        if (IICCheckDmaReady(iic, 0U, 1U) != HAL_OK || IICCheckDmaLength(size) != HAL_OK)
        {
            status = HAL_ERROR;
            break;
        }

        /*
         * DMA先写入RAM_D2内部缓冲区,完成后再复制回上层buffer。
         * 启动接收前先失效cache,避免旧dirty cache line后续写回覆盖DMA新数据。
         */
        IICInvalidateDCacheByAddr(bus->dma_rx_buffer, size);
        IICClearCompleteSem(bus);
        bus->last_status = HAL_BUSY;
        status = HAL_I2C_Master_Seq_Receive_DMA(iic->handle, iic->dev_address, bus->dma_rx_buffer, size,
                                                seq_mode == IIC_SEQ_RELEASE ? I2C_OTHER_AND_LAST_FRAME : I2C_OTHER_FRAME);
        if (status == HAL_OK)
        {
            status = IICWaitAsyncDone(iic);
            if (status == HAL_OK)
            {
                IICInvalidateDCacheByAddr(bus->dma_rx_buffer, size);
                memcpy(data, bus->dma_rx_buffer, size);
            }
        }
        break;
    default:
        LOGERROR("[bsp_iic] IIC接收失败: 工作模式非法");
        status = HAL_ERROR;
        break;
    }

    IICUnlockBus(iic, status, need_unlock);
    if (status == HAL_OK && iic->callback != NULL)
    {
        iic->callback(iic);
    }
    return status;
}

/**
 * @brief 访问I2C从设备内部寄存器/内存
 *
 * 根据实例work_mode选择阻塞、IT或DMA形式的HAL Mem接口。该接口本身是一次完整事务,
 * 因此固定以IIC_SEQ_RELEASE方式获取和释放总线。
 */
HAL_StatusTypeDef IICAccessMem(IICInstance *iic, uint16_t mem_addr, uint8_t *data, uint16_t size, IIC_Mem_Mode_e mem_mode, uint8_t mem8bit_flag)
{
    HAL_StatusTypeDef status;
    uint8_t need_unlock = 0;
    IICBusResource *bus;
    uint16_t bit_flag = mem8bit_flag ? I2C_MEMADD_SIZE_8BIT : I2C_MEMADD_SIZE_16BIT;
    uint8_t should_callback = 0;

    if (iic == NULL || data == NULL || size == 0)
    {
        LOGERROR("[bsp_iic] IIC内存访问失败: 参数非法");
        return HAL_ERROR;
    }

    if (mem_mode != IIC_WRITE_MEM && mem_mode != IIC_READ_MEM)
    {
        LOGERROR("[bsp_iic] IIC内存访问失败: 访问模式非法");
        return HAL_ERROR;
    }

    if (iic->work_mode != IIC_BLOCK_MODE && osKernelGetState() != osKernelRunning)
    {
        LOGERROR("[bsp_iic] IIC内存访问失败: IT/DMA模式需要在FreeRTOS调度器启动后使用");
        return HAL_ERROR;
    }

    status = IICLockBus(iic, IIC_SEQ_RELEASE, &need_unlock);
    if (status != HAL_OK)
    {
        return status;
    }

    bus = IICFindBusByHandle(iic->handle);

    switch (iic->work_mode)
    {
    case IIC_BLOCK_MODE:
        if (mem_mode == IIC_WRITE_MEM)
        {
            status = HAL_I2C_Mem_Write(iic->handle, iic->dev_address, mem_addr, bit_flag, data, size, IIC_HAL_TIMEOUT_MS);
        }
        else
        {
            status = HAL_I2C_Mem_Read(iic->handle, iic->dev_address, mem_addr, bit_flag, data, size, IIC_HAL_TIMEOUT_MS);
        }
        break;
    case IIC_IT_MODE:
        if (bus == NULL)
        {
            status = HAL_ERROR;
            break;
        }
        IICClearCompleteSem(bus);
        bus->last_status = HAL_BUSY;
        if (mem_mode == IIC_WRITE_MEM)
        {
            status = HAL_I2C_Mem_Write_IT(iic->handle, iic->dev_address, mem_addr, bit_flag, data, size);
        }
        else
        {
            status = HAL_I2C_Mem_Read_IT(iic->handle, iic->dev_address, mem_addr, bit_flag, data, size);
        }
        if (status == HAL_OK)
        {
            status = IICWaitAsyncDone(iic);
        }
        break;
    case IIC_DMA_MODE:
        if (bus == NULL)
        {
            status = HAL_ERROR;
            break;
        }
        if (IICCheckDmaLength(size) != HAL_OK)
        {
            status = HAL_ERROR;
            break;
        }

        if (mem_mode == IIC_WRITE_MEM)
        {
            if (IICCheckDmaReady(iic, 1U, 0U) != HAL_OK)
            {
                status = HAL_ERROR;
                break;
            }

            // 内存写DMA同样先复制到RAM_D2中转缓冲区,避免DMA直接读取DTCM。
            memcpy(bus->dma_tx_buffer, data, size);
            IICCleanDCacheByAddr(bus->dma_tx_buffer, size);
            IICClearCompleteSem(bus);
            bus->last_status = HAL_BUSY;
            status = HAL_I2C_Mem_Write_DMA(iic->handle, iic->dev_address, mem_addr, bit_flag, bus->dma_tx_buffer, size);
        }
        else
        {
            if (IICCheckDmaReady(iic, 0U, 1U) != HAL_OK)
            {
                status = HAL_ERROR;
                break;
            }

            // 内存读DMA先写入RAM_D2中转缓冲区,完成后再复制回上层buffer。
            IICInvalidateDCacheByAddr(bus->dma_rx_buffer, size);
            IICClearCompleteSem(bus);
            bus->last_status = HAL_BUSY;
            status = HAL_I2C_Mem_Read_DMA(iic->handle, iic->dev_address, mem_addr, bit_flag, bus->dma_rx_buffer, size);
        }
        if (status == HAL_OK)
        {
            status = IICWaitAsyncDone(iic);
            if (status == HAL_OK && mem_mode == IIC_READ_MEM)
            {
                IICInvalidateDCacheByAddr(bus->dma_rx_buffer, size);
                memcpy(data, bus->dma_rx_buffer, size);
            }
        }
        break;
    default:
        LOGERROR("[bsp_iic] IIC内存访问失败: 工作模式非法");
        status = HAL_ERROR;
        break;
    }

    IICUnlockBus(iic, status, need_unlock);
    should_callback = (mem_mode == IIC_READ_MEM && status == HAL_OK && iic->callback != NULL) ? 1U : 0U;
    if (should_callback != 0U)
    {
        iic->callback(iic);
    }
    return status;
}

/**
 * @brief HAL I2C发送完成回调
 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    IICNotifyDone(hi2c, HAL_OK);
}

/**
 * @brief HAL I2C接收完成回调
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    IICNotifyDone(hi2c, HAL_OK);
}

/**
 * @brief HAL I2C内存写完成回调
 */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    IICNotifyDone(hi2c, HAL_OK);
}

/**
 * @brief HAL I2C内存读完成回调
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    IICNotifyDone(hi2c, HAL_OK);
}

/**
 * @brief HAL I2C错误回调
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    IICNotifyDone(hi2c, HAL_ERROR);
}

/**
 * @brief HAL I2C Abort完成回调
 */
void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
{
    IICNotifyDone(hi2c, HAL_TIMEOUT);
}
