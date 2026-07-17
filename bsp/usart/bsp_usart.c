/**
 * @file bsp_usart.c
 * @author neozng
 * @brief  串口bsp层的实现
 * @version beta
 * @date 2022-11-01
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "bsp_usart.h"
#include "bsp_log.h"
#include "bsp_service.h"
#include "memory.h"

#define USART_DCACHE_LINE_SIZE 32U

/* USART服务实例表,所有通过USARTRegister()注册的模块信息都会保存在这里。 */
static uint8_t idx;
static USARTInstance usart_instance_pool[DEVICE_USART_CNT]; // USART实例静态池,控制结构体放默认.bss/DTCM
static USARTInstance* usart_instance[DEVICE_USART_CNT] = {NULL};

/*
 * STM32H7的DMA1/DMA2不能访问DTCM,而本工程默认heap/bss在DTCM中。
 * 因此USART的DMA收发缓冲区统一放到链接脚本映射到RAM_D2的.dma_buffer段。
 */
static uint8_t usart_rx_dma_buffer[DEVICE_USART_CNT][USART_RXBUFF_LIMIT]
__attribute__ ((section
(
".dma_buffer"
)
,
aligned (
32
)
)
);
static uint8_t usart_tx_dma_buffer[DEVICE_USART_CNT][USART_TXBUFF_LIMIT]
__attribute__ ((section
(
".dma_buffer"
)
,
aligned (
32
)
)
);
static uint8_t usart_parse_buffer[DEVICE_USART_CNT][USART_PARSE_BUFF_CNT][USART_RXBUFF_LIMIT];
static uint16_t usart_parse_len[DEVICE_USART_CNT][USART_PARSE_BUFF_CNT];

static void USARTAlignDCacheRange(uintptr_t address, uint32_t size, uintptr_t* aligned_address, int32_t* aligned_size)
{
    uintptr_t start = address & ~((uintptr_t)USART_DCACHE_LINE_SIZE - 1U);
    uintptr_t end = (address + size + USART_DCACHE_LINE_SIZE - 1U) & ~((uintptr_t)USART_DCACHE_LINE_SIZE - 1U);

    *aligned_address = start;
    *aligned_size = (int32_t)(end - start);
}

static void USARTCleanDCacheByAddr(const void* buffer, uint16_t len)
{
#if USART_USE_DMA_CACHE_MAINTENANCE
    uintptr_t aligned_address;
    int32_t aligned_size;

    if (buffer == NULL || len == 0)
        return;

    USARTAlignDCacheRange((uintptr_t)buffer, len, &aligned_address, &aligned_size);
    SCB_CleanDCache_by_Addr((uint32_t*)aligned_address, aligned_size);
#else
    UNUSED(buffer);
    UNUSED(len);
#endif
}

static void USARTInvalidateDCacheByAddr(const void* buffer, uint16_t len)
{
#if USART_USE_DMA_CACHE_MAINTENANCE
    uintptr_t aligned_address;
    int32_t aligned_size;

    if (buffer == NULL || len == 0)
        return;

    USARTAlignDCacheRange((uintptr_t)buffer, len, &aligned_address, &aligned_size);
    SCB_InvalidateDCache_by_Addr((uint32_t*)aligned_address, aligned_size);
#else
    UNUSED(buffer);
    UNUSED(len);
#endif
}

static USARTInstance* USARTFindInstance(UART_HandleTypeDef* huart)
{
    for (uint8_t i = 0; i < idx; ++i)
    {
        if (usart_instance[i] != NULL && usart_instance[i]->usart_handle == huart)
            return usart_instance[i];
    }

    return NULL;
}

static uint8_t USARTTryAcquireTx(USARTInstance* _instance)
{
    uint8_t acquired = 0;
    uint32_t primask;

    if (_instance == NULL || _instance->usart_handle == NULL)
        return 0;

    primask = __get_PRIMASK();
    __disable_irq();
    if (_instance->tx_busy == 0 && _instance->usart_handle->gState == HAL_UART_STATE_READY)
    {
        _instance->tx_busy = 1;
        acquired = 1;
    }
    __set_PRIMASK(primask);

    return acquired;
}

static void USARTReleaseTx(USARTInstance* _instance)
{
    uint32_t primask;

    if (_instance == NULL)
        return;

    primask = __get_PRIMASK();
    __disable_irq();
    _instance->tx_busy = 0;
    __set_PRIMASK(primask);
}

static void USARTSaveReceivedFrame(USARTInstance* _instance, uint16_t size)
{
    uint16_t copy_size;

    if (_instance == NULL || size == 0)
        return;

    copy_size = size;
    if (copy_size > _instance->recv_buff_size)
        copy_size = _instance->recv_buff_size;
    if (copy_size > USART_RXBUFF_LIMIT)
        copy_size = USART_RXBUFF_LIMIT;

    if (_instance->usart_handle != NULL && _instance->usart_handle->hdmarx != NULL)
    {
        // DMA刚把数据写入RAM_D2,CPU拷贝前先失效DCache,避免读到旧缓存。
        USARTInvalidateDCacheByAddr(_instance->rx_dma_buff, copy_size);
    }

    if (BSPFrameQueuePush(&_instance->rx_queue, _instance->rx_dma_buff, copy_size) == 0U)
    {
        return;
    }

    // 已经有完整帧等待任务解析,立即唤醒BSP服务任务,避免依赖固定1ms轮询。
    BSPServiceNotify();
}

static void USARTStartReceive(USARTInstance* _instance)
{
    HAL_StatusTypeDef status;

    if (_instance == NULL || _instance->usart_handle == NULL)
    {
        if (__get_IPSR() == 0U)
        {
            LOGERROR("[bsp_usart] USART receive start with null instance or handle");
        }
        return;
    }

    if (_instance->usart_handle->hdmarx != NULL)
    {
        // 重新启动DMA接收前失效接收缓冲,避免旧Cache行后续写回覆盖DMA新数据。
        USARTInvalidateDCacheByAddr(_instance->rx_dma_buff, _instance->recv_buff_size);
        status = HAL_UARTEx_ReceiveToIdle_DMA(_instance->usart_handle, _instance->rx_dma_buff,
                                              _instance->recv_buff_size);
        // 关闭DMA半传输中断,避免同一帧数据在半满和IDLE/满传输时被重复回调处理。
        __HAL_DMA_DISABLE_IT(_instance->usart_handle->hdmarx, DMA_IT_HT);
    }
    else
    {
        // 当前UART5没有配置RX DMA,但已经开启UART5全局中断,因此使用IDLE IT模式接收遥控器数据。
        status = HAL_UARTEx_ReceiveToIdle_IT(_instance->usart_handle, _instance->rx_dma_buff,
                                             _instance->recv_buff_size);
    }

    if (status != HAL_OK)
    {
        if (__get_IPSR() != 0U)
        {
            _instance->error_count++;
        }
        else
        {
            LOGWARNING("[bsp_usart] USART receive start failed, status [%d]", status);
        }
    }
}

/**
 * @brief 启动串口接收服务。
 *
 * @note 注册成功后会自动调用本函数。若串口配置了RX DMA,使用ReceiveToIdle DMA;
 *       未配置RX DMA但开启了UART全局中断时,使用ReceiveToIdle IT。
 *
 * @param _instance instance owned by module,模块拥有的串口实例
 */
void USARTServiceInit(USARTInstance* _instance)
{
    USARTStartReceive(_instance);
}

USARTInstance* USARTRegister(USART_Init_Config_s* init_config)
{
    if (init_config == NULL || init_config->usart_handle == NULL)
    {
        LOGERROR("[bsp_usart] USART register with invalid config!");
        return NULL;
    }

    if (init_config->recv_buff_size == 0 || init_config->recv_buff_size > USART_RXBUFF_LIMIT)
    {
        LOGERROR("[bsp_usart] USART recv buff size [%d] invalid, limit [%d]!",
                 init_config->recv_buff_size, USART_RXBUFF_LIMIT);
        return NULL;
    }

    if (idx >= DEVICE_USART_CNT) // 超过最大实例数
    {
        LOGERROR("[bsp_usart] USART exceed max instance count!");
        return NULL;
    }

    for (uint8_t i = 0; i < idx; i++) // 检查是否已经注册过
    {
        if (usart_instance[i]->usart_handle == init_config->usart_handle)
        {
            LOGERROR("[bsp_usart] USART instance already registered!");
            return NULL;
        }
    }

    USARTInstance* instance = &usart_instance_pool[idx];
    memset(instance, 0, sizeof(USARTInstance));

    instance->usart_handle = init_config->usart_handle;
    instance->recv_buff_size = init_config->recv_buff_size;
    instance->module_callback = init_config->module_callback;
    instance->rx_dma_buff = usart_rx_dma_buffer[idx];
    instance->tx_dma_buff = usart_tx_dma_buffer[idx];
    instance->recv_buff = usart_parse_buffer[idx][0];
    BSPFrameQueueInit(&instance->rx_queue,
                      &usart_parse_buffer[idx][0][0],
                      usart_parse_len[idx],
                      USART_PARSE_BUFF_CNT,
                      USART_RXBUFF_LIMIT);
    // .dma_buffer段为NOLOAD,上电后内容不保证为0,注册时主动清空保持旧版buffer语义。
    memset(instance->rx_dma_buff, 0, USART_RXBUFF_LIMIT);
    memset(instance->tx_dma_buff, 0, USART_TXBUFF_LIMIT);

    usart_instance[idx++] = instance;
    USARTServiceInit(instance);
    return instance;
}

HAL_StatusTypeDef USARTSend(USARTInstance* _instance, uint8_t* send_buf, uint16_t send_size, USART_TRANSFER_MODE mode)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (_instance == NULL || _instance->usart_handle == NULL || (send_buf == NULL && send_size > 0))
    {
        LOGERROR("[bsp_usart] USART send with invalid argument");
        return HAL_ERROR;
    }

    if (send_size == 0)
        return HAL_OK;

    if (USARTTryAcquireTx(_instance) == 0)
        return HAL_BUSY;

    switch (mode)
    {
    case USART_TRANSFER_BLOCKING:
        status = HAL_UART_Transmit(_instance->usart_handle, send_buf, send_size, 100);
        USARTReleaseTx(_instance);
        break;
    case USART_TRANSFER_IT:
        if (send_size > USART_TXBUFF_LIMIT)
        {
            LOGERROR("[bsp_usart] USART IT tx size [%d] exceed limit [%d]", send_size, USART_TXBUFF_LIMIT);
            USARTReleaseTx(_instance);
            return HAL_ERROR;
        }
        // IT发送同样使用实例内部缓冲,避免上层局部变量在发送完成前失效。
        memcpy(_instance->tx_dma_buff, send_buf, send_size);
        status = HAL_UART_Transmit_IT(_instance->usart_handle, _instance->tx_dma_buff, send_size);
        if (status != HAL_OK)
            USARTReleaseTx(_instance);
        break;
    case USART_TRANSFER_DMA:
        if (send_size > USART_TXBUFF_LIMIT)
        {
            LOGERROR("[bsp_usart] USART DMA tx size [%d] exceed limit [%d]", send_size, USART_TXBUFF_LIMIT);
            USARTReleaseTx(_instance);
            return HAL_ERROR;
        }
        // DMA不能访问DTCM中的普通局部/全局buffer,发送前先复制到RAM_D2缓冲区。
        memcpy(_instance->tx_dma_buff, send_buf, send_size);
        // CPU刚写入发送缓冲,DMA读取前需要清理DCache,确保外设读到最新数据。
        USARTCleanDCacheByAddr(_instance->tx_dma_buff, send_size);
        status = HAL_UART_Transmit_DMA(_instance->usart_handle, _instance->tx_dma_buff, send_size);
        if (status != HAL_OK)
            USARTReleaseTx(_instance);
        break;
    default:
        LOGERROR("[bsp_usart] USART send with illegal transfer mode [%d]", mode);
        USARTReleaseTx(_instance);
        return HAL_ERROR;
    }

    if (status != HAL_OK && status != HAL_BUSY)
    {
        LOGWARNING("[bsp_usart] USART send failed, status [%d]", status);
    }

    return status;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    USARTInstance* instance = USARTFindInstance(huart);

    if (instance != NULL)
    {
        USARTReleaseTx(instance);
    }
}

void HAL_UART_AbortCpltCallback(UART_HandleTypeDef* huart)
{
    USARTInstance* instance = USARTFindInstance(huart);

    if (instance != NULL)
    {
        USARTReleaseTx(instance);
    }
}

void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef* huart)
{
    USARTInstance* instance = USARTFindInstance(huart);

    if (instance != NULL)
    {
        USARTReleaseTx(instance);
    }
}

/* 串口发送时,gstate会被设为BUSY_TX */
uint8_t USARTIsReady(USARTInstance* _instance)
{
    if (_instance == NULL || _instance->usart_handle == NULL)
        return 0;

    /*
     * gState只描述发送/全局状态,RxState才描述接收状态。
     * 串口长期挂着ReceiveToIdle接收时仍然允许发送,因此这里只判断TX是否空闲。
     */
    return (_instance->tx_busy == 0) && (_instance->usart_handle->gState == HAL_UART_STATE_READY);
}

uint32_t USARTGetErrorCount(USARTInstance* _instance)
{
    if (_instance == NULL)
        return 0U;

    return _instance->error_count;
}

void USARTProcess(void)
{
    USARTInstance* instance;
    uint8_t* recv_buff;
    uint16_t recv_len;

    for (uint8_t i = 0; i < idx; ++i)
    {
        instance = usart_instance[i];
        if (instance == NULL)
            continue;

        while (BSPFrameQueuePeek(&instance->rx_queue, &recv_buff, &recv_len) != 0U)
        {
            instance->recv_buff = recv_buff;
            instance->recv_len = recv_len;

            if (instance->module_callback != NULL)
            {
                // module_callback在任务上下文执行,因此可以安全进行协议解析和普通模块状态更新。
                instance->module_callback();
            }

            /*
             * 当前帧处理完之后再释放解析缓冲,避免ISR在回调执行期间覆盖正在解析的数据。
             * 清空数据本身不需要关中断;读索引和待处理计数由公共帧队列维护。
             */
            memset(recv_buff, 0, recv_len);
            BSPFrameQueuePop(&instance->rx_queue);
        }
    }
}

/**
 * @brief 每次dma/idle中断发生时，都会调用此函数.这里只保存数据并重新启动接收
 *        视觉协议解析/遥控器解析/裁判系统解析会在USARTProcess()任务上下文中执行
 *
 * @note  通过__HAL_DMA_DISABLE_IT(huart->hdmarx,DMA_IT_HT)关闭dma half transfer中断防止两次进入HAL_UARTEx_RxEventCallback()
 *        这是HAL库的一个设计失误,发生DMA传输完成/半完成以及串口IDLE中断都会触发HAL_UARTEx_RxEventCallback()
 *        我们只希望处理传输完成和IDLE两种情况,因此直接关闭DMA半传输中断。
 *
 * @param huart 发生中断的串口
 * @param Size 此次接收到的数据量
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
    for (uint8_t i = 0; i < idx; ++i)
    {
        if (huart == usart_instance[i]->usart_handle)
        {
            USARTSaveReceivedFrame(usart_instance[i], Size);
            USARTStartReceive(usart_instance[i]);
            return;
        }
    }
}

/**
 * @brief 当串口发送/接收出现错误时,会调用此函数,此时这个函数要做的就是重新启动接收
 *
 * @note  最常见的错误:奇偶校验/溢出/帧错误
 *
 * @param huart 发生错误的串口
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    for (uint8_t i = 0; i < idx; ++i)
    {
        if (huart == usart_instance[i]->usart_handle)
        {
            usart_instance[i]->error_count++;
            USARTReleaseTx(usart_instance[i]);
            USARTStartReceive(usart_instance[i]);
            return;
        }
    }
}
