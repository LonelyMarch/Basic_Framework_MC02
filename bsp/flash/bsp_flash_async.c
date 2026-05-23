#include "bsp_flash_async.h"

#include "bsp_flash.h"
#include "bsp_log.h"
#include "bsp_qspi_flash.h"
#include "FreeRTOS.h"
#include "main.h"
#include "queue.h"
#include <string.h>

typedef enum
{
    BSP_FLASH_ASYNC_JOB_ONCHIP_ERASE = 0,
    BSP_FLASH_ASYNC_JOB_ONCHIP_WRITE,
    BSP_FLASH_ASYNC_JOB_QSPI_ERASE,
    BSP_FLASH_ASYNC_JOB_QSPI_WRITE,
} BSP_FlashAsyncJobType;

typedef struct
{
    BSP_FlashAsyncJobType type;                 // 任务类型
    uint32_t address;                           // 片上Flash偏移或QSPI内部地址
    uint32_t size;                              // 擦除/写入长度
    uint8_t data[BSP_FLASH_ASYNC_DATA_SIZE];    // 写任务的数据副本
} BSP_FlashAsyncJob;

static StaticQueue_t flash_async_queue_cb;
static uint8_t flash_async_queue_storage[BSP_FLASH_ASYNC_JOB_CNT * sizeof(BSP_FlashAsyncJob)];
static QueueHandle_t flash_async_queue;
static volatile uint8_t flash_async_running;
static volatile uint32_t flash_async_dropped_cnt;
static volatile uint32_t flash_async_processed_cnt;
static volatile uint32_t flash_async_failed_cnt;
static volatile int8_t flash_async_last_status = BSP_FLASH_ASYNC_OK;

static void BSP_FlashAsyncDropOne(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    flash_async_dropped_cnt++;
    __set_PRIMASK(primask);
}

void BSP_FlashAsyncInit(void)
{
    if (flash_async_queue == NULL)
    {
        /*
         * Flash异步任务使用FreeRTOS静态Queue保存擦写请求,避免手写环形队列,
         * 也不依赖FreeRTOS heap。BSPInit()会在系统启动早期调用本函数。
         */
        flash_async_queue = xQueueCreateStatic(BSP_FLASH_ASYNC_JOB_CNT,
                                               sizeof(BSP_FlashAsyncJob),
                                               flash_async_queue_storage,
                                               &flash_async_queue_cb);
        if (flash_async_queue == NULL)
        {
            LOGERROR("[flash_async] FreeRTOS queue create failed");
            Error_Handler();
        }
    }
}

static QueueHandle_t BSP_FlashAsyncGetQueue(void)
{
    if (flash_async_queue == NULL && __get_IPSR() == 0U)
    {
        BSP_FlashAsyncInit();
    }

    return flash_async_queue;
}

static int8_t BSP_FlashAsyncPush(BSP_FlashAsyncJob const *job)
{
    QueueHandle_t queue = BSP_FlashAsyncGetQueue();
    int8_t status = BSP_FLASH_ASYNC_ERROR_QUEUE_FULL;

    if (job == NULL)
    {
        return BSP_FLASH_ASYNC_ERROR_INVALID_PARAM;
    }

    /*
     * 提交写任务需要复制最多BSP_FLASH_ASYNC_DATA_SIZE字节数据。
     * 为避免ISR里长时间关中断,异步Flash任务不允许从中断上下文提交。
     */
    if (__get_IPSR() != 0U)
    {
        return BSP_FLASH_ASYNC_ERROR_IN_ISR;
    }

    if (queue == NULL)
    {
        return BSP_FLASH_ASYNC_ERROR_QUEUE_FULL;
    }

    if (xQueueSendToBack(queue, job, 0U) == pdPASS)
    {
        return BSP_FLASH_ASYNC_OK;
    }

    BSP_FlashAsyncDropOne();
    return status;
}

static int8_t BSP_FlashAsyncExecute(BSP_FlashAsyncJob const *job)
{
    if (job == NULL)
    {
        return BSP_FLASH_ASYNC_ERROR_INVALID_PARAM;
    }

    switch (job->type)
    {
    case BSP_FLASH_ASYNC_JOB_ONCHIP_ERASE:
        return BSP_Flash_Erase(job->address, job->size);
    case BSP_FLASH_ASYNC_JOB_ONCHIP_WRITE:
        return BSP_Flash_Write(job->address, job->data, job->size);
    case BSP_FLASH_ASYNC_JOB_QSPI_ERASE:
        return BSP_QSPI_Flash_EraseRange(job->address, job->size);
    case BSP_FLASH_ASYNC_JOB_QSPI_WRITE:
        return BSP_QSPI_Flash_Write(job->address, job->data, job->size);
    default:
        return BSP_FLASH_ASYNC_ERROR_INVALID_PARAM;
    }
}

static int8_t BSP_FlashAsyncPostWrite(BSP_FlashAsyncJobType type, uint32_t address, const void *data, uint32_t size)
{
    BSP_FlashAsyncJob job;

    if ((data == NULL) || (size == 0U))
    {
        LOGERROR("[flash_async] write invalid param, addr = 0x%X, size = %u", address, size);
        return BSP_FLASH_ASYNC_ERROR_INVALID_PARAM;
    }

    if (size > BSP_FLASH_ASYNC_DATA_SIZE)
    {
        LOGERROR("[flash_async] write too large, size = %u, limit = %u",
                 size,
                 (unsigned int)BSP_FLASH_ASYNC_DATA_SIZE);
        return BSP_FLASH_ASYNC_ERROR_TOO_LARGE;
    }

    memset(&job, 0, sizeof(job));
    job.type = type;
    job.address = address;
    job.size = size;
    memcpy(job.data, data, size);

    return BSP_FlashAsyncPush(&job);
}

static int8_t BSP_FlashAsyncPostErase(BSP_FlashAsyncJobType type, uint32_t address, uint32_t size)
{
    BSP_FlashAsyncJob job;

    if (size == 0U)
    {
        LOGERROR("[flash_async] erase invalid param, addr = 0x%X, size = %u", address, size);
        return BSP_FLASH_ASYNC_ERROR_INVALID_PARAM;
    }

    memset(&job, 0, sizeof(job));
    job.type = type;
    job.address = address;
    job.size = size;

    return BSP_FlashAsyncPush(&job);
}

int8_t BSP_FlashAsyncPostOnchipErase(uint32_t offset, uint32_t size)
{
    return BSP_FlashAsyncPostErase(BSP_FLASH_ASYNC_JOB_ONCHIP_ERASE, offset, size);
}

int8_t BSP_FlashAsyncPostOnchipWrite(uint32_t offset, const void *data, uint32_t size)
{
    return BSP_FlashAsyncPostWrite(BSP_FLASH_ASYNC_JOB_ONCHIP_WRITE, offset, data, size);
}

int8_t BSP_FlashAsyncPostQspiErase(uint32_t addr, uint32_t size)
{
    return BSP_FlashAsyncPostErase(BSP_FLASH_ASYNC_JOB_QSPI_ERASE, addr, size);
}

int8_t BSP_FlashAsyncPostQspiWrite(uint32_t addr, const void *data, uint32_t size)
{
    return BSP_FlashAsyncPostWrite(BSP_FLASH_ASYNC_JOB_QSPI_WRITE, addr, data, size);
}

uint8_t BSP_FlashAsyncIsBusy(void)
{
    return (flash_async_running != 0U) || (BSP_FlashAsyncGetPendingCount() != 0U);
}

uint8_t BSP_FlashAsyncGetPendingCount(void)
{
    QueueHandle_t queue = BSP_FlashAsyncGetQueue();
    UBaseType_t count;

    if (queue == NULL)
    {
        return 0U;
    }

    count = uxQueueMessagesWaiting(queue);
    return (count > 255U) ? 255U : (uint8_t)count;
}

uint32_t BSP_FlashAsyncGetDroppedCount(void)
{
    return flash_async_dropped_cnt;
}

uint32_t BSP_FlashAsyncGetProcessedCount(void)
{
    return flash_async_processed_cnt;
}

uint32_t BSP_FlashAsyncGetFailedCount(void)
{
    return flash_async_failed_cnt;
}

int8_t BSP_FlashAsyncGetLastStatus(void)
{
    return flash_async_last_status;
}

__attribute__((noreturn)) void BSP_FlashAsyncTask(void *argument)
{
    BSP_FlashAsyncJob job;
    int8_t status;

    (void)argument;
    BSP_FlashAsyncInit();
    LOGINFO("[flash_async] Flash async task start");

    for (;;)
    {
        if (flash_async_queue == NULL)
        {
            LOGERROR("[flash_async] queue create failed");
            Error_Handler();
        }

        if (xQueueReceive(flash_async_queue, &job, portMAX_DELAY) != pdPASS)
        {
            continue;
        }

        flash_async_running = 1U;
        status = BSP_FlashAsyncExecute(&job);
        flash_async_last_status = status;
        flash_async_processed_cnt++;
        if (status != BSP_FLASH_ASYNC_OK)
        {
            flash_async_failed_cnt++;
            LOGERROR("[flash_async] flash job failed, type = %u, addr = 0x%X, size = %u, status = %d",
                     (unsigned int)job.type,
                     (unsigned int)job.address,
                     (unsigned int)job.size,
                     (int)status);
        }
        flash_async_running = 0U;
    }
}
