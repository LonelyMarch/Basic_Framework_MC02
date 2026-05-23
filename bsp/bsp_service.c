#include "bsp_service.h"

#include "bsp_log.h"
#include "bsp_usart.h"
#include "bsp_usb.h"
#include "FreeRTOS.h"
#include "main.h"
#include "queue.h"
#include "task.h"

#define BSP_SERVICE_IDLE_TIMEOUT_MS 50U

typedef struct
{
    BSPServiceEventCallback callback;
    void *arg;
} BSPServiceEvent;

static StaticQueue_t bsp_service_queue_cb;
static uint8_t bsp_service_queue_storage[BSP_SERVICE_EVENT_CNT * sizeof(BSPServiceEvent)];
static QueueHandle_t bsp_service_queue;
static TaskHandle_t bsp_service_task_handle;
static volatile uint32_t bsp_service_dropped_cnt;

static void BSPServiceNotifyFromISR(BaseType_t *higher_priority_task_woken)
{
    if ((bsp_service_task_handle != NULL) && (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING))
    {
        /*
         * BSPServiceTask阻塞等待任务通知。ISR中只负责唤醒任务,
         * 真正的日志输出、协议解析和模块回调仍在任务上下文执行。
         */
        vTaskNotifyGiveFromISR(bsp_service_task_handle, higher_priority_task_woken);
    }
}

void BSPServiceInit(void)
{
    if (bsp_service_queue == NULL)
    {
        /*
         * 使用FreeRTOS静态Queue,不依赖heap。BSPInit()会在关中断的初始化阶段调用,
         * 因此后续ISR投递事件时队列已经存在。
         */
        bsp_service_queue = xQueueCreateStatic(BSP_SERVICE_EVENT_CNT,
                                               sizeof(BSPServiceEvent),
                                               bsp_service_queue_storage,
                                               &bsp_service_queue_cb);
        if (bsp_service_queue == NULL)
        {
            LOGERROR("[bsp_service] FreeRTOS queue create failed");
            Error_Handler();
        }
    }
}

static void BSPServiceDropOne(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    bsp_service_dropped_cnt++;
    __set_PRIMASK(primask);
}

static QueueHandle_t BSPServiceGetQueue(void)
{
    if (bsp_service_queue == NULL && __get_IPSR() == 0U)
    {
        BSPServiceInit();
    }

    return bsp_service_queue;
}

void BSPServiceNotify(void)
{
    if ((bsp_service_task_handle == NULL) || (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING))
    {
        return;
    }

    if (__get_IPSR() != 0U)
    {
        BaseType_t higher_priority_task_woken = pdFALSE;

        BSPServiceNotifyFromISR(&higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
    else
    {
        /*
         * 任务上下文中产生新的BSP待处理事件时,直接通知服务任务。
         * 通知是计数型的,即使服务任务还没进入等待,也不会丢失这次唤醒。
         */
        (void)xTaskNotifyGive(bsp_service_task_handle);
    }
}

uint8_t BSPServicePost(BSPServiceEventCallback callback, void *arg)
{
    BSPServiceEvent event;
    QueueHandle_t queue = BSPServiceGetQueue();

    if (callback == NULL || queue == NULL)
    {
        return 0;
    }

    event.callback = callback;
    event.arg = arg;

    if (xQueueSendToBack(queue, &event, 0U) != pdPASS)
    {
        BSPServiceDropOne();
        return 0;
    }

    BSPServiceNotify();
    return 1;
}

uint8_t BSPServicePostFromISR(BSPServiceEventCallback callback, void *arg)
{
    BSPServiceEvent event;
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (callback == NULL || bsp_service_queue == NULL)
    {
        BSPServiceDropOne();
        return 0;
    }

    event.callback = callback;
    event.arg = arg;

    if (xQueueSendToBackFromISR(bsp_service_queue, &event, &higher_priority_task_woken) != pdPASS)
    {
        BSPServiceDropOne();
        return 0;
    }

    BSPServiceNotifyFromISR(&higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
    return 1;
}

void BSPServiceProcess(void)
{
    BSPServiceEvent event;
    QueueHandle_t queue = BSPServiceGetQueue();

    BSPLogProcess();

    if (queue != NULL)
    {
        while (xQueueReceive(queue, &event, 0U) == pdPASS)
        {
            if (event.callback != NULL)
            {
                event.callback(event.arg);
            }
        }
    }

    USARTProcess();
    USBProcess();
    BSPLogProcess();
}

uint32_t BSPServiceGetDroppedEventCount(void)
{
    return bsp_service_dropped_cnt;
}

__attribute__((noreturn)) void BSPServiceTask(void *argument)
{
    (void)argument;

    /*
     * 记录当前任务句柄,供USART/USB/log/通用事件队列在产生待处理数据时唤醒本任务。
     */
    bsp_service_task_handle = xTaskGetCurrentTaskHandle();

    LOGINFO("[bsp_service] BSP Service Task Start");

    for (;;)
    {
        BSPServiceProcess();
        /*
         * 正常情况下由BSPServiceNotify()事件唤醒;超时只是兜底,
         * 避免极端情况下某个模块漏通知后数据长时间得不到处理。
         */
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(BSP_SERVICE_IDLE_TIMEOUT_MS));
    }
}
