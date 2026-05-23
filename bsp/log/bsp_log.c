#include "bsp_log.h"

#include "bsp_service.h"
#include "SEGGER_RTT.h"
#include "cmsis_os2.h"
#include "main.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#if BSP_LOG_USE_USB
#include "bsp_usb.h"
#endif

typedef struct
{
    uint16_t len;                   // 当前日志实际长度
    char text[BSP_LOG_LINE_SIZE];   // 已完成格式化的短日志
} BSPLogMessage;

static BSPLogMessage log_queue[BSP_LOG_QUEUE_SIZE];
static volatile uint16_t log_write_idx;
static volatile uint16_t log_read_idx;
static volatile uint16_t log_pending_cnt;
static volatile uint32_t log_dropped_cnt;

static void BSPLogDropOne(void)
{
    uint32_t primask;

    /*
     * 丢弃计数可能同时被任务和ISR更新,用极短临界区保护读改写过程,
     * 避免两个上下文同时递增时覆盖彼此的结果。
     */
    primask = __get_PRIMASK();
    __disable_irq();
    log_dropped_cnt++;
    __set_PRIMASK(primask);
}

static uint8_t BSPLogKernelIsRunning(void)
{
    return osKernelGetState() == osKernelRunning;
}

static void BSPLogDirectWrite(const char *text, uint16_t len)
{
    if (text == NULL || len == 0U)
    {
        return;
    }

    /*
     * RTT作为默认日志后端,适合调试阶段持续打开。
     * USB输出默认关闭,避免和视觉/VOFA等业务通道共用CDC时互相抢占。
     */
    (void)SEGGER_RTT_Write(BUFFER_INDEX, text, len);

#if BSP_LOG_USE_USB
    (void)USBTransmit((const uint8_t *)text, len);
#endif
}

static uint8_t BSPLogPush(const char *text, uint16_t len)
{
    uint8_t ret = 0U;
    uint32_t primask;

    if (text == NULL || len == 0U)
    {
        return 0U;
    }

    if (len >= BSP_LOG_LINE_SIZE)
    {
        len = BSP_LOG_LINE_SIZE - 1U;
    }

    /*
     * 任务和中断都可能写日志队列,这里只保护索引和计数。
     * 格式化已经在进入临界区之前完成,不会长时间关中断。
     */
    primask = __get_PRIMASK();
    __disable_irq();

    if (log_pending_cnt < BSP_LOG_QUEUE_SIZE)
    {
        log_queue[log_write_idx].len = len;
        memcpy(log_queue[log_write_idx].text, text, len);
        log_queue[log_write_idx].text[len] = '\0';
        log_write_idx = (uint16_t)((log_write_idx + 1U) % BSP_LOG_QUEUE_SIZE);
        log_pending_cnt++;
        ret = 1U;
    }
    else
    {
        BSPLogDropOne();
    }

    __set_PRIMASK(primask);

    if (ret != 0U)
    {
        /*
         * 日志已经进入队列后唤醒BSP服务任务,由任务统一输出RTT/USB。
         * ISR中误调用日志会在更早的位置被丢弃,因此这里正常只会从任务上下文触发。
         */
        BSPServiceNotify();
    }

    return ret;
}

static int BSPLogVPrintf(const char *type, const char *color, const char *format, va_list args)
{
    char line[BSP_LOG_LINE_SIZE];
    int prefix_len;
    int body_len;
    int total_len;
    size_t remain;

    if (format == NULL)
    {
        return -1;
    }

    /*
     * 日志格式化和RTT/USB输出都不适合在ISR中执行。
     * 若中断里误调用LOG宏,这里直接丢弃并计数,避免在中断上下文运行vsnprintf。
     */
    if (__get_IPSR() != 0U)
    {
        BSPLogDropOne();
        return 0;
    }

    if (type == NULL)
    {
        type = "";
    }
    if (color == NULL)
    {
        color = "";
    }

    prefix_len = snprintf(line, sizeof(line), "  %s%s", color, type);
    if (prefix_len < 0)
    {
        return -1;
    }
    if ((uint32_t)prefix_len >= sizeof(line))
    {
        prefix_len = (int)sizeof(line) - 1;
    }

    remain = sizeof(line) - (size_t)prefix_len;
    body_len = vsnprintf(&line[prefix_len], remain, format, args);
    if (body_len < 0)
    {
        return -1;
    }

    total_len = prefix_len + body_len;
    if ((uint32_t)total_len >= sizeof(line))
    {
        total_len = (int)sizeof(line) - 1;
    }

    remain = sizeof(line) - (size_t)total_len;
    if (remain > 1U)
    {
        int suffix_len = snprintf(&line[total_len], remain, "\r\n%s", RTT_CTRL_RESET);
        if (suffix_len > 0)
        {
            total_len += suffix_len;
            if ((uint32_t)total_len >= sizeof(line))
            {
                total_len = (int)sizeof(line) - 1;
            }
        }
    }

    line[total_len] = '\0';

    if (BSPLogKernelIsRunning() == 0U)
    {
        /*
         * FreeRTOS启动前还没有BSPServiceTask,此时直接输出,保证早期初始化日志可见。
         */
        BSPLogDirectWrite(line, (uint16_t)total_len);
    }
    else
    {
        (void)BSPLogPush(line, (uint16_t)total_len);
    }

    return total_len;
}

void BSPLogInit(void)
{
    SEGGER_RTT_Init();
    memset(log_queue, 0, sizeof(log_queue));
    log_write_idx = 0U;
    log_read_idx = 0U;
    log_pending_cnt = 0U;
    log_dropped_cnt = 0U;
}

void BSPLogProcess(void)
{
    BSPLogMessage msg;
    uint32_t primask;

    while (log_pending_cnt > 0U)
    {
        memset(&msg, 0, sizeof(msg));

        primask = __get_PRIMASK();
        __disable_irq();
        if (log_pending_cnt > 0U)
        {
            msg = log_queue[log_read_idx];
            log_queue[log_read_idx].len = 0U;
            log_queue[log_read_idx].text[0] = '\0';
            log_read_idx = (uint16_t)((log_read_idx + 1U) % BSP_LOG_QUEUE_SIZE);
            log_pending_cnt--;
        }
        __set_PRIMASK(primask);

        BSPLogDirectWrite(msg.text, msg.len);
    }
}

uint32_t BSPLogGetDroppedCount(void)
{
    return log_dropped_cnt;
}

int BSPLogPrintf(const char *type, const char *color, const char *format, ...)
{
    int n;
    va_list args;

    va_start(args, format);
    n = BSPLogVPrintf(type, color, format, args);
    va_end(args);

    return n;
}

int PrintLog(const char *fmt, ...)
{
    int n;
    va_list args;

    va_start(args, fmt);
    n = BSPLogVPrintf("", "", fmt, args);
    va_end(args);

    return n;
}
