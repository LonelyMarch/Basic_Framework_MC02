/**
 * @file bsp_frame_queue.c
 * @brief BSP内部固定帧队列,用于USART/USB这类“ISR生产帧、任务消费帧”的场景。
 */

#include "bsp_frame_queue.h"
#include "main.h"
#include "memory.h"

static uint8_t BSPFrameQueueIsValid(const BSPFrameQueue *queue)
{
    return (queue != NULL &&
            queue->buffer != NULL &&
            queue->len_table != NULL &&
            queue->frame_count > 0U &&
            queue->frame_size > 0U);
}

void BSPFrameQueueInit(BSPFrameQueue *queue,
                       uint8_t *buffer,
                       uint16_t *len_table,
                       uint8_t frame_count,
                       uint16_t frame_size)
{
    if (queue == NULL)
    {
        return;
    }

    memset(queue, 0, sizeof(BSPFrameQueue));
    /*
     * 队列本身不申请内存。调用者提供实际帧缓存和长度表,
     * 这样USART/USB可以自行决定缓存所在内存域和槽位数量。
     */
    queue->buffer = buffer;
    queue->len_table = len_table;
    queue->frame_count = frame_count;
    queue->frame_size = frame_size;

    if (buffer != NULL && frame_count > 0U && frame_size > 0U)
    {
        memset(buffer, 0, (uint32_t)frame_count * frame_size);
    }

    if (len_table != NULL && frame_count > 0U)
    {
        memset(len_table, 0, (uint32_t)frame_count * sizeof(uint16_t));
    }
}

uint8_t BSPFrameQueuePush(BSPFrameQueue *queue, const uint8_t *data, uint16_t len)
{
    uint8_t write_idx;
    uint16_t copy_len;
    uint32_t primask;

    if (!BSPFrameQueueIsValid(queue) || data == NULL || len == 0U)
    {
        return 0U;
    }

    copy_len = (len > queue->frame_size) ? queue->frame_size : len;

    /*
     * 先预留槽位,再拷贝数据,最后发布pending计数。
     * 这样消费者只会看到已经写完长度和内容的完整帧。
     */
    primask = __get_PRIMASK();
    __disable_irq();
    if ((queue->pending_cnt + queue->reserved_cnt) >= queue->frame_count)
    {
        queue->dropped_cnt++;
        __set_PRIMASK(primask);
        return 0U;
    }

    write_idx = queue->write_idx;
    queue->write_idx = (uint8_t)((queue->write_idx + 1U) % queue->frame_count);
    queue->reserved_cnt++;
    __set_PRIMASK(primask);

    memcpy(&queue->buffer[(uint32_t)write_idx * queue->frame_size], data, copy_len);

    primask = __get_PRIMASK();
    __disable_irq();
    queue->len_table[write_idx] = copy_len;
    if (queue->reserved_cnt > 0U)
    {
        queue->reserved_cnt--;
    }
    queue->pending_cnt++;
    __set_PRIMASK(primask);

    return 1U;
}

uint8_t BSPFrameQueuePeek(BSPFrameQueue *queue, uint8_t **data, uint16_t *len)
{
    uint8_t read_idx;
    uint32_t primask;

    if (!BSPFrameQueueIsValid(queue) || data == NULL || len == NULL)
    {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (queue->pending_cnt == 0U)
    {
        __set_PRIMASK(primask);
        return 0U;
    }

    /*
     * Peek只返回当前读槽位,不移动read_idx。
     * 消费者完成协议解析后必须调用BSPFrameQueuePop()释放槽位。
     */
    read_idx = queue->read_idx;
    *data = &queue->buffer[(uint32_t)read_idx * queue->frame_size];
    *len = queue->len_table[read_idx];
    __set_PRIMASK(primask);

    return 1U;
}

void BSPFrameQueuePop(BSPFrameQueue *queue)
{
    uint32_t primask;

    if (!BSPFrameQueueIsValid(queue))
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (queue->pending_cnt > 0U)
    {
        /*
         * 只在Pop阶段释放槽位,避免消费者还在解析时ISR复用同一块缓存。
         */
        queue->len_table[queue->read_idx] = 0U;
        queue->read_idx = (uint8_t)((queue->read_idx + 1U) % queue->frame_count);
        queue->pending_cnt--;
    }
    __set_PRIMASK(primask);
}

uint8_t BSPFrameQueuePendingCount(const BSPFrameQueue *queue)
{
    if (!BSPFrameQueueIsValid(queue))
    {
        return 0U;
    }

    return queue->pending_cnt;
}

uint32_t BSPFrameQueueDroppedCount(const BSPFrameQueue *queue)
{
    if (!BSPFrameQueueIsValid(queue))
    {
        return 0U;
    }

    return queue->dropped_cnt;
}
