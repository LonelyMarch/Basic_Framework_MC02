/**
 * @file bsp_frame_queue.h
 * @brief BSP内部固定大小帧队列。
 *
 * @note 该队列面向“中断/回调快速复制一帧,任务上下文稍后解析”的场景。
 *       它不做动态内存分配,也不负责唤醒任务;唤醒动作由USART/USB等调用者完成。
 */
#ifndef BSP_FRAME_QUEUE_H
#define BSP_FRAME_QUEUE_H

#include <stdint.h>

typedef struct
{
    uint8_t *buffer;              // 帧数据存储区,按 frame_count * frame_size 连续排列
    uint16_t *len_table;          // 每个帧槽位对应的有效数据长度
    uint16_t frame_size;          // 单帧最大字节数
    uint8_t frame_count;          // 帧槽位数量
    volatile uint8_t write_idx;   // 中断/生产者写入索引
    volatile uint8_t read_idx;    // 任务/消费者读取索引
    volatile uint8_t reserved_cnt;// 已预留但尚未发布给消费者的槽位数量,用于避免生产者写一半就被消费者看到
    volatile uint8_t pending_cnt; // 等待任务处理的帧数量
    volatile uint32_t dropped_cnt;// 队列满时丢弃的新帧数量
} BSPFrameQueue;

/**
 * @brief 初始化固定大小帧队列。
 *
 * @note 该队列不分配内存,外部必须提供静态 buffer 和 len_table。
 */
void BSPFrameQueueInit(BSPFrameQueue *queue,
                       uint8_t *buffer,
                       uint16_t *len_table,
                       uint8_t frame_count,
                       uint16_t frame_size);

/**
 * @brief 从中断或普通上下文写入一帧数据。
 *
 * @note 若len超过frame_size,队列会截断为frame_size字节保存。
 *
 * @return uint8_t 1表示写入成功,0表示参数非法或队列满。
 */
uint8_t BSPFrameQueuePush(BSPFrameQueue *queue, const uint8_t *data, uint16_t len);

/**
 * @brief 查看当前最早一帧数据,不释放槽位。
 *
 * @return uint8_t 1表示取到帧,0表示队列为空或参数非法。
 */
uint8_t BSPFrameQueuePeek(BSPFrameQueue *queue, uint8_t **data, uint16_t *len);

/**
 * @brief 释放当前最早一帧数据槽位。
 */
void BSPFrameQueuePop(BSPFrameQueue *queue);

/**
 * @brief 获取等待处理的帧数量。
 */
uint8_t BSPFrameQueuePendingCount(const BSPFrameQueue *queue);

/**
 * @brief 获取队列满导致的丢帧计数。
 */
uint32_t BSPFrameQueueDroppedCount(const BSPFrameQueue *queue);

#endif
