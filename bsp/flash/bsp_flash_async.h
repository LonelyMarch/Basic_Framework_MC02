/**
 * @file bsp_flash_async.h
 * @brief 片上Flash和QSPI Flash的低优先级异步擦写队列。
 */
#ifndef BSP_FLASH_ASYNC_H
#define BSP_FLASH_ASYNC_H

#include <stdint.h>

#define BSP_FLASH_ASYNC_OK                    0
#define BSP_FLASH_ASYNC_ERROR_INVALID_PARAM  -1
#define BSP_FLASH_ASYNC_ERROR_QUEUE_FULL     -2
#define BSP_FLASH_ASYNC_ERROR_TOO_LARGE      -3
#define BSP_FLASH_ASYNC_ERROR_IN_ISR         -4

#ifndef BSP_FLASH_ASYNC_JOB_CNT
#define BSP_FLASH_ASYNC_JOB_CNT               8U
#endif

#ifndef BSP_FLASH_ASYNC_DATA_SIZE
#define BSP_FLASH_ASYNC_DATA_SIZE             256U
#endif

/**
 * @brief 初始化异步Flash任务队列
 *
 * @note 当前使用FreeRTOS静态Queue保存任务,由BSPInit()在系统启动早期调用。
 */
void BSP_FlashAsyncInit(void);

/**
 * @brief 低优先级异步Flash服务任务
 *
 * @note 由application层在任务初始化阶段创建。业务层通常不直接调用本函数。
 *       任务空闲时阻塞等待FreeRTOS Queue中的新任务。
 */
void BSP_FlashAsyncTask(void *argument);

/**
 * @brief 提交片上Flash擦除任务
 *
 * @param offset 用户Flash区内偏移
 * @param size 擦除长度,必须满足bsp_flash同步接口的对齐要求
 * @return int8_t BSP_FLASH_ASYNC_OK表示任务已入队
 */
int8_t BSP_FlashAsyncPostOnchipErase(uint32_t offset, uint32_t size);

/**
 * @brief 提交片上Flash写入任务
 *
 * @attention data会被复制到异步队列内部缓冲区,调用者函数返回后可释放原buffer。
 *
 * @param offset 用户Flash区内偏移
 * @param data 待写入数据
 * @param size 写入长度,不能超过BSP_FLASH_ASYNC_DATA_SIZE
 * @return int8_t BSP_FLASH_ASYNC_OK表示任务已入队
 */
int8_t BSP_FlashAsyncPostOnchipWrite(uint32_t offset, const void *data, uint32_t size);

/**
 * @brief 提交QSPI Flash擦除任务
 *
 * @param addr W25Q64内部地址
 * @param size 擦除长度,必须4KB对齐
 * @return int8_t BSP_FLASH_ASYNC_OK表示任务已入队
 */
int8_t BSP_FlashAsyncPostQspiErase(uint32_t addr, uint32_t size);

/**
 * @brief 提交QSPI Flash写入任务
 *
 * @attention data会被复制到异步队列内部缓冲区。建议单次不超过一页256字节。
 *
 * @param addr W25Q64内部地址
 * @param data 待写入数据
 * @param size 写入长度,不能超过BSP_FLASH_ASYNC_DATA_SIZE
 * @return int8_t BSP_FLASH_ASYNC_OK表示任务已入队
 */
int8_t BSP_FlashAsyncPostQspiWrite(uint32_t addr, const void *data, uint32_t size);

/**
 * @brief 判断异步Flash服务是否忙
 *
 * @return uint8_t 1表示队列中还有任务或当前正在执行任务
 */
uint8_t BSP_FlashAsyncIsBusy(void);

/**
 * @brief 获取等待处理的任务数量
 */
uint8_t BSP_FlashAsyncGetPendingCount(void);

/**
 * @brief 获取队列满导致的任务丢弃次数
 */
uint32_t BSP_FlashAsyncGetDroppedCount(void);

/**
 * @brief 获取已经处理完成的任务数量
 */
uint32_t BSP_FlashAsyncGetProcessedCount(void);

/**
 * @brief 获取执行失败的任务数量
 */
uint32_t BSP_FlashAsyncGetFailedCount(void);

/**
 * @brief 获取最近一次底层同步Flash接口返回值
 */
int8_t BSP_FlashAsyncGetLastStatus(void);

#endif
