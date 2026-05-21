#ifndef BSP_RC_H
#define BSP_RC_H

#include <stdint.h>
#include "main.h"

#define DEVICE_USART_CNT 8     // MC02串口数量8个，其中
#define USART_RXBUFF_LIMIT 256 // 如果协议需要更大的接收buff,请修改这里
#define USART_TXBUFF_LIMIT 256 // 如果协议需要更大的DMA发送buff,请修改这里
#define USART_PARSE_BUFF_CNT 2 // 每个串口使用双解析缓冲,避免任务解析时被下一帧覆盖

#ifndef USART_USE_DMA_CACHE_MAINTENANCE
#define USART_USE_DMA_CACHE_MAINTENANCE 1U // STM32H7开启DCache后,DMA缓冲区需要做Cache维护
#endif

// 模块回调函数,用于解析协议
typedef void (*usart_module_callback)();

/* 发送模式枚举 */
typedef enum
{
    USART_TRANSFER_NONE=0,
    USART_TRANSFER_BLOCKING,
    USART_TRANSFER_IT,
    USART_TRANSFER_DMA,
} USART_TRANSFER_MODE;

// 串口实例结构体,每个module都要包含一个实例.
// 由于串口是独占的点对点通信,所以不需要考虑多个module同时使用一个串口的情况,因此不用加入id;当然也可以选择加入,这样在bsp层可以访问到module的其他信息
typedef struct
{
    uint8_t *recv_buff;                    // 当前供module解析使用的buffer,在USARTProcess()中切换
    uint8_t *rx_dma_buff;                  // UART DMA/IT接收buffer,实际存放在RAM_D2的.dma_buffer段
    uint8_t *tx_dma_buff;                  // DMA发送buffer,实际存放在RAM_D2的.dma_buffer段
    uint8_t *parse_buff[USART_PARSE_BUFF_CNT]; // 双解析缓冲,任务上下文解析时使用
    volatile uint16_t recv_len;            // 最近一次接收到的数据长度
    volatile uint16_t parse_len[USART_PARSE_BUFF_CNT]; // 双解析缓冲各自对应的数据长度
    volatile uint8_t parse_write_idx;      // ISR写入的解析缓冲索引
    volatile uint8_t parse_read_idx;       // 任务读取的解析缓冲索引
    volatile uint8_t pending_frame_cnt;    // 等待任务处理的帧数量
    volatile uint8_t dropped_frame_cnt;    // 解析任务来不及处理时丢弃的帧计数
    volatile uint8_t tx_busy;              // IT/DMA/阻塞发送占用标志,防止重复发送覆盖TX缓冲
    uint16_t recv_buff_size;               // 模块接收一包数据的大小,最大值由USART_RXBUFF_LIMIT限制
    UART_HandleTypeDef *usart_handle;      // 实例对应的usart_handle
    usart_module_callback module_callback; // 解析收到的数据的回调函数
} USARTInstance;

/* usart 初始化配置结构体 */
typedef struct
{
    uint16_t recv_buff_size;               // 模块接收一包数据的大小,最大值由USART_RXBUFF_LIMIT限制
    UART_HandleTypeDef *usart_handle;      // 实例对应的usart_handle
    usart_module_callback module_callback; // 解析收到的数据的回调函数
} USART_Init_Config_s;

/**
 * @brief 注册一个串口实例,返回一个串口实例指针
 *
 * @param init_config 传入串口初始化结构体
 * @return USARTInstance* 注册成功返回实例指针,失败返回NULL
 */
USARTInstance *USARTRegister(USART_Init_Config_s *init_config);

/**
 * @brief 启动串口服务,需要传入一个usart实例.一般用于lost callback的情况(使用串口的模块daemon)
 *
 * @param _instance
 */
void USARTServiceInit(USARTInstance *_instance);

/**
 * @brief 在任务上下文中处理所有已接收的串口数据,不能在中断中调用
 */
void USARTProcess(void);


/**
 * @brief 通过调用该函数可以发送一帧数据,需要传入一个usart实例,发送buff以及这一帧的长度
 * @note 在短时间内连续调用此接口,若采用IT/DMA会导致上一次的发送未完成而新的发送取消.
 * @note 若希望连续使用DMA/IT进行发送,请配合USARTIsReady()使用,或自行为你的module实现一个发送队列和任务.
 * @todo 是否考虑为USARTInstance增加发送队列以进行连续发送?
 * 
 * @param _instance 串口实例
 * @param send_buf 待发送数据的buffer
 * @param send_size how many bytes to send
 * @return HAL_StatusTypeDef HAL_OK表示已发送或已成功启动异步发送,HAL_BUSY表示上一次发送尚未完成
 */
HAL_StatusTypeDef USARTSend(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size, USART_TRANSFER_MODE mode);

/**
 * @brief 判断串口是否准备好,用于连续或异步的IT/DMA发送
 *
 * @param _instance 要判断的串口实例
 * @return uint8_t ready 1, busy 0
 */
uint8_t USARTIsReady(USARTInstance *_instance);

#endif
