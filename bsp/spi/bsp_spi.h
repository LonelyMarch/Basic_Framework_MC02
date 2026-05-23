/**
 * @file bsp_spi.h
 * @brief SPI总线型外设BSP封装,支持阻塞/中断/DMA同步事务。
 */
#ifndef BSP_SPI_H
#define BSP_SPI_H

#include "spi.h"
#include "stdint.h"
#include "gpio.h"

/* 根据开发板引出的SPI引脚以及CubeMX中的初始化配置设定 */
#define SPI_BUS_CNT 3          // 当前主控板引出3路SPI硬件总线
#define MX_SPI_BUS_SLAVE_CNT 4 // 单个SPI总线上最多挂载的从机数目
#define SPI_MX_INSTANCE_CNT (SPI_BUS_CNT * MX_SPI_BUS_SLAVE_CNT) // BSP层最多注册的SPI从设备实例数量

#ifndef SPI_HAL_TIMEOUT_MS
#define SPI_HAL_TIMEOUT_MS 5U // HAL阻塞SPI事务超时时间,单位ms
#endif

#ifndef SPI_OS_TIMEOUT_TICK
#define SPI_OS_TIMEOUT_TICK 100U // FreeRTOS等待SPI互斥锁或完成信号的超时时间,单位tick
#endif

#ifndef SPI_DMA_BOUNCE_BUFFER_SIZE
#define SPI_DMA_BOUNCE_BUFFER_SIZE 1024U // SPI DMA内部RAM_D2中转缓冲区大小
#endif

// 是否启用SPI DMA的D-Cache一致性维护。若后续确认DMA缓冲区位于non-cacheable区域,可以改为0。
#ifndef SPI_USE_DMA_CACHE_MAINTENANCE
#define SPI_USE_DMA_CACHE_MAINTENANCE 1U
#endif

/* SPI传输模式枚举。IT/DMA模式内部仍会等待事务完成,对上层表现为同步接口。 */
typedef enum
{
    SPI_BLOCK_MODE = 0, // 默认使用阻塞模式
    SPI_IT_MODE,
    SPI_DMA_MODE,
} SPI_TXRX_MODE_e;

/* SPI实例结构体定义 */
typedef struct spi_ins_temp
{
    SPI_HandleTypeDef *spi_handle; // SPI外设handle
    GPIO_TypeDef *GPIOx;           // 片选信号对应的GPIO,如GPIOA,GPIOB等等
    uint16_t cs_pin;               // 片选信号对应的引脚号,GPIO_PIN_1,GPIO_PIN_2等等

    SPI_TXRX_MODE_e spi_work_mode; // 传输工作模式
    uint16_t rx_size;              // 本次接收的数据长度,与HAL SPI的Size参数保持一致
    uint8_t *rx_buffer;            // 本次接收的数据缓冲区
    uint8_t CS_State;              // 片选信号状态,用于记录当前片选电平
    void (*callback)(struct spi_ins_temp *); // 接收回调函数
    void *id;                                // 模块指针
} SPIInstance;

/* 接收回调函数定义,包含SPI的module按照此格式构建回调函数 */
typedef void (*spi_rx_callback)(SPIInstance *);

/* SPI初始化配置。模块通过该结构体注册一个SPI从设备实例。 */
typedef struct
{
    SPI_HandleTypeDef *spi_handle; // SPI外设handle
    GPIO_TypeDef *GPIOx;           // 片选信号对应的GPIO,如GPIOA,GPIOB等等
    uint16_t cs_pin;               // 片选信号对应的引脚号,GPIO_PIN_1,GPIO_PIN_2等等

    SPI_TXRX_MODE_e spi_work_mode; // 传输工作模式

    spi_rx_callback callback; // 接收回调函数
    void *id;                 // 模块指针
} SPI_Init_Config_s;

/**
 * @brief 注册一个spi instance
 *
 * @param conf 传入spi配置
 * @return SPIInstance* 返回一个spi实例指针,之后通过该指针操作spi外设
 */
SPIInstance *SPIRegister(SPI_Init_Config_s *conf);

/**
 * @brief 为已经注册的SPI硬件总线创建RTOS互斥锁和完成信号量。
 *
 * @note 由BSPTaskInit()在osKernelInitialize()之后、任务启动前统一调用。
 */
HAL_StatusTypeDef SPIBusOsInit(void);

/**
 * @brief 通过spi向对应从机发送数据
 *
 * @param spi_ins spi实例指针
 * @param ptr_data 要发送的数据
 * @param len 待发送的数据长度
 * @return HAL_StatusTypeDef HAL_OK表示发送完成,其他值表示失败或超时
 */
HAL_StatusTypeDef SPITransmit(SPIInstance *spi_ins, uint8_t *ptr_data, uint16_t len);

/**
 * @brief 通过spi从从机获取数据
 *
 * @note 函数返回HAL_OK时本次接收事务已经完成;若配置了callback,回调也已在调用者任务上下文执行。
 * 
 * @param spi_ins spi实例指针
 * @param ptr_data 接受数据buffer的首地址
 * @param len 待接收的长度
 * @return HAL_StatusTypeDef HAL_OK表示接收完成,其他值表示失败或超时
 */
HAL_StatusTypeDef SPIRecv(SPIInstance *spi_ins, uint8_t *ptr_data, uint16_t len);

/**
 * @brief 通过spi利用移位寄存器同时收发数据
 *
 * @note 函数返回HAL_OK时本次收发事务已经完成;若配置了callback,回调也已在调用者任务上下文执行。
 * 
 * @param spi_ins spi实例指针
 * @param ptr_data_rx 接收数据地址
 * @param ptr_data_tx 发送数据地址
 * @param len 接收&发送的长度
 * @return HAL_StatusTypeDef HAL_OK表示收发完成,其他值表示失败或超时
 */
HAL_StatusTypeDef SPITransRecv(SPIInstance *spi_ins, uint8_t *ptr_data_rx, uint8_t *ptr_data_tx, uint16_t len);

#endif // BSP_SPI_H
