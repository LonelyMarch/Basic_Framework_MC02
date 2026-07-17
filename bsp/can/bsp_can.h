#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdint.h>
#include "fdcan.h"

#define CAN_MX_REGISTER_CNT 32     // 所有FDCAN外设共用的CAN实例注册上限
#define DEVICE_CAN_CNT 3           // STM32H723VG提供3个FDCAN外设

/* CAN实例由模块注册获得,用于保存该模块在某一路FDCAN上的收发ID、缓存和回调。 */
#pragma pack(1)
typedef struct _
{
    FDCAN_HandleTypeDef* can_handle; // can句柄
    FDCAN_TxHeaderTypeDef txconf; // CAN报文发送配置
    uint32_t tx_id; // 发送id
    uint8_t tx_buff[8]; // 发送缓存,发送消息长度可以通过CANSetDLC()设定,最大为8
    uint8_t rx_buff[8]; // 接收缓存,最大消息长度为8
    uint32_t rx_id; // 接收id
    uint8_t rx_len; // 接收长度,可能为0-8
    // 接收的回调函数,由CANProcessTask在任务上下文调用,参数用于区分不同注册实例
    void (*can_module_callback)(struct _*);


    void* id; // 使用can外设的模块指针(即id指向的模块拥有此can实例,是父子关系)
} CANInstance;
#pragma pack()

/* CAN实例初始化结构体,将此结构体指针传入注册函数 */
typedef struct
{
    FDCAN_HandleTypeDef* can_handle; // can句柄
    uint32_t tx_id; // 发送id
    uint32_t rx_id; // 接收id
    void (*can_module_callback)(CANInstance*); // 处理接收数据的回调函数
    void* id; // 拥有can实例的模块地址,用于区分不同的模块(如果有需要的话),如果不需要可以不传入
} CAN_Init_Config_s;

/**
 * @brief 注册一个CAN实例,使用CAN收发前必须先调用本函数。
 *
 * @param config CAN初始化配置
 * @return CANInstance* 注册成功返回实例指针,失败进入Error_Handler或返回NULL
 */
CANInstance* CANRegister(CAN_Init_Config_s* config);


/**
 * @brief CAN接收处理任务
 *
 * @note FDCAN中断只负责把报文复制到BSP内部队列,真正的模块回调由本任务执行。
 *       当前工程由 BSPTaskInit() 创建该任务，APP 不直接调用任务入口。
 */
void CANProcessTask(void* argument);


/**
 * @brief 修改CAN发送报文的数据帧长度;注意最大长度为8,在没有进行修改的时候,默认长度为8
 *
 * @param _instance 要修改长度的can实例
 * @param length    设定长度
 */
void CANSetDLC(CANInstance* _instance, uint8_t length);


/**
 * @brief 提交一帧消息到 FDCAN 硬件 TX FIFO。
 *        发送前需要向 CANInstance 的 tx_buff 写入完整数据；提交成功后，
 *        实际总线发送由 FDCAN 硬件异步完成。
 *
 * @note tx_buff 由持有该 CANInstance 的模块负责写入，调用期间不得被其他
 *       上下文同时改写。该函数只保护总线进入 HAL 发送接口的过程。
 *
 * @attention 超时时间不应该超过调用此函数的任务的周期,否则会导致任务阻塞
 *
 * @param timeout 超时时间,单位为ms。小于1ms的超时会保持DWT短忙等,毫秒级等待会在任务态让出CPU
 * @param _instance 模块持有的CAN实例
 */
uint8_t CANTransmit(CANInstance* _instance, float timeout);


/**
 * @brief 获取CAN接收延后事件队列丢帧次数
 *
 * @return uint32_t CAN接收事件队列满导致的丢弃计数
 */
uint32_t CANGetDroppedRxEventCount(void);


/**
 * @brief 获取FDCAN硬件FIFO丢帧次数
 *
 * @param hfdcan FDCAN句柄
 * @param fifo FDCAN_RX_FIFO0 或 FDCAN_RX_FIFO1
 * @return uint32_t 指定FIFO的硬件丢帧计数
 */
uint32_t CANGetFifoLostCount(FDCAN_HandleTypeDef* hfdcan, uint32_t fifo);


/**
 * @brief 获取所有CAN硬件FIFO丢帧总次数
 *
 * @return uint32_t 硬件FIFO丢帧总计数
 */
uint32_t CANGetHardwareLostCount(void);


/**
 * @brief 获取CAN接收HAL错误计数
 *
 * @note 该计数在接收中断中递增,用于替代中断里直接打印日志。
 */
uint32_t CANGetRxHalErrorCount(void);


/**
 * @brief 获取CAN接收DLC非法计数
 *
 * @note 该计数在接收中断中递增,用于替代中断里直接打印日志。
 */
uint32_t CANGetRxInvalidDlcCount(void);

#endif
