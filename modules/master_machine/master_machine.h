#ifndef MASTER_MACHINE_H
#define MASTER_MACHINE_H

#include "main.h"
#include "master_protocol.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MASTER_MACHINE_DEFAULT_BRIDGE_TIMEOUT_MS 1000U /**< 任意合法帧超过该时间未到达，桥接端视为离线。 */
#define MASTER_MACHINE_DEFAULT_COMMAND_TIMEOUT_MS 200U /**< Command最新值默认有效时间。 */
#define MASTER_MACHINE_DEFAULT_VISION_TIMEOUT_MS 1000U /**< Vision最新值默认有效时间。 */
#define MASTER_MACHINE_EVENT_QUEUE_CAPACITY       8U   /**< 离散事件FIFO容量。 */

/**
 * @brief master_machine初始化配置。
 *
 * @note 三个超时字段传0时使用本文件定义的默认值；USART波特率和8N1格式由CubeMX配置。
 */
typedef struct
{
    UART_HandleTypeDef *uart_handle; /**< 与ESP32-S3连接的UART句柄，协议要求921600 8N1。 */
    uint32_t bridge_timeout_ms;      /**< 桥接端在线超时。 */
    uint32_t command_timeout_ms;     /**< Command数据新鲜度超时。 */
    uint32_t vision_timeout_ms;      /**< Vision数据新鲜度超时。 */
} MasterMachine_Init_Config_s;

/**
 * @brief 最新值通道的消费者游标。
 *
 * @note 每个消费者必须持有自己的游标，初始化为{0}即可。读取成功后模块会自动更新generation。
 */
typedef struct
{
    uint32_t generation; /**< 消费者上一次读取到的数据代数。 */
} MasterMachineCursor_s;

/** 单一帧类型的编号连续性统计。 */
typedef struct
{
    uint32_t accepted_count;       /**< 已发布给上层的数据帧数量。 */
    uint32_t duplicate_count;      /**< ID与上一帧相同而被丢弃的数量。 */
    uint32_t out_of_order_count;   /**< ID逆序而被丢弃的数量。 */
    uint32_t estimated_lost_count; /**< 根据相邻递增ID差值估算的丢帧数量。 */
} MasterMachineTypeStatistics_s;

/** master_machine完整运行统计快照。 */
typedef struct
{
    uint32_t received_byte_count;        /**< USART回调累计交给本模块的字节数。 */
    uint32_t valid_frame_count;          /**< 校验和及语义均合法的帧数，包含重复和乱序帧。 */
    uint32_t discarded_noise_byte_count; /**< 重同步过程中跳过的噪声字节数。 */
    uint32_t stream_overflow_count;      /**< 流缓存空间不足时的保护性丢弃次数。 */
    uint32_t unknown_type_count;         /**< 未知type计数。 */
    uint32_t length_error_count;         /**< payload长度与type不匹配计数。 */
    uint32_t checksum_error_count;       /**< 帧尾校验和错误计数。 */
    uint32_t semantic_error_count;       /**< payload字段范围或浮点值非法计数。 */
    uint32_t event_queue_overflow_count; /**< Event FIFO已满而丢弃新事件的数量。 */
    uint32_t uart_error_count;           /**< BSP USART错误回调累计次数。 */
    MasterMachineTypeStatistics_s command;
    MasterMachineTypeStatistics_s vision;
    MasterMachineTypeStatistics_s event;
} MasterMachineStatistics_s;

/**
 * @brief 初始化ESP32-S3上位机通信模块。
 *
 * @param config UART句柄和新鲜度超时配置。
 * @return uint8_t 初始化成功返回1，参数错误、重复初始化或USART注册失败返回0。
 */
uint8_t MasterMachineInit(const MasterMachine_Init_Config_s *config);

/**
 * @brief 判断模块是否已经成功初始化。
 *
 * @return uint8_t 已初始化返回1，否则返回0。
 */
uint8_t MasterMachineIsInitialized(void);

/**
 * @brief 判断ESP32-S3桥接端是否在线。
 *
 * @note 任意通过校验和与语义校验的帧都会刷新桥接端在线时间，重复/乱序帧也能证明链路仍然存在。
 * @return uint8_t 在线返回1，否则返回0。
 */
uint8_t MasterMachineIsBridgeOnline(void);

/**
 * @brief 判断最新Command是否仍在配置的新鲜度时间内。
 *
 * @return uint8_t 已收到过有效Command且未超时返回1，否则返回0。
 */
uint8_t MasterMachineIsCommandFresh(void);

/**
 * @brief 判断最新Vision是否仍在配置的新鲜度时间内。
 *
 * @return uint8_t 已收到过有效Vision且未超时返回1，否则返回0。
 */
uint8_t MasterMachineIsVisionFresh(void);

/**
 * @brief 按消费者游标读取一份新的Command快照。
 *
 * @param command 输出Command。
 * @param cursor 当前消费者独占的游标。
 * @return uint8_t 存在尚未被该消费者读取的新数据时返回1，否则返回0。
 */
uint8_t MasterMachineReadCommand(MasterCommandPayload *command, MasterMachineCursor_s *cursor);

/**
 * @brief 无条件读取当前最新Command快照。
 *
 * @param command 输出Command。
 * @return uint8_t 模块已收到过Command时返回1，否则返回0。
 */
uint8_t MasterMachinePeekCommand(MasterCommandPayload *command);

/**
 * @brief 按消费者游标读取一份新的Vision快照。
 *
 * @param vision 输出Vision。
 * @param cursor 当前消费者独占的游标。
 * @return uint8_t 存在尚未被该消费者读取的新数据时返回1，否则返回0。
 */
uint8_t MasterMachineReadVision(MasterVisionPayload *vision, MasterMachineCursor_s *cursor);

/**
 * @brief 无条件读取当前最新Vision快照。
 *
 * @param vision 输出Vision。
 * @return uint8_t 模块已收到过Vision时返回1，否则返回0。
 */
uint8_t MasterMachinePeekVision(MasterVisionPayload *vision);

/**
 * @brief 从离散事件FIFO中取出最早尚未处理的事件。
 *
 * @note Event FIFO设计为单一事件分发者消费；多个业务模块需要事件时，应由app层统一取出后再分发。
 * @param event_payload 输出事件。
 * @return uint8_t 成功取出返回1，队列为空或参数非法返回0。
 */
uint8_t MasterMachinePopEvent(MasterEventPayload *event_payload);

/**
 * @brief 获取当前等待处理的事件数量。
 *
 * @return uint8_t Event FIFO中的元素数量。
 */
uint8_t MasterMachineGetPendingEventCount(void);

/**
 * @brief 查询紧急停机锁存状态。
 *
 * @note 只要收到合法的紧急停机事件就立即锁存，即使此时Event FIFO已满也不会遗漏该安全状态。
 * @return uint8_t 已锁存返回1，否则返回0。
 */
uint8_t MasterMachineIsEmergencyStopLatched(void);

/**
 * @brief 由上层在完成安全复位流程后显式清除紧急停机锁存。
 *
 * @note 普通事件和新的Command不会自动解除急停，避免通信恢复时意外重新使能执行机构。
 */
void MasterMachineClearEmergencyStop(void);

/**
 * @brief 获取运行统计的线程安全快照。
 *
 * @param statistics 输出统计结构体。
 * @return uint8_t 参数有效时返回1，否则返回0。
 */
uint8_t MasterMachineGetStatistics(MasterMachineStatistics_s *statistics);

#ifdef __cplusplus
}
#endif

#endif
