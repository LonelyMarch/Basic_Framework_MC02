/**
 * @file message_center.h
 * @brief APP层轻量发布-订阅消息中心。
 *
 * @details
 * message_center用于在APP之间传递固定长度结构体消息。模块内部使用静态资源池
 * 和FreeRTOS静态Queue，不依赖运行期malloc。发布者和订阅者按MessageCenterTopic_e
 * 枚举ID绑定消息通道，每个订阅者都有独立Queue，因此同一个topic ID可以被多个APP
 * 订阅，且互不抢消息。
 */

#ifndef MESSAGE_CENTER_H
#define MESSAGE_CENTER_H

#include <stddef.h>
#include <stdint.h>

/**
 * @brief 最大订阅者数量。
 *
 * @note 一个APP订阅多个topic时，需要为每个topic各注册一个订阅者。
 */
#define MESSAGE_CENTER_MAX_SUBSCRIBER_COUNT 24U

/**
 * @brief 单条消息的最大字节数。
 *
 * @note 消息长度必须大于0。超过该大小的结构体不适合通过message_center直接传递，应改为拆分消息或传递索引/句柄。
 */
#define MESSAGE_CENTER_MAX_MESSAGE_BYTES 128U

/**
 * @brief 每个订阅者Queue支持的最大深度。
 *
 * @note 当前控制命令默认使用深度1，即“最新帧邮箱”语义。
 */
#define MESSAGE_CENTER_MAX_QUEUE_DEPTH 4U

/**
 * @brief 默认订阅者Queue深度。
 */
#define MESSAGE_CENTER_DEFAULT_QUEUE_DEPTH 1U

/**
 * @brief 消息中心topic ID。
 *
 * @details
 * topic在编译期固定为枚举值，避免在MCU RAM中保存topic字符串，也避免注册阶段做
 * 字符串比较。新增APP间消息时，应在这里添加新的topic ID，并在.c文件中的调试名
 * 表同步添加可读名称。
 */
typedef enum
{
    MESSAGE_TOPIC_GIMBAL_CMD = 0, /*!< robot_cmd发布给gimbal的云台控制命令 */
    MESSAGE_TOPIC_GIMBAL_FEED, /*!< gimbal发布给robot_cmd的云台反馈 */
    MESSAGE_TOPIC_SHOOT_CMD, /*!< robot_cmd发布给shoot的发射控制命令 */
    MESSAGE_TOPIC_SHOOT_FEED, /*!< shoot发布给robot_cmd的发射反馈 */
    MESSAGE_TOPIC_CHASSIS_CMD, /*!< robot_cmd发布给chassis的底盘控制命令 */
    MESSAGE_TOPIC_CHASSIS_FEED, /*!< chassis发布给robot_cmd的底盘反馈 */
    MESSAGE_TOPIC_COUNT, /*!< topic数量，必须放在枚举末尾 */
} MessageCenterTopic_e;

/**
 * @brief 发布者句柄。
 *
 * @note 句柄内容由message_center内部维护，应用层只保存指针并传回接口。
 */
typedef struct MessageCenterPublisher MessageCenterPublisher_t;

/**
 * @brief 订阅者句柄。
 *
 * @note 句柄内容由message_center内部维护，应用层只保存指针并传回接口。
 */
typedef struct MessageCenterSubscriber MessageCenterSubscriber_t;

/**
 * @brief 初始化消息中心。
 *
 * @details
 * 该函数会清空内部静态池。工程必须在所有APP注册发布者/订阅者之前显式调用一次。
 * 未调用该函数时，注册接口会拒绝注册并返回NULL。
 *
 * @note 当前工程建议在RobotInit()中调用。该函数不创建动态内存。
 */
void MessageCenterInit(void);


/**
 * @brief 锁定注册阶段。
 *
 * @details
 * 锁定后不允许继续注册新的发布者或订阅者，但仍允许发布和读取消息。
 * 这样运行期不会修改topic/订阅者链表，只剩FreeRTOS Queue收发路径。
 */
void MessageCenterLockRegistration(void);


/**
 * @brief 获取topic调试名称。
 *
 * @param topic_id topic枚举ID。
 * @return 合法topic返回Flash中的只读调试名，非法topic返回"invalid_topic"。
 */
const char* MessageCenterTopicName(MessageCenterTopic_e topic_id);


/**
 * @brief 注册一个topic发布者。
 *
 * @param topic_id topic枚举ID。
 * @param data_len 每条消息的字节数，通常传入sizeof(消息结构体)。
 * @return 成功返回发布者句柄，失败返回NULL。
 *
 * @note 同一个topic允许重复注册发布者，但消息长度必须一致。重复注册会返回同一个topic句柄。
 */
MessageCenterPublisher_t* MessageCenterRegisterPublisher(MessageCenterTopic_e topic_id, size_t data_len);


/**
 * @brief 注册一个topic订阅者。
 *
 * @param topic_id    topic枚举ID。
 * @param data_len    每条消息的字节数，必须与同一topic ID的发布者一致。
 * @param queue_depth 该订阅者的Queue深度，传0时使用MESSAGE_CENTER_DEFAULT_QUEUE_DEPTH。
 * @return 成功返回订阅者句柄，失败返回NULL。
 *
 * @note 订阅者可以早于发布者注册；此时会先创建topic，发布者后续注册时会绑定到同一topic。
 */
MessageCenterSubscriber_t* MessageCenterRegisterSubscriber(MessageCenterTopic_e topic_id,
                                                           size_t data_len,
                                                           uint8_t queue_depth);


/**
 * @brief 向topic发布一条消息。
 *
 * @param publisher 发布者句柄。
 * @param data      待发布消息地址，必须为非空指针。
 * @return 成功写入的订阅者Queue数量。返回0表示无订阅者或参数/上下文非法。
 *
 * @note
 * Queue满时会丢弃最旧消息并写入新消息，因此控制类topic总是尽量保留最新帧。
 * 本接口只允许在任务上下文或调度器启动前调用，不允许在ISR中调用。
 */
uint8_t MessageCenterPublish(MessageCenterPublisher_t* publisher, const void* data);


/**
 * @brief 从订阅者Queue读取一条消息。
 *
 * @param subscriber 订阅者句柄。
 * @param data       接收消息的缓冲区，必须为非空指针。
 * @return 1表示读取到新消息，0表示无消息或参数/上下文非法。
 *
 * @note 本接口只允许在任务上下文或调度器启动前调用，不允许在ISR中调用。
 */
uint8_t MessageCenterFetch(MessageCenterSubscriber_t* subscriber, void* data);


/**
 * @brief 获取订阅者Queue中等待读取的消息数量。
 *
 * @param subscriber 订阅者句柄。
 * @return 等待读取的消息数量；参数非法时返回0。
 */
uint8_t MessageCenterPendingCount(const MessageCenterSubscriber_t* subscriber);


/**
 * @brief 获取订阅者因为Queue满而丢弃的旧消息数量。
 *
 * @param subscriber 订阅者句柄。
 * @return 丢弃计数；参数非法时返回0。
 */
uint32_t MessageCenterDroppedCount(const MessageCenterSubscriber_t* subscriber);

#endif /* MESSAGE_CENTER_H */
