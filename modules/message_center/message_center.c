#include "message_center.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "bsp_log.h"
#include "main.h"
#include "string.h"

/**
 * @brief topic发布者/话题节点。
 *
 * @details
 * 当前实现把“发布者句柄”和“topic节点”合并。多个APP重复注册同一topic ID发布者时，
 * 会拿到同一个节点指针。这样做可以避免为相同topic维护多个冗余节点。
 */
struct MessageCenterPublisher
{
    uint8_t used;                                                       /*!< 当前静态池槽位是否已经被占用 */
    uint8_t publisher_registered;                                       /*!< 是否已经有发布者显式注册过该topic */
    uint16_t data_len;                                                  /*!< 该topic每条消息的有效字节数 */
    MessageCenterSubscriber_t* first_subscriber;                        /*!< 订阅该topic的第一个订阅者 */
};

/**
 * @brief topic订阅者节点。
 *
 * @details
 * 每个订阅者都有一套独立的FreeRTOS静态Queue。发布者向同一topic发布时，会把
 * 消息分别写入所有订阅者Queue，因此不同订阅者之间互不抢数据。
 */
struct MessageCenterSubscriber
{
    uint8_t used;                                                       /*!< 当前静态池槽位是否已经被占用 */
    uint8_t queue_depth;                                                /*!< 本订阅者Queue实际深度 */
    uint16_t data_len;                                                  /*!< 每条消息的有效字节数 */
    uint16_t queue_item_size;                                           /*!< FreeRTOS Queue item大小，等于topic消息长度 */
    QueueHandle_t queue;                                                /*!< FreeRTOS静态Queue句柄 */
    StaticQueue_t queue_cb;                                             /*!< FreeRTOS静态Queue控制块 */
    uint8_t queue_storage[MESSAGE_CENTER_MAX_QUEUE_DEPTH]
                         [MESSAGE_CENTER_MAX_MESSAGE_BYTES];           /*!< Queue底层存储区 */
    MessageCenterSubscriber_t* next_subscriber;                         /*!< 同topic下一个订阅者 */
    uint32_t dropped_count;                                             /*!< Queue满时丢弃旧消息的计数 */
};

static MessageCenterPublisher_t publisher_pool[MESSAGE_TOPIC_COUNT];
static MessageCenterSubscriber_t subscriber_pool[MESSAGE_CENTER_MAX_SUBSCRIBER_COUNT];
static uint8_t message_center_initialized;
static uint8_t registration_locked;

/**
 * @brief topic调试名表。
 *
 * @details
 * 表和字符串常量均为const，只用于日志和调试查看，不在每个topic节点中占用RAM。
 * 枚举顺序必须与MessageCenterTopic_e保持一致。
 */
static const char* const topic_debug_names[MESSAGE_TOPIC_COUNT] = {
    [MESSAGE_TOPIC_GIMBAL_CMD] = "gimbal_cmd",
    [MESSAGE_TOPIC_GIMBAL_FEED] = "gimbal_feed",
    [MESSAGE_TOPIC_SHOOT_CMD] = "shoot_cmd",
    [MESSAGE_TOPIC_SHOOT_FEED] = "shoot_feed",
    [MESSAGE_TOPIC_CHASSIS_CMD] = "chassis_cmd",
    [MESSAGE_TOPIC_CHASSIS_FEED] = "chassis_feed",
};

/**
 * @brief 检查topic ID是否合法。
 *
 * @param topic_id topic枚举ID。
 * @return 1表示合法，0表示非法。
 */
static uint8_t MessageCenterCheckTopicId(MessageCenterTopic_e topic_id)
{
    if ((uint32_t)topic_id >= (uint32_t)MESSAGE_TOPIC_COUNT)
    {
        LOGERROR("[message_center] invalid topic id:%u", (unsigned int)topic_id);
        return 0U;
    }

    return 1U;
}

/**
 * @brief 检查消息长度是否合法。
 *
 * @param data_len 消息有效字节数。
 * @return 1表示合法，0表示非法。
 */
static uint8_t MessageCenterCheckDataLen(size_t data_len)
{
    if (data_len == 0U)
    {
        LOGERROR("[message_center] message len must be greater than 0");
        return 0U;
    }

    if (data_len > MESSAGE_CENTER_MAX_MESSAGE_BYTES)
    {
        LOGERROR("[message_center] message too large:%u/%u",
                 (unsigned int)data_len,
                 (unsigned int)MESSAGE_CENTER_MAX_MESSAGE_BYTES);
        return 0U;
    }

    return 1U;
}

/**
 * @brief 检查消息中心是否已经显式初始化。
 *
 * @return 1表示已经初始化，0表示尚未初始化。
 */
static uint8_t MessageCenterCheckInitialized(void)
{
    if (message_center_initialized == 0U)
    {
        LOGERROR("[message_center] register before MessageCenterInit");
        return 0U;
    }

    return 1U;
}

/**
 * @brief 获取topic节点。
 *
 * @param topic_id topic枚举ID。
 * @return 合法topic返回节点指针，否则返回NULL。
 */
static MessageCenterPublisher_t* MessageCenterGetTopic(MessageCenterTopic_e topic_id)
{
    if (MessageCenterCheckTopicId(topic_id) == 0U)
    {
        return NULL;
    }

    return &publisher_pool[(uint32_t)topic_id];
}

/**
 * @brief 初始化或获取topic节点。
 *
 * @param topic_id topic枚举ID。
 * @param data_len 消息有效字节数。
 * @return 成功返回topic节点指针，失败返回NULL。
 */
static MessageCenterPublisher_t* MessageCenterPrepareTopic(MessageCenterTopic_e topic_id, size_t data_len)
{
    MessageCenterPublisher_t* topic = MessageCenterGetTopic(topic_id);

    if (topic == NULL)
    {
        return NULL;
    }

    if (topic->used == 0U)
    {
        topic->used = 1U;
        topic->publisher_registered = 0U;
        topic->data_len = (uint16_t)data_len;
        topic->first_subscriber = NULL;
        return topic;
    }

    if (topic->data_len != data_len)
    {
        LOGERROR("[message_center] topic [%s] data len mismatch:%u/%u",
                 MessageCenterTopicName(topic_id),
                 (unsigned int)data_len,
                 (unsigned int)topic->data_len);
        return NULL;
    }

    return topic;
}

/**
 * @brief 从静态池分配一个订阅者节点。
 *
 * @return 成功返回订阅者节点指针，失败返回NULL。
 */
static MessageCenterSubscriber_t* MessageCenterAllocSubscriber(void)
{
    for (uint8_t i = 0U; i < MESSAGE_CENTER_MAX_SUBSCRIBER_COUNT; ++i)
    {
        if (subscriber_pool[i].used == 0U)
        {
            subscriber_pool[i].used = 1U; // 先占用槽位，避免后续初始化过程被重复分配
            return &subscriber_pool[i];
        }
    }

    LOGERROR("[message_center] subscriber pool exhausted:%u",
             (unsigned int)MESSAGE_CENTER_MAX_SUBSCRIBER_COUNT);
    return NULL;
}

/**
 * @brief 规范化订阅者Queue深度。
 *
 * @param queue_depth 调用者请求的Queue深度。
 * @return 合法Queue深度。
 */
static uint8_t MessageCenterNormalizeQueueDepth(uint8_t queue_depth)
{
    if (queue_depth == 0U)
    {
        return MESSAGE_CENTER_DEFAULT_QUEUE_DEPTH;
    }

    if (queue_depth > MESSAGE_CENTER_MAX_QUEUE_DEPTH)
    {
        return MESSAGE_CENTER_MAX_QUEUE_DEPTH;
    }

    return queue_depth;
}

/**
 * @brief 将订阅者挂到topic的订阅者链表尾部。
 *
 * @param topic      topic节点。
 * @param subscriber 订阅者节点。
 */
static void MessageCenterAppendSubscriber(MessageCenterPublisher_t* topic,
                                          MessageCenterSubscriber_t* subscriber)
{
    MessageCenterSubscriber_t* iter;

    if (topic->first_subscriber == NULL)
    {
        topic->first_subscriber = subscriber;
        return;
    }

    iter = topic->first_subscriber;
    while (iter->next_subscriber != NULL)
    {
        iter = iter->next_subscriber;
    }
    iter->next_subscriber = subscriber;
}

/**
 * @brief 初始化消息中心静态资源。
 *
 * @details
 * 清空topic池和订阅者池，恢复到可注册状态。该函数应在APP注册发布者/订阅者
 * 之前调用。注册阶段锁定后再次调用会被拒绝，避免运行期破坏Queue。
 */
void MessageCenterInit(void)
{
    // 运行期已经锁定注册后，不允许重新清空消息中心，避免破坏正在使用的Queue。
    if (registration_locked != 0U)
    {
        LOGERROR("[message_center] init after registration locked");
        return;
    }

    memset(publisher_pool, 0, sizeof(publisher_pool));
    memset(subscriber_pool, 0, sizeof(subscriber_pool));
    message_center_initialized = 1U;
    registration_locked = 0U;
}

/**
 * @brief 锁定消息中心注册阶段。
 *
 * @details
 * 锁定后不再允许注册新的发布者或订阅者。运行期只保留FreeRTOS Queue收发路径，
 * 不再修改topic表和订阅者链表。
 */
void MessageCenterLockRegistration(void)
{
    if (message_center_initialized == 0U)
    {
        LOGERROR("[message_center] lock before MessageCenterInit");
        return;
    }

    registration_locked = 1U;
}

/**
 * @brief 获取topic的只读调试名。
 *
 * @param topic_id topic枚举ID。
 * @return 合法topic返回Flash/只读区中的调试名，非法topic返回"invalid_topic"。
 */
const char* MessageCenterTopicName(MessageCenterTopic_e topic_id)
{
    if ((uint32_t)topic_id >= (uint32_t)MESSAGE_TOPIC_COUNT)
    {
        return "invalid_topic";
    }

    return topic_debug_names[(uint32_t)topic_id];
}

/**
 * @brief 注册一个发布者topic句柄。
 *
 * @param topic_id topic枚举ID。
 * @param data_len 每条消息的有效字节数。
 * @return 成功返回发布者句柄，失败返回NULL。
 */
MessageCenterPublisher_t* MessageCenterRegisterPublisher(MessageCenterTopic_e topic_id, size_t data_len)
{
    MessageCenterPublisher_t* topic;

    if (MessageCenterCheckInitialized() == 0U)
    {
        return NULL;
    }
    if (registration_locked != 0U)
    {
        LOGERROR("[message_center] publisher register after locked");
        return NULL;
    }
    if (MessageCenterCheckTopicId(topic_id) == 0U ||
        MessageCenterCheckDataLen(data_len) == 0U)
    {
        return NULL;
    }

    topic = MessageCenterPrepareTopic(topic_id, data_len);
    if (topic != NULL)
    {
        topic->publisher_registered = 1U;
    }

    return topic;
}

/**
 * @brief 注册一个订阅者并创建对应静态Queue。
 *
 * @param topic_id    topic枚举ID。
 * @param data_len    每条消息的有效字节数。
 * @param queue_depth 订阅者Queue深度，传0时使用默认深度。
 * @return 成功返回订阅者句柄，失败返回NULL。
 */
MessageCenterSubscriber_t* MessageCenterRegisterSubscriber(MessageCenterTopic_e topic_id,
                                                           size_t data_len,
                                                           uint8_t queue_depth)
{
    MessageCenterPublisher_t* topic;
    MessageCenterSubscriber_t* subscriber;

    if (MessageCenterCheckInitialized() == 0U)
    {
        return NULL;
    }
    if (registration_locked != 0U)
    {
        LOGERROR("[message_center] subscriber register after locked");
        return NULL;
    }
    if (MessageCenterCheckTopicId(topic_id) == 0U ||
        MessageCenterCheckDataLen(data_len) == 0U)
    {
        return NULL;
    }

    topic = MessageCenterPrepareTopic(topic_id, data_len);
    if (topic == NULL)
    {
        return NULL;
    }

    subscriber = MessageCenterAllocSubscriber();
    if (subscriber == NULL)
    {
        return NULL;
    }

    // 初始化订阅者运行期字段。消息长度已经在注册阶段保证大于0。
    subscriber->data_len = (uint16_t)data_len;
    subscriber->queue_depth = MessageCenterNormalizeQueueDepth(queue_depth);
    subscriber->queue_item_size = (uint16_t)data_len;
    subscriber->next_subscriber = NULL;
    subscriber->dropped_count = 0U;
    memset(subscriber->queue_storage, 0, sizeof(subscriber->queue_storage));

    subscriber->queue = xQueueCreateStatic(subscriber->queue_depth,
                                           subscriber->queue_item_size,
                                           &subscriber->queue_storage[0U][0U],
                                           &subscriber->queue_cb);
    if (subscriber->queue == NULL)
    {
        LOGERROR("[message_center] queue create failed topic:%s", MessageCenterTopicName(topic_id));
        memset(subscriber, 0, sizeof(*subscriber)); // 创建失败时释放静态池槽位
        return NULL;
    }

    MessageCenterAppendSubscriber(topic, subscriber);
    return subscriber;
}

/**
 * @brief 向topic下所有订阅者发布一条消息。
 *
 * @param publisher 发布者句柄。
 * @param data      待发布消息地址，必须为非空指针。
 * @return 成功写入的订阅者Queue数量。
 */
uint8_t MessageCenterPublish(MessageCenterPublisher_t* publisher, const void* data)
{
    MessageCenterSubscriber_t* subscriber;
    uint8_t pushed_count = 0U;
    uint8_t discard_buffer[MESSAGE_CENTER_MAX_MESSAGE_BYTES];

    if (publisher == NULL || publisher->used == 0U || data == NULL || __get_IPSR() != 0U)
    {
        return 0U;
    }

    subscriber = publisher->first_subscriber;
    while (subscriber != NULL)
    {
        if (subscriber->queue != NULL)
        {
            if (subscriber->queue_depth == 1U)
            {
                // 深度1时使用覆盖语义，天然适合控制命令的“最新帧邮箱”。
                if (xQueueOverwrite(subscriber->queue, data) == pdPASS)
                {
                    pushed_count++;
                }
            }
            else if (xQueueSendToBack(subscriber->queue, data, 0U) == pdPASS)
            {
                pushed_count++;
            }
            else
            {
                // 深度大于1且队列已满时，丢弃最旧帧，再尝试写入最新帧。
                (void)xQueueReceive(subscriber->queue, discard_buffer, 0U);
                subscriber->dropped_count++;
                if (xQueueSendToBack(subscriber->queue, data, 0U) == pdPASS)
                {
                    pushed_count++;
                }
            }
        }

        subscriber = subscriber->next_subscriber;
    }

    return pushed_count;
}

/**
 * @brief 从订阅者Queue读取一条消息。
 *
 * @param subscriber 订阅者句柄。
 * @param data       接收消息的缓冲区，必须为非空指针。
 * @return 1表示读取到消息，0表示无消息或参数非法。
 */
uint8_t MessageCenterFetch(MessageCenterSubscriber_t* subscriber, void* data)
{
    if (subscriber == NULL || subscriber->used == 0U || subscriber->queue == NULL ||
        data == NULL || __get_IPSR() != 0U)
    {
        return 0U;
    }

    return (xQueueReceive(subscriber->queue, data, 0U) == pdPASS) ? 1U : 0U;
}

/**
 * @brief 查询订阅者Queue中等待处理的消息数量。
 *
 * @param subscriber 订阅者句柄。
 * @return 等待读取的消息数量。
 */
uint8_t MessageCenterPendingCount(const MessageCenterSubscriber_t* subscriber)
{
    if (subscriber == NULL || subscriber->used == 0U || subscriber->queue == NULL)
    {
        return 0U;
    }

    return (uint8_t)uxQueueMessagesWaiting(subscriber->queue);
}

/**
 * @brief 查询订阅者丢弃旧消息的次数。
 *
 * @param subscriber 订阅者句柄。
 * @return Queue满时丢弃旧消息的累计次数。
 */
uint32_t MessageCenterDroppedCount(const MessageCenterSubscriber_t* subscriber)
{
    if (subscriber == NULL || subscriber->used == 0U)
    {
        return 0U;
    }

    return subscriber->dropped_count;
}
