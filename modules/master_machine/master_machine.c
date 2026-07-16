#include "master_machine.h"

#include "bsp_log.h"
#include "bsp_usart.h"

#include <string.h>

#define MASTER_MACHINE_USART_RECEIVE_SIZE 256U /**< 单次DMA/IDLE接收缓冲，等于当前BSP上限。 */
#define MASTER_MACHINE_STREAM_BUFFER_SIZE 512U /**< 可同时容纳多个完整帧和一个跨回调残帧。 */

/** 最新Command双缓冲通道。 */
typedef struct
{
    MasterCommandPayload slot[2];
    volatile uint8_t active_index;
    volatile uint8_t has_data;
    volatile uint32_t generation;
    volatile uint32_t last_update_tick;
} MasterMachineCommandChannel_s;

/** 最新Vision双缓冲通道。 */
typedef struct
{
    MasterVisionPayload slot[2];
    volatile uint8_t active_index;
    volatile uint8_t has_data;
    volatile uint32_t generation;
    volatile uint32_t last_update_tick;
} MasterMachineVisionChannel_s;

/** 每种type独立维护的ID连续性状态。 */
typedef struct
{
    uint8_t has_id;
    uint32_t last_id;
} MasterMachineIdTracker_s;

static USARTInstance* master_machine_usart;
static MasterMachine_Init_Config_s master_machine_config;
static MasterMachineCommandChannel_s master_machine_command_channel;
static MasterMachineVisionChannel_s master_machine_vision_channel;
static MasterEventPayload master_machine_event_queue[MASTER_MACHINE_EVENT_QUEUE_CAPACITY];
static volatile uint8_t master_machine_event_read_index;
static volatile uint8_t master_machine_event_write_index;
static volatile uint8_t master_machine_event_count;
static volatile uint8_t master_machine_emergency_stop_latched;
static uint8_t master_machine_stream_buffer[MASTER_MACHINE_STREAM_BUFFER_SIZE];
static uint16_t master_machine_stream_length;
static MasterMachineIdTracker_s master_machine_id_tracker[3];
static MasterMachineStatistics_s master_machine_statistics;
static volatile uint32_t master_machine_last_valid_frame_tick;
static volatile uint8_t master_machine_has_valid_frame;
static volatile uint8_t master_machine_initialized;

/**
 * @brief 进入极短临界区，保护任务之间共享的快照索引和FIFO索引。
 *
 * @param primask 输出进入临界区前的中断屏蔽状态。
 */
static void MasterMachineEnterCritical(uint32_t* primask)
{
    if (primask == NULL)
    {
        return;
    }

    *primask = __get_PRIMASK();
    __disable_irq();
}

/**
 * @brief 恢复进入临界区前的中断屏蔽状态。
 *
 * @param primask MasterMachineEnterCritical()保存的状态。
 */
static void MasterMachineExitCritical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

/**
 * @brief 判断一个HAL毫秒时间戳是否仍在超时窗口内。
 *
 * @param last_tick 最后更新时间。
 * @param timeout_ms 允许的最大间隔。
 * @return uint8_t 未超时返回1，否则返回0。
 */
static uint8_t MasterMachineTickIsFresh(uint32_t last_tick, uint32_t timeout_ms)
{
    /* 无符号减法天然兼容HAL_GetTick()约49.7天一次的回绕。 */
    return ((uint32_t)(HAL_GetTick() - last_tick) <= timeout_ms) ? 1U : 0U;
}

/**
 * @brief 把协议type映射到0~2的内部数组下标。
 *
 * @param type 协议帧类型。
 * @return uint8_t 合法类型返回0~2，非法类型返回0xFF。
 */
static uint8_t MasterMachineTypeToIndex(uint8_t type)
{
    if (type < MASTER_PROTOCOL_TYPE_COMMAND || type > MASTER_PROTOCOL_TYPE_EVENT)
    {
        return 0xFFU;
    }

    return (uint8_t)(type - MASTER_PROTOCOL_TYPE_COMMAND);
}

/**
 * @brief 获取指定type对应的统计结构体。
 *
 * @param type 协议帧类型。
 * @return MasterMachineTypeStatistics_s* 合法类型对应的统计对象，非法类型返回NULL。
 */
static MasterMachineTypeStatistics_s* MasterMachineGetTypeStatistics(uint8_t type)
{
    switch (type)
    {
    case MASTER_PROTOCOL_TYPE_COMMAND:
        return &master_machine_statistics.command;

    case MASTER_PROTOCOL_TYPE_VISION:
        return &master_machine_statistics.vision;

    case MASTER_PROTOCOL_TYPE_EVENT:
        return &master_machine_statistics.event;

    default:
        return NULL;
    }
}

/**
 * @brief 校验并更新某种帧类型的递增ID状态。
 *
 * @note 使用模2^32差值判断，既允许uint32_t自然回绕，也能识别重复和反向乱序。
 * @param type 协议帧类型。
 * @param id 当前帧ID。
 * @return uint8_t 应接受并发布该帧返回1；重复或乱序返回0。
 */
static uint8_t MasterMachineAcceptFrameId(uint8_t type, uint32_t id)
{
    uint8_t index;
    uint32_t forward_distance;
    MasterMachineIdTracker_s* tracker;
    MasterMachineTypeStatistics_s* statistics;

    index = MasterMachineTypeToIndex(type);
    statistics = MasterMachineGetTypeStatistics(type);
    if (index == 0xFFU || statistics == NULL)
    {
        return 0U;
    }

    tracker = &master_machine_id_tracker[index];
    if (tracker->has_id == 0U)
    {
        tracker->has_id = 1U;
        tracker->last_id = id;
        return 1U;
    }

    forward_distance = id - tracker->last_id;
    if (forward_distance == 0U)
    {
        statistics->duplicate_count++;
        return 0U;
    }

    /* 小于半个uint32_t范围视为向前推进，大于等于半范围视为旧帧或乱序帧。 */
    if (forward_distance >= 0x80000000UL)
    {
        statistics->out_of_order_count++;
        return 0U;
    }

    if (forward_distance > 1U)
    {
        statistics->estimated_lost_count += forward_distance - 1U;
    }

    tracker->last_id = id;
    return 1U;
}

/**
 * @brief 发布最新Command到双缓冲通道。
 *
 * @param command 已完成协议校验和ID检查的Command。
 */
static void MasterMachinePublishCommand(const MasterCommandPayload* command)
{
    uint8_t inactive_index;
    uint32_t primask;

    inactive_index = (uint8_t)(master_machine_command_channel.active_index ^ 1U);
    memcpy(&master_machine_command_channel.slot[inactive_index], command, sizeof(*command));

    MasterMachineEnterCritical(&primask);
    master_machine_command_channel.active_index = inactive_index;
    master_machine_command_channel.last_update_tick = HAL_GetTick();
    master_machine_command_channel.generation++;
    if (master_machine_command_channel.generation == 0U)
    {
        /* generation=0专门表示消费者尚未读过数据，回绕时主动跳过0。 */
        master_machine_command_channel.generation++;
    }
    master_machine_command_channel.has_data = 1U;
    master_machine_statistics.command.accepted_count++;
    MasterMachineExitCritical(primask);
}

/**
 * @brief 发布最新Vision到双缓冲通道。
 *
 * @param vision 已完成协议校验和ID检查的Vision。
 */
static void MasterMachinePublishVision(const MasterVisionPayload* vision)
{
    uint8_t inactive_index;
    uint32_t primask;

    inactive_index = (uint8_t)(master_machine_vision_channel.active_index ^ 1U);
    memcpy(&master_machine_vision_channel.slot[inactive_index], vision, sizeof(*vision));

    MasterMachineEnterCritical(&primask);
    master_machine_vision_channel.active_index = inactive_index;
    master_machine_vision_channel.last_update_tick = HAL_GetTick();
    master_machine_vision_channel.generation++;
    if (master_machine_vision_channel.generation == 0U)
    {
        /* generation=0专门表示消费者尚未读过数据，回绕时主动跳过0。 */
        master_machine_vision_channel.generation++;
    }
    master_machine_vision_channel.has_data = 1U;
    master_machine_statistics.vision.accepted_count++;
    MasterMachineExitCritical(primask);
}

/**
 * @brief 将合法Event加入FIFO，并独立处理紧急停机锁存。
 *
 * @param event_payload 已完成协议校验和ID检查的Event。
 */
static void MasterMachinePublishEvent(const MasterEventPayload* event_payload)
{
    uint32_t primask;

    MasterMachineEnterCritical(&primask);

    /* 急停是安全状态，必须先锁存，不能受事件队列是否已满影响。 */
    if (event_payload->event == MASTER_PROTOCOL_EVENT_EMERGENCY_STOP)
    {
        master_machine_emergency_stop_latched = 1U;
    }

    if (master_machine_event_count >= MASTER_MACHINE_EVENT_QUEUE_CAPACITY)
    {
        /* 保留队列中尚未处理的旧事件，丢弃新到事件并留下可诊断计数。 */
        master_machine_statistics.event_queue_overflow_count++;
        MasterMachineExitCritical(primask);
        return;
    }

    master_machine_event_queue[master_machine_event_write_index] = *event_payload;
    master_machine_event_write_index =
        (uint8_t)((master_machine_event_write_index + 1U) % MASTER_MACHINE_EVENT_QUEUE_CAPACITY);
    master_machine_event_count++;
    master_machine_statistics.event.accepted_count++;

    MasterMachineExitCritical(primask);
}

/**
 * @brief 从payload中提取ID，并执行分类型连续性判断。
 *
 * @param type 协议帧类型。
 * @param payload 已通过语义校验的payload。
 * @return uint8_t 允许发布返回1，重复或乱序返回0。
 */
static uint8_t MasterMachinePayloadIdIsAccepted(uint8_t type, const uint8_t* payload)
{
    uint32_t id;

    /* 三种payload中的id都位于固定偏移6，使用memcpy避免未对齐访问。 */
    memcpy(&id, &payload[6], sizeof(id));
    return MasterMachineAcceptFrameId(type, id);
}

/**
 * @brief 将一帧已经完整校验的数据分发到对应的数据通道。
 *
 * @param type 协议帧类型。
 * @param payload payload原始字节。
 */
static void MasterMachineDispatchFrame(uint8_t type, const uint8_t* payload)
{
    MasterCommandPayload command;
    MasterVisionPayload vision;
    MasterEventPayload event_payload;
    uint32_t primask;

    /* 合法但重复/乱序的帧仍会刷新bridge在线时间，但不会覆盖上层当前数据。 */
    MasterMachineEnterCritical(&primask);
    master_machine_last_valid_frame_tick = HAL_GetTick();
    master_machine_has_valid_frame = 1U;
    master_machine_statistics.valid_frame_count++;
    MasterMachineExitCritical(primask);

    if (MasterMachinePayloadIdIsAccepted(type, payload) == 0U)
    {
        return;
    }

    switch (type)
    {
    case MASTER_PROTOCOL_TYPE_COMMAND:
        memcpy(&command, payload, sizeof(command));
        MasterMachinePublishCommand(&command);
        break;

    case MASTER_PROTOCOL_TYPE_VISION:
        memcpy(&vision, payload, sizeof(vision));
        MasterMachinePublishVision(&vision);
        break;

    case MASTER_PROTOCOL_TYPE_EVENT:
        memcpy(&event_payload, payload, sizeof(event_payload));
        MasterMachinePublishEvent(&event_payload);
        break;

    default:
        /* type已经在解析阶段检查，正常情况下不会到达这里。 */
        break;
    }
}

/**
 * @brief 在流缓存中查找并解析尽可能多的完整帧。
 *
 * @note 遇到噪声、未知type、错误长度或错误checksum时只前移一个字节重新搜索AA 55，
 *       从而支持粘包、拆包、线路噪声以及错误帧后的自动重同步。
 */
static void MasterMachineParseStream(void)
{
    uint16_t cursor;
    uint16_t payload_length;
    uint16_t expected_length;
    uint16_t complete_frame_length;
    uint16_t remaining_length;
    uint8_t type;
    uint8_t received_checksum;
    uint8_t calculated_checksum;
    const uint8_t* payload;
    MasterProtocolPayloadStatus_e payload_status;

    cursor = 0U;

    while ((uint16_t)(master_machine_stream_length - cursor) >= 2U)
    {
        /* 先寻找连续帧头；跳过的所有字节都记为线路噪声或重同步丢弃。 */
        if (master_machine_stream_buffer[cursor] != MASTER_PROTOCOL_FRAME_HEAD_1 ||
            master_machine_stream_buffer[cursor + 1U] != MASTER_PROTOCOL_FRAME_HEAD_2)
        {
            cursor++;
            master_machine_statistics.discarded_noise_byte_count++;
            continue;
        }

        remaining_length = (uint16_t)(master_machine_stream_length - cursor);
        if (remaining_length < MASTER_PROTOCOL_HEADER_SIZE)
        {
            break;
        }

        type = master_machine_stream_buffer[cursor + 2U];
        expected_length = MasterProtocolGetPayloadSize(type);
        if (expected_length == 0U)
        {
            master_machine_statistics.unknown_type_count++;
            cursor++;
            master_machine_statistics.discarded_noise_byte_count++;
            continue;
        }

        payload_length = (uint16_t)master_machine_stream_buffer[cursor + 3U] |
            ((uint16_t)master_machine_stream_buffer[cursor + 4U] << 8U);
        if (payload_length != expected_length || payload_length > MASTER_PROTOCOL_MAX_PAYLOAD_SIZE)
        {
            master_machine_statistics.length_error_count++;
            cursor++;
            master_machine_statistics.discarded_noise_byte_count++;
            continue;
        }

        complete_frame_length = (uint16_t)(payload_length + MASTER_PROTOCOL_FRAME_OVERHEAD);
        if (remaining_length < complete_frame_length)
        {
            /* 帧被USART IDLE切开，保留残帧等待下一次回调继续追加。 */
            break;
        }

        payload = &master_machine_stream_buffer[cursor + MASTER_PROTOCOL_HEADER_SIZE];
        received_checksum = master_machine_stream_buffer[cursor + complete_frame_length - 1U];
        calculated_checksum = MasterProtocolCalculateChecksum(type, payload_length, payload);
        if (received_checksum != calculated_checksum)
        {
            master_machine_statistics.checksum_error_count++;
            cursor++;
            master_machine_statistics.discarded_noise_byte_count++;
            continue;
        }

        payload_status = MasterProtocolValidatePayload(type, payload, payload_length);
        if (payload_status != MASTER_PROTOCOL_PAYLOAD_VALID)
        {
            /* checksum正确说明帧边界可信，语义错误时直接丢弃整帧，避免把payload中的AA55误当帧头。 */
            master_machine_statistics.semantic_error_count++;
            cursor = (uint16_t)(cursor + complete_frame_length);
            continue;
        }

        MasterMachineDispatchFrame(type, payload);
        cursor = (uint16_t)(cursor + complete_frame_length);
    }

    if (cursor > 0U)
    {
        remaining_length = (uint16_t)(master_machine_stream_length - cursor);
        if (remaining_length > 0U)
        {
            memmove(master_machine_stream_buffer,
                    &master_machine_stream_buffer[cursor],
                    remaining_length);
        }
        master_machine_stream_length = remaining_length;
    }
}

/**
 * @brief 向协议流缓存追加USART本次收到的片段。
 *
 * @param data 接收片段首地址。
 * @param length 接收片段长度。
 */
static void MasterMachineAppendStream(const uint8_t* data, uint16_t length)
{
    uint16_t overflow_length;

    if (data == NULL || length == 0U)
    {
        return;
    }

    master_machine_statistics.received_byte_count += length;

    /* 正常情况下残帧最多130字节，再追加一次256字节接收片段不会超过512字节。 */
    if ((uint32_t)master_machine_stream_length + length > MASTER_MACHINE_STREAM_BUFFER_SIZE)
    {
        MasterMachineParseStream();
    }

    if ((uint32_t)master_machine_stream_length + length > MASTER_MACHINE_STREAM_BUFFER_SIZE)
    {
        overflow_length = (uint16_t)((uint32_t)master_machine_stream_length + length -
            MASTER_MACHINE_STREAM_BUFFER_SIZE);
        if (overflow_length >= master_machine_stream_length)
        {
            master_machine_statistics.discarded_noise_byte_count += master_machine_stream_length;
            master_machine_stream_length = 0U;
        }
        else
        {
            memmove(master_machine_stream_buffer,
                    &master_machine_stream_buffer[overflow_length],
                    master_machine_stream_length - overflow_length);
            master_machine_stream_length = (uint16_t)(master_machine_stream_length - overflow_length);
            master_machine_statistics.discarded_noise_byte_count += overflow_length;
        }
        master_machine_statistics.stream_overflow_count++;
    }

    /* BSP单次回调最大256字节，因此经过上面的空间整理后一定可以完整追加。 */
    memcpy(&master_machine_stream_buffer[master_machine_stream_length], data, length);
    master_machine_stream_length = (uint16_t)(master_machine_stream_length + length);
    MasterMachineParseStream();
}

/**
 * @brief BSP USART在任务上下文调用的接收回调。
 */
static void MasterMachineReceiveCallback(void)
{
    if (master_machine_usart == NULL ||
        master_machine_usart->recv_buff == NULL ||
        master_machine_usart->recv_len == 0U)
    {
        return;
    }

    MasterMachineAppendStream(master_machine_usart->recv_buff,
                              master_machine_usart->recv_len);
}

/**
 * @brief 初始化ESP32-S3上位机通信模块。
 */
uint8_t MasterMachineInit(const MasterMachine_Init_Config_s* config)
{
    USART_Init_Config_s usart_config;

    if (config == NULL || config->uart_handle == NULL)
    {
        LOGERROR("[master_machine] init with invalid config");
        return 0U;
    }

    if (master_machine_initialized != 0U)
    {
        LOGERROR("[master_machine] repeated initialization is not allowed");
        return 0U;
    }

    memset(&master_machine_command_channel, 0, sizeof(master_machine_command_channel));
    memset(&master_machine_vision_channel, 0, sizeof(master_machine_vision_channel));
    memset(master_machine_event_queue, 0, sizeof(master_machine_event_queue));
    memset(master_machine_stream_buffer, 0, sizeof(master_machine_stream_buffer));
    memset(master_machine_id_tracker, 0, sizeof(master_machine_id_tracker));
    memset(&master_machine_statistics, 0, sizeof(master_machine_statistics));

    master_machine_event_read_index = 0U;
    master_machine_event_write_index = 0U;
    master_machine_event_count = 0U;
    master_machine_emergency_stop_latched = 0U;
    master_machine_stream_length = 0U;
    master_machine_last_valid_frame_tick = 0U;
    master_machine_has_valid_frame = 0U;

    master_machine_config = *config;
    if (master_machine_config.bridge_timeout_ms == 0U)
    {
        master_machine_config.bridge_timeout_ms = MASTER_MACHINE_DEFAULT_BRIDGE_TIMEOUT_MS;
    }
    if (master_machine_config.command_timeout_ms == 0U)
    {
        master_machine_config.command_timeout_ms = MASTER_MACHINE_DEFAULT_COMMAND_TIMEOUT_MS;
    }
    if (master_machine_config.vision_timeout_ms == 0U)
    {
        master_machine_config.vision_timeout_ms = MASTER_MACHINE_DEFAULT_VISION_TIMEOUT_MS;
    }

    memset(&usart_config, 0, sizeof(usart_config));
    usart_config.usart_handle = master_machine_config.uart_handle;
    usart_config.recv_buff_size = MASTER_MACHINE_USART_RECEIVE_SIZE;
    usart_config.module_callback = MasterMachineReceiveCallback;

    master_machine_usart = USARTRegister(&usart_config);
    if (master_machine_usart == NULL)
    {
        LOGERROR("[master_machine] USART register failed");
        return 0U;
    }

    master_machine_initialized = 1U;
    LOGINFO("[master_machine] ESP32-S3 UART receiver initialized");
    return 1U;
}

/**
 * @brief 判断模块是否已经成功初始化。
 */
uint8_t MasterMachineIsInitialized(void)
{
    return master_machine_initialized;
}

/**
 * @brief 判断ESP32-S3桥接端是否在线。
 */
uint8_t MasterMachineIsBridgeOnline(void)
{
    uint8_t has_valid_frame;
    uint32_t last_tick;
    uint32_t primask;

    if (master_machine_initialized == 0U)
    {
        return 0U;
    }

    MasterMachineEnterCritical(&primask);
    has_valid_frame = master_machine_has_valid_frame;
    last_tick = master_machine_last_valid_frame_tick;
    MasterMachineExitCritical(primask);

    if (has_valid_frame == 0U)
    {
        return 0U;
    }

    return MasterMachineTickIsFresh(last_tick,
                                    master_machine_config.bridge_timeout_ms);
}

/**
 * @brief 判断最新Command是否仍在配置的新鲜度时间内。
 */
uint8_t MasterMachineIsCommandFresh(void)
{
    uint8_t has_data;
    uint32_t last_tick;
    uint32_t primask;

    MasterMachineEnterCritical(&primask);
    has_data = master_machine_command_channel.has_data;
    last_tick = master_machine_command_channel.last_update_tick;
    MasterMachineExitCritical(primask);

    if (has_data == 0U)
    {
        return 0U;
    }

    return MasterMachineTickIsFresh(last_tick,
                                    master_machine_config.command_timeout_ms);
}

/**
 * @brief 判断最新Vision是否仍在配置的新鲜度时间内。
 */
uint8_t MasterMachineIsVisionFresh(void)
{
    uint8_t has_data;
    uint32_t last_tick;
    uint32_t primask;

    MasterMachineEnterCritical(&primask);
    has_data = master_machine_vision_channel.has_data;
    last_tick = master_machine_vision_channel.last_update_tick;
    MasterMachineExitCritical(primask);

    if (has_data == 0U)
    {
        return 0U;
    }

    return MasterMachineTickIsFresh(last_tick,
                                    master_machine_config.vision_timeout_ms);
}

/**
 * @brief 按消费者游标读取一份新的Command快照。
 */
uint8_t MasterMachineReadCommand(MasterCommandPayload* command, MasterMachineCursor_s* cursor)
{
    uint32_t primask;
    uint32_t generation;

    if (command == NULL || cursor == NULL)
    {
        return 0U;
    }

    MasterMachineEnterCritical(&primask);
    generation = master_machine_command_channel.generation;
    if (master_machine_command_channel.has_data == 0U || cursor->generation == generation)
    {
        MasterMachineExitCritical(primask);
        return 0U;
    }

    memcpy(command,
           &master_machine_command_channel.slot[master_machine_command_channel.active_index],
           sizeof(*command));
    cursor->generation = generation;
    MasterMachineExitCritical(primask);
    return 1U;
}

/**
 * @brief 无条件读取当前最新Command快照。
 */
uint8_t MasterMachinePeekCommand(MasterCommandPayload* command)
{
    uint32_t primask;

    if (command == NULL)
    {
        return 0U;
    }

    MasterMachineEnterCritical(&primask);
    if (master_machine_command_channel.has_data == 0U)
    {
        MasterMachineExitCritical(primask);
        return 0U;
    }

    memcpy(command,
           &master_machine_command_channel.slot[master_machine_command_channel.active_index],
           sizeof(*command));
    MasterMachineExitCritical(primask);
    return 1U;
}

/**
 * @brief 按消费者游标读取一份新的Vision快照。
 */
uint8_t MasterMachineReadVision(MasterVisionPayload* vision, MasterMachineCursor_s* cursor)
{
    uint32_t primask;
    uint32_t generation;

    if (vision == NULL || cursor == NULL)
    {
        return 0U;
    }

    MasterMachineEnterCritical(&primask);
    generation = master_machine_vision_channel.generation;
    if (master_machine_vision_channel.has_data == 0U || cursor->generation == generation)
    {
        MasterMachineExitCritical(primask);
        return 0U;
    }

    memcpy(vision,
           &master_machine_vision_channel.slot[master_machine_vision_channel.active_index],
           sizeof(*vision));
    cursor->generation = generation;
    MasterMachineExitCritical(primask);
    return 1U;
}

/**
 * @brief 无条件读取当前最新Vision快照。
 */
uint8_t MasterMachinePeekVision(MasterVisionPayload* vision)
{
    uint32_t primask;

    if (vision == NULL)
    {
        return 0U;
    }

    MasterMachineEnterCritical(&primask);
    if (master_machine_vision_channel.has_data == 0U)
    {
        MasterMachineExitCritical(primask);
        return 0U;
    }

    memcpy(vision,
           &master_machine_vision_channel.slot[master_machine_vision_channel.active_index],
           sizeof(*vision));
    MasterMachineExitCritical(primask);
    return 1U;
}

/**
 * @brief 从离散事件FIFO中取出最早尚未处理的事件。
 */
uint8_t MasterMachinePopEvent(MasterEventPayload* event_payload)
{
    uint32_t primask;

    if (event_payload == NULL)
    {
        return 0U;
    }

    MasterMachineEnterCritical(&primask);
    if (master_machine_event_count == 0U)
    {
        MasterMachineExitCritical(primask);
        return 0U;
    }

    *event_payload = master_machine_event_queue[master_machine_event_read_index];
    master_machine_event_read_index =
        (uint8_t)((master_machine_event_read_index + 1U) % MASTER_MACHINE_EVENT_QUEUE_CAPACITY);
    master_machine_event_count--;
    MasterMachineExitCritical(primask);
    return 1U;
}

/**
 * @brief 获取当前等待处理的事件数量。
 */
uint8_t MasterMachineGetPendingEventCount(void)
{
    uint8_t count;
    uint32_t primask;

    MasterMachineEnterCritical(&primask);
    count = master_machine_event_count;
    MasterMachineExitCritical(primask);
    return count;
}

/**
 * @brief 查询紧急停机锁存状态。
 */
uint8_t MasterMachineIsEmergencyStopLatched(void)
{
    return master_machine_emergency_stop_latched;
}

/**
 * @brief 由上层在完成安全复位流程后显式清除紧急停机锁存。
 */
void MasterMachineClearEmergencyStop(void)
{
    uint32_t primask;

    MasterMachineEnterCritical(&primask);
    master_machine_emergency_stop_latched = 0U;
    MasterMachineExitCritical(primask);
}

/**
 * @brief 获取运行统计的线程安全快照。
 */
uint8_t MasterMachineGetStatistics(MasterMachineStatistics_s* statistics)
{
    uint32_t primask;

    if (statistics == NULL)
    {
        return 0U;
    }

    MasterMachineEnterCritical(&primask);
    *statistics = master_machine_statistics;
    if (master_machine_usart != NULL)
    {
        statistics->uart_error_count = USARTGetErrorCount(master_machine_usart);
    }
    MasterMachineExitCritical(primask);
    return 1U;
}
