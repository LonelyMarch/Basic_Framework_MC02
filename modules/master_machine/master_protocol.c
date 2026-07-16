#include "master_protocol.h"

#include <math.h>
#include <string.h>

/* 编译期锁定线上结构体尺寸，避免字段或对齐方式被无意修改。 */
_Static_assert(sizeof(MasterCommandPayload) == 23U, "MasterCommandPayload size must be 23 bytes");
_Static_assert(sizeof(MasterVisionPayload) == 125U, "MasterVisionPayload size must be 125 bytes");
_Static_assert(sizeof(MasterEventPayload) == 15U, "MasterEventPayload size must be 15 bytes");

/**
 * @brief 判断一个单精度浮点数是否可用于控制或计算。
 *
 * @param value 待检查数值。
 * @return uint8_t 有限值返回1，NaN或正负无穷返回0。
 */
static uint8_t MasterProtocolFloatIsFinite(float value)
{
    return isfinite(value) ? 1U : 0U;
}

/**
 * @brief 校验所有视觉浮点字段，阻止NaN和无穷值进入上层算法。
 *
 * @param vision 已按协议布局复制出的视觉payload。
 * @return uint8_t 全部为有限值返回1，否则返回0。
 */
static uint8_t MasterProtocolValidateVisionFloats(const MasterVisionPayload *vision)
{
    uint8_t index;

    for (index = 0U; index < 9U; ++index)
    {
        if (MasterProtocolFloatIsFinite(vision->coords[index][0]) == 0U ||
            MasterProtocolFloatIsFinite(vision->coords[index][1]) == 0U)
        {
            return 0U;
        }
    }

    for (index = 0U; index < 3U; ++index)
    {
        if (MasterProtocolFloatIsFinite(vision->rvec[index]) == 0U ||
            MasterProtocolFloatIsFinite(vision->tvec[index]) == 0U)
        {
            return 0U;
        }
    }

    return 1U;
}

/**
 * @brief 获取指定帧类型应有的固定payload长度。
 */
uint16_t MasterProtocolGetPayloadSize(uint8_t type)
{
    switch (type)
    {
    case MASTER_PROTOCOL_TYPE_COMMAND:
        return MASTER_PROTOCOL_COMMAND_PAYLOAD_SIZE;

    case MASTER_PROTOCOL_TYPE_VISION:
        return MASTER_PROTOCOL_VISION_PAYLOAD_SIZE;

    case MASTER_PROTOCOL_TYPE_EVENT:
        return MASTER_PROTOCOL_EVENT_PAYLOAD_SIZE;

    default:
        return 0U;
    }
}

/**
 * @brief 计算协议规定的低8位累加校验和。
 */
uint8_t MasterProtocolCalculateChecksum(uint8_t type,
                                        uint16_t payload_length,
                                        const uint8_t *payload)
{
    uint8_t checksum;
    uint16_t index;

    checksum = type;
    checksum = (uint8_t)(checksum + (uint8_t)(payload_length & 0xFFU));
    checksum = (uint8_t)(checksum + (uint8_t)(payload_length >> 8U));

    if (payload == NULL)
    {
        return checksum;
    }

    for (index = 0U; index < payload_length; ++index)
    {
        checksum = (uint8_t)(checksum + payload[index]);
    }

    return checksum;
}

/**
 * @brief 校验Command payload的字段取值。
 *
 * @param payload 已确认长度正确的payload字节。
 * @return MasterProtocolPayloadStatus_e 校验结果。
 */
static MasterProtocolPayloadStatus_e MasterProtocolValidateCommand(const uint8_t *payload)
{
    MasterCommandPayload command;

    /* 使用memcpy避免把可能未对齐的UART缓冲区直接强转为结构体指针。 */
    memcpy(&command, payload, sizeof(command));

    if (command.ms > 999U ||
        command.w > 1U || command.a > 1U || command.s > 1U || command.d > 1U ||
        command.rotate < -1 || command.rotate > 1 ||
        MasterProtocolFloatIsFinite(command.dx) == 0U ||
        MasterProtocolFloatIsFinite(command.dy) == 0U)
    {
        return MASTER_PROTOCOL_PAYLOAD_INVALID_VALUE;
    }

    return MASTER_PROTOCOL_PAYLOAD_VALID;
}

/**
 * @brief 校验Vision payload的字段取值。
 *
 * @param payload 已确认长度正确的payload字节。
 * @return MasterProtocolPayloadStatus_e 校验结果。
 */
static MasterProtocolPayloadStatus_e MasterProtocolValidateVision(const uint8_t *payload)
{
    MasterVisionPayload vision;
    uint8_t index;

    memcpy(&vision, payload, sizeof(vision));

    if (vision.ms > 999U || vision.result < -1 || vision.result > 8)
    {
        return MASTER_PROTOCOL_PAYLOAD_INVALID_VALUE;
    }

    for (index = 0U; index < 9U; ++index)
    {
        if ((vision.order[index] > 8U && vision.order[index] != 0xFFU) ||
            vision.goal[index] < -1 || vision.goal[index] > 1)
        {
            return MASTER_PROTOCOL_PAYLOAD_INVALID_VALUE;
        }
    }

    if (MasterProtocolValidateVisionFloats(&vision) == 0U)
    {
        return MASTER_PROTOCOL_PAYLOAD_INVALID_VALUE;
    }

    return MASTER_PROTOCOL_PAYLOAD_VALID;
}

/**
 * @brief 校验Event payload的字段取值。
 *
 * @param payload 已确认长度正确的payload字节。
 * @return MasterProtocolPayloadStatus_e 校验结果。
 */
static MasterProtocolPayloadStatus_e MasterProtocolValidateEvent(const uint8_t *payload)
{
    MasterEventPayload event_payload;

    memcpy(&event_payload, payload, sizeof(event_payload));

    if (event_payload.ms > 999U ||
        event_payload.event < MASTER_PROTOCOL_EVENT_EMERGENCY_STOP ||
        event_payload.event > MASTER_PROTOCOL_EVENT_SERVE ||
        MasterProtocolFloatIsFinite(event_payload.value) == 0U)
    {
        return MASTER_PROTOCOL_PAYLOAD_INVALID_VALUE;
    }

    return MASTER_PROTOCOL_PAYLOAD_VALID;
}

/**
 * @brief 校验payload的类型、长度和字段取值。
 */
MasterProtocolPayloadStatus_e MasterProtocolValidatePayload(uint8_t type,
                                                            const uint8_t *payload,
                                                            uint16_t payload_length)
{
    uint16_t expected_length;

    if (payload == NULL)
    {
        return MASTER_PROTOCOL_PAYLOAD_INVALID_ARGUMENT;
    }

    expected_length = MasterProtocolGetPayloadSize(type);
    if (expected_length == 0U)
    {
        return MASTER_PROTOCOL_PAYLOAD_INVALID_TYPE;
    }

    if (payload_length != expected_length)
    {
        return MASTER_PROTOCOL_PAYLOAD_INVALID_LENGTH;
    }

    switch (type)
    {
    case MASTER_PROTOCOL_TYPE_COMMAND:
        return MasterProtocolValidateCommand(payload);

    case MASTER_PROTOCOL_TYPE_VISION:
        return MasterProtocolValidateVision(payload);

    case MASTER_PROTOCOL_TYPE_EVENT:
        return MasterProtocolValidateEvent(payload);

    default:
        return MASTER_PROTOCOL_PAYLOAD_INVALID_TYPE;
    }
}
