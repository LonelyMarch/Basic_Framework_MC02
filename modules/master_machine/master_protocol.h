#ifndef MASTER_PROTOCOL_H
#define MASTER_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** UART帧的两个固定帧头字节。 */
#define MASTER_PROTOCOL_FRAME_HEAD_1 0xAAU
#define MASTER_PROTOCOL_FRAME_HEAD_2 0x55U

/** ESP32-S3发送给STM32的三种帧类型。 */
#define MASTER_PROTOCOL_TYPE_COMMAND 0x01U
#define MASTER_PROTOCOL_TYPE_VISION  0x02U
#define MASTER_PROTOCOL_TYPE_EVENT   0x03U

/** 离散事件编号，与最新版ESP32-S3协议保持一致。 */
#define MASTER_PROTOCOL_EVENT_EMERGENCY_STOP 0x01U
#define MASTER_PROTOCOL_EVENT_START_MATCH    0x02U
#define MASTER_PROTOCOL_EVENT_SERVE          0x03U

#define MASTER_PROTOCOL_HEADER_SIZE       5U   /**< AA 55、type、len_low、len_high。 */
#define MASTER_PROTOCOL_CHECKSUM_SIZE     1U   /**< 帧尾低8位累加和。 */
#define MASTER_PROTOCOL_FRAME_OVERHEAD    6U   /**< 不包含payload的固定开销。 */
#define MASTER_PROTOCOL_MAX_PAYLOAD_SIZE  256U /**< 协议文档建议的payload上限。 */
#define MASTER_PROTOCOL_MAX_FRAME_SIZE    (MASTER_PROTOCOL_MAX_PAYLOAD_SIZE + MASTER_PROTOCOL_FRAME_OVERHEAD)

#if defined(__GNUC__)
#define MASTER_PROTOCOL_PACKED __attribute__((packed))
#else
#define MASTER_PROTOCOL_PACKED
#endif

/**
 * @brief 周期控制命令payload。
 *
 * @note 该结构体直接描述线上的23字节二进制布局，必须保持packed、小端和IEEE754单精度格式。
 */
typedef struct MASTER_PROTOCOL_PACKED
{
    uint32_t sec; /**< 客户端秒级时间戳。 */
    uint16_t ms; /**< 毫秒部分，合法范围为0~999。 */
    uint32_t id; /**< Command类型内独立递增的帧编号。 */
    uint8_t w; /**< W键状态，只允许0或1。 */
    uint8_t a; /**< A键状态，只允许0或1。 */
    uint8_t s; /**< S键状态，只允许0或1。 */
    uint8_t d; /**< D键状态，只允许0或1。 */
    float dx; /**< 本帧鼠标X方向位移增量。 */
    float dy; /**< 本帧鼠标Y方向位移增量。 */
    int8_t rotate; /**< -1逆时针、0停止、1顺时针。 */
}

MasterCommandPayload;

/**
 * @brief 视觉识别结果payload。
 *
 * @note order、goal和coords都按照九宫格0~8的顺序排列。
 */
typedef struct MASTER_PROTOCOL_PACKED
{
    uint32_t sec; /**< 视觉端秒级时间戳。 */
    uint16_t ms; /**< 毫秒部分，合法范围为0~999。 */
    uint32_t id; /**< Vision类型内独立递增的帧编号。 */
    int8_t result; /**< -1表示无结果，0~8表示对应九宫格结果。 */
    uint8_t order[9]; /**< 字符编号0~8，未知字符使用255。 */
    int8_t goal[9]; /**< 各格目标状态，当前协议允许-1、0、1。 */
    float coords[9][2]; /**< 九个目标的二维坐标。 */
    float rvec[3]; /**< 外参旋转向量。 */
    float tvec[3]; /**< 外参平移向量。 */
}

MasterVisionPayload;

/**
 * @brief 离散事件payload。
 */
typedef struct MASTER_PROTOCOL_PACKED
{
    uint32_t sec; /**< 事件源秒级时间戳。 */
    uint16_t ms; /**< 毫秒部分，合法范围为0~999。 */
    uint32_t id; /**< Event类型内独立递增的帧编号。 */
    uint8_t event; /**< 事件编号，当前为1~3。 */
    float value; /**< 可选事件参数，未使用时为0.0f。 */
}

MasterEventPayload;

#define MASTER_PROTOCOL_COMMAND_PAYLOAD_SIZE ((uint16_t)sizeof(MasterCommandPayload))
#define MASTER_PROTOCOL_VISION_PAYLOAD_SIZE  ((uint16_t)sizeof(MasterVisionPayload))
#define MASTER_PROTOCOL_EVENT_PAYLOAD_SIZE   ((uint16_t)sizeof(MasterEventPayload))

/** payload语义校验结果。 */
typedef enum
{
    MASTER_PROTOCOL_PAYLOAD_VALID = 0,
    MASTER_PROTOCOL_PAYLOAD_INVALID_ARGUMENT,
    MASTER_PROTOCOL_PAYLOAD_INVALID_TYPE,
    MASTER_PROTOCOL_PAYLOAD_INVALID_LENGTH,
    MASTER_PROTOCOL_PAYLOAD_INVALID_VALUE,
} MasterProtocolPayloadStatus_e;

/**
 * @brief 获取指定帧类型应有的固定payload长度。
 *
 * @param type 帧类型。
 * @return uint16_t 合法类型对应的payload长度；未知类型返回0。
 */
uint16_t MasterProtocolGetPayloadSize(uint8_t type);


/**
 * @brief 计算协议规定的低8位累加校验和。
 *
 * @param type 帧类型。
 * @param payload_length payload长度。
 * @param payload payload首地址；长度为0时允许为NULL。
 * @return uint8_t type、两个长度字节和payload全部字节累加后的低8位。
 */
uint8_t MasterProtocolCalculateChecksum(uint8_t type,
                                        uint16_t payload_length,
                                        const uint8_t* payload);


/**
 * @brief 校验payload的类型、长度和字段取值。
 *
 * @param type 帧类型。
 * @param payload payload原始字节。
 * @param payload_length payload长度。
 * @return MasterProtocolPayloadStatus_e 校验结果。
 */
MasterProtocolPayloadStatus_e MasterProtocolValidatePayload(uint8_t type,
                                                            const uint8_t* payload,
                                                            uint16_t payload_length);

#ifdef __cplusplus
}
#endif

#endif
