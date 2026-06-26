#ifndef MASTER_PROTOCOL_H
#define MASTER_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MASTER_FRAME_HEAD_1 0xAAu
#define MASTER_FRAME_HEAD_2 0x55u

#define MASTER_FRAME_TYPE_COMMAND 0x01u
#define MASTER_FRAME_TYPE_VISION  0x02u
#define MASTER_FRAME_TYPE_EVENT   0x03u

#define MASTER_EVENT_EMERGENCY_STOP 0x01u
#define MASTER_EVENT_START_MATCH    0x02u
#define MASTER_EVENT_SERVE          0x03u

#define MASTER_UART_MAX_PAYLOAD_SIZE 256u
#define MASTER_UART_HEADER_SIZE      5u
#define MASTER_UART_CHECKSUM_SIZE    1u
#define MASTER_UART_FRAME_OVERHEAD   (MASTER_UART_HEADER_SIZE + MASTER_UART_CHECKSUM_SIZE)
#define MASTER_UART_MAX_FRAME_SIZE   (MASTER_UART_MAX_PAYLOAD_SIZE + MASTER_UART_FRAME_OVERHEAD)

#if defined(__GNUC__)
#define MASTER_PACKED __attribute__((packed))
#else
#define MASTER_PACKED
#endif

typedef struct MASTER_PACKED
{
    uint32_t sec;
    uint16_t ms;
    uint32_t id;

    uint8_t w;
    uint8_t a;
    uint8_t s;
    uint8_t d;

    float dx;
    float dy;

    int8_t rotate;
} CommandPayload;

typedef struct MASTER_PACKED
{
    uint32_t sec;
    uint16_t ms;
    uint32_t id;

    int8_t result;

    uint8_t order[9];
    int8_t goal[9];

    float coords[9][2];

    float rvec[3];
    float tvec[3];
} VisionPayload;

typedef struct MASTER_PACKED
{
    uint32_t sec;
    uint16_t ms;
    uint32_t id;

    uint8_t event;

    float value;
} EventPayload;

#define MASTER_COMMAND_PAYLOAD_SIZE ((uint16_t)sizeof(CommandPayload))
#define MASTER_VISION_PAYLOAD_SIZE  ((uint16_t)sizeof(VisionPayload))
#define MASTER_EVENT_PAYLOAD_SIZE   ((uint16_t)sizeof(EventPayload))

typedef struct
{
    uint8_t type;
    uint16_t payload_len;
    uint8_t checksum;
} MasterFrameHeader_s;

typedef enum
{
    MASTER_PARSE_OK = 0,
    MASTER_PARSE_BAD_TYPE,
    MASTER_PARSE_BAD_LENGTH,
    MASTER_PARSE_BAD_CHECKSUM,
} MasterParseError_e;

typedef enum
{
    MASTER_PARSE_STATE_HEAD_1 = 0,
    MASTER_PARSE_STATE_HEAD_2,
    MASTER_PARSE_STATE_TYPE,
    MASTER_PARSE_STATE_LEN_LOW,
    MASTER_PARSE_STATE_LEN_HIGH,
    MASTER_PARSE_STATE_PAYLOAD,
    MASTER_PARSE_STATE_CHECKSUM,
} MasterParseState_e;

typedef struct
{
    MasterParseState_e state;
    MasterParseError_e error;
    uint8_t type;
    uint16_t payload_len;
    uint16_t index;
    uint8_t checksum;
    uint8_t payload[MASTER_UART_MAX_PAYLOAD_SIZE];
} MasterProtocolParser_s;

uint16_t MasterProtocolExpectedPayloadSize(uint8_t type);
uint8_t MasterProtocolChecksum(uint8_t type, uint16_t payload_len, const uint8_t *payload);
void MasterProtocolParserInit(MasterProtocolParser_s *parser);
uint8_t MasterProtocolFeedByte(MasterProtocolParser_s *parser,
                               uint8_t byte,
                               MasterFrameHeader_s *header,
                               const uint8_t **payload);

#ifdef __cplusplus
}
#endif

#endif
