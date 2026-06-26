#include "master_protocol.h"

#include <string.h>

uint16_t MasterProtocolExpectedPayloadSize(uint8_t type)
{
    switch (type)
    {
    case MASTER_FRAME_TYPE_COMMAND:
        return MASTER_COMMAND_PAYLOAD_SIZE;
    case MASTER_FRAME_TYPE_VISION:
        return MASTER_VISION_PAYLOAD_SIZE;
    case MASTER_FRAME_TYPE_EVENT:
        return MASTER_EVENT_PAYLOAD_SIZE;
    default:
        return 0u;
    }
}

uint8_t MasterProtocolChecksum(uint8_t type, uint16_t payload_len, const uint8_t *payload)
{
    uint8_t checksum = 0u;

    checksum = (uint8_t)(checksum + type);
    checksum = (uint8_t)(checksum + (uint8_t)(payload_len & 0xFFu));
    checksum = (uint8_t)(checksum + (uint8_t)((payload_len >> 8) & 0xFFu));

    if (payload != NULL)
    {
        for (uint16_t i = 0; i < payload_len; ++i)
        {
            checksum = (uint8_t)(checksum + payload[i]);
        }
    }

    return checksum;
}

static void MasterProtocolParserReset(MasterProtocolParser_s *parser)
{
    parser->state = MASTER_PARSE_STATE_HEAD_1;
    parser->type = 0u;
    parser->payload_len = 0u;
    parser->index = 0u;
    parser->checksum = 0u;
}

void MasterProtocolParserInit(MasterProtocolParser_s *parser)
{
    if (parser == NULL)
        return;

    memset(parser, 0, sizeof(MasterProtocolParser_s));
    parser->state = MASTER_PARSE_STATE_HEAD_1;
    parser->error = MASTER_PARSE_OK;
}

uint8_t MasterProtocolFeedByte(MasterProtocolParser_s *parser,
                               uint8_t byte,
                               MasterFrameHeader_s *header,
                               const uint8_t **payload)
{
    uint16_t expected_len;
    uint8_t frame_type;

    if (payload != NULL)
        *payload = NULL;

    if (parser == NULL)
        return 0u;

    switch (parser->state)
    {
    case MASTER_PARSE_STATE_HEAD_1:
        if (byte == MASTER_FRAME_HEAD_1)
            parser->state = MASTER_PARSE_STATE_HEAD_2;
        break;

    case MASTER_PARSE_STATE_HEAD_2:
        if (byte == MASTER_FRAME_HEAD_2)
        {
            parser->state = MASTER_PARSE_STATE_TYPE;
        }
        else if (byte != MASTER_FRAME_HEAD_1)
        {
            parser->state = MASTER_PARSE_STATE_HEAD_1;
        }
        break;

    case MASTER_PARSE_STATE_TYPE:
        parser->type = byte;
        parser->checksum = byte;
        parser->state = MASTER_PARSE_STATE_LEN_LOW;
        break;

    case MASTER_PARSE_STATE_LEN_LOW:
        parser->payload_len = byte;
        parser->checksum = (uint8_t)(parser->checksum + byte);
        parser->state = MASTER_PARSE_STATE_LEN_HIGH;
        break;

    case MASTER_PARSE_STATE_LEN_HIGH:
        parser->payload_len |= (uint16_t)((uint16_t)byte << 8);
        parser->checksum = (uint8_t)(parser->checksum + byte);
        expected_len = MasterProtocolExpectedPayloadSize(parser->type);

        if (expected_len == 0u)
        {
            parser->error = MASTER_PARSE_BAD_TYPE;
            MasterProtocolParserReset(parser);
            return 0xFFu;
        }

        if (parser->payload_len != expected_len || parser->payload_len > MASTER_UART_MAX_PAYLOAD_SIZE)
        {
            parser->error = MASTER_PARSE_BAD_LENGTH;
            MasterProtocolParserReset(parser);
            return 0xFFu;
        }

        parser->index = 0u;
        parser->state = MASTER_PARSE_STATE_PAYLOAD;
        break;

    case MASTER_PARSE_STATE_PAYLOAD:
        parser->payload[parser->index++] = byte;
        parser->checksum = (uint8_t)(parser->checksum + byte);
        if (parser->index >= parser->payload_len)
            parser->state = MASTER_PARSE_STATE_CHECKSUM;
        break;

    case MASTER_PARSE_STATE_CHECKSUM:
        if (byte != parser->checksum)
        {
            parser->error = MASTER_PARSE_BAD_CHECKSUM;
            MasterProtocolParserReset(parser);
            return 0xFFu;
        }

        if (header != NULL)
        {
            header->type = parser->type;
            header->payload_len = parser->payload_len;
            header->checksum = byte;
        }

        if (payload != NULL)
            *payload = parser->payload;

        frame_type = parser->type;
        MasterProtocolParserReset(parser);
        parser->error = MASTER_PARSE_OK;
        return frame_type;

    default:
        MasterProtocolParserReset(parser);
        return 0xFFu;
    }

    return 0u;
}
