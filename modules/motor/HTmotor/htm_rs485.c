#include "htm_rs485.h"

#include <string.h>

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "htm_motor.h"

#define HTM_REQUEST_HEADER 0x3EU
#define HTM_RESPONSE_HEADER 0x3CU
#define HTM_MAX_DATA_LEN 60U
#define HTM_MAX_FRAME_LEN (5U + HTM_MAX_DATA_LEN + 2U)
#define HTM_RX_STREAM_SIZE 160U

#define HTM_CMD_READ_MEASURE 0x2FU
#define HTM_CMD_READ_STATUS 0x40U

struct HTMRS485Bus
{
    USARTInstance* usart;
    USART_TRANSFER_MODE transfer_mode;
    HTMMotorInstance* motors[HTM_RS485_MOTOR_PER_BUS];
    HTMMotorInstance* in_flight_motor;
    uint8_t motor_count;
    uint8_t round_robin_index;
    uint8_t packet_sequence;
    uint8_t in_flight_sequence;
    uint8_t in_flight_command;
    uint16_t response_timeout_ms;
    float request_start_ms;
    uint8_t rx_stream[HTM_RX_STREAM_SIZE];
    uint16_t rx_stream_len;
};

static HTMRS485Bus htm_bus_pool[HTM_RS485_BUS_CNT];
static uint8_t htm_bus_count;

static uint16_t HTMCRC16Modbus(const uint8_t* data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;

    for (uint16_t i = 0U; i < len; ++i)
    {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit)
            crc = (crc & 1U) != 0U ? (uint16_t)((crc >> 1U) ^ 0xA001U) : (uint16_t)(crc >> 1U);
    }
    return crc;
}

static HTMMotorInstance* HTMFindMotor(HTMRS485Bus* bus, uint8_t address)
{
    for (uint8_t i = 0U; i < bus->motor_count; ++i)
    {
        if (bus->motors[i] != NULL && bus->motors[i]->device_address == address)
            return bus->motors[i];
    }
    return NULL;
}

static int16_t HTMReadI16(const uint8_t* data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static int32_t HTMReadI32(const uint8_t* data)
{
    return (int32_t)((uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
        ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U));
}

static void HTMDecodeMeasure(HTMMotorInstance* motor, const uint8_t* data, uint8_t len)
{
    float direction = motor->motor_reverse_flag == MOTOR_DIRECTION_REVERSE ? -1.0f : 1.0f;

    if (len != 8U)
        return;

    motor->measure.single_round_count = (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
    motor->measure.total_count = HTMReadI32(&data[2]);
    motor->measure.speed_0_1rpm = HTMReadI16(&data[6]);
    motor->measure.single_round_angle_deg = direction * (float)motor->measure.single_round_count *
        360.0f / HTM_ENCODER_COUNTS_PER_ROUND;
    motor->measure.total_angle_deg = direction * (float)motor->measure.total_count *
        360.0f / HTM_ENCODER_COUNTS_PER_ROUND;
    motor->measure.speed_rpm = direction * (float)motor->measure.speed_0_1rpm * 0.1f;
}

static void HTMDecodeStatus(HTMMotorInstance* motor, const uint8_t* data, uint8_t len)
{
    if (len != 5U)
        return;
    motor->measure.voltage_v = (float)data[0] * 0.2f;
    motor->measure.current_a = (float)data[1] * 0.03f;
    motor->measure.temperature_c = (float)data[2] * 0.4f;
    motor->measure.fault_code = data[3];
    motor->measure.running_state = data[4];
}

/**
 * @brief 处理一帧已经通过帧头、长度和 CRC 校验的 HTM RS485 回复
 *
 * @note frame_len 由流式拆包函数根据数据长度字段计算后显式传入，避免本函数
 *       访问未定义的局部变量，并为后续增加更严格的长度校验保留完整帧长度信息。
 *
 * @param bus 所属 RS485 总线实例
 * @param frame 完整回复帧首地址
 * @param frame_len 完整帧长度，包含帧头、数据和 CRC
 */
static void HTMHandleFrame(HTMRS485Bus* bus, const uint8_t* frame, uint16_t frame_len)
{
    HTMMotorInstance* motor;
    uint8_t command;
    uint8_t data_len;

    if (frame_len < 7U || frame[0] != HTM_RESPONSE_HEADER)
        return;

    data_len = frame[4];
    motor = HTMFindMotor(bus, frame[2]);
    command = frame[3];
    if (motor == NULL)
        return;

    if (command == HTM_CMD_READ_MEASURE || (command >= 0x50U && command <= 0x56U))
        HTMDecodeMeasure(motor, &frame[5], data_len);
    else if (command == HTM_CMD_READ_STATUS || command == 0x41U)
        HTMDecodeStatus(motor, &frame[5], data_len);

    if (motor->offline != 0U)
    {
        motor->offline = 0U;
        LOGINFO("[htm] motor address %u recovered, remains disabled", (unsigned int)motor->device_address);
    }
    DaemonReload(motor->daemon);

    if (bus->in_flight_motor == motor && bus->in_flight_command == command &&
        bus->in_flight_sequence == frame[1])
    {
        bus->in_flight_motor = NULL;
        bus->in_flight_command = 0U;
    }
}

static void HTMParseStream(HTMRS485Bus* bus)
{
    while (bus->rx_stream_len >= 7U)
    {
        uint16_t frame_len;
        uint16_t crc;

        if (bus->rx_stream[0] != HTM_RESPONSE_HEADER)
        {
            memmove(bus->rx_stream, &bus->rx_stream[1], --bus->rx_stream_len);
            continue;
        }

        if (bus->rx_stream[4] > HTM_MAX_DATA_LEN)
        {
            memmove(bus->rx_stream, &bus->rx_stream[1], --bus->rx_stream_len);
            continue;
        }

        frame_len = (uint16_t)(5U + bus->rx_stream[4] + 2U);
        if (bus->rx_stream_len < frame_len)
            return;

        crc = HTMCRC16Modbus(bus->rx_stream, (uint16_t)(frame_len - 2U));
        if (bus->rx_stream[frame_len - 2U] == (uint8_t)crc &&
            bus->rx_stream[frame_len - 1U] == (uint8_t)(crc >> 8U))
            HTMHandleFrame(bus, bus->rx_stream, frame_len);

        bus->rx_stream_len = (uint16_t)(bus->rx_stream_len - frame_len);
        memmove(bus->rx_stream, &bus->rx_stream[frame_len], bus->rx_stream_len);
    }
}

static void HTMBusReceive(uint8_t bus_index)
{
    HTMRS485Bus* bus;
    uint16_t copy_len;

    if (bus_index >= htm_bus_count)
        return;
    bus = &htm_bus_pool[bus_index];
    if (bus->usart == NULL || bus->usart->recv_len == 0U)
        return;

    copy_len = bus->usart->recv_len;
    if (copy_len > HTM_RX_STREAM_SIZE - bus->rx_stream_len)
    {
        bus->rx_stream_len = 0U;
        if (copy_len > HTM_RX_STREAM_SIZE)
            copy_len = HTM_RX_STREAM_SIZE;
    }
    memcpy(&bus->rx_stream[bus->rx_stream_len], bus->usart->recv_buff, copy_len);
    bus->rx_stream_len = (uint16_t)(bus->rx_stream_len + copy_len);
    HTMParseStream(bus);
}

static void HTMBus0Receive(void) { HTMBusReceive(0U); }
static void HTMBus1Receive(void) { HTMBusReceive(1U); }

static HAL_StatusTypeDef HTMSendRequest(HTMRS485Bus* bus,
                                        HTMMotorInstance* motor,
                                        uint8_t command,
                                        const uint8_t* data,
                                        uint8_t data_len)
{
    uint8_t frame[HTM_MAX_FRAME_LEN];
    uint8_t frame_len;
    uint16_t crc;
    HAL_StatusTypeDef status;

    if (bus == NULL || motor == NULL || data_len > HTM_MAX_DATA_LEN)
        return HAL_ERROR;

    frame[0] = HTM_REQUEST_HEADER;
    frame[1] = bus->packet_sequence++;
    frame[2] = motor->device_address;
    frame[3] = command;
    frame[4] = data_len;
    if (data_len > 0U)
        memcpy(&frame[5], data, data_len);
    crc = HTMCRC16Modbus(frame, (uint16_t)(5U + data_len));
    frame[5U + data_len] = (uint8_t)crc;
    frame[6U + data_len] = (uint8_t)(crc >> 8U);
    frame_len = (uint8_t)(7U + data_len);

    status = USARTSend(bus->usart, frame, frame_len, bus->transfer_mode);
    if (status == HAL_OK)
    {
        bus->in_flight_motor = motor;
        bus->in_flight_command = command;
        bus->in_flight_sequence = frame[1];
        bus->request_start_ms = DWT_GetTimeline_ms();
    }
    return status;
}

HTMRS485Bus* HTMRS485BusInit(const HTMRS485Bus_Init_Config_s* config)
{
    HTMRS485Bus* bus;
    USART_Init_Config_s usart_config;

    if (config == NULL || config->usart_handle == NULL || htm_bus_count >= HTM_RS485_BUS_CNT ||
        (config->transfer_mode != USART_TRANSFER_BLOCKING &&
            config->transfer_mode != USART_TRANSFER_IT &&
            config->transfer_mode != USART_TRANSFER_DMA))
        return NULL;

    bus = &htm_bus_pool[htm_bus_count];
    memset(bus, 0, sizeof(*bus));
    bus->transfer_mode = config->transfer_mode;
    bus->response_timeout_ms = config->response_timeout_ms == 0U ? 20U : config->response_timeout_ms;

    memset(&usart_config, 0, sizeof(usart_config));
    usart_config.usart_handle = config->usart_handle;
    usart_config.recv_buff_size = HTM_MAX_FRAME_LEN;
    usart_config.module_callback = htm_bus_count == 0U ? HTMBus0Receive : HTMBus1Receive;
    bus->usart = USARTRegister(&usart_config);
    if (bus->usart == NULL)
        return NULL;

    htm_bus_count++;
    return bus;
}

HAL_StatusTypeDef HTMRS485RegisterMotor(HTMRS485Bus* bus, HTMMotorInstance* motor)
{
    if (bus == NULL || motor == NULL || bus->motor_count >= HTM_RS485_MOTOR_PER_BUS)
        return HAL_ERROR;
    if (HTMFindMotor(bus, motor->device_address) != NULL)
        return HAL_ERROR;

    bus->motors[bus->motor_count++] = motor;
    return HAL_OK;
}

void HTMRS485Control(void)
{
    for (uint8_t bus_index = 0U; bus_index < htm_bus_count; ++bus_index)
    {
        HTMRS485Bus* bus = &htm_bus_pool[bus_index];
        HTMMotorInstance* motor;

        if (bus->motor_count == 0U)
            continue;

        if (bus->in_flight_motor != NULL)
        {
            if (DWT_GetTimeline_ms() - bus->request_start_ms <= (float)bus->response_timeout_ms)
                continue;
            bus->in_flight_motor = NULL;
            bus->in_flight_command = 0U;
        }

        for (uint8_t checked = 0U; checked < bus->motor_count; ++checked)
        {
            motor = bus->motors[bus->round_robin_index];
            bus->round_robin_index = (uint8_t)((bus->round_robin_index + 1U) % bus->motor_count);
            if (motor == NULL)
                continue;

            if (motor->command_pending != 0U)
            {
                if (HTMSendRequest(bus,
                                   motor,
                                   motor->pending_command,
                                   motor->pending_data,
                                   motor->pending_data_len) == HAL_OK)
                    motor->command_pending = 0U;
                break;
            }

            (void)HTMSendRequest(bus, motor, HTM_CMD_READ_MEASURE, NULL, 0U);
            break;
        }
    }
}
