#include "htm_motor.h"

#include <limits.h>
#include <string.h>

#include "bsp_log.h"

#define HTM_MOTOR_TOTAL_CNT (HTM_RS485_BUS_CNT * HTM_RS485_MOTOR_PER_BUS)

#define HTM_CMD_SET_ZERO 0x21U
#define HTM_CMD_CLEAR_FAULT 0x41U
#define HTM_CMD_CLOSE 0x50U
#define HTM_CMD_OPEN_LOOP 0x53U
#define HTM_CMD_SPEED 0x54U
#define HTM_CMD_ABSOLUTE_POSITION 0x55U
#define HTM_CMD_RELATIVE_POSITION 0x56U

static HTMMotorInstance htm_motor_pool[HTM_MOTOR_TOTAL_CNT];
static uint8_t htm_motor_count;

extern HAL_StatusTypeDef HTMRS485RegisterMotor(HTMRS485Bus * bus, HTMMotorInstance * motor);

static void HTMMotorLostCallback(void* owner)
{
    HTMMotorInstance* motor = (HTMMotorInstance*)owner;

    if (motor == NULL)
        return;

    motor->enabled = 0U;
    motor->command_pending = 0U;
    if (motor->offline == 0U)
    {
        motor->offline = 1U;
        LOGWARNING("[htm] motor address %u lost, output locked", (unsigned int)motor->device_address);
    }
}

static HAL_StatusTypeDef HTMMotorQueueCommand(HTMMotorInstance* motor,
                                              uint8_t command,
                                              const uint8_t* data,
                                              uint8_t data_len)
{
    if (motor == NULL || data_len > sizeof(motor->pending_data))
        return HAL_ERROR;
    if (motor->command_pending != 0U)
        return HAL_BUSY;

    motor->pending_command = command;
    motor->pending_data_len = data_len;
    if (data_len > 0U)
        memcpy(motor->pending_data, data, data_len);
    motor->command_pending = 1U;
    return HAL_OK;
}

HTMMotorInstance* HTMMotorInit(const HTMMotor_Init_Config_s* config)
{
    HTMMotorInstance* motor;
    Daemon_Init_Config_s daemon_config;

    if (config == NULL || config->bus == NULL || config->device_address < 1U ||
        config->device_address > 32U || config->control_mode > HTM_CONTROL_RELATIVE_POSITION ||
        htm_motor_count >= HTM_MOTOR_TOTAL_CNT)
    {
        LOGERROR("[htm] invalid motor init config");
        return NULL;
    }

    motor = &htm_motor_pool[htm_motor_count];
    memset(motor, 0, sizeof(*motor));
    motor->bus = config->bus;
    motor->device_address = config->device_address;
    motor->control_mode = config->control_mode;
    motor->motor_reverse_flag = config->motor_reverse_flag;
    motor->offline = 1U;

    if (HTMRS485RegisterMotor(config->bus, motor) != HAL_OK)
        return NULL;

    memset(&daemon_config, 0, sizeof(daemon_config));
    daemon_config.callback = HTMMotorLostCallback;
    daemon_config.owner_id = motor;
    daemon_config.reload_count = 20U;
    motor->daemon = DaemonRegister(&daemon_config);
    if (motor->daemon == NULL)
        return NULL;

    htm_motor_count++;
    return motor;
}

HAL_StatusTypeDef HTMMotorEnable(HTMMotorInstance* motor)
{
    if (motor == NULL)
        return HAL_ERROR;
    if (motor->offline != 0U)
        return HAL_BUSY;
    motor->enabled = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef HTMMotorStop(HTMMotorInstance* motor)
{
    if (motor == NULL)
        return HAL_ERROR;

    motor->enabled = 0U;
    motor->pending_command = HTM_CMD_CLOSE;
    motor->pending_data_len = 0U;
    motor->command_pending = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef HTMMotorSetOpenLoop(HTMMotorInstance* motor, int16_t power)
{
    uint8_t data[2];

    if (motor == NULL || motor->control_mode != HTM_CONTROL_OPEN_LOOP)
        return HAL_ERROR;
    if (motor->enabled == 0U || motor->offline != 0U)
        return HAL_BUSY;
    if (motor->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
        power = (power == INT16_MIN) ? INT16_MAX : (int16_t) - power;
    data[0] = (uint8_t)power;
    data[1] = (uint8_t)((uint16_t)power >> 8U);
    return HTMMotorQueueCommand(motor, HTM_CMD_OPEN_LOOP, data, 2U);
}

HAL_StatusTypeDef HTMMotorSetSpeed(HTMMotorInstance* motor, float speed_rpm)
{
    int32_t raw;
    uint8_t data[2];

    if (motor == NULL || motor->control_mode != HTM_CONTROL_SPEED)
        return HAL_ERROR;
    if (motor->enabled == 0U || motor->offline != 0U)
        return HAL_BUSY;
    if (motor->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
        speed_rpm = -speed_rpm;
    if (speed_rpm > 3276.7f)
        speed_rpm = 3276.7f;
    if (speed_rpm < -3276.8f)
        speed_rpm = -3276.8f;
    raw = (int32_t)(speed_rpm * 10.0f);
    if (raw > INT16_MAX)
        raw = INT16_MAX;
    if (raw < INT16_MIN)
        raw = INT16_MIN;
    data[0] = (uint8_t)raw;
    data[1] = (uint8_t)((uint16_t)raw >> 8U);
    return HTMMotorQueueCommand(motor, HTM_CMD_SPEED, data, 2U);
}

HAL_StatusTypeDef HTMMotorSetAbsolutePosition(HTMMotorInstance* motor, float position_deg)
{
    int64_t signed_count;
    uint32_t count;
    uint8_t data[4];

    if (motor == NULL || motor->control_mode != HTM_CONTROL_ABSOLUTE_POSITION)
        return HAL_ERROR;
    if (position_deg < 0.0f)
        return HAL_ERROR;
    if (motor->enabled == 0U || motor->offline != 0U)
        return HAL_BUSY;
    if (motor->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
        return HAL_ERROR;
    if (position_deg > (float)UINT32_MAX * 360.0f / HTM_ENCODER_COUNTS_PER_ROUND)
        return HAL_ERROR;

    signed_count = (int64_t)(position_deg * HTM_ENCODER_COUNTS_PER_ROUND / 360.0f);
    if (signed_count < 0 || signed_count > UINT32_MAX)
        return HAL_ERROR;
    count = (uint32_t)signed_count;
    data[0] = (uint8_t)count;
    data[1] = (uint8_t)(count >> 8U);
    data[2] = (uint8_t)(count >> 16U);
    data[3] = (uint8_t)(count >> 24U);
    return HTMMotorQueueCommand(motor, HTM_CMD_ABSOLUTE_POSITION, data, 4U);
}

HAL_StatusTypeDef HTMMotorSetRelativePosition(HTMMotorInstance* motor, float delta_deg)
{
    int32_t count;
    uint8_t data[2];

    if (motor == NULL || motor->control_mode != HTM_CONTROL_RELATIVE_POSITION)
        return HAL_ERROR;
    if (motor->enabled == 0U || motor->offline != 0U)
        return HAL_BUSY;
    if (motor->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
        delta_deg = -delta_deg;
    if (delta_deg > (float)INT16_MAX * 360.0f / HTM_ENCODER_COUNTS_PER_ROUND ||
        delta_deg < (float)INT16_MIN * 360.0f / HTM_ENCODER_COUNTS_PER_ROUND)
        return HAL_ERROR;
    count = (int32_t)(delta_deg * HTM_ENCODER_COUNTS_PER_ROUND / 360.0f);
    if (count > INT16_MAX || count < INT16_MIN)
        return HAL_ERROR;
    data[0] = (uint8_t)count;
    data[1] = (uint8_t)((uint16_t)count >> 8U);
    return HTMMotorQueueCommand(motor, HTM_CMD_RELATIVE_POSITION, data, 2U);
}

HAL_StatusTypeDef HTMMotorSetCurrentPositionAsZero(HTMMotorInstance* motor)
{
    if (motor == NULL)
        return HAL_ERROR;
    motor->enabled = 0U;
    return HTMMotorQueueCommand(motor, HTM_CMD_SET_ZERO, NULL, 0U);
}

HAL_StatusTypeDef HTMMotorClearFault(HTMMotorInstance* motor)
{
    return HTMMotorQueueCommand(motor, HTM_CMD_CLEAR_FAULT, NULL, 0U);
}

uint8_t HTMMotorIsOnline(const HTMMotorInstance* motor)
{
    return motor != NULL && motor->offline == 0U;
}

HTMMotor_Control_Mode_e HTMMotorGetControlMode(const HTMMotorInstance* motor)
{
    return motor != NULL ? motor->control_mode : HTM_CONTROL_OPEN_LOOP;
}
