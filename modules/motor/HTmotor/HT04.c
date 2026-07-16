#include "HT04.h"

#include <string.h>

#include "bsp_dwt.h" // DWT_GetDeltaT()：计算相邻反馈帧之间的时间间隔
#include "bsp_log.h"
#include "general_def.h"
#include "user_lib.h"

#define HT04_SPEED_BUFFER_SIZE 5U
#define HT04_CURRENT_SMOOTH_COEF 0.9f
#define HT04_SPEED_BIAS_RAD_S (-0.0109901428f)
#define HT04_CAN_TIMEOUT_MS 0.5f

#define HT04_KP_MIN 0.0f
#define HT04_KP_MAX 500.0f
#define HT04_KD_MIN 0.0f
#define HT04_KD_MAX 5.0f

#define HT04_CMD_MOTOR_MODE 0xFCU
#define HT04_CMD_RESET_MODE 0xFDU
#define HT04_CMD_ZERO_POSITION 0xFEU

typedef struct
{
    HTMotorInstance instance;
    float speed_buffer[HT04_SPEED_BUFFER_SIZE];
} HTMotorPoolEntry;

static HTMotorPoolEntry ht_motor_pool[HT04_MOTOR_CNT];
static HTMotorInstance* ht_motor_instances[HT04_MOTOR_CNT];
static uint8_t ht_motor_count;

static uint16_t HTMotorFloatToUint(float value, float min_value, float max_value, uint8_t bits)
{
    const float span = max_value - min_value;
    const uint32_t max_integer = (1UL << bits) - 1UL;

    LIMIT_MIN_MAX(value, min_value, max_value);
    return (uint16_t)((value - min_value) * (float)max_integer / span);
}

static float HTMotorUintToFloat(uint16_t value, float min_value, float max_value, uint8_t bits)
{
    const float span = max_value - min_value;
    const uint32_t max_integer = (1UL << bits) - 1UL;
    return (float)value * span / (float)max_integer + min_value;
}

static void HTMotorPackTorqueFrame(uint8_t frame[8], float torque_nm)
{
    const uint16_t position = HTMotorFloatToUint(0.0f, HT04_POSITION_MIN_RAD, HT04_POSITION_MAX_RAD, 16U);
    const uint16_t velocity = HTMotorFloatToUint(0.0f, HT04_VELOCITY_MIN_RAD_S, HT04_VELOCITY_MAX_RAD_S, 12U);
    const uint16_t kp = HTMotorFloatToUint(0.0f, HT04_KP_MIN, HT04_KP_MAX, 12U);
    const uint16_t kd = HTMotorFloatToUint(0.0f, HT04_KD_MIN, HT04_KD_MAX, 12U);
    const uint16_t torque = HTMotorFloatToUint(torque_nm, HT04_TORQUE_MIN_NM, HT04_TORQUE_MAX_NM, 12U);

    frame[0] = (uint8_t)(position >> 8U);
    frame[1] = (uint8_t)position;
    frame[2] = (uint8_t)(velocity >> 4U);
    frame[3] = (uint8_t)(((velocity & 0x0FU) << 4U) | (kp >> 8U));
    frame[4] = (uint8_t)kp;
    frame[5] = (uint8_t)(kd >> 4U);
    frame[6] = (uint8_t)(((kd & 0x0FU) << 4U) | (torque >> 8U));
    frame[7] = (uint8_t)torque;
}

static HAL_StatusTypeDef HTMotorSendSpecialCommand(HTMotorInstance* motor, uint8_t command)
{
    if (motor == NULL || motor->motor_can_instance == NULL)
        return HAL_ERROR;

    memset(motor->motor_can_instance->tx_buff, 0xFF, 7U);
    motor->motor_can_instance->tx_buff[7] = command;
    return CANTransmit(motor->motor_can_instance, 1.0f) != 0U ? HAL_OK : HAL_ERROR;
}

static void HTMotorDecode(CANInstance* motor_can)
{
    HTMotorInstance* motor;
    HTMotor_Measure_t* measure;
    HTMotorPoolEntry* entry;
    uint16_t raw_position;
    uint16_t raw_velocity;
    uint16_t raw_current;
    float direction = 1.0f;

    if (motor_can == NULL || motor_can->id == NULL || motor_can->rx_len != 6U)
        return;

    motor = (HTMotorInstance*)motor_can->id;
    if (motor->expected_motor_id != 0U && motor_can->rx_buff[0] != motor->expected_motor_id)
        return;

    if (motor->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
        direction = -1.0f;

    measure = &motor->measure;
    entry = (HTMotorPoolEntry*)motor;
    raw_position = (uint16_t)(((uint16_t)motor_can->rx_buff[1] << 8U) | motor_can->rx_buff[2]);
    raw_velocity = (uint16_t)(((uint16_t)motor_can->rx_buff[3] << 4U) | (motor_can->rx_buff[4] >> 4U));
    raw_current = (uint16_t)((((uint16_t)motor_can->rx_buff[4] & 0x0FU) << 8U) | motor_can->rx_buff[5]);

    measure->last_position_rad = measure->position_rad;
    measure->position_rad = direction * HTMotorUintToFloat(raw_position,
                                                           HT04_POSITION_MIN_RAD,
                                                           HT04_POSITION_MAX_RAD,
                                                           16U);
    measure->velocity_rad_s = AverageFilter(direction *
                                            (HTMotorUintToFloat(raw_velocity,
                                                                HT04_VELOCITY_MIN_RAD_S,
                                                                HT04_VELOCITY_MAX_RAD_S,
                                                                12U) -
                                                HT04_SPEED_BIAS_RAD_S),
                                            entry->speed_buffer,
                                            HT04_SPEED_BUFFER_SIZE);
    measure->phase_current_a = HT04_CURRENT_SMOOTH_COEF * direction *
        HTMotorUintToFloat(raw_current,
                           HT04_CURRENT_MIN_A,
                           HT04_CURRENT_MAX_A,
                           12U) +
        (1.0f - HT04_CURRENT_SMOOTH_COEF) * measure->phase_current_a;
    measure->motor_id = motor_can->rx_buff[0];
    measure->feed_dt = DWT_GetDeltaT(&measure->feed_cnt);

    if (motor->offline != 0U)
    {
        motor->offline = 0U;
        LOGINFO("[ht04] motor %u feedback recovered, remains stopped",
                (unsigned int)motor->expected_motor_id);
    }
    DaemonReload(motor->motor_daemon);
}

static void HTMotorLostCallback(void* motor_ptr)
{
    HTMotorInstance* motor = (HTMotorInstance*)motor_ptr;

    if (motor == NULL)
        return;

    motor->torque_ref_nm = 0.0f;
    motor->stop_flag = MOTOR_STOP;
    motor->enabled = 0U;
    if (motor->offline == 0U)
    {
        motor->offline = 1U;
        LOGWARNING("[ht04] motor %u lost, output locked",
                   (unsigned int)motor->expected_motor_id);
    }
}

HTMotorInstance* HTMotorInit(const HTMotor_Init_Config_s* config)
{
    HTMotorPoolEntry* entry;
    HTMotorInstance* motor;
    CAN_Init_Config_s can_config;
    Daemon_Init_Config_s daemon_config;

    if (config == NULL || config->can_init_config.can_handle == NULL || ht_motor_count >= HT04_MOTOR_CNT)
    {
        LOGERROR("[ht04] invalid init config or instance limit reached");
        return NULL;
    }

    entry = &ht_motor_pool[ht_motor_count];
    memset(entry, 0, sizeof(*entry));
    motor = &entry->instance;
    motor->motor_reverse_flag = config->motor_reverse_flag;
    motor->expected_motor_id = config->expected_motor_id;
    motor->stop_flag = MOTOR_STOP;
    motor->offline = 1U;

    can_config = config->can_init_config;
    can_config.can_module_callback = HTMotorDecode;
    can_config.id = motor;
    motor->motor_can_instance = CANRegister(&can_config);
    if (motor->motor_can_instance == NULL)
        return NULL;

    memset(&daemon_config, 0, sizeof(daemon_config));
    daemon_config.callback = HTMotorLostCallback;
    daemon_config.owner_id = motor;
    daemon_config.reload_count = 5U;
    motor->motor_daemon = DaemonRegister(&daemon_config);
    if (motor->motor_daemon == NULL)
        return NULL;

    ht_motor_instances[ht_motor_count++] = motor;

    if (config->enable_on_init != 0U && HTMotorEnable(motor) != HAL_OK)
        LOGWARNING("[ht04] initial motor-mode command submit failed");
    if (config->set_zero_on_init != 0U && HTMotorSetCurrentPositionAsZero(motor) != HAL_OK)
        LOGWARNING("[ht04] initial zero-position command submit failed");

    return motor;
}

HAL_StatusTypeDef HTMotorSetTorque(HTMotorInstance* motor, float torque_nm)
{
    if (motor == NULL)
        return HAL_ERROR;
    if (motor->offline != 0U || motor->enabled == 0U || motor->stop_flag == MOTOR_STOP)
        return HAL_BUSY;

    LIMIT_MIN_MAX(torque_nm, HT04_TORQUE_MIN_NM, HT04_TORQUE_MAX_NM);
    motor->torque_ref_nm = torque_nm;
    return HAL_OK;
}

HAL_StatusTypeDef HTMotorStop(HTMotorInstance* motor)
{
    if (motor == NULL)
        return HAL_ERROR;
    motor->torque_ref_nm = 0.0f;
    motor->stop_flag = MOTOR_STOP;
    return HAL_OK;
}

HAL_StatusTypeDef HTMotorEnable(HTMotorInstance* motor)
{
    if (motor == NULL)
        return HAL_ERROR;
    if (HTMotorSendSpecialCommand(motor, HT04_CMD_MOTOR_MODE) != HAL_OK)
        return HAL_ERROR;

    motor->torque_ref_nm = 0.0f;
    motor->stop_flag = MOTOR_ENABLED;
    motor->enabled = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef HTMotorDisable(HTMotorInstance* motor)
{
    if (motor == NULL)
        return HAL_ERROR;

    motor->torque_ref_nm = 0.0f;
    motor->stop_flag = MOTOR_STOP;
    if (HTMotorSendSpecialCommand(motor, HT04_CMD_RESET_MODE) != HAL_OK)
        return HAL_ERROR;

    motor->enabled = 0U;
    return HAL_OK;
}

HAL_StatusTypeDef HTMotorSetCurrentPositionAsZero(HTMotorInstance* motor)
{
    if (motor == NULL)
        return HAL_ERROR;

    motor->torque_ref_nm = 0.0f;
    motor->stop_flag = MOTOR_STOP;
    if (HTMotorSendSpecialCommand(motor, HT04_CMD_ZERO_POSITION) != HAL_OK)
        return HAL_ERROR;

    motor->measure.position_rad = 0.0f;
    motor->measure.last_position_rad = 0.0f;
    return HAL_OK;
}

void HTMotorControl(void)
{
    for (uint8_t i = 0U; i < ht_motor_count; ++i)
    {
        HTMotorInstance* motor = ht_motor_instances[i];
        float torque_nm;

        if (motor == NULL || motor->motor_can_instance == NULL || motor->enabled == 0U)
            continue;

        torque_nm = (motor->stop_flag == MOTOR_ENABLED && motor->offline == 0U) ? motor->torque_ref_nm : 0.0f;
        if (motor->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            torque_nm = -torque_nm;

        HTMotorPackTorqueFrame(motor->motor_can_instance->tx_buff, torque_nm);
        (void)CANTransmit(motor->motor_can_instance, HT04_CAN_TIMEOUT_MS);
    }
}

uint8_t HTMotorIsOnline(const HTMotorInstance* motor)
{
    return motor != NULL && motor->offline == 0U;
}

uint8_t HTMotorIsEnabled(const HTMotorInstance* motor)
{
    return motor != NULL && motor->enabled != 0U;
}
