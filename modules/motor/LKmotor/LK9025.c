#include "LK9025.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "cmsis_os2.h"
#include "daemon.h"
#include <string.h>

#define LK_SINGLE_CMD_TIMEOUT_MS 1.0f
#define LK_MULTI_CMD_TIMEOUT_MS 0.2f
#define LK_COMMAND_QUEUE_DEPTH 8U

typedef struct
{
    uint8_t data[LK_CAN_DLC];
} LKMotorCommandFrame;

struct LKMotorInstance
{
    LKMotor_Measure_t measure;

    uint8_t motor_id;
    uint8_t message_num;
    LKMotor_Work_Mode_e work_mode;
    volatile float ref;
    volatile Motor_Working_Type_e stop_flag;

    CANInstance* motor_can_ins;
    DaemonInstance* daemon;

    LKMotorCommandFrame command_queue[LK_COMMAND_QUEUE_DEPTH];
    volatile uint8_t command_read_idx;
    volatile uint8_t command_write_idx;
    volatile uint8_t command_count;

    uint8_t active_command[LK_CAN_DLC];
    volatile uint8_t active_command_valid;
    volatile uint32_t active_command_generation;

    volatile uint8_t offline_reported;
    volatile uint8_t off_latched;
};

static uint8_t idx;
static LKMotorInstance lkmotor_pool[LK_MOTOR_MX_CNT];
static LKMotorInstance* lkmotor_instance[LK_MOTOR_MX_CNT];

/* 当前模块只维护一个 0x280 多电机组；单电机实例不受该总线限制。 */
static CANInstance* multi_sender_instance;
static FDCAN_HandleTypeDef* multi_can_handle;
static uint32_t lkmotor_control_fail_cnt;

static uint32_t LKEnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void LKExitCritical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

static int16_t LKConstrainI16(int32_t value, int16_t min, int16_t max)
{
    if (value < min)
    {
        return min;
    }
    if (value > max)
    {
        return max;
    }
    return (int16_t)value;
}

static void LKWriteU16LE(uint8_t* buff, uint16_t value)
{
    buff[0] = (uint8_t)(value & 0xffU);
    buff[1] = (uint8_t)((value >> 8U) & 0xffU);
}

static void LKWriteI16LE(uint8_t* buff, int16_t value)
{
    LKWriteU16LE(buff, (uint16_t)value);
}

static void LKWriteU32LE(uint8_t* buff, uint32_t value)
{
    buff[0] = (uint8_t)(value & 0xffU);
    buff[1] = (uint8_t)((value >> 8U) & 0xffU);
    buff[2] = (uint8_t)((value >> 16U) & 0xffU);
    buff[3] = (uint8_t)((value >> 24U) & 0xffU);
}

static void LKWriteI32LE(uint8_t* buff, int32_t value)
{
    LKWriteU32LE(buff, (uint32_t)value);
}

static uint16_t LKReadU16LE(const uint8_t* buff)
{
    return ((uint16_t)buff[0]) | ((uint16_t)buff[1] << 8U);
}

static int16_t LKReadI16LE(const uint8_t* buff)
{
    return (int16_t)LKReadU16LE(buff);
}

static uint32_t LKReadU32LE(const uint8_t* buff)
{
    return ((uint32_t)buff[0]) |
        ((uint32_t)buff[1] << 8U) |
        ((uint32_t)buff[2] << 16U) |
        ((uint32_t)buff[3] << 24U);
}

static int32_t LKReadI32LE(const uint8_t* buff)
{
    return (int32_t)LKReadU32LE(buff);
}

static int64_t LKReadI56LE(const uint8_t* buff)
{
    uint64_t value = 0U;

    for (uint8_t i = 0U; i < 7U; i++)
    {
        value |= ((uint64_t)buff[i]) << (8U * i);
    }

    if ((value & (1ULL << 55U)) != 0U)
    {
        value |= 0xff00000000000000ULL;
    }

    return (int64_t)value;
}

static uint8_t LKMotorIsModeValid(LKMotor_Work_Mode_e mode)
{
    return mode >= LK_MOTOR_MODE_MULTI_TORQUE && mode <= LK_MOTOR_MODE_SINGLE_POSITION;
}

static uint8_t LKMotorIsSingleMode(const LKMotorInstance* motor)
{
    return motor != NULL && motor->work_mode != LK_MOTOR_MODE_MULTI_TORQUE;
}

static HAL_StatusTypeDef LKMotorRequireControlMode(const LKMotorInstance* motor,
                                                   LKMotor_Work_Mode_e required_mode)
{
    if (motor == NULL)
    {
        LOGERROR("[LKMotor] control with null motor");
        return HAL_ERROR;
    }

    if (motor->work_mode != required_mode)
    {
        LOGERROR("[LKMotor] control mode mismatch, id:%u registered:%u required:%u",
                 (unsigned int)motor->motor_id,
                 (unsigned int)motor->work_mode,
                 (unsigned int)required_mode);
        return HAL_ERROR;
    }

    if (motor->stop_flag != MOTOR_ENABLED)
    {
        return HAL_BUSY;
    }

    return HAL_OK;
}

static uint8_t LKMotorValidateConfig(const LKMotor_Init_Config_s* config)
{
    uint32_t motor_id;

    if (config == NULL || config->can_init_config.can_handle == NULL)
    {
        LOGERROR("[LKMotor] invalid init config");
        return 0U;
    }

    if (LKMotorIsModeValid(config->work_mode) == 0U)
    {
        LOGERROR("[LKMotor] invalid work mode:%u", (unsigned int)config->work_mode);
        return 0U;
    }

    motor_id = config->can_init_config.tx_id;
    if (config->work_mode == LK_MOTOR_MODE_MULTI_TORQUE)
    {
        if (motor_id < LK_MULTI_MOTOR_ID_MIN || motor_id > LK_MULTI_MOTOR_ID_MAX)
        {
            LOGERROR("[LKMotor] multi mode motor id must be 1-4, got:%u", (unsigned int)motor_id);
            return 0U;
        }

        if (multi_can_handle != NULL && multi_can_handle != config->can_init_config.can_handle)
        {
            LOGERROR("[LKMotor] all 0x280 motors must use the same FDCAN bus");
            return 0U;
        }
    }
    else if (motor_id < LK_SINGLE_MOTOR_ID_MIN || motor_id > LK_SINGLE_MOTOR_ID_MAX)
    {
        LOGERROR("[LKMotor] single mode motor id must be 1-32, got:%u", (unsigned int)motor_id);
        return 0U;
    }

    for (uint8_t i = 0U; i < idx; i++)
    {
        if (lkmotor_instance[i] != NULL &&
            lkmotor_instance[i]->motor_can_ins != NULL &&
            lkmotor_instance[i]->motor_can_ins->can_handle == config->can_init_config.can_handle &&
            lkmotor_instance[i]->motor_id == motor_id)
        {
            LOGERROR("[LKMotor] duplicate motor id:%u on the same FDCAN bus", (unsigned int)motor_id);
            return 0U;
        }
    }

    return 1U;
}

static void LKMotorClearCommandQueue(LKMotorInstance* motor)
{
    uint32_t primask;

    if (motor == NULL)
    {
        return;
    }

    primask = LKEnterCritical();
    motor->command_read_idx = 0U;
    motor->command_write_idx = 0U;
    motor->command_count = 0U;
    motor->active_command_valid = 0U;
    motor->active_command_generation++;
    LKExitCritical(primask);
}

static HAL_StatusTypeDef LKMotorQueueFrame(LKMotorInstance* motor,
                                           const uint8_t tx_data[LK_CAN_DLC])
{
    uint32_t primask;
    uint8_t write_idx;

    if (motor == NULL || tx_data == NULL || LKMotorIsSingleMode(motor) == 0U)
    {
        return HAL_ERROR;
    }

    primask = LKEnterCritical();
    if (motor->command_count >= LK_COMMAND_QUEUE_DEPTH)
    {
        LKExitCritical(primask);
        return HAL_BUSY;
    }

    write_idx = motor->command_write_idx;
    memcpy(motor->command_queue[write_idx].data, tx_data, LK_CAN_DLC);
    motor->command_write_idx = (uint8_t)((write_idx + 1U) % LK_COMMAND_QUEUE_DEPTH);
    motor->command_count++;
    LKExitCritical(primask);
    return HAL_OK;
}

/* 停止/关闭命令覆盖普通队列，并抢占尚未成功发送的普通命令。 */
static HAL_StatusTypeDef LKMotorQueueUrgentCommand(LKMotorInstance* motor, LKMotor_Command_e cmd)
{
    uint8_t tx_data[LK_CAN_DLC] = {0};
    uint32_t primask;

    if (motor == NULL || LKMotorIsSingleMode(motor) == 0U)
    {
        return HAL_ERROR;
    }

    tx_data[0] = (uint8_t)cmd;
    primask = LKEnterCritical();
    motor->command_read_idx = 0U;
    motor->command_write_idx = 0U;
    motor->command_count = 0U;
    memcpy(motor->active_command, tx_data, LK_CAN_DLC);
    motor->active_command_valid = 1U;
    motor->active_command_generation++;
    LKExitCritical(primask);
    return HAL_OK;
}

static uint8_t LKMotorSafetyCommandPending(LKMotorInstance* motor)
{
    uint32_t primask;
    uint8_t pending;

    if (motor == NULL)
    {
        return 0U;
    }

    primask = LKEnterCritical();
    pending = motor->active_command_valid != 0U &&
    (motor->active_command[0] == (uint8_t)LK_CMD_MOTOR_STOP ||
        motor->active_command[0] == (uint8_t)LK_CMD_MOTOR_OFF);
    LKExitCritical(primask);
    return pending;
}

static HAL_StatusTypeDef LKMotorSendSingleCommand(LKMotorInstance* motor,
                                                  LKMotor_Command_e cmd,
                                                  const uint8_t data[LK_CAN_DLC])
{
    uint8_t tx_data[LK_CAN_DLC] = {0};

    if (motor == NULL || LKMotorIsSingleMode(motor) == 0U)
    {
        LOGERROR("[LKMotor] single command is unavailable in 0x280 mode");
        return HAL_ERROR;
    }

    if (data != NULL)
    {
        memcpy(tx_data, data, LK_CAN_DLC);
    }
    tx_data[0] = (uint8_t)cmd;
    return LKMotorQueueFrame(motor, tx_data);
}

static void LKMotorFlushPendingCommand(LKMotorInstance* motor)
{
    uint8_t tx_data[LK_CAN_DLC];
    uint32_t generation;
    uint32_t primask;

    if (motor == NULL || motor->motor_can_ins == NULL || LKMotorIsSingleMode(motor) == 0U)
    {
        return;
    }

    primask = LKEnterCritical();
    if (motor->active_command_valid == 0U && motor->command_count > 0U)
    {
        memcpy(motor->active_command,
               motor->command_queue[motor->command_read_idx].data,
               LK_CAN_DLC);
        motor->command_read_idx = (uint8_t)((motor->command_read_idx + 1U) % LK_COMMAND_QUEUE_DEPTH);
        motor->command_count--;
        motor->active_command_valid = 1U;
        motor->active_command_generation++;
    }

    if (motor->active_command_valid == 0U)
    {
        LKExitCritical(primask);
        return;
    }

    memcpy(tx_data, motor->active_command, LK_CAN_DLC);
    generation = motor->active_command_generation;
    LKExitCritical(primask);

    motor->motor_can_ins->tx_id = LK_SINGLE_CAN_ID_BASE + motor->motor_id;
    motor->motor_can_ins->txconf.Identifier = LK_SINGLE_CAN_ID_BASE + motor->motor_id;
    motor->motor_can_ins->txconf.DataLength = FDCAN_DLC_BYTES_8;
    memcpy(motor->motor_can_ins->tx_buff, tx_data, LK_CAN_DLC);

    if (CANTransmit(motor->motor_can_ins, LK_SINGLE_CMD_TIMEOUT_MS) != 0U)
    {
        primask = LKEnterCritical();
        if (motor->active_command_generation == generation)
        {
            motor->active_command_valid = 0U;
        }
        LKExitCritical(primask);
    }
}

static void LKMotorUpdateCommonFeedback(LKMotorInstance* motor,
                                        const uint8_t* rx_buff,
                                        LKMotor_Command_e cmd)
{
    LKMotor_Measure_t* measure = &motor->measure;
    uint16_t current_ecd;
    int32_t ecd_delta;

    measure->temperature = (int8_t)rx_buff[1];
    if (cmd == LK_CMD_OPEN_TORQUE)
    {
        measure->open_power = LKReadI16LE(&rx_buff[2]);
        measure->open_power_valid = 1U;
        measure->real_current_valid = 0U;
    }
    else
    {
        measure->real_current = LKReadI16LE(&rx_buff[2]);
        measure->real_current_valid = 1U;
        measure->open_power_valid = 0U;
    }
    measure->speed_dps = LKReadI16LE(&rx_buff[4]);
    current_ecd = LKReadU16LE(&rx_buff[6]) & (LK_ENCODER_RANGE - 1U);

    if (measure->feedback_initialized == 0U)
    {
        measure->last_ecd = current_ecd;
        measure->ecd = current_ecd;
        measure->angle_single_round = LK_ECD_ANGLE_COEF * (float)current_ecd;
        measure->total_round = 0;
        measure->total_angle = measure->angle_single_round;
        measure->feedback_initialized = 1U;
        return;
    }

    measure->last_ecd = measure->ecd;
    measure->ecd = current_ecd;
    measure->angle_single_round = LK_ECD_ANGLE_COEF * (float)current_ecd;
    ecd_delta = (int32_t)measure->ecd - (int32_t)measure->last_ecd;
    if (ecd_delta > LK_ENCODER_HALF_RANGE)
    {
        measure->total_round--;
    }
    else if (ecd_delta < -LK_ENCODER_HALF_RANGE)
    {
        measure->total_round++;
    }
    measure->total_angle = (float)measure->total_round * 360.0f + measure->angle_single_round;
}

static void LKMotorResetAccumulatedAngle(LKMotorInstance* motor)
{
    motor->measure.feedback_initialized = 0U;
    motor->measure.last_ecd = 0U;
    motor->measure.ecd = 0U;
    motor->measure.total_round = 0;
    motor->measure.total_angle = 0.0f;
}

static void LKMotorDecode(CANInstance* instance)
{
    LKMotorInstance* motor;
    LKMotor_Measure_t* measure;
    uint8_t* rx_buff;
    LKMotor_Command_e cmd;

    if (instance == NULL || instance->id == NULL || instance->rx_len < LK_CAN_DLC)
    {
        return;
    }

    motor = (LKMotorInstance*)instance->id;
    measure = &motor->measure;
    rx_buff = instance->rx_buff;
    cmd = (LKMotor_Command_e)rx_buff[0];

    if (motor->offline_reported != 0U)
    {
        motor->offline_reported = 0U;
        LOGINFO("[LKMotor] motor feedback recovered, id:%u", (unsigned int)motor->motor_id);
    }
    DaemonReload(motor->daemon);
    measure->feed_dt = DWT_GetDeltaT(&measure->feed_dwt_cnt);

    switch (cmd)
    {
    case LK_CMD_READ_PID:
    case LK_CMD_WRITE_PID_RAM:
    case LK_CMD_WRITE_PID_ROM:
        measure->pid_param.angle_kp = rx_buff[2];
        measure->pid_param.angle_ki = rx_buff[3];
        measure->pid_param.speed_kp = rx_buff[4];
        measure->pid_param.speed_ki = rx_buff[5];
        measure->pid_param.iq_kp = rx_buff[6];
        measure->pid_param.iq_ki = rx_buff[7];
        break;

    case LK_CMD_READ_ACCEL:
    case LK_CMD_WRITE_ACCEL_RAM:
        measure->accel_dps2 = LKReadI32LE(&rx_buff[4]);
        break;

    case LK_CMD_READ_ENCODER:
        measure->encoder.encoder = LKReadU16LE(&rx_buff[2]) & (LK_ENCODER_RANGE - 1U);
        measure->encoder.encoder_raw = LKReadU16LE(&rx_buff[4]) & (LK_ENCODER_RANGE - 1U);
        measure->encoder.encoder_offset = LKReadU16LE(&rx_buff[6]) & (LK_ENCODER_RANGE - 1U);
        break;

    case LK_CMD_WRITE_ENCODER_ZERO_ROM:
    case LK_CMD_WRITE_CURRENT_POS_ZERO_ROM:
        measure->encoder.encoder_offset = LKReadU16LE(&rx_buff[6]) & (LK_ENCODER_RANGE - 1U);
        LKMotorResetAccumulatedAngle(motor);
        break;

    case LK_CMD_READ_MULTI_TURN_ANGLE:
        measure->multi_turn_angle_0p01deg = LKReadI56LE(&rx_buff[1]);
        break;

    case LK_CMD_READ_SINGLE_TURN_ANGLE:
        measure->single_turn_angle_0p01deg = LKReadU32LE(&rx_buff[4]);
        break;

    case LK_CMD_READ_STATUS1_ERROR:
    case LK_CMD_CLEAR_ERROR:
        measure->status1.temperature = (int8_t)rx_buff[1];
        measure->status1.voltage_0p1v = LKReadU16LE(&rx_buff[3]);
        measure->status1.error_state = rx_buff[7];
        break;

    case LK_CMD_READ_STATUS3:
        measure->status3.temperature = (int8_t)rx_buff[1];
        measure->status3.phase_a = LKReadI16LE(&rx_buff[2]);
        measure->status3.phase_b = LKReadI16LE(&rx_buff[4]);
        measure->status3.phase_c = LKReadI16LE(&rx_buff[6]);
        break;

    case LK_CMD_READ_STATUS2:
    case LK_CMD_OPEN_TORQUE:
    case LK_CMD_TORQUE_CONTROL:
    case LK_CMD_SPEED_CONTROL:
    case LK_CMD_MULTI_TURN_POSITION:
    case LK_CMD_MULTI_TURN_POSITION_WITH_SPEED:
    case LK_CMD_SINGLE_TURN_POSITION:
    case LK_CMD_SINGLE_TURN_POSITION_WITH_SPEED:
    case LK_CMD_INCREMENT_POSITION:
    case LK_CMD_INCREMENT_POSITION_WITH_SPEED:
        LKMotorUpdateCommonFeedback(motor, rx_buff, cmd);
        break;

    case LK_CMD_CLEAR_ANGLE:
        LKMotorResetAccumulatedAngle(motor);
        break;

    case LK_CMD_MOTOR_OFF:
    case LK_CMD_MOTOR_STOP:
    case LK_CMD_MOTOR_RUN:
    default:
        break;
    }
}

static void LKMotorLostCallback(void* motor_ptr)
{
    LKMotorInstance* motor = (LKMotorInstance*)motor_ptr;

    if (motor == NULL || motor->offline_reported != 0U)
    {
        return;
    }

    motor->offline_reported = 1U;
    motor->off_latched = 1U;
    motor->stop_flag = MOTOR_STOP;
    motor->ref = 0.0f;
    if (LKMotorIsSingleMode(motor) != 0U)
    {
        (void)LKMotorQueueUrgentCommand(motor, LK_CMD_MOTOR_OFF);
    }
    else
    {
        LKMotorClearCommandQueue(motor);
    }
    LOGWARNING("[LKMotor] motor lost and stopped, id:%u", (unsigned int)motor->motor_id);
}

LKMotorInstance* LKMotorInit(const LKMotor_Init_Config_s* config)
{
    LKMotorInstance* motor;
    CAN_Init_Config_s can_config;
    Daemon_Init_Config_s daemon_config;
    uint8_t motor_id;

    if (LKMotorValidateConfig(config) == 0U || idx >= LK_MOTOR_MX_CNT)
    {
        return NULL;
    }

    motor_id = (uint8_t)config->can_init_config.tx_id;
    motor = &lkmotor_pool[idx];
    memset(motor, 0, sizeof(*motor));
    motor->motor_id = motor_id;
    motor->work_mode = config->work_mode;
    motor->stop_flag = MOTOR_ENABLED;
    if (config->work_mode == LK_MOTOR_MODE_MULTI_TORQUE)
    {
        motor->message_num = (uint8_t)(motor_id - 1U);
    }

    can_config = config->can_init_config;
    can_config.id = motor;
    can_config.can_module_callback = LKMotorDecode;
    can_config.rx_id = LK_SINGLE_CAN_ID_BASE + motor_id;
    can_config.tx_id = LK_SINGLE_CAN_ID_BASE + motor_id;
    motor->motor_can_ins = CANRegister(&can_config);
    if (motor->motor_can_ins == NULL)
    {
        LOGERROR("[LKMotor] CAN register failed");
        memset(motor, 0, sizeof(*motor));
        return NULL;
    }

    memset(&daemon_config, 0, sizeof(daemon_config));
    daemon_config.callback = LKMotorLostCallback;
    daemon_config.owner_id = motor;
    daemon_config.reload_count = 5U;
    motor->daemon = DaemonRegister(&daemon_config);
    if (motor->daemon == NULL)
    {
        LOGERROR("[LKMotor] daemon register failed");
        return NULL;
    }

    if (config->work_mode == LK_MOTOR_MODE_MULTI_TORQUE && multi_sender_instance == NULL)
    {
        multi_sender_instance = motor->motor_can_ins;
        multi_can_handle = config->can_init_config.can_handle;
    }

    DWT_GetDeltaT(&motor->measure.feed_dwt_cnt);
    lkmotor_instance[idx++] = motor;
    return motor;
}

LKMotor_Work_Mode_e LKMotorGetMode(const LKMotorInstance* motor)
{
    return motor == NULL ? LK_MOTOR_MODE_MULTI_TORQUE : motor->work_mode;
}

HAL_StatusTypeDef LKMotorSetMultiTorque(LKMotorInstance* motor, int16_t iq)
{
    HAL_StatusTypeDef status = LKMotorRequireControlMode(motor, LK_MOTOR_MODE_MULTI_TORQUE);

    if (status != HAL_OK)
    {
        return status;
    }

    motor->ref = (float)LKConstrainI16(iq, LK_MULTI_TORQUE_MIN, LK_MULTI_TORQUE_MAX);
    return HAL_OK;
}

HAL_StatusTypeDef LKMotorSetOpenTorque(LKMotorInstance* motor, int16_t power_control)
{
    uint8_t tx_data[LK_CAN_DLC] = {0};
    HAL_StatusTypeDef status = LKMotorRequireControlMode(motor, LK_MOTOR_MODE_SINGLE_OPEN_TORQUE);

    if (status != HAL_OK)
    {
        return status;
    }
    power_control = LKConstrainI16(power_control, LK_OPEN_POWER_MIN, LK_OPEN_POWER_MAX);
    LKWriteI16LE(&tx_data[4], power_control);
    return LKMotorSendSingleCommand(motor, LK_CMD_OPEN_TORQUE, tx_data);
}

HAL_StatusTypeDef LKMotorSetTorque(LKMotorInstance* motor, int16_t iq_control)
{
    uint8_t tx_data[LK_CAN_DLC] = {0};
    HAL_StatusTypeDef status = LKMotorRequireControlMode(motor, LK_MOTOR_MODE_SINGLE_TORQUE);

    if (status != HAL_OK)
    {
        return status;
    }
    iq_control = LKConstrainI16(iq_control, LK_TORQUE_CURRENT_MIN, LK_TORQUE_CURRENT_MAX);
    LKWriteI16LE(&tx_data[4], iq_control);
    return LKMotorSendSingleCommand(motor, LK_CMD_TORQUE_CONTROL, tx_data);
}

HAL_StatusTypeDef LKMotorSetSpeed(LKMotorInstance* motor, int32_t speed_0p01dps)
{
    uint8_t tx_data[LK_CAN_DLC] = {0};
    HAL_StatusTypeDef status = LKMotorRequireControlMode(motor, LK_MOTOR_MODE_SINGLE_SPEED);

    if (status != HAL_OK)
    {
        return status;
    }
    LKWriteI32LE(&tx_data[4], speed_0p01dps);
    return LKMotorSendSingleCommand(motor, LK_CMD_SPEED_CONTROL, tx_data);
}

HAL_StatusTypeDef LKMotorSetMultiTurnPosition(LKMotorInstance* motor, int32_t angle_0p01deg)
{
    uint8_t tx_data[LK_CAN_DLC] = {0};
    HAL_StatusTypeDef status = LKMotorRequireControlMode(motor, LK_MOTOR_MODE_SINGLE_POSITION);

    if (status != HAL_OK)
    {
        return status;
    }
    LKWriteI32LE(&tx_data[4], angle_0p01deg);
    return LKMotorSendSingleCommand(motor, LK_CMD_MULTI_TURN_POSITION, tx_data);
}

HAL_StatusTypeDef LKMotorSetMultiTurnPositionWithSpeed(LKMotorInstance* motor,
                                                       int32_t angle_0p01deg,
                                                       uint16_t max_speed_dps)
{
    uint8_t tx_data[LK_CAN_DLC] = {0};
    HAL_StatusTypeDef status = LKMotorRequireControlMode(motor, LK_MOTOR_MODE_SINGLE_POSITION);

    if (status != HAL_OK)
    {
        return status;
    }
    LKWriteU16LE(&tx_data[2], max_speed_dps);
    LKWriteI32LE(&tx_data[4], angle_0p01deg);
    return LKMotorSendSingleCommand(motor, LK_CMD_MULTI_TURN_POSITION_WITH_SPEED, tx_data);
}

static HAL_StatusTypeDef LKMotorValidatePositionDirection(LKMotor_Spin_Direction_e direction)
{
    return (direction == LK_MOTOR_SPIN_CW || direction == LK_MOTOR_SPIN_CCW) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef LKMotorSetSingleTurnPosition(LKMotorInstance* motor,
                                               LKMotor_Spin_Direction_e direction,
                                               uint16_t angle_0p01deg)
{
    uint8_t tx_data[LK_CAN_DLC] = {0};
    HAL_StatusTypeDef status = LKMotorRequireControlMode(motor, LK_MOTOR_MODE_SINGLE_POSITION);

    if (status != HAL_OK)
    {
        return status;
    }
    if (LKMotorValidatePositionDirection(direction) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (angle_0p01deg > LK_SINGLE_CIRCLE_ANGLE_MAX)
    {
        angle_0p01deg = LK_SINGLE_CIRCLE_ANGLE_MAX;
    }
    tx_data[1] = (uint8_t)direction;
    LKWriteU16LE(&tx_data[4], angle_0p01deg);
    return LKMotorSendSingleCommand(motor, LK_CMD_SINGLE_TURN_POSITION, tx_data);
}

HAL_StatusTypeDef LKMotorSetSingleTurnPositionWithSpeed(LKMotorInstance* motor,
                                                        LKMotor_Spin_Direction_e direction,
                                                        uint16_t angle_0p01deg,
                                                        uint16_t max_speed_dps)
{
    uint8_t tx_data[LK_CAN_DLC] = {0};
    HAL_StatusTypeDef status = LKMotorRequireControlMode(motor, LK_MOTOR_MODE_SINGLE_POSITION);

    if (status != HAL_OK)
    {
        return status;
    }
    if (LKMotorValidatePositionDirection(direction) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (angle_0p01deg > LK_SINGLE_CIRCLE_ANGLE_MAX)
    {
        angle_0p01deg = LK_SINGLE_CIRCLE_ANGLE_MAX;
    }
    tx_data[1] = (uint8_t)direction;
    LKWriteU16LE(&tx_data[2], max_speed_dps);
    LKWriteU16LE(&tx_data[4], angle_0p01deg);
    return LKMotorSendSingleCommand(motor, LK_CMD_SINGLE_TURN_POSITION_WITH_SPEED, tx_data);
}

HAL_StatusTypeDef LKMotorSetIncrementPosition(LKMotorInstance* motor, int32_t delta_angle_0p01deg)
{
    uint8_t tx_data[LK_CAN_DLC] = {0};
    HAL_StatusTypeDef status = LKMotorRequireControlMode(motor, LK_MOTOR_MODE_SINGLE_POSITION);

    if (status != HAL_OK)
    {
        return status;
    }
    LKWriteI32LE(&tx_data[4], delta_angle_0p01deg);
    return LKMotorSendSingleCommand(motor, LK_CMD_INCREMENT_POSITION, tx_data);
}

HAL_StatusTypeDef LKMotorSetIncrementPositionWithSpeed(LKMotorInstance* motor,
                                                       int32_t delta_angle_0p01deg,
                                                       uint16_t max_speed_dps)
{
    uint8_t tx_data[LK_CAN_DLC] = {0};
    HAL_StatusTypeDef status = LKMotorRequireControlMode(motor, LK_MOTOR_MODE_SINGLE_POSITION);

    if (status != HAL_OK)
    {
        return status;
    }
    LKWriteU16LE(&tx_data[2], max_speed_dps);
    LKWriteI32LE(&tx_data[4], delta_angle_0p01deg);
    return LKMotorSendSingleCommand(motor, LK_CMD_INCREMENT_POSITION_WITH_SPEED, tx_data);
}

HAL_StatusTypeDef LKMotorStop(LKMotorInstance* motor)
{
    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    motor->stop_flag = MOTOR_STOP;
    motor->off_latched = 0U;
    motor->ref = 0.0f;
    if (LKMotorIsSingleMode(motor) != 0U)
    {
        return LKMotorQueueUrgentCommand(motor, LK_CMD_MOTOR_STOP);
    }
    return HAL_OK;
}

HAL_StatusTypeDef LKMotorEnable(LKMotorInstance* motor)
{
    if (motor == NULL || motor->daemon == NULL)
    {
        return HAL_ERROR;
    }

    if (DaemonIsOnline(motor->daemon) == 0U)
    {
        return HAL_BUSY;
    }

    if (LKMotorIsSingleMode(motor) != 0U && LKMotorSafetyCommandPending(motor) != 0U)
    {
        return HAL_BUSY;
    }

    if (LKMotorIsSingleMode(motor) != 0U && motor->off_latched == 0U)
    {
        if (LKMotorQueueUrgentCommand(motor, LK_CMD_MOTOR_RUN) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }
    motor->off_latched = 0U;
    motor->stop_flag = MOTOR_ENABLED;
    return HAL_OK;
}

HAL_StatusTypeDef LKMotorOff(LKMotorInstance* motor)
{
    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    motor->stop_flag = MOTOR_STOP;
    motor->off_latched = 1U;
    motor->ref = 0.0f;
    if (LKMotorIsSingleMode(motor) != 0U)
    {
        return LKMotorQueueUrgentCommand(motor, LK_CMD_MOTOR_OFF);
    }
    return HAL_OK;
}

uint8_t LKMotorIsOnline(const LKMotorInstance* motor)
{
    return motor != NULL && motor->daemon != NULL ? DaemonIsOnline(motor->daemon) : 0U;
}

uint8_t LKMotorGetMeasure(LKMotorInstance* motor, LKMotor_Measure_t* measure)
{
    int32_t kernel_lock = -1;

    if (motor == NULL || measure == NULL || __get_IPSR() != 0U)
    {
        return 0U;
    }

    if (osKernelGetState() == osKernelRunning)
    {
        kernel_lock = osKernelLock();
        if (kernel_lock < 0)
        {
            return 0U;
        }
    }

    *measure = motor->measure;

    if (kernel_lock >= 0)
    {
        (void)osKernelRestoreLock(kernel_lock);
    }
    return 1U;
}

HAL_StatusTypeDef LKMotorReadPID(LKMotorInstance* motor)
{
    return LKMotorSendSingleCommand(motor, LK_CMD_READ_PID, NULL);
}

HAL_StatusTypeDef LKMotorWritePIDToRAM(LKMotorInstance* motor, const LKMotor_PID_Param_s* pid)
{
    uint8_t tx_data[LK_CAN_DLC] = {0};

    if (pid == NULL)
    {
        return HAL_ERROR;
    }
    tx_data[2] = pid->angle_kp;
    tx_data[3] = pid->angle_ki;
    tx_data[4] = pid->speed_kp;
    tx_data[5] = pid->speed_ki;
    tx_data[6] = pid->iq_kp;
    tx_data[7] = pid->iq_ki;
    return LKMotorSendSingleCommand(motor, LK_CMD_WRITE_PID_RAM, tx_data);
}

HAL_StatusTypeDef LKMotorWritePIDToROM(LKMotorInstance* motor, const LKMotor_PID_Param_s* pid)
{
    uint8_t tx_data[LK_CAN_DLC] = {0};

    if (pid == NULL)
    {
        return HAL_ERROR;
    }
    tx_data[2] = pid->angle_kp;
    tx_data[3] = pid->angle_ki;
    tx_data[4] = pid->speed_kp;
    tx_data[5] = pid->speed_ki;
    tx_data[6] = pid->iq_kp;
    tx_data[7] = pid->iq_ki;
    return LKMotorSendSingleCommand(motor, LK_CMD_WRITE_PID_ROM, tx_data);
}

HAL_StatusTypeDef LKMotorReadAcceleration(LKMotorInstance* motor)
{
    return LKMotorSendSingleCommand(motor, LK_CMD_READ_ACCEL, NULL);
}

HAL_StatusTypeDef LKMotorWriteAccelerationToRAM(LKMotorInstance* motor, int32_t accel_dps2)
{
    uint8_t tx_data[LK_CAN_DLC] = {0};
    LKWriteI32LE(&tx_data[4], accel_dps2);
    return LKMotorSendSingleCommand(motor, LK_CMD_WRITE_ACCEL_RAM, tx_data);
}

HAL_StatusTypeDef LKMotorReadEncoder(LKMotorInstance* motor)
{
    return LKMotorSendSingleCommand(motor, LK_CMD_READ_ENCODER, NULL);
}

HAL_StatusTypeDef LKMotorWriteEncoderZeroToROM(LKMotorInstance* motor, uint16_t encoder_offset)
{
    uint8_t tx_data[LK_CAN_DLC] = {0};
    encoder_offset &= (LK_ENCODER_RANGE - 1U);
    LKWriteU16LE(&tx_data[6], encoder_offset);
    return LKMotorSendSingleCommand(motor, LK_CMD_WRITE_ENCODER_ZERO_ROM, tx_data);
}

HAL_StatusTypeDef LKMotorSetCurrentPositionAsZero(LKMotorInstance* motor)
{
    return LKMotorSendSingleCommand(motor, LK_CMD_WRITE_CURRENT_POS_ZERO_ROM, NULL);
}

HAL_StatusTypeDef LKMotorReadMultiTurnAngle(LKMotorInstance* motor)
{
    return LKMotorSendSingleCommand(motor, LK_CMD_READ_MULTI_TURN_ANGLE, NULL);
}

HAL_StatusTypeDef LKMotorReadSingleTurnAngle(LKMotorInstance* motor)
{
    return LKMotorSendSingleCommand(motor, LK_CMD_READ_SINGLE_TURN_ANGLE, NULL);
}

HAL_StatusTypeDef LKMotorClearAngle(LKMotorInstance* motor)
{
    return LKMotorSendSingleCommand(motor, LK_CMD_CLEAR_ANGLE, NULL);
}

HAL_StatusTypeDef LKMotorReadStatus1AndError(LKMotorInstance* motor)
{
    return LKMotorSendSingleCommand(motor, LK_CMD_READ_STATUS1_ERROR, NULL);
}

HAL_StatusTypeDef LKMotorClearError(LKMotorInstance* motor)
{
    return LKMotorSendSingleCommand(motor, LK_CMD_CLEAR_ERROR, NULL);
}

HAL_StatusTypeDef LKMotorReadStatus2(LKMotorInstance* motor)
{
    return LKMotorSendSingleCommand(motor, LK_CMD_READ_STATUS2, NULL);
}

HAL_StatusTypeDef LKMotorReadStatus3(LKMotorInstance* motor)
{
    return LKMotorSendSingleCommand(motor, LK_CMD_READ_STATUS3, NULL);
}

void LKMotorControl(void)
{
    uint8_t tx_data[LK_CAN_DLC] = {0};
    uint8_t need_multi_send = 0U;

    for (uint8_t i = 0U; i < idx; i++)
    {
        LKMotorInstance* motor = lkmotor_instance[i];
        if (motor == NULL)
        {
            continue;
        }

        if (LKMotorIsSingleMode(motor) != 0U)
        {
            LKMotorFlushPendingCommand(motor);
            continue;
        }

        int16_t set = LKConstrainI16((int32_t)motor->ref, LK_MULTI_TORQUE_MIN, LK_MULTI_TORQUE_MAX);
        uint8_t offset = (uint8_t)(motor->message_num * 2U);
        if (motor->stop_flag != MOTOR_ENABLED)
        {
            set = 0;
        }
        LKWriteI16LE(&tx_data[offset], set);
        need_multi_send = 1U;
    }

    if (need_multi_send == 0U || multi_sender_instance == NULL)
    {
        return;
    }

    multi_sender_instance->tx_id = LK_MULTI_TORQUE_CAN_ID;
    multi_sender_instance->txconf.Identifier = LK_MULTI_TORQUE_CAN_ID;
    multi_sender_instance->txconf.DataLength = FDCAN_DLC_BYTES_8;
    memcpy(multi_sender_instance->tx_buff, tx_data, LK_CAN_DLC);

    if (CANTransmit(multi_sender_instance, LK_MULTI_CMD_TIMEOUT_MS) != 0U)
    {
        if (lkmotor_control_fail_cnt >= 10U)
        {
            LOGINFO("[LKMotor] 0x280 frame resumed");
        }
        lkmotor_control_fail_cnt = 0U;
    }
    else
    {
        lkmotor_control_fail_cnt++;
        if (lkmotor_control_fail_cnt == 10U)
        {
            LOGWARNING("[LKMotor] 0x280 frame send failed 10 times consecutively");
        }
    }
}
