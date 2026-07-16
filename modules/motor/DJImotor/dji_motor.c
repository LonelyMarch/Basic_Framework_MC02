#include "dji_motor.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "general_def.h"
#include <string.h>

#define DJI_SENDER_MAX 12U
#define DJI_ECD_RANGE 8192
#define DJI_ECD_HALF_RANGE 4096
#define DJI_ECD_TO_DEGREE (360.0f / (float)DJI_ECD_RANGE)

typedef struct
{
    CANInstance can;
    uint8_t used;
    uint8_t active;
} DJISender_s;

static uint8_t dji_motor_count;
static DJIMotorInstance dji_motor_pool[DJI_MOTOR_CNT];
static DJIMotorInstance* dji_motor_instances[DJI_MOTOR_CNT];
static DJISender_s dji_sender_pool[DJI_SENDER_MAX];

static int16_t DJIClampI16(int32_t value, int16_t min, int16_t max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return (int16_t)value;
}

static int16_t DJIGetCommandLimit(DJIMotorProtocolMode_e mode)
{
    switch (mode)
    {
    case DJI_CONTROL_C610_CURRENT:
        return DJI_C610_CURRENT_RAW_MAX;
    case DJI_CONTROL_C620_CURRENT:
        return DJI_C620_CURRENT_RAW_MAX;
    case DJI_CONTROL_GM6020_CURRENT:
        return DJI_GM6020_CURRENT_RAW_MAX;
    case DJI_CONTROL_GM6020_VOLTAGE:
        return DJI_GM6020_VOLTAGE_RAW_MAX;
    default:
        return 0;
    }
}

static void DJIResetPID(PIDInstance* pid)
{
    PID_Init_Config_s config;
    if (pid == NULL)
        return;

    config.Kp = pid->Kp;
    config.Ki = pid->Ki;
    config.Kd = pid->Kd;
    config.MaxOut = pid->MaxOut;
    config.DeadBand = pid->DeadBand;
    config.Improve = pid->Improve;
    config.IntegralLimit = pid->IntegralLimit;
    config.CoefA = pid->CoefA;
    config.CoefB = pid->CoefB;
    config.Output_LPF_RC = pid->Output_LPF_RC;
    config.Derivative_LPF_RC = pid->Derivative_LPF_RC;
    PIDInit(pid, &config);
}

static void DJIResetController(DJIMotorInstance* motor)
{
    if (motor == NULL)
        return;
    if (motor->control_mode == DJI_CONTROL_POSITION)
        DJIResetPID(&motor->position_pid);
    if (motor->control_mode == DJI_CONTROL_SPEED || motor->control_mode == DJI_CONTROL_POSITION)
        DJIResetPID(&motor->speed_pid);
}

static DJISender_s* DJIGetOrCreateSender(FDCAN_HandleTypeDef* can_handle, uint32_t tx_id)
{
    DJISender_s* free_sender = NULL;

    for (uint8_t i = 0U; i < DJI_SENDER_MAX; i++)
    {
        DJISender_s* sender = &dji_sender_pool[i];
        if (sender->used != 0U)
        {
            if (sender->can.can_handle == can_handle && sender->can.txconf.Identifier == tx_id)
                return sender;
        }
        else if (free_sender == NULL)
        {
            free_sender = sender;
        }
    }

    if (free_sender == NULL)
    {
        LOGERROR("[dji_motor] sender pool exhausted");
        return NULL;
    }

    memset(free_sender, 0, sizeof(*free_sender));
    free_sender->used = 1U;
    free_sender->can.can_handle = can_handle;
    free_sender->can.tx_id = tx_id;
    free_sender->can.txconf.Identifier = tx_id;
    free_sender->can.txconf.IdType = FDCAN_STANDARD_ID;
    free_sender->can.txconf.TxFrameType = FDCAN_DATA_FRAME;
    free_sender->can.txconf.DataLength = FDCAN_DLC_BYTES_8;
    free_sender->can.txconf.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    free_sender->can.txconf.BitRateSwitch = FDCAN_BRS_OFF;
    free_sender->can.txconf.FDFormat = FDCAN_CLASSIC_CAN;
    free_sender->can.txconf.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    free_sender->can.txconf.MessageMarker = 0U;
    return free_sender;
}

static uint8_t DJIMapProtocol(Motor_Type_e motor_type,
                              DJIMotorProtocolMode_e protocol_mode,
                              uint8_t motor_id,
                              uint32_t* tx_id,
                              uint32_t* rx_id,
                              uint8_t* message_num)
{
    if (tx_id == NULL || rx_id == NULL || message_num == NULL)
        return 0U;

    switch (motor_type)
    {
    case M2006:
        if (protocol_mode != DJI_CONTROL_C610_CURRENT || motor_id < 1U || motor_id > 8U)
            return 0U;
        *tx_id = motor_id <= 4U ? 0x200U : 0x1FFU;
        *rx_id = 0x200U + motor_id;
        *message_num = (uint8_t)((motor_id - 1U) % 4U);
        return 1U;

    case M3508:
        if (protocol_mode != DJI_CONTROL_C620_CURRENT || motor_id < 1U || motor_id > 8U)
            return 0U;
        *tx_id = motor_id <= 4U ? 0x200U : 0x1FFU;
        *rx_id = 0x200U + motor_id;
        *message_num = (uint8_t)((motor_id - 1U) % 4U);
        return 1U;

    case GM6020:
        if (motor_id < 1U || motor_id > 7U)
            return 0U;
        if (protocol_mode == DJI_CONTROL_GM6020_CURRENT)
            *tx_id = motor_id <= 4U ? 0x1FEU : 0x2FEU;
        else if (protocol_mode == DJI_CONTROL_GM6020_VOLTAGE)
            *tx_id = motor_id <= 4U ? 0x1FFU : 0x2FFU;
        else
            return 0U;
        *rx_id = 0x204U + motor_id;
        *message_num = (uint8_t)((motor_id - 1U) % 4U);
        return 1U;

    default:
        return 0U;
    }
}

static uint8_t DJIHasRxIdConflict(FDCAN_HandleTypeDef* can_handle, uint32_t rx_id)
{
    for (uint8_t i = 0U; i < dji_motor_count; i++)
    {
        DJIMotorInstance* motor = dji_motor_instances[i];
        if (motor != NULL && motor->motor_can_instance != NULL &&
            motor->motor_can_instance->can_handle == can_handle &&
            motor->motor_can_instance->rx_id == rx_id)
        {
            return 1U;
        }
    }
    return 0U;
}

static uint8_t DJIHasSenderSlotConflict(CANInstance* sender, uint8_t message_num)
{
    for (uint8_t i = 0U; i < dji_motor_count; i++)
    {
        DJIMotorInstance* motor = dji_motor_instances[i];
        if (motor != NULL && motor->sender_can_instance == sender && motor->message_num == message_num)
            return 1U;
    }
    return 0U;
}

static void DJIDecodeFeedback(CANInstance* can_instance)
{
    DJIMotorInstance* motor;
    DJI_Motor_Measure_s* measure;
    uint8_t* data;
    uint16_t new_ecd;
    int32_t delta_ecd;

    if (can_instance == NULL || can_instance->id == NULL || can_instance->rx_len != 8U)
        return;

    motor = (DJIMotorInstance*)can_instance->id;
    measure = &motor->measure;
    data = can_instance->rx_buff;
    new_ecd = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);

    if (motor->offline_reported != 0U)
    {
        motor->offline_reported = 0U;
        LOGINFO("[dji_motor] motor resumed, rx id [0x%lx]", can_instance->rx_id);
    }
    DaemonReload(motor->daemon);
    motor->dt = DWT_GetDeltaT(&motor->feed_cnt);

    measure->ecd = new_ecd;
    measure->speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    measure->torque_current_raw = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    measure->angle_single_round = (float)new_ecd * DJI_ECD_TO_DEGREE;
    measure->speed_aps = (float)measure->speed_rpm * RPM_2_ANGLE_PER_SEC;

    if (motor->motor_type == M2006)
    {
        measure->temperature = 0U;
        measure->temperature_valid = 0U;
        measure->torque_current_a = (float)measure->torque_current_raw * 10.0f / 10000.0f;
    }
    else if (motor->motor_type == M3508)
    {
        measure->temperature = data[6];
        measure->temperature_valid = 1U;
        measure->torque_current_a = (float)measure->torque_current_raw * 20.0f / 16384.0f;
    }
    else
    {
        measure->temperature = data[6];
        measure->temperature_valid = 1U;
        measure->torque_current_a = (float)measure->torque_current_raw * 3.0f / 16384.0f;
    }

    if (measure->feedback_initialized == 0U)
    {
        measure->feedback_initialized = 1U;
        measure->last_ecd = new_ecd;
        measure->total_ecd = 0;
        measure->total_angle = 0.0f;
        return;
    }

    delta_ecd = (int32_t)new_ecd - (int32_t)measure->last_ecd;
    if (delta_ecd > DJI_ECD_HALF_RANGE)
        delta_ecd -= DJI_ECD_RANGE;
    else if (delta_ecd < -DJI_ECD_HALF_RANGE)
        delta_ecd += DJI_ECD_RANGE;

    measure->total_ecd += delta_ecd;
    measure->total_angle = (float)measure->total_ecd * DJI_ECD_TO_DEGREE;
    measure->last_ecd = new_ecd;
}

static void DJIMotorLostCallback(void* owner)
{
    DJIMotorInstance* motor = (DJIMotorInstance*)owner;
    if (motor == NULL || motor->motor_can_instance == NULL)
        return;

    motor->command_raw = 0;
    motor->stop_flag = MOTOR_STOP;
    motor->controller_reset_pending = 1U;
    if (motor->offline_reported == 0U)
    {
        motor->offline_reported = 1U;
        LOGWARNING("[dji_motor] motor lost, rx id [0x%lx]", motor->motor_can_instance->rx_id);
    }
}

static DJIMotorInstance* DJIMotorInitInternal(const DJIMotor_Init_Config_s* config,
                                              Motor_Type_e motor_type,
                                              DJIMotorProtocolMode_e protocol_mode)
{
    DJIMotorInstance* motor;
    DJISender_s* sender;
    CAN_Init_Config_s can_config;
    Daemon_Init_Config_s daemon_config = {0};
    uint32_t tx_id;
    uint32_t rx_id;
    uint8_t message_num;
    PID_Init_Config_s position_pid_config;
    PID_Init_Config_s speed_pid_config;

    if (config == NULL || config->can_init_config.can_handle == NULL)
    {
        LOGERROR("[dji_motor] invalid init config");
        return NULL;
    }
    if (dji_motor_count >= DJI_MOTOR_CNT)
    {
        LOGERROR("[dji_motor] instance pool exhausted");
        return NULL;
    }
    if (config->control_mode > DJI_CONTROL_POSITION || config->feedback_source > DJI_FEEDBACK_EXTERNAL)
    {
        LOGERROR("[dji_motor] invalid controller or feedback mode");
        return NULL;
    }
    if (DJIMapProtocol(motor_type, protocol_mode, (uint8_t)config->can_init_config.tx_id,
                       &tx_id, &rx_id, &message_num) == 0U)
    {
        LOGERROR("[dji_motor] invalid motor id or control mode");
        return NULL;
    }
    if (config->control_mode == DJI_CONTROL_SPEED &&
        config->feedback_source == DJI_FEEDBACK_EXTERNAL && config->speed_feedback_ptr == NULL)
    {
        LOGERROR("[dji_motor] external speed feedback is null");
        return NULL;
    }
    if (config->control_mode == DJI_CONTROL_POSITION &&
        config->feedback_source == DJI_FEEDBACK_EXTERNAL &&
        (config->position_feedback_ptr == NULL || config->speed_feedback_ptr == NULL))
    {
        LOGERROR("[dji_motor] external position/speed feedback is null");
        return NULL;
    }
    if (DJIHasRxIdConflict(config->can_init_config.can_handle, rx_id) != 0U)
    {
        LOGERROR("[dji_motor] feedback id conflict [0x%lx]", rx_id);
        return NULL;
    }

    sender = DJIGetOrCreateSender(config->can_init_config.can_handle, tx_id);
    if (sender == NULL)
        return NULL;
    if (DJIHasSenderSlotConflict(&sender->can, message_num) != 0U)
    {
        LOGERROR("[dji_motor] command slot conflict, tx [0x%lx], slot [%u]", tx_id,
                 (unsigned int)message_num);
        return NULL;
    }

    motor = &dji_motor_pool[dji_motor_count];
    memset(motor, 0, sizeof(*motor));
    motor->motor_type = motor_type;
    motor->protocol_mode = protocol_mode;
    motor->control_mode = config->control_mode;
    motor->feedback_source = config->feedback_source;
    motor->motor_reverse_flag = config->motor_reverse_flag;
    motor->position_feedback_ptr = config->position_feedback_ptr;
    motor->speed_feedback_ptr = config->speed_feedback_ptr;
    if (motor->control_mode == DJI_CONTROL_POSITION &&
        motor->feedback_source == DJI_FEEDBACK_EXTERNAL)
    {
        motor->control_ref = *motor->position_feedback_ptr;
    }
    motor->sender_can_instance = &sender->can;
    motor->message_num = message_num;
    motor->stop_flag = MOTOR_STOP;
    if (motor->control_mode == DJI_CONTROL_POSITION)
    {
        position_pid_config = config->position_pid;
        PIDInit(&motor->position_pid, &position_pid_config);
    }
    if (motor->control_mode == DJI_CONTROL_SPEED || motor->control_mode == DJI_CONTROL_POSITION)
    {
        speed_pid_config = config->speed_pid;
        PIDInit(&motor->speed_pid, &speed_pid_config);
    }

    can_config = config->can_init_config;
    can_config.rx_id = rx_id;
    can_config.tx_id = tx_id;
    can_config.id = motor;
    can_config.can_module_callback = DJIDecodeFeedback;
    motor->motor_can_instance = CANRegister(&can_config);
    if (motor->motor_can_instance == NULL)
    {
        LOGERROR("[dji_motor] CAN register failed");
        memset(motor, 0, sizeof(*motor));
        return NULL;
    }

    daemon_config.callback = DJIMotorLostCallback;
    daemon_config.owner_id = motor;
    daemon_config.reload_count = 2U;
    motor->daemon = DaemonRegister(&daemon_config);
    if (motor->daemon == NULL)
    {
        LOGERROR("[dji_motor] daemon register failed");
        memset(motor, 0, sizeof(*motor));
        return NULL;
    }

    sender->active = 1U;
    DJIMotorEnable(motor);
    dji_motor_instances[dji_motor_count++] = motor;
    return motor;
}

DJIMotorInstance* DJIMotorInitM2006(const DJIMotor_Init_Config_s* config)
{
    return DJIMotorInitInternal(config, M2006, DJI_CONTROL_C610_CURRENT);
}

DJIMotorInstance* DJIMotorInitM3508(const DJIMotor_Init_Config_s* config)
{
    return DJIMotorInitInternal(config, M3508, DJI_CONTROL_C620_CURRENT);
}

DJIMotorInstance* DJIMotorInitGM6020Voltage(const DJIMotor_Init_Config_s* config)
{
    return DJIMotorInitInternal(config, GM6020, DJI_CONTROL_GM6020_VOLTAGE);
}

DJIMotorInstance* DJIMotorInitGM6020Current(const DJIMotor_Init_Config_s* config)
{
    return DJIMotorInitInternal(config, GM6020, DJI_CONTROL_GM6020_CURRENT);
}

void DJIMotorSetCurrentRaw(DJIMotorInstance* motor, int16_t current_raw)
{
    int16_t limit;
    if (motor == NULL)
        return;
    if (motor->control_mode != DJI_CONTROL_DIRECT)
    {
        LOGERROR("[dji_motor] direct current command used for closed-loop instance");
        return;
    }
    if (motor->protocol_mode == DJI_CONTROL_GM6020_VOLTAGE)
    {
        LOGERROR("[dji_motor] current command used for voltage-mode GM6020");
        return;
    }

    limit = DJIGetCommandLimit(motor->protocol_mode);
    motor->command_raw = DJIClampI16(current_raw, (int16_t)-limit, limit);
}

void DJIMotorSetVoltageRaw(DJIMotorInstance* motor, int16_t voltage_raw)
{
    if (motor == NULL)
        return;
    if (motor->control_mode != DJI_CONTROL_DIRECT)
    {
        LOGERROR("[dji_motor] direct voltage command used for closed-loop instance");
        return;
    }
    if (motor->protocol_mode != DJI_CONTROL_GM6020_VOLTAGE)
    {
        LOGERROR("[dji_motor] voltage command used for current-mode motor");
        return;
    }
    motor->command_raw = DJIClampI16(voltage_raw,
                                     -DJI_GM6020_VOLTAGE_RAW_MAX,
                                     DJI_GM6020_VOLTAGE_RAW_MAX);
}

void DJIMotorSetSpeed(DJIMotorInstance* motor, float speed_aps)
{
    if (motor == NULL)
        return;
    if (motor->control_mode != DJI_CONTROL_SPEED)
    {
        LOGERROR("[dji_motor] speed reference used for non-speed instance");
        return;
    }
    motor->control_ref = speed_aps;
}

void DJIMotorSetPosition(DJIMotorInstance* motor, float position_degree)
{
    if (motor == NULL)
        return;
    if (motor->control_mode != DJI_CONTROL_POSITION)
    {
        LOGERROR("[dji_motor] position reference used for non-position instance");
        return;
    }
    motor->control_ref = position_degree;
}

void DJIMotorStop(DJIMotorInstance* motor)
{
    if (motor == NULL)
        return;
    motor->command_raw = 0;
    motor->stop_flag = MOTOR_STOP;
    motor->controller_reset_pending = 1U;
}

void DJIMotorEnable(DJIMotorInstance* motor)
{
    if (motor == NULL)
        return;
    if (motor->daemon != NULL && DaemonIsOnline(motor->daemon) == 0U)
    {
        motor->command_raw = 0;
        motor->stop_flag = MOTOR_STOP;
        return;
    }
    motor->stop_flag = MOTOR_ENABLED;
}

void DJIMotorControl(void)
{
    for (uint8_t i = 0U; i < dji_motor_count; i++)
    {
        DJIMotorInstance* motor = dji_motor_instances[i];
        int32_t command;
        uint16_t command_u16;
        uint8_t offset;

        if (motor == NULL || motor->sender_can_instance == NULL)
            continue;

        if (motor->controller_reset_pending != 0U)
        {
            DJIResetController(motor);
            motor->controller_reset_pending = 0U;
        }

        if (motor->stop_flag != MOTOR_ENABLED)
        {
            command = 0;
        }
        else if (motor->control_mode == DJI_CONTROL_DIRECT)
        {
            command = motor->command_raw;
        }
        else
        {
            float speed_measure;
            float output;

            if (motor->feedback_source == DJI_FEEDBACK_MOTOR &&
                motor->measure.feedback_initialized == 0U)
            {
                command = 0;
                goto dji_pack_command;
            }

            if (motor->feedback_source == DJI_FEEDBACK_EXTERNAL)
                speed_measure = *motor->speed_feedback_ptr;
            else
                speed_measure = motor->motor_reverse_flag == MOTOR_DIRECTION_REVERSE
                                    ? -motor->measure.speed_aps
                                    : motor->measure.speed_aps;

            if (motor->control_mode == DJI_CONTROL_POSITION)
            {
                float position_measure;
                float speed_ref;
                if (motor->feedback_source == DJI_FEEDBACK_EXTERNAL)
                    position_measure = *motor->position_feedback_ptr;
                else
                    position_measure = motor->motor_reverse_flag == MOTOR_DIRECTION_REVERSE
                                           ? -motor->measure.total_angle
                                           : motor->measure.total_angle;
                speed_ref = PIDCalculate(&motor->position_pid, position_measure, motor->control_ref);
                output = PIDCalculate(&motor->speed_pid, speed_measure, speed_ref);
            }
            else
            {
                output = PIDCalculate(&motor->speed_pid, speed_measure, motor->control_ref);
            }

            command = DJIClampI16((int32_t)output,
                                  (int16_t)-DJIGetCommandLimit(motor->protocol_mode),
                                  DJIGetCommandLimit(motor->protocol_mode));
        }

dji_pack_command:
        if (motor->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            command = -command;

        command_u16 = (uint16_t)(int16_t)command;
        offset = (uint8_t)(motor->message_num * 2U);
        motor->sender_can_instance->tx_buff[offset] = (uint8_t)(command_u16 >> 8);
        motor->sender_can_instance->tx_buff[offset + 1U] = (uint8_t)command_u16;
    }

    for (uint8_t i = 0U; i < DJI_SENDER_MAX; i++)
    {
        if (dji_sender_pool[i].active != 0U)
            CANTransmit(&dji_sender_pool[i].can, 1.0f);
    }
}
