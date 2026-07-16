/**
 * @file ddt_motor.c
 * @brief 本末科技 M0601C 电机 RS485 通信驱动实现
 */

#include "ddt_motor.h"

#include <math.h>
#include <string.h>

#include "bsp_dwt.h"
#include "bsp_log.h"

#define DDT_MOTOR_TOTAL_COUNT (DDT_MOTOR_BUS_COUNT * DDT_MOTOR_COUNT_PER_BUS)
#define DDT_MOTOR_DEFAULT_RESPONSE_TIMEOUT_MS 20U
#define DDT_MOTOR_DEFAULT_INTER_FRAME_INTERVAL_MS 10U
#define DDT_MOTOR_COMMAND_DRIVE 0x64U
#define DDT_MOTOR_COMMAND_QUERY_STATUS 0x74U

typedef enum
{
    DDT_WAIT_NONE = 0U,
    DDT_WAIT_DRIVE_FEEDBACK,
    DDT_WAIT_STATUS_FEEDBACK,
} DDTWaitingResponseType_e;

struct DDTMotorBus
{
    USARTInstance* usart;
    DDTMotorInstance* motors[DDT_MOTOR_COUNT_PER_BUS];
    DDTMotorInstance* waiting_motor;
    DDTMotorRawFeedback_t raw_feedback;
    USART_TRANSFER_MODE transfer_mode;
    uint8_t motor_count;
    uint8_t round_robin_index;
    uint8_t waiting_bus_response;
    DDTWaitingResponseType_e waiting_response_type;
    uint8_t special_frame[DDT_MOTOR_FRAME_LENGTH];
    uint8_t special_repeat_count;
    uint8_t special_wait_response;
    uint16_t response_timeout_ms;
    uint16_t inter_frame_interval_ms;
    float request_start_ms;
    float last_tx_ms;
    uint32_t power_on_feedback_count;
};

static DDTMotorBus ddt_bus_pool[DDT_MOTOR_BUS_COUNT];
static DDTMotorInstance ddt_motor_pool[DDT_MOTOR_TOTAL_COUNT];
static uint8_t ddt_bus_count;
static uint8_t ddt_motor_count;


/**
 * @brief 计算官方要求的 CRC-8/MAXIM 校验值
 *
 * @note 当前工程公共 crc_8() 使用的是另一张 CRC-8 表，与官方示例不一致。
 *       本函数使用反射多项式 0x8C，初值、异或输出均为 0；例如
 *       `01 64 00 00 00 00 00 00 00`的结果为 0x50。
 *
 * @param data 待校验数据
 * @param length 数据长度
 * @return uint8_t CRC-8/MAXIM 校验值
 */
static uint8_t DDTCRC8Maxim(const uint8_t* data, uint16_t length)
{
    uint8_t crc = 0U;

    if (data == NULL) return 0U;

    for (uint16_t index = 0U; index < length; index++)
    {
        crc ^= data[index];
        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            crc = (crc & 0x01U) != 0U ? (uint8_t)((crc >> 1U) ^ 0x8CU) : (uint8_t)(crc >> 1U);
        }
    }

    return crc;
}


/**
 * @brief 按设备 ID 查找指定总线上的电机实例
 *
 * @param bus RS485 总线
 * @param device_id 设备 ID
 * @return DDTMotorInstance* 找到返回实例，否则返回 NULL
 */
static DDTMotorInstance* DDTFindMotor(DDTMotorBus* bus, uint8_t device_id)
{
    if (bus == NULL) return NULL;

    for (uint8_t index = 0U; index < bus->motor_count; index++)
    {
        DDTMotorInstance* motor = bus->motors[index];
        if (motor != NULL && motor->device_id == device_id) return motor;
    }

    return NULL;
}


/**
 * @brief 保存一段未经字段解释的电机回复数据
 *
 * @param feedback 原始反馈容器
 * @param data 回复数据首地址
 * @param length 回复长度
 */
static void DDTStoreRawFeedback(DDTMotorRawFeedback_t* feedback,
                                const uint8_t* data,
                                uint16_t length)
{
    uint16_t copy_length;

    if (feedback == NULL || data == NULL || length == 0U) return;

    copy_length = length > DDT_MOTOR_RAW_FEEDBACK_MAX_LENGTH
                      ? DDT_MOTOR_RAW_FEEDBACK_MAX_LENGTH
                      : length;
    memcpy(feedback->data, data, copy_length);
    if (copy_length < DDT_MOTOR_RAW_FEEDBACK_MAX_LENGTH)
    {
        memset(&feedback->data[copy_length], 0,
               DDT_MOTOR_RAW_FEEDBACK_MAX_LENGTH - copy_length);
    }
    feedback->length = copy_length;
    feedback->receive_count++;
    feedback->crc_valid = (length == DDT_MOTOR_FRAME_LENGTH &&
        DDTCRC8Maxim(data, DDT_MOTOR_FRAME_LENGTH - 1U) ==
        data[DDT_MOTOR_FRAME_LENGTH - 1U]);
}


/**
 * @brief 按官方 README 和换算表解析一帧 10 字节电机反馈
 *
 * @note 两类反馈共同字段为：DATA[1]模式、DATA[2:3]电流、DATA[4:5]速度、
 *       DATA[8]错误码。普通驱动反馈使用 DATA[6:7] 表示 16 位位置；0x74
 *       状态反馈使用 DATA[6]表示温度、DATA[7]表示未给出换算公式的位置原始值。
 *
 * @param motor 反馈所属电机实例
 * @param frame 10 字节反馈帧
 * @param feedback_type 本帧对应的请求类型
 */
static void DDTParseMotorFeedback(DDTMotorInstance* motor,
                                  const uint8_t frame[DDT_MOTOR_FRAME_LENGTH],
                                  DDTMotorFeedbackType_e feedback_type)
{
    int16_t current_raw;
    int16_t speed_raw;
    float direction;

    if (motor == NULL || frame == NULL) return;

    current_raw = (int16_t)(((uint16_t)frame[2] << 8U) | frame[3]);
    speed_raw = (int16_t)(((uint16_t)frame[4] << 8U) | frame[5]);
    direction = motor->motor_reverse_flag == MOTOR_DIRECTION_REVERSE ? -1.0f : 1.0f;

    motor->measure.mode = frame[1];
    motor->measure.mode_mismatch = frame[1] == (uint8_t)motor->control_mode ? 0U : 1U;
    motor->measure.current_raw = current_raw;
    motor->measure.speed_raw = speed_raw;
    motor->measure.current_a = direction * (float)current_raw * 8.0f / 32767.0f;
    if (motor->measure.current_a > 8.0f) motor->measure.current_a = 8.0f;
    if (motor->measure.current_a < -8.0f) motor->measure.current_a = -8.0f;
    motor->measure.speed_rpm = direction * (float)speed_raw;
    motor->measure.error_code = frame[8];
    motor->measure.feedback_type = feedback_type;
    motor->measure.feedback_received = 1U;
    motor->measure.last_feedback_ms = DWT_GetTimeline_ms();

    if (motor->measure.mode_mismatch != 0U)
    {
        /*
         * 上位机配置与实例声明不一致时，后续同一控制字会被电机按其它模式解释。
         * 收到首帧不匹配反馈后立即锁定输出，必须修正上位机配置或实例配置。
         */
        motor->enabled = 0U;
        motor->control_pending = 0U;
    }

    if (feedback_type == DDT_MOTOR_FEEDBACK_STATUS)
    {
        motor->measure.temperature_c = frame[6];
        motor->measure.status_position_raw = frame[7];
    }
    else
    {
        motor->measure.position_raw = (uint16_t)(((uint16_t)frame[6] << 8U) | frame[7]);
        motor->measure.position_deg = (float)motor->measure.position_raw * 360.0f / 32768.0f;
    }
}


/**
 * @brief 处理一条 USART ReceiveToIdle 事件中的 M0601C 数据
 *
 * @note 官方 README 明确回复长度为 10 字节。本驱动优先把回复交给当前总线唯一
 *       在途请求对应的电机；没有在途请求时，再按 DATA[0] 的设备 ID 尝试分发。
 *
 * @param bus_index 总线池索引
 */
static void DDTBusReceive(uint8_t bus_index)
{
    DDTMotorBus* bus;
    const uint8_t* data;
    uint16_t length;

    if (bus_index >= ddt_bus_count) return;
    bus = &ddt_bus_pool[bus_index];
    if (bus->usart == NULL || bus->usart->recv_len == 0U) return;

    data = bus->usart->recv_buff;
    length = bus->usart->recv_len;

    if (length == 3U && data[0] == 0xAAU && data[1] == 0x55U && data[2] == 0xFFU)
    {
        bus->power_on_feedback_count++;
        return;
    }

    if (bus->waiting_motor != NULL)
    {
        DDTMotorInstance* motor = bus->waiting_motor;
        DDTWaitingResponseType_e response_type = bus->waiting_response_type;

        DDTStoreRawFeedback(&motor->raw_feedback, data, length);
        if (length == DDT_MOTOR_FRAME_LENGTH)
        {
            DDTParseMotorFeedback(motor, data,
                                  response_type == DDT_WAIT_STATUS_FEEDBACK
                                      ? DDT_MOTOR_FEEDBACK_STATUS
                                      : DDT_MOTOR_FEEDBACK_DRIVE);
        }
        bus->waiting_motor = NULL;
        bus->waiting_response_type = DDT_WAIT_NONE;
        return;
    }

    if (bus->waiting_bus_response != 0U)
    {
        DDTStoreRawFeedback(&bus->raw_feedback, data, length);
        bus->waiting_bus_response = 0U;
        return;
    }

    if ((length % DDT_MOTOR_FRAME_LENGTH) == 0U)
    {
        for (uint16_t offset = 0U; offset < length; offset += DDT_MOTOR_FRAME_LENGTH)
        {
            DDTMotorInstance* motor = DDTFindMotor(bus, data[offset]);
            if (motor == NULL) continue;

            DDTStoreRawFeedback(&motor->raw_feedback, &data[offset], DDT_MOTOR_FRAME_LENGTH);
            DDTParseMotorFeedback(motor, &data[offset], DDT_MOTOR_FEEDBACK_DRIVE);
        }
    }
}


static void DDTBus0Receive(void)
{
    DDTBusReceive(0U);
}


static void DDTBus1Receive(void)
{
    DDTBusReceive(1U);
}


/**
 * @brief 构造带 CRC-8/MAXIM 的普通驱动帧
 *
 * @param motor 电机实例
 * @param frame 输出的 10 字节帧
 */
static void DDTBuildDriveFrame(const DDTMotorInstance* motor,
                               uint8_t frame[DDT_MOTOR_FRAME_LENGTH])
{
    uint16_t raw = (uint16_t)motor->target_raw;

    memset(frame, 0, DDT_MOTOR_FRAME_LENGTH);
    frame[0] = motor->device_id;
    frame[1] = DDT_MOTOR_COMMAND_DRIVE;
    frame[2] = (uint8_t)(raw >> 8U); // PDF：给定值高 8 位在 DATA[2]
    frame[3] = (uint8_t)raw; // PDF：给定值低 8 位在 DATA[3]
    frame[6] = motor->acceleration_time;
    frame[7] = (motor->control_mode == DDT_MOTOR_MODE_SPEED && motor->brake != 0U) ? 0xFFU : 0x00U;
    frame[9] = DDTCRC8Maxim(frame, DDT_MOTOR_FRAME_LENGTH - 1U);
}


/**
 * @brief 构造 0x74“获取其他反馈”查询帧
 *
 * @param motor 电机实例
 * @param frame 输出的 10 字节帧
 */
static void DDTBuildStatusQueryFrame(const DDTMotorInstance* motor,
                                     uint8_t frame[DDT_MOTOR_FRAME_LENGTH])
{
    memset(frame, 0, DDT_MOTOR_FRAME_LENGTH);
    frame[0] = motor->device_id;
    frame[1] = DDT_MOTOR_COMMAND_QUERY_STATUS;
    frame[9] = DDTCRC8Maxim(frame, DDT_MOTOR_FRAME_LENGTH - 1U);
}


/**
 * @brief 向指定 DDT RS485 总线提交一帧数据
 *
 * @param bus 总线实例
 * @param frame 10 字节发送帧
 * @return HAL_StatusTypeDef BSP USART 发送结果
 */
static HAL_StatusTypeDef DDTSendFrame(DDTMotorBus* bus,
                                      uint8_t frame[DDT_MOTOR_FRAME_LENGTH])
{
    HAL_StatusTypeDef status;

    if (bus == NULL || bus->usart == NULL) return HAL_ERROR;
    status = USARTSend(bus->usart, frame, DDT_MOTOR_FRAME_LENGTH, bus->transfer_mode);
    if (status == HAL_OK) bus->last_tx_ms = DWT_GetTimeline_ms();
    return status;
}


DDTMotorBus* DDTMotorBusInit(const DDTMotorBusInitConfig_t* config)
{
    DDTMotorBus* bus;
    USART_Init_Config_s usart_config;

    if (config == NULL || config->usart_handle == NULL || ddt_bus_count >= DDT_MOTOR_BUS_COUNT ||
        (config->transfer_mode != USART_TRANSFER_BLOCKING &&
            config->transfer_mode != USART_TRANSFER_IT &&
            config->transfer_mode != USART_TRANSFER_DMA))
    {
        LOGERROR("[ddt_motor] invalid RS485 bus config");
        return NULL;
    }

    bus = &ddt_bus_pool[ddt_bus_count];
    memset(bus, 0, sizeof(*bus));
    bus->transfer_mode = config->transfer_mode;
    bus->response_timeout_ms = config->response_timeout_ms == 0U
                                   ? DDT_MOTOR_DEFAULT_RESPONSE_TIMEOUT_MS
                                   : config->response_timeout_ms;
    bus->inter_frame_interval_ms = config->inter_frame_interval_ms == 0U
                                       ? DDT_MOTOR_DEFAULT_INTER_FRAME_INTERVAL_MS
                                       : config->inter_frame_interval_ms;

    memset(&usart_config, 0, sizeof(usart_config));
    usart_config.usart_handle = config->usart_handle;
    usart_config.recv_buff_size = DDT_MOTOR_RAW_FEEDBACK_MAX_LENGTH;
    usart_config.module_callback = ddt_bus_count == 0U ? DDTBus0Receive : DDTBus1Receive;
    bus->usart = USARTRegister(&usart_config);
    if (bus->usart == NULL) return NULL;

    ddt_bus_count++;
    return bus;
}


DDTMotorInstance* DDTMotorInit(const DDTMotorInitConfig_t* config)
{
    DDTMotorInstance* motor;

    if (config == NULL || config->bus == NULL || config->device_id == 0U ||
        config->control_mode < DDT_MOTOR_MODE_CURRENT ||
        config->control_mode > DDT_MOTOR_MODE_POSITION ||
        config->motor_reverse_flag > MOTOR_DIRECTION_REVERSE ||
        (config->control_mode == DDT_MOTOR_MODE_POSITION &&
            config->motor_reverse_flag == MOTOR_DIRECTION_REVERSE) ||
        config->bus->motor_count >= DDT_MOTOR_COUNT_PER_BUS ||
        ddt_motor_count >= DDT_MOTOR_TOTAL_COUNT ||
        DDTFindMotor(config->bus, config->device_id) != NULL)
    {
        LOGERROR("[ddt_motor] invalid motor init config");
        return NULL;
    }

    motor = &ddt_motor_pool[ddt_motor_count];
    memset(motor, 0, sizeof(*motor));
    motor->bus = config->bus;
    motor->device_id = config->device_id;
    motor->control_mode = config->control_mode;
    motor->motor_reverse_flag = config->motor_reverse_flag;
    motor->acceleration_time = config->acceleration_time;
    motor->refresh_period_ms = config->refresh_period_ms;

    config->bus->motors[config->bus->motor_count++] = motor;
    ddt_motor_count++;
    return motor;
}


HAL_StatusTypeDef DDTMotorEnable(DDTMotorInstance* motor)
{
    if (motor == NULL || motor->target_initialized == 0U) return HAL_ERROR;
    if (motor->measure.feedback_received != 0U && motor->measure.mode_mismatch != 0U)
        return HAL_ERROR;
    motor->enabled = 1U;
    motor->control_pending = 1U; // 使能后发送当前已保存的安全目标
    return HAL_OK;
}


HAL_StatusTypeDef DDTMotorStop(DDTMotorInstance* motor)
{
    if (motor == NULL) return HAL_ERROR;

    motor->enabled = 0U;
    if (motor->control_mode == DDT_MOTOR_MODE_POSITION)
    {
        /* 官方位置模式没有通用失能或刹车指令，不能伪造“停止”语义。 */
        motor->control_pending = 0U;
        return HAL_ERROR;
    }

    motor->target_raw = 0;
    motor->brake = motor->control_mode == DDT_MOTOR_MODE_SPEED ? 1U : 0U;
    motor->control_pending = 1U;
    return HAL_OK;
}


HAL_StatusTypeDef DDTMotorSetCurrent(DDTMotorInstance* motor, float current_a)
{
    float directed_current;

    if (motor == NULL || motor->control_mode != DDT_MOTOR_MODE_CURRENT || !isfinite(current_a))
        return HAL_ERROR;

    directed_current = motor->motor_reverse_flag == MOTOR_DIRECTION_REVERSE ? -current_a : current_a;
    if (directed_current > 8.0f) directed_current = 8.0f;
    if (directed_current < -8.0f) directed_current = -8.0f;
    motor->target_raw = (int16_t)(directed_current * (32767.0f / 8.0f));
    motor->brake = 0U;
    motor->target_initialized = 1U;
    if (motor->enabled != 0U) motor->control_pending = 1U;
    return HAL_OK;
}


HAL_StatusTypeDef DDTMotorSetSpeed(DDTMotorInstance* motor, float speed_rpm, uint8_t brake)
{
    float directed_speed;

    if (motor == NULL || motor->control_mode != DDT_MOTOR_MODE_SPEED || !isfinite(speed_rpm))
        return HAL_ERROR;

    directed_speed = motor->motor_reverse_flag == MOTOR_DIRECTION_REVERSE ? -speed_rpm : speed_rpm;
    if (directed_speed > 330.0f) directed_speed = 330.0f;
    if (directed_speed < -330.0f) directed_speed = -330.0f;
    motor->target_raw = (int16_t)(directed_speed >= 0.0f
                                      ? directed_speed + 0.5f
                                      : directed_speed - 0.5f);
    motor->brake = brake != 0U ? 1U : 0U;
    motor->target_initialized = 1U;
    if (motor->enabled != 0U) motor->control_pending = 1U;
    return HAL_OK;
}


HAL_StatusTypeDef DDTMotorSetPosition(DDTMotorInstance* motor, float position_deg)
{
    if (motor == NULL || motor->control_mode != DDT_MOTOR_MODE_POSITION || !isfinite(position_deg))
        return HAL_ERROR;

    if (position_deg > 360.0f) position_deg = 360.0f;
    if (position_deg < 0.0f) position_deg = 0.0f;
    motor->target_raw = (int16_t)(position_deg * (32767.0f / 360.0f));
    motor->brake = 0U;
    motor->target_initialized = 1U;
    if (motor->enabled != 0U) motor->control_pending = 1U;
    return HAL_OK;
}


HAL_StatusTypeDef DDTMotorSetAccelerationTime(DDTMotorInstance* motor, uint8_t acceleration_time)
{
    if (motor == NULL) return HAL_ERROR;
    motor->acceleration_time = acceleration_time;
    if (motor->enabled != 0U) motor->control_pending = 1U;
    return HAL_OK;
}


HAL_StatusTypeDef DDTMotorRequestStatus(DDTMotorInstance* motor)
{
    if (motor == NULL) return HAL_ERROR;
    if (motor->status_query_pending != 0U || motor->bus->waiting_motor != NULL ||
        motor->bus->waiting_bus_response != 0U)
    {
        return HAL_BUSY;
    }

    motor->status_query_pending = 1U;
    return HAL_OK;
}


const DDTMotorRawFeedback_t* DDTMotorGetRawFeedback(const DDTMotorInstance* motor)
{
    return motor != NULL ? &motor->raw_feedback : NULL;
}


const DDTMotorMeasure_t* DDTMotorGetMeasure(const DDTMotorInstance* motor)
{
    return motor != NULL ? &motor->measure : NULL;
}


uint8_t DDTMotorIsOnline(const DDTMotorInstance* motor, float timeout_ms)
{
    if (motor == NULL || motor->measure.feedback_received == 0U ||
        !isfinite(timeout_ms) || timeout_ms <= 0.0f)
    {
        return 0U;
    }

    return DWT_GetTimeline_ms() - motor->measure.last_feedback_ms <= timeout_ms ? 1U : 0U;
}


HAL_StatusTypeDef DDTMotorBusSetDeviceId(DDTMotorBus* bus, uint8_t new_id)
{
    if (bus == NULL || new_id == 0U || bus->motor_count != 0U ||
        bus->special_repeat_count != 0U || bus->waiting_bus_response != 0U)
    {
        return HAL_ERROR;
    }

    memset(bus->special_frame, 0, sizeof(bus->special_frame));
    bus->special_frame[0] = 0xAAU;
    bus->special_frame[1] = 0x55U;
    bus->special_frame[2] = 0x53U;
    bus->special_frame[3] = new_id;
    bus->special_repeat_count = 5U; // PDF：电机收到 5 次 ID 设置帧后执行设置
    bus->special_wait_response = 0U;
    return HAL_OK;
}


HAL_StatusTypeDef DDTMotorBusRequestDeviceId(DDTMotorBus* bus)
{
    if (bus == NULL || bus->motor_count != 0U || bus->special_repeat_count != 0U ||
        bus->waiting_bus_response != 0U)
    {
        return HAL_ERROR;
    }

    memset(bus->special_frame, 0, sizeof(bus->special_frame));
    bus->special_frame[0] = 0xC8U;
    bus->special_frame[1] = DDT_MOTOR_COMMAND_DRIVE;
    bus->special_frame[9] = DDTCRC8Maxim(bus->special_frame, DDT_MOTOR_FRAME_LENGTH - 1U);
    bus->special_repeat_count = 1U;
    bus->special_wait_response = 1U;
    return HAL_OK;
}


const DDTMotorRawFeedback_t* DDTMotorBusGetRawFeedback(const DDTMotorBus* bus)
{
    return bus != NULL ? &bus->raw_feedback : NULL;
}


/**
 * @brief 提交一帧普通控制并登记对应的在途反馈
 *
 * @param bus RS485 总线
 * @param motor 电机实例
 * @param now_ms 当前时间戳
 * @return HAL_StatusTypeDef BSP USART 发送结果
 */
static HAL_StatusTypeDef DDTSubmitMotorControl(DDTMotorBus* bus,
                                               DDTMotorInstance* motor,
                                               float now_ms)
{
    uint8_t frame[DDT_MOTOR_FRAME_LENGTH];
    HAL_StatusTypeDef status;

    DDTBuildDriveFrame(motor, frame);
    status = DDTSendFrame(bus, frame);
    if (status == HAL_OK)
    {
        motor->control_pending = 0U;
        motor->last_control_tx_ms = now_ms;
        /* README/usart.c：每次普通驱动指令后电机会返回一帧 10 字节状态。 */
        bus->waiting_motor = motor;
        bus->waiting_response_type = DDT_WAIT_DRIVE_FEEDBACK;
        bus->request_start_ms = now_ms;
    }
    else if (status != HAL_BUSY)
    {
        motor->control_pending = 0U;
    }

    return status;
}


/**
 * @brief 调度一条 DDT RS485 总线的一次发送机会
 *
 * @param bus 总线实例
 */
static void DDTControlBus(DDTMotorBus* bus)
{
    uint8_t frame[DDT_MOTOR_FRAME_LENGTH];
    float now_ms;

    if (bus == NULL || bus->usart == NULL || USARTIsReady(bus->usart) == 0U) return;

    now_ms = DWT_GetTimeline_ms();
    if (now_ms - bus->last_tx_ms < (float)bus->inter_frame_interval_ms) return;

    if (bus->waiting_motor != NULL || bus->waiting_bus_response != 0U)
    {
        if (now_ms - bus->request_start_ms <= (float)bus->response_timeout_ms) return;
        bus->waiting_motor = NULL;
        bus->waiting_bus_response = 0U;
        bus->waiting_response_type = DDT_WAIT_NONE;
    }

    if (bus->special_repeat_count != 0U)
    {
        HAL_StatusTypeDef status = DDTSendFrame(bus, bus->special_frame);
        if (status == HAL_OK)
        {
            bus->special_repeat_count--;
            if (bus->special_repeat_count == 0U && bus->special_wait_response != 0U)
            {
                bus->waiting_bus_response = 1U;
                bus->request_start_ms = now_ms;
                bus->special_wait_response = 0U;
            }
        }
        else if (status != HAL_BUSY)
        {
            bus->special_repeat_count = 0U;
            bus->special_wait_response = 0U;
        }
        return;
    }

    for (uint8_t checked = 0U; checked < bus->motor_count; checked++)
    {
        DDTMotorInstance* motor = bus->motors[bus->round_robin_index];
        HAL_StatusTypeDef status;

        bus->round_robin_index = (uint8_t)((bus->round_robin_index + 1U) % bus->motor_count);
        if (motor == NULL) continue;

        if (motor->control_pending != 0U)
        {
            (void)DDTSubmitMotorControl(bus, motor, now_ms);
            return;
        }

        if (motor->status_query_pending != 0U)
        {
            DDTBuildStatusQueryFrame(motor, frame);
            status = DDTSendFrame(bus, frame);
            if (status == HAL_OK)
            {
                motor->status_query_pending = 0U;
                bus->waiting_motor = motor;
                bus->waiting_response_type = DDT_WAIT_STATUS_FEEDBACK;
                bus->request_start_ms = now_ms;
            }
            else if (status != HAL_BUSY)
            {
                motor->status_query_pending = 0U;
            }
            return;
        }

        /* 显式状态查询优先于周期刷新，避免 refresh_period 较短时查询长期饥饿。 */
        if (motor->enabled != 0U && motor->refresh_period_ms > 0U &&
            now_ms - motor->last_control_tx_ms >= (float)motor->refresh_period_ms)
        {
            (void)DDTSubmitMotorControl(bus, motor, now_ms);
            return;
        }
    }
}


void DDTMotorControl(void)
{
    for (uint8_t bus_index = 0U; bus_index < ddt_bus_count; bus_index++)
    {
        DDTControlBus(&ddt_bus_pool[bus_index]);
    }
}
