#include "dm_imu.h"

#include "bsp_log.h"

#include <math.h>
#include <string.h>

#define DM_IMU_CAN_FRAME_LENGTH             8U
#define DM_IMU_CAN_REQUEST_HEAD             0xCCU
#define DM_IMU_CAN_REQUEST_DATA_HEAD        0xDDU
#define DM_IMU_OPERATION_READ               0x00U
#define DM_IMU_OPERATION_WRITE              0x01U

#define DM_IMU_RS485_ACTIVE_HEAD_1          0x55U
#define DM_IMU_RS485_ACTIVE_HEAD_2          0xAAU
#define DM_IMU_RS485_ACTIVE_TAIL            0x0AU
#define DM_IMU_RS485_NORMAL_FRAME_LENGTH    19U
#define DM_IMU_RS485_QUAT_FRAME_LENGTH      23U
#define DM_IMU_RS485_RESPONSE_HEAD          0xA5U
#define DM_IMU_RS485_RESPONSE_TAIL          0x5AU
#define DM_IMU_RS485_RESPONSE_FRAME_LENGTH  24U
#define DM_IMU_RS485_TYPE_COMMAND           0x0CU
#define DM_IMU_RS485_TYPE_DATA              0x0DU
#define DM_IMU_RS485_STREAM_SIZE            256U

#define DM_IMU_ACCEL_MIN_MPS2               (-235.2F)
#define DM_IMU_ACCEL_MAX_MPS2               (235.2F)
#define DM_IMU_GYRO_MIN_RAD_S                (-34.88F)
#define DM_IMU_GYRO_MAX_RAD_S                (34.88F)
#define DM_IMU_PITCH_MIN_DEG                 (-90.0F)
#define DM_IMU_PITCH_MAX_DEG                 (90.0F)
#define DM_IMU_ROLL_MIN_DEG                  (-180.0F)
#define DM_IMU_ROLL_MAX_DEG                  (180.0F)
#define DM_IMU_YAW_MIN_DEG                   (-180.0F)
#define DM_IMU_YAW_MAX_DEG                   (180.0F)
#define DM_IMU_QUATERNION_MIN                (-1.0F)
#define DM_IMU_QUATERNION_MAX                (1.0F)
#define DM_IMU_TEMPERATURE_MIN_C             0.0F
#define DM_IMU_TEMPERATURE_MAX_C             60.0F

/* DM-IMU的RS485主动协议直接传输IEEE754单精度原始字节，编译期锁定float尺寸。 */
_Static_assert (
sizeof
(
float
)
==
4U
,
"DM-IMU protocol requires 32-bit float"
);

/**
 * @brief 驱动内部实例结构体。
 *
 * @note transport及其ID在DMIMURegister()后不再改变，确保BSP CAN过滤器和USART独占关系稳定。
 */
struct DMIMUInstance
{
    DMIMUTransport_e transport;
    uint32_t online_timeout_ms;
    volatile uint8_t has_received_frame;
    volatile uint32_t last_receive_tick_ms;
    DMIMUMeasure_s measure;
    DMIMURegisterResponse_s register_response;
    DMIMUStatistics_s statistics;
    uint8_t last_request_register;
    uint8_t last_request_operation;

    CANInstance* can_instance;
    float can_transmit_timeout_ms;

    USARTInstance* usart_instance;
    USART_TRANSFER_MODE rs485_transfer_mode;
    uint8_t rs485_slave_id;
    uint8_t rs485_stream[DM_IMU_RS485_STREAM_SIZE];
    uint16_t rs485_stream_length;
};

static DMIMUInstance dm_imu_instance_pool[DM_IMU_MAX_INSTANCE_COUNT];
static uint8_t dm_imu_instance_count;
static DMIMUInstance* dm_imu_rs485_instances[DM_IMU_MAX_RS485_INSTANCE_COUNT];
static uint8_t dm_imu_rs485_instance_count;

/**
 * @brief 进入短临界区，保护BSP后台任务和application任务之间共享的测量快照。
 *
 * @param primask 输出进入临界区之前的中断屏蔽状态。
 */
static void DMIMUEnterCritical(uint32_t* primask)
{
    if (primask == NULL)
    {
        return;
    }

    *primask = __get_PRIMASK();
    __disable_irq();
}

/**
 * @brief 恢复短临界区之前的中断屏蔽状态。
 *
 * @param primask DMIMUEnterCritical()保存的状态。
 */
static void DMIMUExitCritical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

/**
 * @brief 将无符号映射值还原为官方定义范围内的浮点值。
 *
 * @param raw 无符号映射整数。
 * @param minimum 物理量下限。
 * @param maximum 物理量上限。
 * @param bits 映射整数位数，当前为16或14。
 * @return float 解码后的物理量。
 */
static float DMIMUUnsignedToFloat(uint32_t raw, float minimum, float maximum, uint8_t bits)
{
    uint32_t maximum_raw = (1UL << bits) - 1UL;

    return (float)raw * (maximum - minimum) / (float)maximum_raw + minimum;
}

/**
 * @brief 按小端序读取16位整数。
 */
static uint16_t DMIMUReadU16LE(const uint8_t* data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

/**
 * @brief 按小端序读取32位整数。
 */
static uint32_t DMIMUReadU32LE(const uint8_t* data)
{
    return (uint32_t)data[0] |
        ((uint32_t)data[1] << 8U) |
        ((uint32_t)data[2] << 16U) |
        ((uint32_t)data[3] << 24U);
}

/**
 * @brief 按小端IEEE754格式读取单精度浮点数。
 *
 * @param data 四字节原始数据。
 * @return float 解码结果。
 */
static float DMIMUReadFloatLE(const uint8_t* data)
{
    uint32_t raw = DMIMUReadU32LE(data);
    float value;

    memcpy(&value, &raw, sizeof(value));
    return value;
}

/**
 * @brief 把32位整数按小端序写入发送帧。
 */
static void DMIMUWriteU32LE(uint8_t* data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
    data[2] = (uint8_t)(value >> 16U);
    data[3] = (uint8_t)(value >> 24U);
}

/**
 * @brief 计算CRC16-CCITT查表项。
 *
 * @note PDF附录给出的256项表对应多项式0x1021。运行时计算单个表项可避免复制大表，
 *       RS485主动帧最长仅23字节，对任务负载影响可以忽略。
 */
static uint16_t DMIMUCRC16TableEntry(uint8_t index)
{
    uint16_t value = (uint16_t)index << 8U;

    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
        value = (value & 0x8000U) != 0U
                    ? (uint16_t)((value << 1U) ^ 0x1021U)
                    : (uint16_t)(value << 1U);
    }

    return value;
}

/**
 * @brief 计算标准CRC16-CCITT-FALSE结果。
 *
 * @note 初值0xFFFF、多项式0x1021，与PDF附录的表内容一致。
 */
static uint16_t DMIMUCRC16Standard(const uint8_t* data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;

    for (uint16_t index = 0U; index < length; ++index)
    {
        uint8_t table_index = (uint8_t)((crc >> 8U) ^ data[index]);
        crc = (uint16_t)((crc << 8U) ^ DMIMUCRC16TableEntry(table_index));
    }

    return crc;
}

/**
 * @brief 按PDF V1.2附录四原样计算其示例函数结果。
 *
 * @note PDF示例写作“crc << 1”，与常见CCITT查表实现的“crc << 8”不同。
 *       PDF内上位机截图中的实际数据可验证设备采用本算法，并以低字节在前发送CRC。
 */
static uint16_t DMIMUCRC16DocumentVariant(const uint8_t* data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;

    for (uint16_t index = 0U; index < length; ++index)
    {
        uint8_t table_index = (uint8_t)((crc >> 8U) ^ data[index]);
        crc = (uint16_t)((crc << 1U) ^ DMIMUCRC16TableEntry(table_index));
    }

    return crc;
}

/**
 * @brief 校验RS485主动帧CRC，并识别是否使用了兼容路径。
 *
 * @param frame 完整主动帧。
 * @param frame_length 19或23字节。
 * @param compatibility 输出是否通过PDF原样算法或相反字节序通过。
 * @return uint8_t CRC匹配返回1，否则返回0。
 */
static uint8_t DMIMUValidateActiveCRC(const uint8_t* frame,
                                      uint16_t frame_length,
                                      uint8_t* compatibility)
{
    uint16_t payload_length = (uint16_t)(frame_length - 3U);
    uint16_t received_le = DMIMUReadU16LE(&frame[payload_length]);
    uint16_t received_be = (uint16_t)frame[payload_length + 1U] |
        ((uint16_t)frame[payload_length] << 8U);
    uint16_t standard_crc = DMIMUCRC16Standard(frame, payload_length);
    uint16_t document_crc = DMIMUCRC16DocumentVariant(frame, payload_length);

    if (compatibility != NULL)
    {
        *compatibility = 0U;
    }

    /* PDF截图实测帧与附录原样算法、小端CRC字节序一致，因此将其作为严格主路径。 */
    if (received_le == document_crc)
    {
        return 1U;
    }

    if (received_be == document_crc || received_le == standard_crc || received_be == standard_crc)
    {
        if (compatibility != NULL)
        {
            *compatibility = 1U;
        }
        return 1U;
    }

    return 0U;
}

/**
 * @brief 记录一帧合法接收，刷新实例在线时间。
 */
static void DMIMURecordValidReceive(DMIMUInstance* instance)
{
    uint32_t primask;

    DMIMUEnterCritical(&primask);
    instance->has_received_frame = 1U;
    instance->last_receive_tick_ms = HAL_GetTick();
    instance->statistics.received_frame_count++;
    DMIMUExitCritical(primask);
}

/**
 * @brief 发布三轴加速度及可选温度。
 */
static void DMIMUPublishAccel(DMIMUInstance* instance,
                              const float accel[3],
                              uint8_t has_temperature,
                              uint8_t temperature_raw)
{
    uint32_t primask;
    uint32_t tick = HAL_GetTick();

    DMIMUEnterCritical(&primask);
    memcpy(instance->measure.accel_mps2, accel, sizeof(instance->measure.accel_mps2));
    if (has_temperature != 0U)
    {
        instance->measure.temperature_raw = temperature_raw;
        instance->measure.temperature_c = DMIMUUnsignedToFloat(temperature_raw,
                                                               DM_IMU_TEMPERATURE_MIN_C,
                                                               DM_IMU_TEMPERATURE_MAX_C,
                                                               8U);
    }
    instance->measure.accel_last_tick_ms = tick;
    instance->measure.accel_update_count++;
    instance->measure.valid_mask |= DM_IMU_VALID_ACCEL;
    instance->statistics.sensor_frame_count++;
    DMIMUExitCritical(primask);
}

/**
 * @brief 发布三轴角速度。
 */
static void DMIMUPublishGyro(DMIMUInstance* instance, const float gyro[3])
{
    uint32_t primask;

    DMIMUEnterCritical(&primask);
    memcpy(instance->measure.gyro_rad_s, gyro, sizeof(instance->measure.gyro_rad_s));
    instance->measure.gyro_last_tick_ms = HAL_GetTick();
    instance->measure.gyro_update_count++;
    instance->measure.valid_mask |= DM_IMU_VALID_GYRO;
    instance->statistics.sensor_frame_count++;
    DMIMUExitCritical(primask);
}

/**
 * @brief 发布欧拉角，公共接口统一为roll、pitch、yaw字段。
 */
static void DMIMUPublishEuler(DMIMUInstance* instance, float roll, float pitch, float yaw)
{
    uint32_t primask;

    DMIMUEnterCritical(&primask);
    instance->measure.roll_deg = roll;
    instance->measure.pitch_deg = pitch;
    instance->measure.yaw_deg = yaw;
    instance->measure.euler_last_tick_ms = HAL_GetTick();
    instance->measure.euler_update_count++;
    instance->measure.valid_mask |= DM_IMU_VALID_EULER;
    instance->statistics.sensor_frame_count++;
    DMIMUExitCritical(primask);
}

/**
 * @brief 发布w、x、y、z四元数。
 */
static void DMIMUPublishQuaternion(DMIMUInstance* instance, const float quaternion[4])
{
    uint32_t primask;

    DMIMUEnterCritical(&primask);
    memcpy(instance->measure.quaternion, quaternion, sizeof(instance->measure.quaternion));
    instance->measure.quaternion_last_tick_ms = HAL_GetTick();
    instance->measure.quaternion_update_count++;
    instance->measure.valid_mask |= DM_IMU_VALID_QUATERNION;
    instance->statistics.sensor_frame_count++;
    DMIMUExitCritical(primask);
}

/**
 * @brief 保存寄存器应答快照。
 */
static void DMIMUPublishRegisterResponse(DMIMUInstance* instance,
                                         uint8_t register_id,
                                         uint8_t operation,
                                         uint8_t ack,
                                         uint32_t data)
{
    uint32_t primask;

    DMIMUEnterCritical(&primask);
    instance->register_response.received = 1U;
    instance->register_response.register_id = register_id;
    instance->register_response.operation = operation;
    instance->register_response.ack = ack <= DM_IMU_ACK_OPERATION_FAILED
                                          ? (DMIMUAck_e)ack
                                          : DM_IMU_ACK_UNKNOWN;
    instance->register_response.data = data;
    instance->register_response.update_count++;
    instance->statistics.register_response_count++;
    DMIMUExitCritical(primask);
}

/**
 * @brief 判断一组浮点值是否全部有限。
 */
static uint8_t DMIMUFloatsAreFinite(const float* values, uint8_t count)
{
    for (uint8_t index = 0U; index < count; ++index)
    {
        if (!isfinite(values[index]))
        {
            return 0U;
        }
    }

    return 1U;
}

/**
 * @brief 解析CAN加速度帧。
 *
 * @note PDF第10页规定DATA[2:7]为三个16位小端映射值；DATA[1]为温度原始字节。
 */
static void DMIMUDecodeCANAccel(DMIMUInstance* instance, const uint8_t* data)
{
    float accel[3];

    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
        uint16_t raw = DMIMUReadU16LE(&data[2U + axis * 2U]);
        accel[axis] = DMIMUUnsignedToFloat(raw,
                                           DM_IMU_ACCEL_MIN_MPS2,
                                           DM_IMU_ACCEL_MAX_MPS2,
                                           16U);
    }

    DMIMUPublishAccel(instance, accel, 1U, data[1]);
}

/**
 * @brief 解析CAN角速度帧。
 */
static void DMIMUDecodeCANGyro(DMIMUInstance* instance, const uint8_t* data)
{
    float gyro[3];

    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
        uint16_t raw = DMIMUReadU16LE(&data[2U + axis * 2U]);
        gyro[axis] = DMIMUUnsignedToFloat(raw,
                                          DM_IMU_GYRO_MIN_RAD_S,
                                          DM_IMU_GYRO_MAX_RAD_S,
                                          16U);
    }

    DMIMUPublishGyro(instance, gyro);
}

/**
 * @brief 解析CAN欧拉角帧。
 *
 * @note CAN顺序由PDF规定为Pitch、Yaw、Roll，与RS485主动浮点帧的Roll、Pitch、Yaw不同。
 */
static void DMIMUDecodeCANEuler(DMIMUInstance* instance, const uint8_t* data)
{
    float pitch = DMIMUUnsignedToFloat(DMIMUReadU16LE(&data[2]),
                                       DM_IMU_PITCH_MIN_DEG,
                                       DM_IMU_PITCH_MAX_DEG,
                                       16U);
    float yaw = DMIMUUnsignedToFloat(DMIMUReadU16LE(&data[4]),
                                     DM_IMU_YAW_MIN_DEG,
                                     DM_IMU_YAW_MAX_DEG,
                                     16U);
    float roll = DMIMUUnsignedToFloat(DMIMUReadU16LE(&data[6]),
                                      DM_IMU_ROLL_MIN_DEG,
                                      DM_IMU_ROLL_MAX_DEG,
                                      16U);

    DMIMUPublishEuler(instance, roll, pitch, yaw);
}

/**
 * @brief 解析CAN四元数帧。
 *
 * @note PDF第10页将四个14位值连续压入DATA[1:7]。官方示例对W低6位使用0xF8，
 *       会遗漏DATA[2]的bit2；这里按PDF位域表使用0xFC完整提取W[5:0]。
 */
static void DMIMUDecodeCANQuaternion(DMIMUInstance* instance, const uint8_t* data)
{
    uint16_t raw[4];
    float quaternion[4];

    raw[0] = ((uint16_t)data[1] << 6U) | ((uint16_t)(data[2] & 0xFCU) >> 2U);
    raw[1] = ((uint16_t)(data[2] & 0x03U) << 12U) |
        ((uint16_t)data[3] << 4U) |
        ((uint16_t)(data[4] & 0xF0U) >> 4U);
    raw[2] = ((uint16_t)(data[4] & 0x0FU) << 10U) |
        ((uint16_t)data[5] << 2U) |
        ((uint16_t)(data[6] & 0xC0U) >> 6U);
    raw[3] = ((uint16_t)(data[6] & 0x3FU) << 8U) | data[7];

    for (uint8_t index = 0U; index < 4U; ++index)
    {
        quaternion[index] = DMIMUUnsignedToFloat(raw[index],
                                                 DM_IMU_QUATERNION_MIN,
                                                 DM_IMU_QUATERNION_MAX,
                                                 14U);
    }

    DMIMUPublishQuaternion(instance, quaternion);
}

/**
 * @brief BSP CAN任务上下文中的DM-IMU接收回调。
 */
static void DMIMUCANReceiveCallback(CANInstance* can_instance)
{
    DMIMUInstance* instance;
    const uint8_t* data;

    if (can_instance == NULL || can_instance->id == NULL)
    {
        return;
    }

    instance = (DMIMUInstance*)can_instance->id;
    data = can_instance->rx_buff;

    if (can_instance->rx_len != DM_IMU_CAN_FRAME_LENGTH)
    {
        instance->statistics.invalid_length_count++;
        return;
    }

    if (data[0] >= DM_IMU_DATA_ACCEL && data[0] <= DM_IMU_DATA_QUATERNION)
    {
        DMIMURecordValidReceive(instance);
        switch ((DMIMUDataType_e)data[0])
        {
        case DM_IMU_DATA_ACCEL:
            DMIMUDecodeCANAccel(instance, data);
            break;

        case DM_IMU_DATA_GYRO:
            DMIMUDecodeCANGyro(instance, data);
            break;

        case DM_IMU_DATA_EULER:
            DMIMUDecodeCANEuler(instance, data);
            break;

        case DM_IMU_DATA_QUATERNION:
            DMIMUDecodeCANQuaternion(instance, data);
            break;

        default:
            break;
        }
        return;
    }

    if (data[0] == DM_IMU_CAN_REQUEST_HEAD && data[2] == DM_IMU_CAN_REQUEST_DATA_HEAD)
    {
        DMIMURecordValidReceive(instance);
        DMIMUPublishRegisterResponse(instance,
                                     data[1],
                                     instance->last_request_operation,
                                     data[3],
                                     DMIMUReadU32LE(&data[4]));
        return;
    }

    instance->statistics.unknown_type_count++;
}

/**
 * @brief 解析RS485主动模式浮点数据帧。
 */
static void DMIMUDecodeRS485ActiveFrame(DMIMUInstance* instance,
                                        const uint8_t* frame,
                                        DMIMUDataType_e data_type)
{
    float values[4];
    uint8_t value_count = data_type == DM_IMU_DATA_QUATERNION ? 4U : 3U;

    for (uint8_t index = 0U; index < value_count; ++index)
    {
        values[index] = DMIMUReadFloatLE(&frame[4U + index * 4U]);
    }

    if (DMIMUFloatsAreFinite(values, value_count) == 0U)
    {
        instance->statistics.invalid_value_count++;
        return;
    }

    DMIMURecordValidReceive(instance);
    switch (data_type)
    {
    case DM_IMU_DATA_ACCEL:
        DMIMUPublishAccel(instance, values, 0U, 0U);
        break;

    case DM_IMU_DATA_GYRO:
        DMIMUPublishGyro(instance, values);
        break;

    case DM_IMU_DATA_EULER:
        /* USB/RS485主动协议顺序为Roll、Pitch、Yaw。 */
        DMIMUPublishEuler(instance, values[0], values[1], values[2]);
        break;

    case DM_IMU_DATA_QUATERNION:
        DMIMUPublishQuaternion(instance, values);
        break;

    default:
        break;
    }
}

/**
 * @brief 解析RS485应答模式的24字节A5...5A帧。
 */
static void DMIMUDecodeRS485ResponseFrame(DMIMUInstance* instance, const uint8_t* frame)
{
    uint8_t type = frame[1];
    uint8_t register_id = frame[3];
    uint8_t operation = frame[4];
    uint8_t ack = frame[21];
    float values[4];

    if ((type != DM_IMU_RS485_TYPE_COMMAND && type != DM_IMU_RS485_TYPE_DATA) ||
        operation > DM_IMU_OPERATION_WRITE)
    {
        instance->statistics.unknown_type_count++;
        return;
    }

    DMIMURecordValidReceive(instance);
    DMIMUPublishRegisterResponse(instance,
                                 register_id,
                                 operation,
                                 ack,
                                 DMIMUReadU32LE(&frame[5]));

    if (type != DM_IMU_RS485_TYPE_DATA || ack != DM_IMU_ACK_SUCCESS || register_id > 3U)
    {
        return;
    }

    for (uint8_t index = 0U; index < 4U; ++index)
    {
        values[index] = DMIMUReadFloatLE(&frame[5U + index * 4U]);
    }

    if (DMIMUFloatsAreFinite(values, register_id == 3U ? 4U : 3U) == 0U)
    {
        instance->statistics.invalid_value_count++;
        return;
    }

    switch (register_id)
    {
    case 0U:
        DMIMUPublishAccel(instance, values, 0U, 0U);
        break;

    case 1U:
        DMIMUPublishGyro(instance, values);
        break;

    case 2U:
        DMIMUPublishEuler(instance, values[0], values[1], values[2]);
        break;

    case 3U:
        DMIMUPublishQuaternion(instance, values);
        break;

    default:
        break;
    }
}

/**
 * @brief 从RS485流缓存头部移除指定字节。
 */
static void DMIMURS485Consume(DMIMUInstance* instance, uint16_t length)
{
    if (length >= instance->rs485_stream_length)
    {
        instance->rs485_stream_length = 0U;
        return;
    }

    instance->rs485_stream_length = (uint16_t)(instance->rs485_stream_length - length);
    memmove(instance->rs485_stream,
            &instance->rs485_stream[length],
            instance->rs485_stream_length);
}

/**
 * @brief 流式解析RS485主动模式和应答模式帧。
 *
 * @note 同一实例可同时识别两种帧格式，因此设备在上位机中切换主动/应答模式后，
 *       STM32驱动不需要切换解析器状态。
 */
static void DMIMUParseRS485Stream(DMIMUInstance* instance)
{
    while (instance->rs485_stream_length > 0U)
    {
        uint8_t* stream = instance->rs485_stream;

        if (stream[0] == DM_IMU_RS485_ACTIVE_HEAD_1)
        {
            uint16_t frame_length;
            uint8_t compatibility;
            DMIMUDataType_e data_type;

            if (instance->rs485_stream_length < 4U)
            {
                return;
            }
            if (stream[1] != DM_IMU_RS485_ACTIVE_HEAD_2)
            {
                instance->statistics.invalid_header_count++;
                DMIMURS485Consume(instance, 1U);
                continue;
            }

            data_type = (DMIMUDataType_e)stream[3];
            if (data_type < DM_IMU_DATA_ACCEL || data_type > DM_IMU_DATA_QUATERNION)
            {
                instance->statistics.unknown_type_count++;
                DMIMURS485Consume(instance, 1U);
                continue;
            }

            frame_length = data_type == DM_IMU_DATA_QUATERNION
                               ? DM_IMU_RS485_QUAT_FRAME_LENGTH
                               : DM_IMU_RS485_NORMAL_FRAME_LENGTH;
            if (instance->rs485_stream_length < frame_length)
            {
                return;
            }
            if (stream[frame_length - 1U] != DM_IMU_RS485_ACTIVE_TAIL)
            {
                instance->statistics.invalid_header_count++;
                DMIMURS485Consume(instance, 1U);
                continue;
            }
            if (stream[2] != instance->rs485_slave_id)
            {
                instance->statistics.invalid_id_count++;
                DMIMURS485Consume(instance, frame_length);
                continue;
            }
            if (DMIMUValidateActiveCRC(stream, frame_length, &compatibility) == 0U)
            {
                instance->statistics.crc_error_count++;
                DMIMURS485Consume(instance, 1U);
                continue;
            }
            if (compatibility != 0U)
            {
                instance->statistics.crc_compatibility_count++;
            }

            DMIMUDecodeRS485ActiveFrame(instance, stream, data_type);
            DMIMURS485Consume(instance, frame_length);
            continue;
        }

        if (stream[0] == DM_IMU_RS485_RESPONSE_HEAD)
        {
            if (instance->rs485_stream_length < DM_IMU_RS485_RESPONSE_FRAME_LENGTH)
            {
                return;
            }
            if (stream[DM_IMU_RS485_RESPONSE_FRAME_LENGTH - 1U] != DM_IMU_RS485_RESPONSE_TAIL)
            {
                instance->statistics.invalid_header_count++;
                DMIMURS485Consume(instance, 1U);
                continue;
            }
            if (stream[2] != instance->rs485_slave_id)
            {
                instance->statistics.invalid_id_count++;
                DMIMURS485Consume(instance, DM_IMU_RS485_RESPONSE_FRAME_LENGTH);
                continue;
            }

            DMIMUDecodeRS485ResponseFrame(instance, stream);
            DMIMURS485Consume(instance, DM_IMU_RS485_RESPONSE_FRAME_LENGTH);
            continue;
        }

        instance->statistics.invalid_header_count++;
        DMIMURS485Consume(instance, 1U);
    }
}

/**
 * @brief 处理指定RS485实例本次由BSP USART提交的接收片段。
 */
static void DMIMURS485Receive(uint8_t callback_index)
{
    DMIMUInstance* instance;
    uint16_t copy_length;

    if (callback_index >= dm_imu_rs485_instance_count)
    {
        return;
    }

    instance = dm_imu_rs485_instances[callback_index];
    if (instance == NULL || instance->usart_instance == NULL ||
        instance->usart_instance->recv_buff == NULL || instance->usart_instance->recv_len == 0U)
    {
        return;
    }

    copy_length = instance->usart_instance->recv_len;
    if ((uint32_t)instance->rs485_stream_length + copy_length > DM_IMU_RS485_STREAM_SIZE)
    {
        /* 先尝试解析旧残帧；正常残帧最多22字节，通常解析后即可腾出足够空间。 */
        DMIMUParseRS485Stream(instance);
    }
    if ((uint32_t)instance->rs485_stream_length + copy_length > DM_IMU_RS485_STREAM_SIZE)
    {
        instance->rs485_stream_length = 0U;
        instance->statistics.stream_overflow_count++;
    }
    if (copy_length > DM_IMU_RS485_STREAM_SIZE)
    {
        copy_length = DM_IMU_RS485_STREAM_SIZE;
        instance->statistics.stream_overflow_count++;
    }

    memcpy(&instance->rs485_stream[instance->rs485_stream_length],
           instance->usart_instance->recv_buff,
           copy_length);
    instance->rs485_stream_length = (uint16_t)(instance->rs485_stream_length + copy_length);
    DMIMUParseRS485Stream(instance);
}

static void DMIMURS485Receive0(void) { DMIMURS485Receive(0U); }
static void DMIMURS485Receive1(void) { DMIMURS485Receive(1U); }

/**
 * @brief 通过注册时固定的物理接口发送一条原始寄存器请求。
 */
static uint8_t DMIMUSendRegister(DMIMUInstance* instance,
                                 uint8_t rs485_type,
                                 uint8_t register_id,
                                 uint8_t operation,
                                 uint32_t data)
{
    uint8_t success = 0U;

    if (instance->transport == DM_IMU_TRANSPORT_CAN)
    {
        uint8_t* frame = instance->can_instance->tx_buff;

        memset(frame, 0, DM_IMU_CAN_FRAME_LENGTH);
        frame[0] = DM_IMU_CAN_REQUEST_HEAD;
        frame[1] = register_id;
        frame[2] = operation;
        frame[3] = DM_IMU_CAN_REQUEST_DATA_HEAD;
        DMIMUWriteU32LE(&frame[4], data);
        success = CANTransmit(instance->can_instance, instance->can_transmit_timeout_ms);
    }
    else
    {
        uint8_t frame[DM_IMU_RS485_RESPONSE_FRAME_LENGTH] = {0};

        frame[0] = DM_IMU_RS485_RESPONSE_HEAD;
        frame[1] = rs485_type;
        frame[2] = instance->rs485_slave_id;
        frame[3] = register_id;
        frame[4] = operation;
        DMIMUWriteU32LE(&frame[5], data);
        frame[DM_IMU_RS485_RESPONSE_FRAME_LENGTH - 1U] = DM_IMU_RS485_RESPONSE_TAIL;

        success = USARTSend(instance->usart_instance,
                            frame,
                            sizeof(frame),
                            instance->rs485_transfer_mode) == HAL_OK
                      ? 1U
                      : 0U;
    }

    if (success != 0U)
    {
        instance->last_request_register = register_id;
        instance->last_request_operation = operation;
    }
    else
    {
        instance->statistics.transmit_failure_count++;
    }

    return success;
}

/**
 * @brief 将公共命令映射到CAN寄存器编号。
 */
static uint8_t DMIMUMapCANCommand(DMIMUCommand_e command, uint8_t* register_id)
{
    switch (command)
    {
    case DM_IMU_COMMAND_REBOOT:
        *register_id = 0x00U;
        break;
    case DM_IMU_COMMAND_ZERO_EULER:
        *register_id = 0x05U;
        break;
    case DM_IMU_COMMAND_ACCEL_CALIBRATION:
        *register_id = 0x06U;
        break;
    case DM_IMU_COMMAND_GYRO_CALIBRATION:
        *register_id = 0x07U;
        break;
    case DM_IMU_COMMAND_SET_OUTPUT_INTERVAL:
        *register_id = 0x0AU;
        break;
    case DM_IMU_COMMAND_SET_ACTIVE_MODE:
        *register_id = 0x0BU;
        break;
    case DM_IMU_COMMAND_SAVE_PARAMETERS:
        *register_id = 0xFEU;
        break;
    case DM_IMU_COMMAND_RESTORE_FACTORY:
        *register_id = 0xFFU;
        break;
    default:
        return 0U;
    }

    return 1U;
}

/**
 * @brief 将公共命令映射到RS485指令域寄存器编号。
 */
static uint8_t DMIMUMapRS485Command(DMIMUCommand_e command, uint8_t* register_id)
{
    switch (command)
    {
    case DM_IMU_COMMAND_REBOOT:
        *register_id = 0x00U;
        break;
    case DM_IMU_COMMAND_SAVE_PARAMETERS:
        *register_id = 0x01U;
        break;
    case DM_IMU_COMMAND_ZERO_EULER:
        *register_id = 0x02U;
        break;
    case DM_IMU_COMMAND_GYRO_CALIBRATION:
        *register_id = 0x03U;
        break;
    case DM_IMU_COMMAND_ACCEL_CALIBRATION:
        *register_id = 0x04U;
        break;
    case DM_IMU_COMMAND_RESTORE_FACTORY:
        *register_id = 0x05U;
        break;
    case DM_IMU_COMMAND_SET_ACTIVE_MODE:
        *register_id = 0x06U;
        break;
    case DM_IMU_COMMAND_SET_OUTPUT_INTERVAL:
        *register_id = 0x08U;
        break;
    default:
        return 0U;
    }

    return 1U;
}

/**
 * @brief 注册一个外部DM-IMU-L1实例。
 */
DMIMUInstance* DMIMURegister(const DMIMU_Init_Config_s* config)
{
    DMIMUInstance* instance;

    if (config == NULL || dm_imu_instance_count >= DM_IMU_MAX_INSTANCE_COUNT)
    {
        LOGERROR("[dm_imu] invalid config or instance pool full");
        return NULL;
    }

    instance = &dm_imu_instance_pool[dm_imu_instance_count];
    memset(instance, 0, sizeof(*instance));
    instance->transport = config->transport;
    instance->online_timeout_ms = config->online_timeout_ms == 0U
                                      ? DM_IMU_DEFAULT_ONLINE_TIMEOUT_MS
                                      : config->online_timeout_ms;
    instance->register_response.ack = DM_IMU_ACK_UNKNOWN;

    if (config->transport == DM_IMU_TRANSPORT_CAN)
    {
        CAN_Init_Config_s can_config;

        if (config->communication.can.can_handle == NULL ||
            config->communication.can.can_id > 0x7FFU ||
            config->communication.can.mst_id > 0x7FFU ||
            !isfinite(config->communication.can.transmit_timeout_ms) ||
            config->communication.can.transmit_timeout_ms < 0.0F)
        {
            LOGERROR("[dm_imu] invalid CAN config");
            return NULL;
        }

        memset(&can_config, 0, sizeof(can_config));
        can_config.can_handle = config->communication.can.can_handle;
        can_config.tx_id = config->communication.can.can_id;
        can_config.rx_id = config->communication.can.mst_id;
        can_config.can_module_callback = DMIMUCANReceiveCallback;
        can_config.id = instance;
        instance->can_instance = CANRegister(&can_config);
        if (instance->can_instance == NULL)
        {
            LOGERROR("[dm_imu] CAN register failed");
            return NULL;
        }

        CANSetDLC(instance->can_instance, DM_IMU_CAN_FRAME_LENGTH);
        instance->can_transmit_timeout_ms = config->communication.can.transmit_timeout_ms > 0.0F
                                                ? config->communication.can.transmit_timeout_ms
                                                : 1.0F;
    }
    else if (config->transport == DM_IMU_TRANSPORT_RS485)
    {
        USART_Init_Config_s usart_config;

        if (config->communication.rs485.uart_handle == NULL ||
            dm_imu_rs485_instance_count >= DM_IMU_MAX_RS485_INSTANCE_COUNT ||
            (config->communication.rs485.transfer_mode != USART_TRANSFER_BLOCKING &&
                config->communication.rs485.transfer_mode != USART_TRANSFER_IT &&
                config->communication.rs485.transfer_mode != USART_TRANSFER_DMA))
        {
            LOGERROR("[dm_imu] invalid RS485 config");
            return NULL;
        }

        instance->rs485_slave_id = config->communication.rs485.slave_id;
        instance->rs485_transfer_mode = config->communication.rs485.transfer_mode;
        dm_imu_rs485_instances[dm_imu_rs485_instance_count] = instance;

        memset(&usart_config, 0, sizeof(usart_config));
        usart_config.usart_handle = config->communication.rs485.uart_handle;
        usart_config.recv_buff_size = USART_RXBUFF_LIMIT;
        usart_config.module_callback = dm_imu_rs485_instance_count == 0U
                                           ? DMIMURS485Receive0
                                           : DMIMURS485Receive1;
        instance->usart_instance = USARTRegister(&usart_config);
        if (instance->usart_instance == NULL)
        {
            dm_imu_rs485_instances[dm_imu_rs485_instance_count] = NULL;
            LOGERROR("[dm_imu] RS485 USART register failed");
            return NULL;
        }

        dm_imu_rs485_instance_count++;
    }
    else
    {
        LOGERROR("[dm_imu] unknown transport");
        return NULL;
    }

    dm_imu_instance_count++;
    LOGINFO("[dm_imu] instance registered, transport=%u", (unsigned int)instance->transport);
    return instance;
}

/**
 * @brief 获取实例注册时固定的通信接口。
 */
DMIMUTransport_e DMIMUGetTransport(const DMIMUInstance* instance)
{
    return instance == NULL ? DM_IMU_TRANSPORT_INVALID : instance->transport;
}

/**
 * @brief 获取线程安全的完整测量快照。
 */
uint8_t DMIMUGetMeasure(const DMIMUInstance* instance, DMIMUMeasure_s* measure)
{
    uint32_t primask;

    if (instance == NULL || measure == NULL)
    {
        return 0U;
    }

    DMIMUEnterCritical(&primask);
    *measure = instance->measure;
    DMIMUExitCritical(primask);
    return 1U;
}

/**
 * @brief 获取最近一次寄存器应答。
 */
uint8_t DMIMUGetRegisterResponse(const DMIMUInstance* instance, DMIMURegisterResponse_s* response)
{
    uint32_t primask;

    if (instance == NULL || response == NULL)
    {
        return 0U;
    }

    DMIMUEnterCritical(&primask);
    if (instance->register_response.received == 0U)
    {
        DMIMUExitCritical(primask);
        return 0U;
    }
    *response = instance->register_response;
    DMIMUExitCritical(primask);
    return 1U;
}

/**
 * @brief 判断实例是否在线。
 */
uint8_t DMIMUIsOnline(const DMIMUInstance* instance)
{
    uint8_t has_received;
    uint32_t last_tick;
    uint32_t primask;

    if (instance == NULL)
    {
        return 0U;
    }

    DMIMUEnterCritical(&primask);
    has_received = instance->has_received_frame;
    last_tick = instance->last_receive_tick_ms;
    DMIMUExitCritical(primask);

    return has_received != 0U &&
           (uint32_t)(HAL_GetTick() - last_tick) <= instance->online_timeout_ms
               ? 1U
               : 0U;
}

/**
 * @brief 判断指定类型的传感器数据是否新鲜。
 */
uint8_t DMIMUIsDataFresh(const DMIMUInstance* instance,
                         DMIMUDataType_e data_type,
                         uint32_t timeout_ms)
{
    uint8_t valid_mask;
    uint8_t required_mask;
    uint32_t last_tick;
    uint32_t primask;

    if (instance == NULL)
    {
        return 0U;
    }

    if (timeout_ms == 0U)
    {
        timeout_ms = instance->online_timeout_ms;
    }

    DMIMUEnterCritical(&primask);
    valid_mask = instance->measure.valid_mask;
    switch (data_type)
    {
    case DM_IMU_DATA_ACCEL:
        required_mask = DM_IMU_VALID_ACCEL;
        last_tick = instance->measure.accel_last_tick_ms;
        break;
    case DM_IMU_DATA_GYRO:
        required_mask = DM_IMU_VALID_GYRO;
        last_tick = instance->measure.gyro_last_tick_ms;
        break;
    case DM_IMU_DATA_EULER:
        required_mask = DM_IMU_VALID_EULER;
        last_tick = instance->measure.euler_last_tick_ms;
        break;
    case DM_IMU_DATA_QUATERNION:
        required_mask = DM_IMU_VALID_QUATERNION;
        last_tick = instance->measure.quaternion_last_tick_ms;
        break;
    default:
        DMIMUExitCritical(primask);
        return 0U;
    }
    DMIMUExitCritical(primask);

    return (valid_mask & required_mask) != 0U &&
           (uint32_t)(HAL_GetTick() - last_tick) <= timeout_ms
               ? 1U
               : 0U;
}

/**
 * @brief 在请求模式下请求一种传感器数据。
 */
uint8_t DMIMURequestData(DMIMUInstance* instance, DMIMUDataType_e data_type)
{
    uint8_t register_id;
    uint8_t rs485_type;

    if (instance == NULL || data_type < DM_IMU_DATA_ACCEL || data_type > DM_IMU_DATA_QUATERNION)
    {
        return 0U;
    }

    if (instance->transport == DM_IMU_TRANSPORT_CAN)
    {
        register_id = (uint8_t)data_type;
        rs485_type = 0U;
    }
    else
    {
        /* RS485数据域寄存器为0~3，比公共数据类型枚举小1。 */
        register_id = (uint8_t)data_type - 1U;
        rs485_type = DM_IMU_RS485_TYPE_DATA;
    }

    return DMIMUSendRegister(instance,
                             rs485_type,
                             register_id,
                             DM_IMU_OPERATION_READ,
                             0U);
}

/**
 * @brief 执行一个CAN和RS485共有的高层命令。
 */
uint8_t DMIMUExecuteCommand(DMIMUInstance* instance,
                            DMIMUCommand_e command,
                            uint32_t value)
{
    uint8_t register_id;
    uint8_t mapped;

    if (instance == NULL)
    {
        return 0U;
    }

    if (command == DM_IMU_COMMAND_SET_ACTIVE_MODE && value > 1U)
    {
        return 0U;
    }

    mapped = instance->transport == DM_IMU_TRANSPORT_CAN
                 ? DMIMUMapCANCommand(command, &register_id)
                 : DMIMUMapRS485Command(command, &register_id);
    if (mapped == 0U)
    {
        return 0U;
    }

    return DMIMUSendRegister(instance,
                             instance->transport == DM_IMU_TRANSPORT_RS485
                                 ? DM_IMU_RS485_TYPE_COMMAND
                                 : 0U,
                             register_id,
                             DM_IMU_OPERATION_WRITE,
                             value);
}

/**
 * @brief 获取运行统计快照。
 */
uint8_t DMIMUGetStatistics(const DMIMUInstance* instance, DMIMUStatistics_s* statistics)
{
    uint32_t primask;

    if (instance == NULL || statistics == NULL)
    {
        return 0U;
    }

    DMIMUEnterCritical(&primask);
    *statistics = instance->statistics;
    DMIMUExitCritical(primask);
    return 1U;
}
