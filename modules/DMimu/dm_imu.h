#ifndef DM_IMU_H
#define DM_IMU_H

#include "bsp_can.h"
#include "bsp_usart.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DM_IMU_MAX_INSTANCE_COUNT        4U
#define DM_IMU_MAX_RS485_INSTANCE_COUNT  2U
#define DM_IMU_DEFAULT_ONLINE_TIMEOUT_MS 100U

#define DM_IMU_VALID_ACCEL      (1U << 0U)
#define DM_IMU_VALID_GYRO       (1U << 1U)
#define DM_IMU_VALID_EULER      (1U << 2U)
#define DM_IMU_VALID_QUATERNION (1U << 3U)

/**
 * @brief DM-IMU-L1实例采用的物理通信接口。
 *
 * @note 通信接口在注册时固定，驱动不提供运行期切换接口。
 */
typedef enum
{
    DM_IMU_TRANSPORT_CAN = 0,
    DM_IMU_TRANSPORT_RS485,
    DM_IMU_TRANSPORT_INVALID = 0xFF,
} DMIMUTransport_e;

/** 官方定义的四类传感器数据。枚举值与CAN DATA[0]保持一致。 */
typedef enum
{
    DM_IMU_DATA_ACCEL = 0x01,
    DM_IMU_DATA_GYRO = 0x02,
    DM_IMU_DATA_EULER = 0x03,
    DM_IMU_DATA_QUATERNION = 0x04,
} DMIMUDataType_e;

/**
 * @brief 驱动对外提供的公共高层命令。
 *
 * @note CAN与RS485的寄存器编号不同，驱动会根据注册时固定的通信接口自动映射。
 */
typedef enum
{
    DM_IMU_COMMAND_REBOOT = 0,
    DM_IMU_COMMAND_ZERO_EULER,
    DM_IMU_COMMAND_ACCEL_CALIBRATION,
    DM_IMU_COMMAND_GYRO_CALIBRATION,
    DM_IMU_COMMAND_SAVE_PARAMETERS,
    DM_IMU_COMMAND_RESTORE_FACTORY,
    DM_IMU_COMMAND_SET_ACTIVE_MODE,
    DM_IMU_COMMAND_SET_OUTPUT_INTERVAL,
} DMIMUCommand_e;

/** 官方寄存器应答码。 */
typedef enum
{
    DM_IMU_ACK_SUCCESS = 0x00,
    DM_IMU_ACK_REGISTER_NOT_FOUND = 0x01,
    DM_IMU_ACK_INVALID_DATA = 0x02,
    DM_IMU_ACK_OPERATION_FAILED = 0x03,
    DM_IMU_ACK_UNKNOWN = 0xFF,
} DMIMUAck_e;

/** CAN通信注册参数。 */
typedef struct
{
    FDCAN_HandleTypeDef* can_handle; /**< 使用的FDCAN句柄。 */
    uint16_t can_id; /**< STM32向IMU发送请求时使用的标准帧ID。 */
    uint16_t mst_id; /**< IMU向STM32反馈时使用的标准帧ID。 */
    float transmit_timeout_ms; /**< CANTransmit()提交等待上限，单位ms。 */
} DMIMUCANConfig_s;

/** RS485通信注册参数。 */
typedef struct
{
    UART_HandleTypeDef* uart_handle; /**< 使用的UART句柄，硬件DE由CubeMX配置。 */
    uint8_t slave_id; /**< RS485协议中的IMU从机ID。 */
    USART_TRANSFER_MODE transfer_mode; /**< 请求发送使用阻塞、IT或DMA模式。 */
} DMIMURS485Config_s;

/**
 * @brief DM-IMU-L1实例注册参数。
 */
typedef struct
{
    DMIMUTransport_e transport; /**< 注册时固定为CAN或RS485。 */
    uint32_t online_timeout_ms; /**< 任意合法数据或应答超过该时间未到达则视为离线；0使用默认值。 */
    union
    {
        DMIMUCANConfig_s can;
        DMIMURS485Config_s rs485;
    } communication;
} DMIMU_Init_Config_s;

/**
 * @brief 外部DM-IMU-L1的完整测量快照。
 */
typedef struct
{
    float accel_mps2[3]; /**< 三轴加速度，单位m/s^2。 */
    float gyro_rad_s[3]; /**< 三轴角速度，单位rad/s。 */
    float roll_deg; /**< 横滚角，单位deg。 */
    float pitch_deg; /**< 俯仰角，单位deg。 */
    float yaw_deg; /**< 航向角，单位deg。 */
    float quaternion[4]; /**< 四元数，顺序为w、x、y、z。 */
    float temperature_c; /**< CAN加速度帧携带的温度估计，单位摄氏度。 */
    uint8_t temperature_raw; /**< CAN加速度帧DATA[1]原始温度字节。 */
    uint8_t valid_mask; /**< 已经至少收到过的数据类型，使用DM_IMU_VALID_xxx位。 */
    uint32_t accel_update_count;
    uint32_t gyro_update_count;
    uint32_t euler_update_count;
    uint32_t quaternion_update_count;
    uint32_t accel_last_tick_ms;
    uint32_t gyro_last_tick_ms;
    uint32_t euler_last_tick_ms;
    uint32_t quaternion_last_tick_ms;
} DMIMUMeasure_s;

/** 最近一次寄存器应答快照。 */
typedef struct
{
    uint8_t received; /**< 是否已经收到过寄存器应答。 */
    uint8_t register_id; /**< 应答对应的原始寄存器编号。 */
    uint8_t operation; /**< 0为读，1为写。CAN应答不回传该字段时保持最近请求值。 */
    DMIMUAck_e ack; /**< 官方应答码。 */
    uint32_t data; /**< 应答中的第一个32位原始数据。 */
    uint32_t update_count; /**< 寄存器应答累计次数。 */
} DMIMURegisterResponse_s;

/** 运行统计快照。 */
typedef struct
{
    uint32_t received_frame_count;
    uint32_t sensor_frame_count;
    uint32_t register_response_count;
    uint32_t invalid_length_count;
    uint32_t invalid_header_count;
    uint32_t invalid_id_count;
    uint32_t invalid_value_count;
    uint32_t unknown_type_count;
    uint32_t crc_error_count;
    uint32_t crc_compatibility_count; /**< 使用标准CCITT或相反CRC字节序通过兼容校验的帧数。 */
    uint32_t stream_overflow_count;
    uint32_t transmit_failure_count;
} DMIMUStatistics_s;

typedef struct DMIMUInstance DMIMUInstance;

/**
 * @brief 注册一个外部DM-IMU-L1实例。
 *
 * @param config 通信接口、句柄、ID和在线超时配置。
 * @return DMIMUInstance* 注册成功返回实例，配置非法或BSP注册失败返回NULL。
 */
DMIMUInstance* DMIMURegister(const DMIMU_Init_Config_s* config);


/**
 * @brief 获取实例注册时固定的通信接口。
 *
 * @param instance DM-IMU实例。
 * @return DMIMUTransport_e 合法实例返回固定接口；NULL返回DM_IMU_TRANSPORT_INVALID。
 */
DMIMUTransport_e DMIMUGetTransport(const DMIMUInstance* instance);


/**
 * @brief 获取线程安全的完整测量快照。
 *
 * @param instance DM-IMU实例。
 * @param measure 输出测量快照。
 * @return uint8_t 参数合法返回1，否则返回0。
 */
uint8_t DMIMUGetMeasure(const DMIMUInstance* instance, DMIMUMeasure_s* measure);


/**
 * @brief 获取最近一次寄存器应答。
 *
 * @param instance DM-IMU实例。
 * @param response 输出应答快照。
 * @return uint8_t 已收到过应答且参数合法返回1，否则返回0。
 */
uint8_t DMIMUGetRegisterResponse(const DMIMUInstance* instance, DMIMURegisterResponse_s* response);


/**
 * @brief 判断实例是否在线。
 *
 * @note CAN传感器帧、RS485主动数据帧或合法寄存器应答都会刷新在线时间。
 * @param instance DM-IMU实例。
 * @return uint8_t 未超过注册时配置的在线超时返回1，否则返回0。
 */
uint8_t DMIMUIsOnline(const DMIMUInstance* instance);


/**
 * @brief 判断指定类型的传感器数据是否新鲜。
 *
 * @param instance DM-IMU实例。
 * @param data_type 数据类型。
 * @param timeout_ms 数据新鲜度上限；0使用实例在线超时。
 * @return uint8_t 已收到该数据且未超时返回1，否则返回0。
 */
uint8_t DMIMUIsDataFresh(const DMIMUInstance* instance,
                         DMIMUDataType_e data_type,
                         uint32_t timeout_ms);


/**
 * @brief 在请求模式下请求一种传感器数据。
 *
 * @note 主动模式通常不需要调用本函数。RS485每次只提交一帧，请等待应答后再请求下一类数据。
 * @param instance DM-IMU实例。
 * @param data_type 要请求的数据类型。
 * @return uint8_t 成功提交到底层发送接口返回1，否则返回0。
 */
uint8_t DMIMURequestData(DMIMUInstance* instance, DMIMUDataType_e data_type);


/**
 * @brief 执行一个CAN和RS485共有的高层命令。
 *
 * @param instance DM-IMU实例。
 * @param command 命令类型。
 * @param value 命令参数；主动模式使用0/1，发送间隔使用设备原始间隔值，其他命令填0。
 * @return uint8_t 成功提交到底层发送接口返回1，否则返回0。
 */
uint8_t DMIMUExecuteCommand(DMIMUInstance* instance,
                            DMIMUCommand_e command,
                            uint32_t value);


/**
 * @brief 获取运行统计快照。
 *
 * @param instance DM-IMU实例。
 * @param statistics 输出统计数据。
 * @return uint8_t 参数合法返回1，否则返回0。
 */
uint8_t DMIMUGetStatistics(const DMIMUInstance* instance, DMIMUStatistics_s* statistics);

#ifdef __cplusplus
}
#endif

#endif
