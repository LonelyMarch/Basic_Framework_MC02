/**
 * @file ddt_motor.h
 * @brief 本末科技 M0601C 电机 RS485 通信驱动
 *
 * @details 驱动严格按照《M0601C+电机使用参考_20240613.pdf》实现已公开的
 *          驱动、状态查询、ID 设置和 ID 查询指令。控制模式由官方上位机预先
 *          写入电机，并在实例注册时固定声明，运行期不发送模式切换指令。
 */

#ifndef DDT_MOTOR_H
#define DDT_MOTOR_H

#include <stdint.h>

#include "bsp_usart.h"
#include "motor_def.h"

#define DDT_MOTOR_BUS_COUNT 2U
#define DDT_MOTOR_COUNT_PER_BUS 8U
#define DDT_MOTOR_FRAME_LENGTH 10U
#define DDT_MOTOR_RAW_FEEDBACK_MAX_LENGTH 64U

typedef struct DDTMotorBus DDTMotorBus;

/**
 * @brief M0601C 固定控制模式
 *
 * @note 枚举值与官方模式编号完全一致，用于注册实例和核对反馈模式。
 */
typedef enum
{
    DDT_MOTOR_MODE_CURRENT = 0x01U,
    DDT_MOTOR_MODE_SPEED = 0x02U,
    DDT_MOTOR_MODE_POSITION = 0x03U,
} DDTMotorMode_e;

/**
 * @brief 最近一次已解析反馈的类型
 */
typedef enum
{
    DDT_MOTOR_FEEDBACK_NONE = 0U,
    DDT_MOTOR_FEEDBACK_DRIVE,  /* 普通 0x64 驱动指令的反馈 */
    DDT_MOTOR_FEEDBACK_STATUS, /* 0x74“获取其他反馈”指令的反馈 */
} DDTMotorFeedbackType_e;

/**
 * @brief RS485 总线初始化配置
 */
typedef struct
{
    UART_HandleTypeDef *usart_handle;       /* USART2/USART3 等硬件 RS485 串口句柄 */
    USART_TRANSFER_MODE transfer_mode;      /* 推荐使用 DMA 或 IT，阻塞模式也受支持 */
    uint16_t response_timeout_ms;           /* 查询指令等待回复的超时，0 表示默认 20 ms */
    uint16_t inter_frame_interval_ms;       /* 帧间最小间隔，0 表示按官方例程默认 10 ms */
} DDTMotorBusInitConfig_t;

/**
 * @brief M0601C 电机实例初始化配置
 */
typedef struct
{
    DDTMotorBus *bus;                       /* 电机所属 RS485 总线 */
    uint8_t device_id;                      /* 电机 ID，官方示例使用 1、2、3、4 */
    DDTMotorMode_e control_mode;             /* 注册时固定的控制模式 */
    Motor_Reverse_Flag_e motor_reverse_flag; /* 只作用于电流和速度的命令与目标 */
    uint8_t acceleration_time;               /* 驱动帧 DATA[6]，0 表示最快加速 */
    uint16_t refresh_period_ms;              /* 周期刷新间隔，0 表示仅在目标改变时发送 */
} DDTMotorInitConfig_t;

/**
 * @brief 原始回复数据
 */
typedef struct
{
    uint8_t data[DDT_MOTOR_RAW_FEEDBACK_MAX_LENGTH];
    uint16_t length;
    uint32_t receive_count;
    uint8_t crc_valid;                      /* 长度为 10 字节时，对前 9 字节执行 MAXIM 校验 */
} DDTMotorRawFeedback_t;

/**
 * @brief M0601C 已解析反馈数据
 *
 * @note 普通驱动反馈的 DATA[6:7] 是 16 位位置；状态查询反馈的 DATA[6]
 *       是温度、DATA[7] 仅是单字节位置原始值。后者无法使用官方表中的
 *       16 位位置公式，因此不会覆盖 position_raw 和 position_deg。
 */
typedef struct
{
    float current_a;                       /* 电流，换算公式：signed_raw × 8 / 32767 */
    float speed_rpm;                       /* 速度，16 位有符号原始值，单位 rpm */
    float position_deg;                    /* 普通驱动反馈位置：raw × 360 / 32768 */
    int16_t current_raw;
    int16_t speed_raw;
    uint16_t position_raw;
    uint8_t status_position_raw;           /* 0x74 状态反馈 DATA[7]，官方未给换算公式 */
    uint8_t temperature_c;                 /* 0x74 状态反馈 DATA[6]，单位 ℃ */
    uint8_t mode;
    uint8_t error_code;
    uint8_t feedback_received;
    uint8_t mode_mismatch;                  /* 反馈模式与实例注册模式不一致时置 1 */
    DDTMotorFeedbackType_e feedback_type;
    float last_feedback_ms;
} DDTMotorMeasure_t;

/**
 * @brief M0601C 电机运行实例
 */
typedef struct DDTMotorInstance
{
    DDTMotorBus *bus;
    DDTMotorRawFeedback_t raw_feedback;
    DDTMotorMeasure_t measure;
    DDTMotorMode_e control_mode;
    Motor_Reverse_Flag_e motor_reverse_flag;
    uint8_t device_id;
    uint8_t acceleration_time;
    uint8_t enabled;
    uint8_t brake;
    uint8_t target_initialized;             /* 首次使能前是否已通过模式专用接口设置安全目标 */
    volatile uint8_t control_pending;
    volatile uint8_t status_query_pending;
    uint16_t refresh_period_ms;
    float last_control_tx_ms;
    int16_t target_raw;
} DDTMotorInstance;

/**
 * @brief 注册一条 M0601C RS485 总线
 *
 * @param config 总线配置
 * @return DDTMotorBus* 成功返回总线实例，失败返回 NULL
 */
DDTMotorBus *DDTMotorBusInit(const DDTMotorBusInitConfig_t *config);

/**
 * @brief 在指定 RS485 总线上注册一个 M0601C 电机实例
 *
 * @param config 电机配置，控制模式在此固定
 * @return DDTMotorInstance* 成功返回实例，失败返回 NULL
 */
DDTMotorInstance *DDTMotorInit(const DDTMotorInitConfig_t *config);

/**
 * @brief 允许实例按照注册时固定的模式发送已配置控制目标
 *
 * @note 首次使能前必须先调用对应模式的 SetCurrent/SetSpeed/SetPosition 设置安全目标。
 *
 * @param motor 电机实例
 * @return HAL_StatusTypeDef 参数合法返回 HAL_OK
 */
HAL_StatusTypeDef DDTMotorEnable(DDTMotorInstance *motor);

/**
 * @brief 停止电机输出
 *
 * @note 电流模式发送 0 电流；速度模式发送 0 rpm 并置刹车字节为 0xFF。
 *       官方位置模式没有定义通用停止/刹车指令，因此位置模式返回 HAL_ERROR。
 *
 * @param motor 电机实例
 * @return HAL_StatusTypeDef 已排队安全停止命令返回 HAL_OK
 */
HAL_StatusTypeDef DDTMotorStop(DDTMotorInstance *motor);

/**
 * @brief 设置电流模式目标
 *
 * @param motor 电流模式实例
 * @param current_a 目标电流，单位 A，驱动内部限制到 -8～8 A
 * @return HAL_StatusTypeDef 成功更新目标返回 HAL_OK
 */
HAL_StatusTypeDef DDTMotorSetCurrent(DDTMotorInstance *motor, float current_a);

/**
 * @brief 设置速度模式目标
 *
 * @param motor 速度模式实例
 * @param speed_rpm 目标速度，单位 rpm，驱动内部限制到 -330～330 rpm
 * @param brake 非 0 时 DATA[7] 写 0xFF，官方说明只在速度模式有效
 * @return HAL_StatusTypeDef 成功更新目标返回 HAL_OK
 */
HAL_StatusTypeDef DDTMotorSetSpeed(DDTMotorInstance *motor, float speed_rpm, uint8_t brake);

/**
 * @brief 设置位置模式目标
 *
 * @param motor 位置模式实例
 * @param position_deg 单圈目标位置，单位度，驱动内部限制到 0～360°
 * @return HAL_StatusTypeDef 成功更新目标返回 HAL_OK
 */
HAL_StatusTypeDef DDTMotorSetPosition(DDTMotorInstance *motor, float position_deg);

/**
 * @brief 修改后续驱动帧的加速时间字段
 *
 * @param motor 电机实例
 * @param acceleration_time 官方驱动帧 DATA[6] 原始值，0 表示最快
 * @return HAL_StatusTypeDef 参数合法返回 HAL_OK
 */
HAL_StatusTypeDef DDTMotorSetAccelerationTime(DDTMotorInstance *motor, uint8_t acceleration_time);

/**
 * @brief 请求官方 0x74“获取其他反馈”指令
 *
 * @param motor 电机实例
 * @return HAL_StatusTypeDef 成功排队返回 HAL_OK，已有查询待处理返回 HAL_BUSY
 */
HAL_StatusTypeDef DDTMotorRequestStatus(DDTMotorInstance *motor);

/**
 * @brief 获取最近一次原始回复
 *
 * @param motor 电机实例
 * @return const DDTMotorRawFeedback_t* 原始回复地址，参数为空返回 NULL
 */
const DDTMotorRawFeedback_t *DDTMotorGetRawFeedback(const DDTMotorInstance *motor);

/**
 * @brief 获取最近一次解析后的电机反馈
 *
 * @param motor 电机实例
 * @return const DDTMotorMeasure_t* 反馈地址，参数为空返回 NULL
 */
const DDTMotorMeasure_t *DDTMotorGetMeasure(const DDTMotorInstance *motor);

/**
 * @brief 根据最近一次反馈时间判断电机是否在线
 *
 * @param motor 电机实例
 * @param timeout_ms 在线超时，必须大于 0
 * @return uint8_t 在超时时间内收到过反馈返回 1，否则返回 0
 */
uint8_t DDTMotorIsOnline(const DDTMotorInstance *motor, float timeout_ms);

/**
 * @brief 在尚未注册电机实例的总线上设置设备 ID
 *
 * @note 官方要求总线上只能有一台电机，并且同一上电周期只允许设置一次。
 *       本函数只负责排队，统一控制任务会连续发送 5 次设置帧。
 *
 * @param bus RS485 总线实例
 * @param new_id 新设备 ID，必须非 0
 * @return HAL_StatusTypeDef 成功排队返回 HAL_OK
 */
HAL_StatusTypeDef DDTMotorBusSetDeviceId(DDTMotorBus *bus, uint8_t new_id);

/**
 * @brief 在尚未注册电机实例的总线上请求查询设备 ID
 *
 * @param bus RS485 总线实例
 * @return HAL_StatusTypeDef 成功排队返回 HAL_OK
 */
HAL_StatusTypeDef DDTMotorBusRequestDeviceId(DDTMotorBus *bus);

/**
 * @brief 获取总线级 ID 查询的最近一次原始回复
 *
 * @param bus RS485 总线实例
 * @return const DDTMotorRawFeedback_t* 总线原始回复地址，参数为空返回 NULL
 */
const DDTMotorRawFeedback_t *DDTMotorBusGetRawFeedback(const DDTMotorBus *bus);

/**
 * @brief 由统一 MotorControlTask 约 1 kHz 调用，调度所有 DDT 总线和电机
 */
void DDTMotorControl(void);

#endif /* DDT_MOTOR_H */
