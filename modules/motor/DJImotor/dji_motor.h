/**
 * @file dji_motor.h
 * @brief DJI C610/C620/GM6020 CAN 通信驱动
 */

#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "bsp_can.h"
#include "daemon.h"
#include "motor_def.h"
#include "controller.h"
#include <stdint.h>

#define DJI_MOTOR_CNT 12U

/* 官方协议原始指令范围。 */
#define DJI_C610_CURRENT_RAW_MAX       10000
#define DJI_C620_CURRENT_RAW_MAX       16384
#define DJI_GM6020_CURRENT_RAW_MAX     16384
#define DJI_GM6020_VOLTAGE_RAW_MAX     25000
#define ECD_ANGLE_COEF_DJI             (360.0f / 8192.0f)

typedef enum
{
    DJI_CONTROL_C610_CURRENT = 0,
    DJI_CONTROL_C620_CURRENT,
    DJI_CONTROL_GM6020_VOLTAGE,
    DJI_CONTROL_GM6020_CURRENT,
} DJIMotorProtocolMode_e;

typedef enum
{
    DJI_CONTROL_DIRECT = 0,
    DJI_CONTROL_SPEED,
    DJI_CONTROL_POSITION,
} DJIMotorControlMode_e;

typedef enum
{
    DJI_FEEDBACK_MOTOR = 0,
    DJI_FEEDBACK_EXTERNAL,
} DJIMotorFeedbackSource_e;

typedef struct
{
    CAN_Init_Config_s can_init_config;
    Motor_Reverse_Flag_e motor_reverse_flag;
    DJIMotorControlMode_e control_mode;
    DJIMotorFeedbackSource_e feedback_source;
    volatile const float* position_feedback_ptr; // external 时单位 degree
    volatile const float* speed_feedback_ptr;    // external 时单位 degree/s
    PID_Init_Config_s position_pid;
    PID_Init_Config_s speed_pid;
} DJIMotor_Init_Config_s;

typedef struct
{
    uint16_t last_ecd;
    uint16_t ecd;                    // 协议原始机械角度 0~8191
    int16_t speed_rpm;               // 协议原始转速 rpm
    int16_t torque_current_raw;      // 协议原始转矩电流
    uint8_t temperature;             // C620/GM6020 有效，C610 无此字段
    uint8_t temperature_valid;
    uint8_t feedback_initialized;

    float angle_single_round;        // 绝对单圈角度，单位 degree
    int32_t total_ecd;               // 相对首次反馈的累计编码器增量
    float total_angle;               // 相对首次反馈的累计角度，单位 degree
    float speed_aps;                 // 由 speed_rpm 换算，单位 degree/s
    float torque_current_a;          // 按型号量程换算的转矩电流，单位 A
} DJI_Motor_Measure_s;

typedef struct
{
    DJI_Motor_Measure_s measure;
    Motor_Reverse_Flag_e motor_reverse_flag;
    DJIMotorProtocolMode_e protocol_mode;
    DJIMotorControlMode_e control_mode;
    DJIMotorFeedbackSource_e feedback_source;
    Motor_Type_e motor_type;

    volatile int16_t command_raw;
    volatile float control_ref;
    volatile Motor_Working_Type_e stop_flag;
    volatile uint8_t controller_reset_pending;

    volatile const float* position_feedback_ptr;
    volatile const float* speed_feedback_ptr;
    PIDInstance position_pid;
    PIDInstance speed_pid;

    CANInstance* motor_can_instance;
    CANInstance* sender_can_instance;
    uint8_t message_num;

    DaemonInstance* daemon;
    uint8_t offline_reported;
    uint32_t feed_cnt;
    float dt;
} DJIMotorInstance;

/* C610 + M2006：CAN 转矩电流控制，ID 1~8。 */
DJIMotorInstance* DJIMotorInitM2006(const DJIMotor_Init_Config_s* config);

/* C620 + M3508：CAN 转矩电流控制，ID 1~8。 */
DJIMotorInstance* DJIMotorInitM3508(const DJIMotor_Init_Config_s* config);

/* GM6020：CAN 转矩电压控制，ID 1~7。 */
DJIMotorInstance* DJIMotorInitGM6020Voltage(const DJIMotor_Init_Config_s* config);

/* GM6020：CAN 转矩电流控制，ID 1~7；需在 Assistant 中提前打开电流环。 */
DJIMotorInstance* DJIMotorInitGM6020Current(const DJIMotor_Init_Config_s* config);

/* 设置官方协议原始电流指令；函数会按注册模式自动限幅。 */
void DJIMotorSetCurrentRaw(DJIMotorInstance* motor, int16_t current_raw);

/* 仅适用于 GM6020 电压模式；函数会按官方范围自动限幅。 */
void DJIMotorSetVoltageRaw(DJIMotorInstance* motor, int16_t voltage_raw);

/* 仅适用于注册为 DJI_CONTROL_SPEED 的实例，单位 degree/s。 */
void DJIMotorSetSpeed(DJIMotorInstance* motor, float speed_aps);

/* 仅适用于注册为 DJI_CONTROL_POSITION 的实例，单位 degree。 */
void DJIMotorSetPosition(DJIMotorInstance* motor, float position_degree);

void DJIMotorStop(DJIMotorInstance* motor);
void DJIMotorEnable(DJIMotorInstance* motor);

/* 由统一 MotorControlTask 以 1kHz 调用，完成分组打包与发送。 */
void DJIMotorControl(void);

#endif
