#ifndef LK9025_H
#define LK9025_H

#include "bsp_can.h"
#include "motor_def.h"
#include <stdint.h>

#define LK_MOTOR_MX_CNT 32U
#define LK_MULTI_MOTOR_MAX 4U
#define LK_SINGLE_MOTOR_ID_MIN 1U
#define LK_SINGLE_MOTOR_ID_MAX 32U
#define LK_MULTI_MOTOR_ID_MIN 1U
#define LK_MULTI_MOTOR_ID_MAX 4U
#define LK_MULTI_TORQUE_CAN_ID 0x280U
#define LK_SINGLE_CAN_ID_BASE 0x140U
#define LK_CAN_DLC 8U

#define LK_ENCODER_RANGE 16384U
#define LK_ENCODER_HALF_RANGE 8192
#define LK_ECD_ANGLE_COEF (360.0f / 16384.0f)

#define LK_MULTI_TORQUE_MIN (-2000)
#define LK_MULTI_TORQUE_MAX 2000
#define LK_OPEN_POWER_MIN (-1000)
#define LK_OPEN_POWER_MAX 1000
#define LK_TORQUE_CURRENT_MIN (-2000)
#define LK_TORQUE_CURRENT_MAX 2000
#define LK_SINGLE_CIRCLE_ANGLE_MAX 35999U

typedef enum
{
    LK_CMD_READ_PID = 0x30,
    LK_CMD_WRITE_PID_RAM = 0x31,
    LK_CMD_WRITE_PID_ROM = 0x32,
    LK_CMD_READ_ACCEL = 0x33,
    LK_CMD_WRITE_ACCEL_RAM = 0x34,
    LK_CMD_READ_ENCODER = 0x90,
    LK_CMD_WRITE_ENCODER_ZERO_ROM = 0x91,
    LK_CMD_WRITE_CURRENT_POS_ZERO_ROM = 0x19,
    LK_CMD_READ_MULTI_TURN_ANGLE = 0x92,
    LK_CMD_READ_SINGLE_TURN_ANGLE = 0x94,
    LK_CMD_CLEAR_ANGLE = 0x95,
    LK_CMD_READ_STATUS1_ERROR = 0x9A,
    LK_CMD_CLEAR_ERROR = 0x9B,
    LK_CMD_READ_STATUS2 = 0x9C,
    LK_CMD_READ_STATUS3 = 0x9D,
    LK_CMD_MOTOR_OFF = 0x80,
    LK_CMD_MOTOR_STOP = 0x81,
    LK_CMD_MOTOR_RUN = 0x88,
    LK_CMD_OPEN_TORQUE = 0xA0,
    LK_CMD_TORQUE_CONTROL = 0xA1,
    LK_CMD_SPEED_CONTROL = 0xA2,
    LK_CMD_MULTI_TURN_POSITION = 0xA3,
    LK_CMD_MULTI_TURN_POSITION_WITH_SPEED = 0xA4,
    LK_CMD_SINGLE_TURN_POSITION = 0xA5,
    LK_CMD_SINGLE_TURN_POSITION_WITH_SPEED = 0xA6,
    LK_CMD_INCREMENT_POSITION = 0xA7,
    LK_CMD_INCREMENT_POSITION_WITH_SPEED = 0xA8,
} LKMotor_Command_e;

/*
 * 控制模式在注册时固定，运行期不可更改。
 * MULTI_TORQUE 对应通过配置工具启用的 0x280 多电机模式；其余模式均使用
 * 0x140 + ID 单电机协议，并限制上层只能调用与注册模式匹配的控制接口。
 */
typedef enum
{
    LK_MOTOR_MODE_MULTI_TORQUE = 0,
    LK_MOTOR_MODE_SINGLE_OPEN_TORQUE,
    LK_MOTOR_MODE_SINGLE_TORQUE,
    LK_MOTOR_MODE_SINGLE_SPEED,
    LK_MOTOR_MODE_SINGLE_POSITION,
} LKMotor_Work_Mode_e;

typedef enum
{
    LK_MOTOR_SPIN_CW = 0x00,
    LK_MOTOR_SPIN_CCW = 0x01,
} LKMotor_Spin_Direction_e;

typedef struct
{
    uint8_t angle_kp;
    uint8_t angle_ki;
    uint8_t speed_kp;
    uint8_t speed_ki;
    uint8_t iq_kp;
    uint8_t iq_ki;
} LKMotor_PID_Param_s;

typedef struct
{
    uint16_t encoder;
    uint16_t encoder_raw;
    uint16_t encoder_offset;
} LKMotor_Encoder_s;

typedef struct
{
    int8_t temperature;
    uint16_t voltage_0p1v;
    uint8_t error_state;
} LKMotor_Status1_s;

typedef struct
{
    int8_t temperature;
    int16_t phase_a;
    int16_t phase_b;
    int16_t phase_c;
} LKMotor_Status3_s;

typedef struct
{
    uint16_t last_ecd;
    uint16_t ecd;
    float angle_single_round;
    int16_t speed_dps;
    int16_t real_current;
    int16_t open_power;
    int8_t temperature;
    uint8_t feedback_initialized;
    uint8_t real_current_valid;
    uint8_t open_power_valid;

    float total_angle;
    int32_t total_round;

    int64_t multi_turn_angle_0p01deg;
    uint32_t single_turn_angle_0p01deg;
    int32_t accel_dps2;

    LKMotor_PID_Param_s pid_param;
    LKMotor_Encoder_s encoder;
    LKMotor_Status1_s status1;
    LKMotor_Status3_s status3;

    float feed_dt;
    uint32_t feed_dwt_cnt;
} LKMotor_Measure_t;

typedef struct
{
    CAN_Init_Config_s can_init_config;
    LKMotor_Work_Mode_e work_mode;
} LKMotor_Init_Config_s;

typedef struct LKMotorInstance LKMotorInstance;


/* can_init_config.tx_id 填写逻辑电机 ID。模式注册后不可修改。 */
LKMotorInstance* LKMotorInit(const LKMotor_Init_Config_s* config);


LKMotor_Work_Mode_e LKMotorGetMode(const LKMotorInstance* motor);


HAL_StatusTypeDef LKMotorSetMultiTorque(LKMotorInstance* motor, int16_t iq);


HAL_StatusTypeDef LKMotorSetOpenTorque(LKMotorInstance* motor, int16_t power_control);


HAL_StatusTypeDef LKMotorSetTorque(LKMotorInstance* motor, int16_t iq_control);


HAL_StatusTypeDef LKMotorSetSpeed(LKMotorInstance* motor, int32_t speed_0p01dps);


HAL_StatusTypeDef LKMotorSetMultiTurnPosition(LKMotorInstance* motor, int32_t angle_0p01deg);


HAL_StatusTypeDef LKMotorSetMultiTurnPositionWithSpeed(LKMotorInstance* motor,
                                                       int32_t angle_0p01deg,
                                                       uint16_t max_speed_dps);


HAL_StatusTypeDef LKMotorSetSingleTurnPosition(LKMotorInstance* motor,
                                               LKMotor_Spin_Direction_e direction,
                                               uint16_t angle_0p01deg);


HAL_StatusTypeDef LKMotorSetSingleTurnPositionWithSpeed(LKMotorInstance* motor,
                                                        LKMotor_Spin_Direction_e direction,
                                                        uint16_t angle_0p01deg,
                                                        uint16_t max_speed_dps);


HAL_StatusTypeDef LKMotorSetIncrementPosition(LKMotorInstance* motor, int32_t delta_angle_0p01deg);


HAL_StatusTypeDef LKMotorSetIncrementPositionWithSpeed(LKMotorInstance* motor,
                                                       int32_t delta_angle_0p01deg,
                                                       uint16_t max_speed_dps);


HAL_StatusTypeDef LKMotorStop(LKMotorInstance* motor);


HAL_StatusTypeDef LKMotorEnable(LKMotorInstance* motor);


HAL_StatusTypeDef LKMotorOff(LKMotorInstance* motor);


uint8_t LKMotorIsOnline(const LKMotorInstance* motor);


uint8_t LKMotorGetMeasure(LKMotorInstance* motor, LKMotor_Measure_t* measure);


HAL_StatusTypeDef LKMotorReadPID(LKMotorInstance* motor);


HAL_StatusTypeDef LKMotorWritePIDToRAM(LKMotorInstance* motor, const LKMotor_PID_Param_s* pid);


HAL_StatusTypeDef LKMotorWritePIDToROM(LKMotorInstance* motor, const LKMotor_PID_Param_s* pid);


HAL_StatusTypeDef LKMotorReadAcceleration(LKMotorInstance* motor);


HAL_StatusTypeDef LKMotorWriteAccelerationToRAM(LKMotorInstance* motor, int32_t accel_dps2);


HAL_StatusTypeDef LKMotorReadEncoder(LKMotorInstance* motor);


HAL_StatusTypeDef LKMotorWriteEncoderZeroToROM(LKMotorInstance* motor, uint16_t encoder_offset);


HAL_StatusTypeDef LKMotorSetCurrentPositionAsZero(LKMotorInstance* motor);


HAL_StatusTypeDef LKMotorReadMultiTurnAngle(LKMotorInstance* motor);


HAL_StatusTypeDef LKMotorReadSingleTurnAngle(LKMotorInstance* motor);


HAL_StatusTypeDef LKMotorClearAngle(LKMotorInstance* motor);


HAL_StatusTypeDef LKMotorReadStatus1AndError(LKMotorInstance* motor);


HAL_StatusTypeDef LKMotorClearError(LKMotorInstance* motor);


HAL_StatusTypeDef LKMotorReadStatus2(LKMotorInstance* motor);


HAL_StatusTypeDef LKMotorReadStatus3(LKMotorInstance* motor);


/* 由统一 MotorControlTask 约 1 kHz 调用。 */
void LKMotorControl(void);

#endif // LK9025_H
