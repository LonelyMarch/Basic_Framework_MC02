#ifndef HTM_MOTOR_H
#define HTM_MOTOR_H

#include <stdint.h>

#include "daemon.h"
#include "htm_rs485.h"
#include "motor_def.h"

#define HTM_ENCODER_COUNTS_PER_ROUND 16384.0f

typedef enum
{
    HTM_CONTROL_OPEN_LOOP = 0,
    HTM_CONTROL_SPEED,
    HTM_CONTROL_ABSOLUTE_POSITION,
    HTM_CONTROL_RELATIVE_POSITION,
} HTMMotor_Control_Mode_e;

typedef struct
{
    uint16_t single_round_count;
    int32_t total_count;
    int16_t speed_0_1rpm;
    float single_round_angle_deg;
    float total_angle_deg;
    float speed_rpm;
    float voltage_v;
    float current_a;
    float temperature_c;
    uint8_t fault_code;
    uint8_t running_state;
} HTMMotor_Measure_t;

typedef struct
{
    HTMRS485Bus *bus;
    uint8_t device_address;
    HTMMotor_Control_Mode_e control_mode;
    Motor_Reverse_Flag_e motor_reverse_flag;
} HTMMotor_Init_Config_s;

typedef struct HTMMotorInstance
{
    HTMMotor_Measure_t measure;
    HTMRS485Bus *bus;
    DaemonInstance *daemon;
    HTMMotor_Control_Mode_e control_mode;
    Motor_Reverse_Flag_e motor_reverse_flag;
    uint8_t device_address;
    volatile uint8_t offline;
    volatile uint8_t enabled;
    volatile uint8_t command_pending;
    uint8_t pending_command;
    uint8_t pending_data[4];
    uint8_t pending_data_len;
} HTMMotorInstance;

HTMMotorInstance *HTMMotorInit(const HTMMotor_Init_Config_s *config);

HAL_StatusTypeDef HTMMotorEnable(HTMMotorInstance *motor);
HAL_StatusTypeDef HTMMotorStop(HTMMotorInstance *motor);
HAL_StatusTypeDef HTMMotorSetOpenLoop(HTMMotorInstance *motor, int16_t power);
HAL_StatusTypeDef HTMMotorSetSpeed(HTMMotorInstance *motor, float speed_rpm);
HAL_StatusTypeDef HTMMotorSetAbsolutePosition(HTMMotorInstance *motor, float position_deg);
HAL_StatusTypeDef HTMMotorSetRelativePosition(HTMMotorInstance *motor, float delta_deg);
HAL_StatusTypeDef HTMMotorSetCurrentPositionAsZero(HTMMotorInstance *motor);
HAL_StatusTypeDef HTMMotorClearFault(HTMMotorInstance *motor);

uint8_t HTMMotorIsOnline(const HTMMotorInstance *motor);
HTMMotor_Control_Mode_e HTMMotorGetControlMode(const HTMMotorInstance *motor);

#endif
