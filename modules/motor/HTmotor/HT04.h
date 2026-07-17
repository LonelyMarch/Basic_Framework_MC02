#ifndef HT04_H
#define HT04_H

#include <stdint.h>

#include "bsp_can.h"
#include "daemon.h"
#include "motor_def.h"

#define HT04_MOTOR_CNT 4U

#define HT04_POSITION_MIN_RAD (-95.5f)
#define HT04_POSITION_MAX_RAD (95.5f)
#define HT04_VELOCITY_MIN_RAD_S (-45.0f)
#define HT04_VELOCITY_MAX_RAD_S (45.0f)
#define HT04_TORQUE_MIN_NM (-18.0f)
#define HT04_TORQUE_MAX_NM (18.0f)
#define HT04_CURRENT_MIN_A (-40.0f)
#define HT04_CURRENT_MAX_A (40.0f)

typedef struct
{
    float position_rad;
    float last_position_rad;
    float velocity_rad_s;
    float phase_current_a;
    float feed_dt;
    uint32_t feed_cnt;
    uint8_t motor_id;
} HTMotor_Measure_t;

typedef struct
{
    CAN_Init_Config_s can_init_config;
    Motor_Reverse_Flag_e motor_reverse_flag;
    uint8_t expected_motor_id;
    uint8_t enable_on_init;
    uint8_t set_zero_on_init;
} HTMotor_Init_Config_s;

typedef struct
{
    HTMotor_Measure_t measure;
    Motor_Reverse_Flag_e motor_reverse_flag;
    float torque_ref_nm;
    Motor_Working_Type_e stop_flag;
    CANInstance* motor_can_instance;
    DaemonInstance* motor_daemon;
    uint8_t expected_motor_id;
    volatile uint8_t offline;
    volatile uint8_t enabled;
} HTMotorInstance;


HTMotorInstance* HTMotorInit(const HTMotor_Init_Config_s* config);


HAL_StatusTypeDef HTMotorSetTorque(HTMotorInstance* motor, float torque_nm);


HAL_StatusTypeDef HTMotorStop(HTMotorInstance* motor);


HAL_StatusTypeDef HTMotorEnable(HTMotorInstance* motor);


HAL_StatusTypeDef HTMotorDisable(HTMotorInstance* motor);


HAL_StatusTypeDef HTMotorSetCurrentPositionAsZero(HTMotorInstance* motor);


void HTMotorControl(void);


uint8_t HTMotorIsOnline(const HTMotorInstance* motor);


uint8_t HTMotorIsEnabled(const HTMotorInstance* motor);

#endif
