#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include "stdint.h"
#include "bsp_usart.h"
#include "bsp_pwm.h"

/*
 * 舵机最大实例数。Bus_Servo 用 servo_id 作为索引,
 * 若 ID 可能 > 7 请增大此值。
 */
#define SERVO_MOTOR_CNT 7

/* 串行总线舵机协议常量 */
#define Servo_Frame_First 0x55
#define Servo_Frame_Second 0x55
#define Servo_MAX_BUFF 10
#define SERVO_MOVE_CMD 0x03
#define SERVO_UNLOAD_CMD 0x14
#define SERVO_POS_READ_CMD 0x15

/* 舵机类型 */
typedef enum
{
    Servo_None_Type = 0,
    Bus_Servo = 1, // 串行总线舵机,走 USART
    PWM_Servo = 2, // 传统 PWM 舵机,走 BSP PWM
} ServoType_e;

/* 舵机初始化配置 */
typedef struct
{
    PWM_Init_Config_s pwm_init_config; // PWM 舵机需要
    ServoType_e servo_type; // 舵机类型
    UART_HandleTypeDef* _handle; // 总线舵机需要,对应的 UART 句柄
    uint8_t servo_id; // 总线舵机 ID
} Servo_Init_Config_s;

/* 舵机实例 */
typedef struct
{
    uint8_t servo_id; // 总线舵机 ID
    float angle; // 当前目标角度
    uint16_t recv_angle; // 最近一次读取的角度 (总线舵机)
    PWMInstance* pwm_instance; // PWM 舵机的 PWM 实例
    USARTInstance* usart_instance; // 总线舵机的 USART 实例
    ServoType_e servo_type; // 舵机类型
} ServoInstance;


ServoInstance* ServoInit(Servo_Init_Config_s* config);


void ServoSetAngle(ServoInstance* servo, float angle);

#endif // SERVO_MOTOR_H
