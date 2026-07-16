/**
 * @file motor_def.h
 * @author neozng
 * @brief  电机通用的数据结构定义
 * @version beta
 * @date 2022-11-01
 *
 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 *
 */

#ifndef MOTOR_DEF_H
#define MOTOR_DEF_H

#include "stdint.h"
#include "bsp_can.h"

#define LIMIT_MIN_MAX(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

/* 电机正反转标志 */
typedef enum
{
    MOTOR_DIRECTION_NORMAL = 0,
    MOTOR_DIRECTION_REVERSE = 1
} Motor_Reverse_Flag_e;

typedef enum
{
    MOTOR_STOP = 0,
    MOTOR_ENABLED = 1,
} Motor_Working_Type_e;

/* 电机类型枚举 */
typedef enum
{
    MOTOR_TYPE_NONE = 0,
    GM6020,
    M3508,
    M2006,
    LK9025,
    HT04,
    DM_MOTOR,
} Motor_Type_e;

/* 用于初始化CAN电机的结构体,各类电机通用 */
typedef struct
{
    Motor_Reverse_Flag_e motor_reverse_flag;
    Motor_Type_e motor_type;
    CAN_Init_Config_s can_init_config;
} Motor_Init_Config_s;

#endif // !MOTOR_DEF_H
