/**
 ******************************************************************************
 * @file	 user_lib.h
 * @author  Wang Hongxi
 * @version V1.0.0
 * @date    2021/2/18
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _USER_LIB_H
#define _USER_LIB_H

#include <stddef.h>
#include "stdint.h"
#include "main.h"
#include "cmsis_os.h"
#include "arm_math.h"

#ifndef user_malloc
#if defined(CMSIS_OS_H_) || defined(CMSIS_OS2_H_)
#include "FreeRTOS.h"
#define user_malloc pvPortMalloc
#else
#include <stdlib.h>
#define user_malloc malloc
#endif
#endif

#define msin(x) (arm_sin_f32(x))
#define mcos(x) (arm_cos_f32(x))

typedef arm_matrix_instance_f32 mat;
// 若运算速度不够,可以使用q31代替f32,但是精度会降低
#define MatAdd arm_mat_add_f32
#define MatSubtract arm_mat_sub_f32
#define MatMultiply arm_mat_mult_f32
#define MatTranspose arm_mat_trans_f32
#define MatInverse arm_mat_inverse_f32


void MatInit(mat* m, uint8_t row, uint8_t col);


/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

/* circumference ratio */
#ifndef PI
#define PI 3.14159265354f
#endif

#define VAL_LIMIT(val, min, max) \
    do                           \
    {                            \
        if ((val) <= (min))      \
        {                        \
            (val) = (min);       \
        }                        \
        else if ((val) >= (max)) \
        {                        \
            (val) = (max);       \
        }                        \
    } while (0)

#define ANGLE_LIMIT_360(val, angle)     \
    do                                  \
    {                                   \
        (val) = (angle) - (int)(angle); \
        (val) += (int)(angle) % 360;    \
    } while (0)

#define ANGLE_LIMIT_360_TO_180(val) \
    do                              \
    {                               \
        if ((val) > 180)            \
            (val) -= 360;           \
    } while (0)

#define VAL_MIN(a, b) ((a) < (b) ? (a) : (b))
#define VAL_MAX(a, b) ((a) > (b) ? (a) : (b))


/* 申请并清零一块内存,返回值仍需要由调用者转换为目标类型。 */
void* zmalloc(size_t size);


// 快速开方
float Sqrt(float x);


// 绝对值限幅
float abs_limit(float num, float Limit);


// 符号判断
float sign(float value);


// 浮点死区
float float_deadband(float Value, float minValue, float maxValue);


// 浮点限幅
float float_constrain(float Value, float minValue, float maxValue);


// int16限幅
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);


// 循环限幅
float loop_float_constrain(float Input, float minValue, float maxValue);


// 角度格式化到 -180 ~ 180
float theta_format(float Ang);


int float_rounding(float raw);


float* Norm3d(float* v);


float NormOf3d(float* v);


void Cross3d(float* v1, float* v2, float* res);


float Dot3d(float* v1, float* v2);


float AverageFilter(float new_data, float* buf, uint8_t len);


#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)

#endif
