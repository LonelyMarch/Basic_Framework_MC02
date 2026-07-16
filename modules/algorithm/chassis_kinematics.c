/**
 ******************************************************************************
 * @file    chassis_kinematics.c
 * @brief   麦克纳姆轮/全向轮底盘运动学解算
 ******************************************************************************
 */
#include "chassis_kinematics.h"
#include <math.h>
#include <string.h>

#define KINEMATICS_MIN_DETERMINANT 1.0e-6f
#define KINEMATICS_MIN_VECTOR_NORM 1.0e-6f


static ChassisKinematicsStatus_e Invert3x3(const float a[3][3], float inv[3][3]);


ChassisKinematicsStatus_e OmniKinematicsPrepareConfig(const OmniKinematicsConfig_s* config,
                                                      OmniKinematicsFastConfig_s* fast_config)
{
    if (config == NULL || fast_config == NULL || config->wheel_config == NULL)
    {
        return CHASSIS_KINEMATICS_ERROR_NULL_PTR;
    }
    if (config->wheel_num == 0U || config->wheel_num > CHASSIS_KINEMATICS_MAX_WHEEL_NUM)
    {
        return CHASSIS_KINEMATICS_ERROR_INVALID_PARAM;
    }

    memset(fast_config, 0, sizeof(*fast_config));
    fast_config->wheel_num = config->wheel_num;
    fast_config->wheel_radius = (config->wheel_radius > 0.0f) ? config->wheel_radius : 1.0f;
    fast_config->wheel_radius_inv = (config->wheel_radius > 0.0f) ? (1.0f / config->wheel_radius) : 1.0f;

    float normal_matrix[3][3] = {{0.0f}};
    for (uint8_t i = 0; i < config->wheel_num; ++i)
    {
        const OmniWheelConfig_s* wheel = &config->wheel_config[i];
        float dir_norm_sq = wheel->drive_dir_x * wheel->drive_dir_x + wheel->drive_dir_y * wheel->drive_dir_y;
        if (dir_norm_sq < KINEMATICS_MIN_VECTOR_NORM)
        {
            return CHASSIS_KINEMATICS_ERROR_INVALID_PARAM;
        }

        /*
         * 轮子安装位置和主动滚动方向在运行期通常不变,这里提前归一化方向并计算旋转项系数。
         * 高频控制任务中只需要做乘加运算,避免每次解算都调用sqrtf和重复组矩阵。
         */
        float dir_inv_norm = 1.0f / sqrtf(dir_norm_sq);
        fast_config->drive_x[i] = wheel->drive_dir_x * dir_inv_norm;
        fast_config->drive_y[i] = wheel->drive_dir_y * dir_inv_norm;
        fast_config->rotate_coeff[i] = fast_config->drive_y[i] * wheel->x - fast_config->drive_x[i] * wheel->y;

        float row[3] = {
            fast_config->drive_x[i],
            fast_config->drive_y[i],
            fast_config->rotate_coeff[i],
        };
        for (uint8_t r = 0; r < 3U; ++r)
        {
            for (uint8_t c = 0; c < 3U; ++c)
            {
                normal_matrix[r][c] += row[r] * row[c];
            }
        }
    }

    if (config->wheel_num >= 3U &&
        Invert3x3(normal_matrix, fast_config->inverse_matrix) == CHASSIS_KINEMATICS_OK)
    {
        fast_config->inverse_valid = 1U;
    }

    return CHASSIS_KINEMATICS_OK;
}

ChassisKinematicsStatus_e OmniKinematicsCalculateWheelSpeed(const OmniKinematicsFastConfig_s* fast_config,
                                                            const ChassisVelocity_s* chassis_speed,
                                                            float* wheel_speed)
{
    if (fast_config == NULL || chassis_speed == NULL || wheel_speed == NULL)
    {
        return CHASSIS_KINEMATICS_ERROR_NULL_PTR;
    }
    if (fast_config->wheel_num == 0U || fast_config->wheel_num > CHASSIS_KINEMATICS_MAX_WHEEL_NUM)
    {
        return CHASSIS_KINEMATICS_ERROR_INVALID_PARAM;
    }
    if (fast_config->wheel_radius <= 0.0f || fast_config->wheel_radius_inv <= 0.0f)
    {
        return CHASSIS_KINEMATICS_ERROR_INVALID_PARAM;
    }

    for (uint8_t i = 0; i < fast_config->wheel_num; ++i)
    {
        wheel_speed[i] = (fast_config->drive_x[i] * chassis_speed->vx +
                fast_config->drive_y[i] * chassis_speed->vy +
                fast_config->rotate_coeff[i] * chassis_speed->wz) *
            fast_config->wheel_radius_inv;
    }

    return CHASSIS_KINEMATICS_OK;
}

ChassisKinematicsStatus_e OmniKinematicsEstimateChassisSpeed(const OmniKinematicsFastConfig_s* fast_config,
                                                             const float* wheel_speed,
                                                             ChassisVelocity_s* chassis_speed)
{
    if (fast_config == NULL || wheel_speed == NULL || chassis_speed == NULL)
    {
        return CHASSIS_KINEMATICS_ERROR_NULL_PTR;
    }
    if (fast_config->wheel_num < 3U || fast_config->wheel_num > CHASSIS_KINEMATICS_MAX_WHEEL_NUM)
    {
        return CHASSIS_KINEMATICS_ERROR_INVALID_PARAM;
    }
    if (fast_config->wheel_radius <= 0.0f || fast_config->wheel_radius_inv <= 0.0f)
    {
        return CHASSIS_KINEMATICS_ERROR_INVALID_PARAM;
    }
    if (fast_config->inverse_valid == 0U)
    {
        return CHASSIS_KINEMATICS_ERROR_SINGULAR;
    }

    float normal_vector[3] = {0.0f};
    for (uint8_t i = 0; i < fast_config->wheel_num; ++i)
    {
        float speed = wheel_speed[i] * fast_config->wheel_radius;
        normal_vector[0] += fast_config->drive_x[i] * speed;
        normal_vector[1] += fast_config->drive_y[i] * speed;
        normal_vector[2] += fast_config->rotate_coeff[i] * speed;
    }

    chassis_speed->vx = fast_config->inverse_matrix[0][0] * normal_vector[0] +
        fast_config->inverse_matrix[0][1] * normal_vector[1] +
        fast_config->inverse_matrix[0][2] * normal_vector[2];
    chassis_speed->vy = fast_config->inverse_matrix[1][0] * normal_vector[0] +
        fast_config->inverse_matrix[1][1] * normal_vector[1] +
        fast_config->inverse_matrix[1][2] * normal_vector[2];
    chassis_speed->wz = fast_config->inverse_matrix[2][0] * normal_vector[0] +
        fast_config->inverse_matrix[2][1] * normal_vector[1] +
        fast_config->inverse_matrix[2][2] * normal_vector[2];

    return CHASSIS_KINEMATICS_OK;
}

ChassisKinematicsStatus_e MecanumKinematicsPrepareConfig(const MecanumKinematicsConfig_s* config,
                                                         MecanumKinematicsFastConfig_s* fast_config)
{
    if (config == NULL || fast_config == NULL)
    {
        return CHASSIS_KINEMATICS_ERROR_NULL_PTR;
    }
    if (config->half_wheel_base <= 0.0f || config->half_track_width <= 0.0f)
    {
        return CHASSIS_KINEMATICS_ERROR_INVALID_PARAM;
    }

    fast_config->rotate_scale = config->half_wheel_base + config->half_track_width;
    fast_config->rotate_scale_inv = 1.0f / fast_config->rotate_scale;
    fast_config->wheel_radius = (config->wheel_radius > 0.0f) ? config->wheel_radius : 1.0f;
    fast_config->wheel_radius_inv = (config->wheel_radius > 0.0f) ? (1.0f / config->wheel_radius) : 1.0f;
    return CHASSIS_KINEMATICS_OK;
}

ChassisKinematicsStatus_e MecanumKinematicsCalculateWheelSpeed(const MecanumKinematicsFastConfig_s* fast_config,
                                                               const ChassisVelocity_s* chassis_speed,
                                                               MecanumWheelSpeed_s* wheel_speed)
{
    if (fast_config == NULL || chassis_speed == NULL || wheel_speed == NULL)
    {
        return CHASSIS_KINEMATICS_ERROR_NULL_PTR;
    }
    if (fast_config->rotate_scale <= 0.0f ||
        fast_config->wheel_radius <= 0.0f ||
        fast_config->wheel_radius_inv <= 0.0f)
    {
        return CHASSIS_KINEMATICS_ERROR_INVALID_PARAM;
    }

    float rotate = fast_config->rotate_scale * chassis_speed->wz;
    float vx_minus_vy = chassis_speed->vx - chassis_speed->vy;
    float vx_plus_vy = chassis_speed->vx + chassis_speed->vy;

    wheel_speed->lf = (vx_minus_vy - rotate) * fast_config->wheel_radius_inv;
    wheel_speed->rf = (vx_plus_vy + rotate) * fast_config->wheel_radius_inv;
    wheel_speed->lb = (vx_plus_vy - rotate) * fast_config->wheel_radius_inv;
    wheel_speed->rb = (vx_minus_vy + rotate) * fast_config->wheel_radius_inv;
    return CHASSIS_KINEMATICS_OK;
}

ChassisKinematicsStatus_e MecanumKinematicsEstimateChassisSpeed(const MecanumKinematicsFastConfig_s* fast_config,
                                                                const MecanumWheelSpeed_s* wheel_speed,
                                                                ChassisVelocity_s* chassis_speed)
{
    if (fast_config == NULL || wheel_speed == NULL || chassis_speed == NULL)
    {
        return CHASSIS_KINEMATICS_ERROR_NULL_PTR;
    }
    if (fast_config->rotate_scale <= 0.0f ||
        fast_config->wheel_radius <= 0.0f ||
        fast_config->wheel_radius_inv <= 0.0f)
    {
        return CHASSIS_KINEMATICS_ERROR_INVALID_PARAM;
    }

    float lf = wheel_speed->lf * fast_config->wheel_radius;
    float rf = wheel_speed->rf * fast_config->wheel_radius;
    float lb = wheel_speed->lb * fast_config->wheel_radius;
    float rb = wheel_speed->rb * fast_config->wheel_radius;

    chassis_speed->vx = (lf + rf + lb + rb) * 0.25f;
    chassis_speed->vy = (-lf + rf + lb - rb) * 0.25f;
    chassis_speed->wz = (-lf + rf - lb + rb) * (0.25f * fast_config->rotate_scale_inv);
    return CHASSIS_KINEMATICS_OK;
}

static ChassisKinematicsStatus_e Invert3x3(const float a[3][3], float inv[3][3])
{
    float det =
        a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
        a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
        a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);

    if (fabsf(det) < KINEMATICS_MIN_DETERMINANT)
    {
        return CHASSIS_KINEMATICS_ERROR_SINGULAR;
    }

    float inv_det = 1.0f / det;
    inv[0][0] = (a[1][1] * a[2][2] - a[1][2] * a[2][1]) * inv_det;
    inv[0][1] = (a[0][2] * a[2][1] - a[0][1] * a[2][2]) * inv_det;
    inv[0][2] = (a[0][1] * a[1][2] - a[0][2] * a[1][1]) * inv_det;
    inv[1][0] = (a[1][2] * a[2][0] - a[1][0] * a[2][2]) * inv_det;
    inv[1][1] = (a[0][0] * a[2][2] - a[0][2] * a[2][0]) * inv_det;
    inv[1][2] = (a[0][2] * a[1][0] - a[0][0] * a[1][2]) * inv_det;
    inv[2][0] = (a[1][0] * a[2][1] - a[1][1] * a[2][0]) * inv_det;
    inv[2][1] = (a[0][1] * a[2][0] - a[0][0] * a[2][1]) * inv_det;
    inv[2][2] = (a[0][0] * a[1][1] - a[0][1] * a[1][0]) * inv_det;

    return CHASSIS_KINEMATICS_OK;
}
