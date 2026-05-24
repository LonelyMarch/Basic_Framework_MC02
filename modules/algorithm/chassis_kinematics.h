/**
 ******************************************************************************
 * @file    chassis_kinematics.h
 * @brief   麦克纳姆轮/全向轮底盘运动学解算接口
 ******************************************************************************
 */
#ifndef CHASSIS_KINEMATICS_H
#define CHASSIS_KINEMATICS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CHASSIS_KINEMATICS_MAX_WHEEL_NUM 8U

typedef enum
{
    CHASSIS_KINEMATICS_OK = 0,
    CHASSIS_KINEMATICS_ERROR_NULL_PTR,
    CHASSIS_KINEMATICS_ERROR_INVALID_PARAM,
    CHASSIS_KINEMATICS_ERROR_SINGULAR,
} ChassisKinematicsStatus_e;

typedef struct
{
    float vx; // 底盘前进方向速度,x轴向前为正
    float vy; // 底盘横移方向速度,y轴向右为正
    float wz; // 底盘角速度,从上往下看逆时针为正
} ChassisVelocity_s;

typedef struct
{
    float x;         // 轮子相对底盘中心的x坐标
    float y;         // 轮子相对底盘中心的y坐标
    float drive_dir_x; // 轮子主动滚动方向在底盘x轴上的分量
    float drive_dir_y; // 轮子主动滚动方向在底盘y轴上的分量
} OmniWheelConfig_s;

typedef struct
{
    const OmniWheelConfig_s *wheel_config;
    uint8_t wheel_num;
    float wheel_radius; // 轮半径。>0时输出轮角速度,<=0时输出轮接地点线速度
} OmniKinematicsConfig_s;

typedef struct
{
    uint8_t wheel_num;
    uint8_t inverse_valid;
    float wheel_radius;
    float wheel_radius_inv;
    float drive_x[CHASSIS_KINEMATICS_MAX_WHEEL_NUM];
    float drive_y[CHASSIS_KINEMATICS_MAX_WHEEL_NUM];
    float rotate_coeff[CHASSIS_KINEMATICS_MAX_WHEEL_NUM];
    float inverse_matrix[3][3];
} OmniKinematicsFastConfig_s;

typedef struct
{
    float half_wheel_base;  // 前后轮中心到几何中心的距离
    float half_track_width; // 左右轮中心到几何中心的距离
    float wheel_radius;     // 轮半径。>0时输出轮角速度,<=0时输出轮接地点线速度
} MecanumKinematicsConfig_s;

typedef struct
{
    float rotate_scale;
    float rotate_scale_inv;
    float wheel_radius;
    float wheel_radius_inv;
} MecanumKinematicsFastConfig_s;

typedef struct
{
    float lf;
    float rf;
    float lb;
    float rb;
} MecanumWheelSpeed_s;

/**
 * @brief 预计算通用全向轮固定几何参数,把归一化和正解矩阵求逆移到初始化阶段
 */
ChassisKinematicsStatus_e OmniKinematicsPrepareConfig(const OmniKinematicsConfig_s *config,
                                                      OmniKinematicsFastConfig_s *fast_config);

/**
 * @brief 通用全向轮逆运动学: 底盘速度 -> 各轮速度
 */
ChassisKinematicsStatus_e OmniKinematicsCalculateWheelSpeed(const OmniKinematicsFastConfig_s *fast_config,
                                                            const ChassisVelocity_s *chassis_speed,
                                                            float *wheel_speed);

/**
 * @brief 通用全向轮正运动学: 各轮速度 -> 底盘速度
 */
ChassisKinematicsStatus_e OmniKinematicsEstimateChassisSpeed(const OmniKinematicsFastConfig_s *fast_config,
                                                             const float *wheel_speed,
                                                             ChassisVelocity_s *chassis_speed);

/**
 * @brief 预计算麦克纳姆轮固定参数,适合高频控制任务反复调用
 */
ChassisKinematicsStatus_e MecanumKinematicsPrepareConfig(const MecanumKinematicsConfig_s *config,
                                                         MecanumKinematicsFastConfig_s *fast_config);

/**
 * @brief 标准X型麦克纳姆轮逆运动学: 底盘速度 -> 四轮速度
 */
ChassisKinematicsStatus_e MecanumKinematicsCalculateWheelSpeed(const MecanumKinematicsFastConfig_s *fast_config,
                                                               const ChassisVelocity_s *chassis_speed,
                                                               MecanumWheelSpeed_s *wheel_speed);

/**
 * @brief 标准X型麦克纳姆轮正运动学: 四轮速度 -> 底盘速度
 */
ChassisKinematicsStatus_e MecanumKinematicsEstimateChassisSpeed(const MecanumKinematicsFastConfig_s *fast_config,
                                                                const MecanumWheelSpeed_s *wheel_speed,
                                                                ChassisVelocity_s *chassis_speed);

#ifdef __cplusplus
}
#endif

#endif // CHASSIS_KINEMATICS_H
