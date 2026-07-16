/**
 * @file step_motor.h
 * @brief 步进电机控制模块，基于定时器 PWM 脉冲 + 方向引脚控制。
 *
 * 采用常规 42 步进电机 + 外部驱动器（A4988 / TMC2209 / DM542 等）的控制方式：
 * - PWM 脉冲频率控制电机转速（步进电机每收到一个脉冲走一步/微步）。
 * - DIR 引脚控制方向。
 * - EN 引脚可选，用于使能/失能驱动器。
 * - 支持速度模式、位置模式（梯形加减速 + 预计算减速阈值）。
 * - 软件累加位置，支持相对移动和绝对位置控制。
 *
 * 调用约束：
 * - 同一 StepMotorInstance 的所有接口应在同一任务/中断上下文中调用，
 *   本模块内部不做任务间互斥保护。
 * - StepMotorUpdate() 需要在高频周期中调用（如 1kHz 定时器任务），
 *   频率越高速度曲线越平滑。
 *
 * @note 典型 42 步进电机参数参考（以 1.8° 电机 + 16 微步驱动器为例）：
 *       - steps_per_rev = 200 * 16 = 3200
 *       - max_speed_rps  = 5.0f  (300 RPM)
 *       - accel_rps2     = 25.0f (0→300 RPM 约 0.2s)
 */

#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H

#include <stdint.h>
#include "bsp_pwm.h"
#include "bsp_gpio.h"

/* ========================== 常量定义 ========================== */

/** @brief 最大步进电机实例数。 */
#define STEP_MOTOR_MAX_CNT 4

/**
 * @brief 用于标识"无使能引脚"的特殊值。
 *        在 Init 配置中将 en_gpio_config.GPIO_Pin 设为此值，
 *        模块将跳过 EN 引脚注册。
 */
#define STEP_MOTOR_NO_EN_PIN ((uint32_t)0xff)

/* ========================== 类型定义 ========================== */

/** @brief 步进电机方向 */
typedef enum
{
    STEP_MOTOR_DIR_CW = 0, // 顺时针 / 正向
    STEP_MOTOR_DIR_CCW = 1, // 逆时针 / 反向
} StepMotor_Dir_e;

/** @brief 步进电机控制模式 */
typedef enum
{
    STEP_MOTOR_MODE_STOP = 0, // 停止（无脉冲输出，位置保持）
    STEP_MOTOR_MODE_SPEED, // 速度模式：持续按设定速度旋转
    STEP_MOTOR_MODE_POSITION, // 位置模式：以目标速度移动到目标位置，含减速段
} StepMotor_Mode_e;

/** @brief 步进电机运行状态 */
typedef enum
{
    STEP_MOTOR_STATE_IDLE = 0, // 空闲（无脉冲输出）
    STEP_MOTOR_STATE_RUNNING, // 运行中（有脉冲输出）
} StepMotor_State_e;

/** @brief 步进电机初始化配置 */
typedef struct
{
    PWM_Init_Config_s pulse_pwm_config; // 脉冲 PWM 配置（用于产生步进脉冲，50% 占空比）
    GPIO_Init_Config_s dir_gpio_config; // 方向引脚 GPIO 配置
    GPIO_Init_Config_s en_gpio_config; // 使能引脚 GPIO 配置
    // 如不需要 EN 引脚，将 GPIO_Pin 设为 STEP_MOTOR_NO_EN_PIN
    uint32_t steps_per_rev; // 电机每转步数（含微步细分）
    // 典型值：42 步进 1.8° + 16 微步 = 200 * 16 = 3200
    float max_speed_rps; // 最大转速，单位 转/秒（用于限速）
    float accel_rps2; // 加速度，单位 转/秒平方（用于梯形速度曲线）
} StepMotor_Init_Config_s;

/** @brief 步进电机实例 */
typedef struct
{
    /* -- BSP 资源 -- */
    PWMInstance* pulse_pwm; // 脉冲 PWM 实例
    GPIOInstance* dir_gpio; // 方向引脚 GPIO 实例
    GPIOInstance* en_gpio; // 使能引脚 GPIO 实例（可为 NULL，表示无 EN 引脚）

    /* -- 电机参数 -- */
    uint32_t steps_per_rev; // 每转步数
    float max_speed_rps; // 最大转速（转/秒）
    float accel_rps2; // 加速度（转/秒平方）

    /* -- 控制量 -- */
    StepMotor_Mode_e mode; // 当前控制模式
    StepMotor_Dir_e dir; // 当前方向
    float target_speed_rps; // 目标转速（转/秒），速度模式下设定值
    float current_speed_rps; // 当前实际转速（转/秒），加减速过程中的瞬时值

    /* -- 位置模式减速 -- */
    int32_t decel_threshold; // 减速距离阈值（步数），由 StepMotorMoveAbs() 预计算
    // 当 |target_step - current_step| ≤ decel_threshold 时进入减速段

    /* -- 位置信息 -- */
    int32_t target_step; // 目标位置（步数），位置模式下使用
    int32_t current_step; // 当前位置（步数），软件累加。Stop 不归零，Disable 不归零
    int32_t total_step; // 总步数累加器（绝对值），用于里程统计或调试

    StepMotor_State_e state; // 运行状态
} StepMotorInstance;

/* ========================== 公共接口 ========================== */

/**
 * @brief 注册并初始化步进电机实例。
 *
 * @param config 初始化配置，包含 PWM、GPIO 和电机参数。
 * @return 成功返回实例指针，失败返回 NULL。
 */
StepMotorInstance* StepMotorInit(StepMotor_Init_Config_s* config);


/**
 * @brief 设置步进电机为速度模式，按目标转速持续旋转。
 *        调用后自动退出位置模式。
 *        当目标转速为 0 且当前速度减速到 0 后，自动切回 STOP 模式。
 *
 * @param motor 步进电机实例。
 * @param speed_rps 目标转速（转/秒），正值 CW，负值 CCW。范围受 max_speed_rps 限制。
 */
void StepMotorSetSpeed(StepMotorInstance* motor, float speed_rps);


/**
 * @brief 设置步进电机为位置模式，以目标转速移动到指定相对位置。
 *        含自动减速段，不会冲过目标。
 *
 * @param motor 步进电机实例。
 * @param steps 相对步数，正值 CW，负值 CCW。
 * @param speed_rps 移动速度（转/秒），自动取绝对值。受 max_speed_rps 限制。
 */
void StepMotorMoveRel(StepMotorInstance* motor, int32_t steps, float speed_rps);


/**
 * @brief 设置步进电机为位置模式，以目标转速移动到指定绝对位置。
 *        含预计算减速阈值，不会冲过目标。
 *
 * @param motor 步进电机实例。
 * @param step 绝对步数目标值。
 * @param speed_rps 移动速度（转/秒），自动取绝对值。受 max_speed_rps 限制。
 */
void StepMotorMoveAbs(StepMotorInstance* motor, int32_t step, float speed_rps);


/**
 * @brief 步进电机紧急停止。立即停止脉冲输出。
 *        @note 当前位置 current_step 不会被清零，下次位置模式可从此位置继续。
 *        @note 如需完全复位位置，请在调用 Stop 后手动清零位置。
 *
 * @param motor 步进电机实例。
 */
void StepMotorStop(StepMotorInstance* motor);


/**
 * @brief 步进电机失能（释放力矩，驱动器 EN 引脚拉高）。
 *        同时停止脉冲输出。
 *        @note 失能后重新使能不会自动恢复之前的控制模式，需调用者重新发送控制命令。
 *
 * @param motor 步进电机实例。
 */
void StepMotorDisable(StepMotorInstance* motor);


/**
 * @brief 步进电机使能（上锁力矩，驱动器 EN 引脚拉低）。
 *        @note 仅操作 EN 引脚，不恢复之前的控制模式。调用者需重新发送控制命令。
 *
 * @param motor 步进电机实例。
 */
void StepMotorEnable(StepMotorInstance* motor);


/**
 * @brief 步进电机周期更新函数。应在 FreeRTOS 高频任务中周期调用。
 *        负责速度梯形曲线、位置检查与减速距离判断、脉冲频率实时更新。
 *
 * @param motor 步进电机实例。
 * @param dt_s 距离上次调用的时间（秒），用于速度积分和位置累加。
 */
void StepMotorUpdate(StepMotorInstance* motor, float dt_s);

/**
 * @brief 更新所有已注册步进电机的一次控制周期。
 *
 * @param dt_s 距离上次调用的时间（秒）。
 */
void StepMotorControl(float dt_s);

/**
 * @brief 获取已注册步进电机数量。
 */
uint8_t StepMotorGetCount(void);


/**
 * @brief 获取步进电机当前位置。
 *
 * @param motor 步进电机实例。
 * @return int32_t 当前位置（步数）。
 */
int32_t StepMotorGetPosition(const StepMotorInstance* motor);


/**
 * @brief 获取步进电机当前运行状态。
 *
 * @param motor 步进电机实例。
 * @return StepMotor_State_e 当前状态。
 */
StepMotor_State_e StepMotorGetState(const StepMotorInstance* motor);

#endif // STEP_MOTOR_H
