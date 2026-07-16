/**
 * @file dmmotor.h
 * @brief 达妙(DM)电机模块 —— 支持传统模式(MIT/POS_VEL/VEL/EMIT)和一拖四模式
 * @author Codex
 * @date 2025-06-03
 *
 * @note 电机报文格式来源于达妙官方手册及 robowalker 开源代码交叉验证。
 *       传统模式下每个电机独占一条 CAN 帧,一拖四模式下 4 个电机共享一条 CAN 帧。
 */

#ifndef DMMOTOR_H
#define DMMOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "bsp_can.h"
#include "motor_def.h"
#include "daemon.h"

/* ========================================================================
 * 静态池大小
 * ======================================================================== */
#define DM_MOTOR_CNT 4              /* DM 电机最大实例数 */

/* ========================================================================
 * 控制模式 —— CAN Tx ID 偏移量
 * ======================================================================== */
#define DM_MODE_MIT      0x000      /* MIT 控制 (ID + 0x000) */
#define DM_MODE_POS_VEL  0x100      /* 位置速度控制 (ID + 0x100) */
#define DM_MODE_VEL      0x200      /* 速度控制 (ID + 0x200) */
#define DM_MODE_EMIT     0x300      /* EMIT 控制 (ID + 0x300) */

/* ========================================================================
 * 一拖四模式 —— CAN 广播帧 ID (固定,不按电机变化)
 * ======================================================================== */
#define DM_1TO4_ID_LO  0x3FE        /* 电机 0x301~0x304 的广播帧 ID */
#define DM_1TO4_ID_HI  0x4FE        /* 电机 0x305~0x308 的广播帧 ID */

/* ========================================================================
 * 控制模式枚举
 * ======================================================================== */
typedef enum
{
    DM_MODE_MIT_ENUM = 0x000, /* MIT: 位置+速度+KpKd+扭矩 */
    DM_MODE_POS_VEL_ENUM = 0x100, /* 位置+速度 (float 直发,电调内部闭环) */
    DM_MODE_VEL_ENUM = 0x200, /* 纯速度 (float 直发) */
    DM_MODE_EMIT_ENUM = 0x300, /* EMIT: 位置+速度限幅+电流限幅 */
    DM_MODE_1TO4_ENUM, /* 一拖四模式 */
} DMMotorMode;

/* ========================================================================
 * 映射范围 (PMAX/VMAX/TMAX) 与最大电流 Imax 必须在注册实例时显式传入，
 * 并与电机调试助手中实际写入的参数保持一致。不同型号的 Imax 差异较大，
 * 因此模块不提供跨型号默认值，避免错误标幺化导致限流失效。
 * ======================================================================== */

/* 一拖四广播帧每槽位电流输出量满量程 (协议: ±16384 对应电机最大电流 Imax) */
#define DM_1TO4_OUT_FULL_SCALE  16384.0f
/* 一拖四编码器一圈刻度 (协议: 反馈位置范围 0~8191,对应一圈) */
#define DM_1TO4_ENCODER_PER_ROUND_DEFAULT  8192U

/* ========================================================================
 * MIT 模式 Kp/Kd 范围 (硬件固定,不可修改)
 * ======================================================================== */
#define DM_KP_MIN  0.0f             /* Kp 下限 N·m/rad */
#define DM_KP_MAX  500.0f           /* Kp 上限 N·m/rad */
#define DM_KD_MIN  0.0f             /* Kd 下限 N·m/(rad/s) */
#define DM_KD_MAX  5.0f             /* Kd 上限 N·m/(rad/s) */

/* ========================================================================
 * 电机错误状态码 (传统模式 status 域,低 4 位 + 高 4 位掩码)
 * ======================================================================== */
typedef enum
{
    DM_ERROR_NONE = 0, /* 无错误 */
    DM_ERROR_DISABLE, /* 电机失能 */
    DM_ERROR_ENCODER_UNCALIB, /* 编码器未校准 (官方状态码2) */
    DM_ERROR_OVERVOLTAGE, /* 过压 */
    DM_ERROR_UNDERVOLTAGE, /* 欠压 */
    DM_ERROR_OVERCURRENT, /* 过流 */
    DM_ERROR_MOS_OVERTEMP, /* MOS 过温 */
    DM_ERROR_ROTOR_OVERTEMP, /* 转子过温 */
    DM_ERROR_LOST_CONN, /* 断联 */
    DM_ERROR_MOS_OVERLOAD, /* MOS 过载 */
    DM_ERROR_OFFLINE, /* daemon 判断离线 */
    DM_ERROR_UNKNOWN, /* 未知状态码 */
} DMMotorError;

/* ========================================================================
 * 命令码 (前 7 字节 = 0xFF,第 8 字节 = 命令码)
 * ======================================================================== */
typedef enum
{
    DM_CMD_CLEAR_ERROR = 0xFB, /* 清除错误 */
    DM_CMD_MOTOR_MODE = 0xFC, /* 使能,进入控制模式 */
    DM_CMD_RESET_MODE = 0xFD, /* 停止电机 */
    DM_CMD_ZERO_POSITION = 0xFE, /* 将当前位置设为编码器零位 */
} DMMotorCmd;

/* ========================================================================
 * 传统模式反馈 (MIT/POS_VEL/VEL/EMIT 的反馈报文格式完全一致)
 * ======================================================================== */
typedef struct
{
    float position; /* rad,多圈累加角度 */
    float velocity; /* rad/s */
    float torque; /* N·m */
    float temp_mos; /* ℃,MOS 管温度 */
    float temp_rotor; /* ℃,转子温度 */
    DMMotorError error; /* 解析后的错误枚举 */
    uint8_t raw_status; /* 原始状态字节 (用于自定义错误判断) */

    /* ── 内部使用,上层不应直接读写 ── */
    uint16_t last_encoder; /* 上一帧编码器值 (用于跨圈检测) */
    int32_t total_rounds; /* 累计圈数 (相对初始化时的零位) */
    uint8_t feedback_initialized; /* 是否已经接收过有效首帧，首帧不参与跨圈判断 */
} DMMotorFeedback;

/* ========================================================================
 * 一拖四模式反馈
 * ======================================================================== */
typedef struct
{
    float position; /* rad,多圈累加角度 */
    float velocity; /* rad/s */
    float current; /* A */
    float temp_mos; /* ℃ */
    float temp_rotor; /* ℃ */
    DMMotorError error;

    /* ── 内部使用 ── */
    uint16_t last_encoder;
    int32_t total_rounds;
    uint16_t encoder_per_round; /* 编码器线数 (用于角度换算) */
    uint8_t feedback_initialized; /* 是否已经接收过有效首帧，首帧不参与跨圈判断 */
} DMMotorFeedback1To4;

/* ========================================================================
 * 映射范围 (PMAX/VMAX/TMAX 对应的 ±范围, 必须与电机内部参数一致)
 * 既作初始化入参, 也作运行时状态的一部分。
 * ======================================================================== */
typedef struct
{
    float p_min, p_max;
    float v_min, v_max;
    float t_min, t_max;
} DMMotorRange_t;

/* ========================================================================
 * 各控制模式参数结构体 (配置入参与运行时状态合并为同一类型)
 *
 * 上层初始化时构造该结构体并填写配置字段(range / 初始目标值等),
 * 运行时字段(feedback)保持 0 即可; 模块内部直接 zmalloc 一份并整体拷贝复用,
 * 不再做逐字段搬运, 也不再有独立的 *Config_s 类型。
 * ======================================================================== */

/* 传统模式公共参数: 映射范围 + 反馈解析缓存 */
typedef struct
{
    DMMotorRange_t range; /* [配置] 映射范围 */
    DMMotorFeedback feedback; /* [运行时] 反馈解析缓存, 初始化时置 0 */
} DMMotorNormalArgs;

typedef struct
{
    DMMotorNormalArgs normal; /* [配置 range + 运行时 feedback] */
    float position; /* [配置] 初始目标位置 rad */
    float velocity; /* [配置] 初始目标速度 rad/s */
    float kp; /* [配置] 初始 Kp */
    float kd; /* [配置] 初始 Kd */
    float torque_ff; /* [配置] 初始前馈扭矩 N·m */
} DMMotorMitArgs;

typedef struct
{
    DMMotorNormalArgs normal;
    float position; /* [配置] 初始目标位置 rad */
    float velocity_limit; /* [配置] 速度限幅 rad/s */
} DMMotorPosVelArgs;

typedef struct
{
    DMMotorNormalArgs normal;
    float velocity; /* [配置] 初始目标速度 rad/s */
} DMMotorVelArgs;

typedef struct
{
    DMMotorNormalArgs normal;
    float position; /* [配置] 初始目标位置 rad */
    float vel_limit; /* [配置] 速度限幅 rad/s */
    float cur_limit; /* [配置] 电流限幅 A */
    float imax; /* [配置] 最大电流 A，必须与电机上电打印/调试助手参数一致且大于 0 */
} DMMotorEmitArgs;

typedef struct
{
    /* ── 配置字段 (上层填写) ── */
    uint32_t can_rx_id; /* 反馈 CAN ID (0x301~0x308) */
    uint8_t slot_index; /* 广播帧槽位 (0~3) */
    uint32_t tx_id_1to4; /* 广播帧 CAN ID: DM_1TO4_ID_LO / DM_1TO4_ID_HI */
    uint16_t encoder_per_round_cfg; /* 编码器线数 (0 则用默认 8192) */
    float current_max; /* 最大电流 A (电流限幅) */
    float current_to_out_cfg; /* 电流(A)→广播帧 int16 系数 (0 则用 16384/current_max) */
    float target; /* 初始目标电流 A */
    /* ── 运行时字段 (模块内部维护, 初始化时置 0) ── */
    DMMotorFeedback1To4 feedback;
    float current_to_out; /* 实际生效的电流→输出系数 */
    CANInstance* sender_can;
} DMMotor1To4Args;

/* ========================================================================
 * 电机实例公共状态 (核心句柄)
 * ======================================================================== */
typedef struct DMMotorInstance
{
    Motor_Working_Type_e stop_flag; /* 启停标志 */
    DMMotorMode mode; /* 控制模式 flag,用于分流 args */
    CANInstance* motor_can_instance; /* CAN 通信实例 */
    DaemonInstance* motor_daemon; /* 守护线程实例 (在线监测) */
    void* args; /* 指向当前模式的参数结构体 */
    /* 运行期待发送命令码 (0=无; 否则为 DMMotorCmd: FB/FC/FD/FE)。
     * 运行期命令接口只置此标志,由 DMMotorControl 在自身上下文填充并发送命令帧,
     * 与控制帧分时复用同一 tx_buff,从根本上消除跨任务竞态。 */
    volatile uint8_t pending_cmd;
} DMMotorInstance;

/* ========================================================================
 * 公开接口
 * ======================================================================== */

/* 各 init 接口的 *_args 入参既是配置也是运行时状态容器:
 * 上层填写配置字段(range/初值/Imax 等), 运行时字段(feedback)留 0;
 * 模块内部 zmalloc 一份并整体拷贝, 不再逐字段搬运。 */
DMMotorInstance* DMMotorInitMIT(Motor_Init_Config_s* config,
                                const DMMotorMitArgs* mit_args);


DMMotorInstance* DMMotorInitPosVel(Motor_Init_Config_s* config,
                                   const DMMotorPosVelArgs* posvel_args);


DMMotorInstance* DMMotorInitVel(Motor_Init_Config_s* config,
                                const DMMotorVelArgs* vel_args);


DMMotorInstance* DMMotorInitEmit(Motor_Init_Config_s* config,
                                 const DMMotorEmitArgs* emit_args);


DMMotorInstance* DMMotorInit1To4(Motor_Init_Config_s* config,
                                 const DMMotor1To4Args* args_1to4);


void DMMotorMITSetCommand(DMMotorInstance* motor,
                          float position,
                          float velocity,
                          float kp,
                          float kd,
                          float torque_ff);


void DMMotorPosVelSetCommand(DMMotorInstance* motor,
                             float position,
                             float velocity_limit);


void DMMotorVelSetCommand(DMMotorInstance* motor,
                          float velocity);


void DMMotorEmitSetCommand(DMMotorInstance* motor,
                           float position,
                           float vel_limit,
                           float cur_limit);


void DMMotor1To4SetCurrent(DMMotorInstance* motor,
                           float current);


/**
 * @brief 停止一个 DM 电机实例
 *
 * @note 传统模式发送 0xFD 失能命令；一拖四模式仅清零目标电流，不发送传统特殊命令。
 *
 * @param motor 电机实例
 */
void DMMotorStop(DMMotorInstance* motor);


/**
 * @brief 使能一个 DM 电机实例
 *
 * @note 传统模式发送 0xFC；一拖四模式只解除模块侧停止标志。注册完成后不会自动使能。
 *
 * @param motor 电机实例
 */
void DMMotorEnable(DMMotorInstance* motor);


/**
 * @brief 将传统模式电机当前位置保存为零点
 *
 * @note 一拖四 PDF 未定义保存零点命令，一拖四实例调用该接口只记录警告。
 *
 * @param motor 电机实例
 */
void DMMotorSetZero(DMMotorInstance* motor);


/**
 * @brief 清除传统模式电机错误
 *
 * @note 一拖四 PDF 未定义清错命令，一拖四实例调用该接口只记录警告。
 *
 * @param motor 电机实例
 */
void DMMotorClearError(DMMotorInstance* motor);


DMMotorFeedback* DMMotorGetNormalFeedback(DMMotorInstance* motor);


DMMotorFeedback1To4* DMMotorGet1To4Feedback(DMMotorInstance* motor);


DMMotorError DMMotorGetError(DMMotorInstance* motor);


bool DMMotorIsOnline(DMMotorInstance* motor);


void DMMotorControl(void);

#endif /* DMMOTOR_H */
