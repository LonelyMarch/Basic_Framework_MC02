/**
 * @file dmmotor.c
 * @brief 达妙(DM)电机模块实现
 * @author Codex
 * @date 2025-06-03
 *
 * @details 支持达妙电机传统模式 (MIT/POS_VEL/VEL/EMIT) 和一拖四直接电流模式。
 *          架构:
 *          - DMMotorInstance 与各模式 args 均在初始化期按需 zmalloc 分配, 不用静态池
 *          - DMMotorInstance 仅保存公共状态 + mode flag + args 指针
 *          - 每种控制模式拥有独立 args 结构体
 *          - CAN 中断只收报文 → CANProcessTask 中触发解码回调 → daemon 在线监测
 */

#include "dmmotor.h"
#include "cmsis_os.h"
#include "math.h"
#include "string.h"
#include "daemon.h"
#include "bsp_log.h"
#include "bsp_dwt.h"
#include "user_lib.h"

/* ========================================================================
 * 存储
 * 实例与各模式 args 都在初始化期用 zmalloc 按需分配, 不预留静态池。
 * 仅保留轻量的指针/句柄追踪数组, 供 DMMotorControl 遍历执行周期控制。
 * 实例总数受 DM_MOTOR_CNT 上限约束。
 * ======================================================================== */
static DMMotorInstance* dm_motor_instances[DM_MOTOR_CNT]; /* 实例指针追踪数组 */
static uint8_t dm_instance_count; /* 已注册实例数 */

/*
 * 一拖四广播帧集中发送跟踪。
 * 每路 FDCAN 最多存在 0x3FE、0x4FE 两个广播组；分组键必须同时包含
 * FDCAN 句柄和发送 ID，防止不同 CAN 总线上的同名广播帧错误共用发送缓存。
 */
#define DM_1TO4_SENDER_MAX (DEVICE_CAN_CNT * 2U)
static CANInstance* dm_1to4_senders[DM_1TO4_SENDER_MAX];
static uint8_t dm_1to4_sender_cnt;

/* 预置命令帧: 前 7 字节全 0xFF,第 8 字节为命令码 */
static const uint8_t dm_cmd_template[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};

/* ========================================================================
 * 内部辅助函数
 * ======================================================================== */

static void DM_LostCallback(void* motor_ptr);


/**
 * @brief 判断一个浮点映射区间是否可用于达妙协议的线性量化
 *
 * @param min_value 区间下限
 * @param max_value 区间上限
 * @return bool 两端均为有限数且上限严格大于下限时返回 true
 */
static bool DM_IsValidRangePair(float min_value, float max_value)
{
    return isfinite(min_value) && isfinite(max_value) && max_value > min_value;
}


/**
 * @brief 校验传统模式的 PMAX/VMAX/TMAX 映射范围
 *
 * @note 映射范围同时参与控制帧编码和反馈帧解码，任何一个区间为空或包含
 *       NaN/Inf 都会造成除零或错误比例，因此在注册 CAN 实例前直接拒绝。
 *
 * @param normal 传统模式公共参数
 * @return HAL_StatusTypeDef 合法返回 HAL_OK，否则返回 HAL_ERROR
 */
static HAL_StatusTypeDef DM_ValidateNormalRange(const DMMotorNormalArgs* normal)
{
    if (normal == NULL ||
        !DM_IsValidRangePair(normal->range.p_min, normal->range.p_max) ||
        !DM_IsValidRangePair(normal->range.v_min, normal->range.v_max) ||
        !DM_IsValidRangePair(normal->range.t_min, normal->range.t_max))
    {
        LOGERROR("[dm_motor] invalid PMAX/VMAX/TMAX mapping range");
        return HAL_ERROR;
    }

    return HAL_OK;
}


/**
 * @brief 清除电机的旧控制目标
 *
 * @note 离线期间电机实际位置可能已经变化。清除旧目标并进入停止态，可避免通信
 *       恢复后重新使用失联前命令。不同模式的目标字段互不相同，因此在模块内部
 *       按注册时固定的 mode 分流处理。
 *
 * @param motor 待清除目标的电机实例
 */
static void DM_ClearTarget(DMMotorInstance* motor)
{
    if (motor == NULL || motor->args == NULL) return;

    switch (motor->mode)
    {
    case DM_MODE_MIT_ENUM:
    {
        DMMotorMitArgs* args = (DMMotorMitArgs*)motor->args;
        args->position = 0.0f;
        args->velocity = 0.0f;
        args->kp = 0.0f;
        args->kd = 0.0f;
        args->torque_ff = 0.0f;
        break;
    }

    case DM_MODE_POS_VEL_ENUM:
    {
        DMMotorPosVelArgs* args = (DMMotorPosVelArgs*)motor->args;
        args->position = 0.0f;
        args->velocity_limit = 0.0f;
        break;
    }

    case DM_MODE_VEL_ENUM:
        ((DMMotorVelArgs*)motor->args)->velocity = 0.0f;
        break;

    case DM_MODE_EMIT_ENUM:
    {
        DMMotorEmitArgs* args = (DMMotorEmitArgs*)motor->args;
        args->position = 0.0f;
        args->vel_limit = 0.0f;
        args->cur_limit = 0.0f;
        break;
    }

    case DM_MODE_1TO4_ENUM:
        ((DMMotor1To4Args*)motor->args)->target = 0.0f;
        break;
    }
}


/**
 * @brief 浮点物理量按线性映射转换为无符号定点整数 (达妙 MIT 协议编码)
 *
 * @param x      待转换的浮点值
 * @param x_min  映射区间下限
 * @param x_max  映射区间上限
 * @param bits   定点整数位宽 (位置 16 位, 速度/扭矩/Kp/Kd 12 位)
 * @return uint16_t 映射后的无符号整数
 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief 无符号定点整数按线性映射还原为浮点物理量 (float_to_uint 的逆运算)
 *
 * @param x_int  待还原的整数
 * @param x_min  映射区间下限
 * @param x_max  映射区间上限
 * @param bits   定点整数位宽
 * @return float 还原后的浮点值
 */
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief 向电机同步发送一帧命令帧 (使能/失能/设零点/清错)
 *
 * @note 命令帧格式为前 7 字节 0xFF + 第 8 字节命令码。直接写入并发送 tx_buff,
 *       仅应在初始化期或 DMMotorControl 自身上下文调用, 避免与控制帧打包竞争 tx_buff。
 *
 * @param motor 电机实例
 * @param cmd   命令码 (DMMotorCmd)
 */
static void DM_SendCommand(DMMotorInstance* motor, DMMotorCmd cmd)
{
    if (motor == NULL || motor->motor_can_instance == NULL) return;
    memcpy(motor->motor_can_instance->tx_buff, dm_cmd_template, 7); // 前 7 字节固定 0xFF
    motor->motor_can_instance->tx_buff[7] = (uint8_t)cmd; // 第 8 字节为命令码
    CANSetDLC(motor->motor_can_instance, 8);
    CANTransmit(motor->motor_can_instance, 1);
}

/**
 * @brief 分配并初始化一个电机公共实例 (实例工厂)
 *
 * @note 实例用 zmalloc 分配(已清零)。受 DM_MOTOR_CNT 总数上限约束。
 *       此处只设置公共字段, 不登记进追踪数组, 登记由 DM_CommitMotor 完成。
 *
 * @param mode 控制模式
 * @param args 指向已分配的对应模式 args 结构体
 * @return DMMotorInstance* 成功返回实例指针, 超限/参数非法/分配失败返回 NULL
 */
static DMMotorInstance* DM_AllocBaseMotor(DMMotorMode mode, void* args)
{
    if (dm_instance_count >= DM_MOTOR_CNT || args == NULL)
    {
        return NULL;
    }

    DMMotorInstance* motor = (DMMotorInstance*)zmalloc(sizeof(DMMotorInstance));
    if (motor == NULL)
    {
        return NULL;
    }
    motor->mode = mode;
    motor->args = args;
    motor->stop_flag = MOTOR_STOP; // 初始为停止态, 使能后才发控制帧
    return motor;
}

/**
 * @brief 将初始化完成的实例登记进追踪数组, 供 DMMotorControl 遍历周期控制
 *
 * @param motor 已完成 CAN/daemon 注册的电机实例
 */
static void DM_CommitMotor(DMMotorInstance* motor)
{
    if (motor == NULL || dm_instance_count >= DM_MOTOR_CNT) return;
    dm_motor_instances[dm_instance_count++] = motor;
}

/* ========================================================================
 * args 访问约定: motor->args 由对应模式 init 分配, 使用点按 motor->mode 直接强转,
 * 不再做运行期类型校验(正确使用下 mode 与 args 类型必然匹配)。
 * 传统模式(MIT/POS_VEL/VEL/EMIT) 的 args 首成员均为 DMMotorNormalArgs normal,
 * 故传统模式可直接用 (DMMotorNormalArgs *)motor->args 取公共参数(C 保证首成员零偏移)。
 * ======================================================================== */

/**
 * @brief 判断是否为传统控制模式 (MIT/POS_VEL/VEL/EMIT)
 *
 * @param mode 控制模式
 * @return bool 传统模式返回 true, 一拖四返回 false
 */
static bool DM_IsTraditionalMode(DMMotorMode mode)
{
    return (mode == DM_MODE_MIT_ENUM || mode == DM_MODE_POS_VEL_ENUM ||
        mode == DM_MODE_VEL_ENUM || mode == DM_MODE_EMIT_ENUM);
}

/**
 * @brief 为电机实例注册 CAN 通信实例与在线监测 daemon
 *
 * @note rx_id 必须为电机内部 Master_ID; 同一总线各电机 Master_ID 须唯一,
 *       否则 bsp_can 因重复 rx_id 触发 Error_Handler。
 *
 * @param motor    电机实例
 * @param config   电机初始化配置 (内部 can_init_config 会被本函数填充)
 * @param tx_id    发送 ID (传统模式为 CAN_ID+模式偏移, 一拖四为广播帧 ID)
 * @param rx_id    接收 ID (电机 Master_ID / 一拖四反馈 ID 0x301~0x308)
 * @param callback CAN 接收解码回调
 * @return HAL_StatusTypeDef CAN/daemon 注册成功返回 HAL_OK, 否则 HAL_ERROR
 */
static HAL_StatusTypeDef DM_RegisterCanAndDaemon(DMMotorInstance* motor,
                                                 Motor_Init_Config_s* config,
                                                 uint32_t tx_id,
                                                 uint32_t rx_id,
                                                 void (*callback)(CANInstance*))
{
    config->can_init_config.tx_id = tx_id;
    config->can_init_config.rx_id = rx_id;
    config->can_init_config.id = motor;
    config->can_init_config.can_module_callback = callback;

    motor->motor_can_instance = CANRegister(&config->can_init_config);
    if (motor->motor_can_instance == NULL)
    {
        LOGERROR("[dm_motor] CAN register failed");
        return HAL_ERROR;
    }

    Daemon_Init_Config_s dconf = {
        .callback = DM_LostCallback,
        .owner_id = motor,
        .reload_count = 5, /* 50ms 超时 (5 × 10ms) */
        .init_count = 50, /* 上线等待 500ms */
    };

    motor->motor_daemon = DaemonRegister(&dconf);
    if (motor->motor_daemon == NULL)
    {
        LOGERROR("[dm_motor] daemon register failed");
        return HAL_ERROR;
    }

    return HAL_OK;
}

/* ========================================================================
 * 解码回调函数
 * ======================================================================== */

/**
 * @brief 传统模式 (MIT/POS_VEL/VEL/EMIT) 反馈帧解码回调
 *
 * @note 由 CANProcessTask 在任务上下文调用。反馈帧格式四模式一致:
 *       D0=ID|ERR<<4, D1~D2=位置16位, D3~D4=速度12位, D4~D5=扭矩12位,
 *       D6=MOS温度, D7=转子温度。同时喂狗(DaemonReload)并做多圈累加。
 *
 * @param motor_can 触发回调的 CAN 实例 (其 id 指向 DMMotorInstance)
 */
static void DM_DecodeNormal(CANInstance* motor_can)
{
    uint16_t tmp;
    const uint8_t* rx = motor_can->rx_buff;
    DMMotorInstance* motor = (DMMotorInstance*)motor_can->id;
    if (motor == NULL || motor->args == NULL) return;

    /* 达妙传统反馈协议固定为 8 字节，短帧不能用于解析或喂狗。 */
    if (motor_can->rx_len != 8U)
    {
        return;
    }

    /* D0 低 4 位必须与电机基础 CAN_ID 一致，避免共享 Master_ID 时误收其它电机反馈。 */
    if ((rx[0] & 0x0FU) != (motor_can->tx_id & 0x0FU))
    {
        return;
    }

    /* 传统模式 args 首成员即 normal, 直接取(零偏移) */
    DMMotorNormalArgs* normal = (DMMotorNormalArgs*)motor->args;

    DMMotorFeedback* fb = &normal->feedback;
    int32_t delta;

    DaemonReload(motor->motor_daemon); // 喂狗: 标记本周期收到反馈

    fb->raw_status = (rx[0] >> 4) & 0x0F; // D0 高 4 位为状态/错误码

    tmp = (uint16_t)((rx[1] << 8) | rx[2]); // 16 位位置原始值
    if (fb->feedback_initialized == 0U)
    {
        /* 首帧只建立基准；不能拿清零后的 last_encoder 与任意上电位置判断跨圈。 */
        fb->last_encoder = tmp;
        fb->feedback_initialized = 1U;
    }
    else
    {
        delta = (int32_t)tmp - (int32_t)fb->last_encoder;
        if (delta < -(1 << 15))
        {
            fb->total_rounds++; // 编码器正向跨圈（原始值由大跳变到小）
        }
        else if (delta > (1 << 15))
        {
            fb->total_rounds--; // 编码器反向跨圈（原始值由小跳变到大）
        }
        fb->last_encoder = tmp;
    }

    fb->position = uint_to_float((int)tmp, normal->range.p_min, normal->range.p_max, 16)
        + (float)fb->total_rounds * (normal->range.p_max - normal->range.p_min);

    tmp = (uint16_t)((rx[3] << 4) | (rx[4] >> 4));
    fb->velocity = uint_to_float((int)tmp, normal->range.v_min, normal->range.v_max, 12);

    tmp = (uint16_t)(((rx[4] & 0x0F) << 8) | rx[5]);
    fb->torque = uint_to_float((int)tmp, normal->range.t_min, normal->range.t_max, 12);

    fb->temp_mos = (float)rx[6];
    fb->temp_rotor = (float)rx[7];

    switch (fb->raw_status)
    {
    case 0x00: fb->error = DM_ERROR_DISABLE;
        break;
    case 0x01: fb->error = DM_ERROR_NONE;
        break;
    case 0x02: fb->error = DM_ERROR_ENCODER_UNCALIB;
        break;
    case 0x08: fb->error = DM_ERROR_OVERVOLTAGE;
        break;
    case 0x09: fb->error = DM_ERROR_UNDERVOLTAGE;
        break;
    case 0x0A: fb->error = DM_ERROR_OVERCURRENT;
        break;
    case 0x0B: fb->error = DM_ERROR_MOS_OVERTEMP;
        break;
    case 0x0C: fb->error = DM_ERROR_ROTOR_OVERTEMP;
        break;
    case 0x0D: fb->error = DM_ERROR_LOST_CONN;
        break;
    case 0x0E: fb->error = DM_ERROR_MOS_OVERLOAD;
        break;
    default: fb->error = DM_ERROR_UNKNOWN;
        break;
    }
}

/**
 * @brief 一拖四模式反馈帧解码回调
 *
 * @note 反馈帧: D0~D1=编码器(大端), D2~D3=速度rpm×100(大端有符号),
 *       D4~D5=电流mA(大端有符号), D6=转子温度, D7=MOS温度。
 *       速度换算 rpm→rad/s, 电流换算 mA→A, 位置按线数累加多圈后转 rad。
 *
 * @param motor_can 触发回调的 CAN 实例 (其 id 指向 DMMotorInstance)
 */
static void DM_Decode1To4(CANInstance* motor_can)
{
    DMMotorInstance* motor = (DMMotorInstance*)motor_can->id;
    if (motor == NULL || motor->args == NULL) return;

    /* 一拖四反馈协议固定为 8 字节，短帧不能用于解析或喂狗。 */
    if (motor_can->rx_len != 8U)
    {
        return;
    }

    DMMotor1To4Args* args = (DMMotor1To4Args*)motor->args;

    DMMotorFeedback1To4* fb = &args->feedback;
    int32_t delta;
    uint16_t encoder;
    int16_t omega_raw, current_raw;
    const uint8_t* rx = motor_can->rx_buff;

    DaemonReload(motor->motor_daemon); // 喂狗

    encoder = (uint16_t)((rx[0] << 8) | rx[1]); // 编码器值(大端)
    omega_raw = (int16_t)((rx[2] << 8) | rx[3]); // 速度原始值(rpm×100)
    current_raw = (int16_t)((rx[4] << 8) | rx[5]); // 电流原始值(mA)

    if (fb->feedback_initialized == 0U)
    {
        /* 首帧只建立基准，避免初始位置位于后半圈时被误判为反向跨圈。 */
        fb->last_encoder = encoder;
        fb->feedback_initialized = 1U;
    }
    else
    {
        delta = (int32_t)encoder - (int32_t)fb->last_encoder;
        if (delta < -(int32_t)(fb->encoder_per_round / 2U))
        {
            fb->total_rounds++; // 正向跨圈
        }
        else if (delta > (int32_t)(fb->encoder_per_round / 2U))
        {
            fb->total_rounds--; // 反向跨圈
        }
        fb->last_encoder = encoder;
    }

    /* 多圈累加编码器 → 弧度 (×2π/线数) */
    fb->position = (float)((int32_t)fb->total_rounds * (int32_t)fb->encoder_per_round + (int32_t)encoder)
        / (float)fb->encoder_per_round * 6.283185307f;
    fb->velocity = (float)omega_raw / 100.0f * 0.104719755f; // rpm×100 → rad/s (×2π/60/100)
    fb->current = (float)current_raw / 1000.0f; // mA → A
    fb->temp_rotor = (float)rx[6];
    fb->temp_mos = (float)rx[7];
    fb->error = DM_ERROR_NONE;
}

/**
 * @brief 电机离线回调 (daemon 超时未喂狗时触发)
 *
 * @note 离线后立即进入停止态并清除旧目标，不自动发送重连命令。电机实际位置在
 *       失联期间不可知，只有上层重新设置目标并显式调用 DMMotorEnable() 才能恢复。
 *
 * @param motor_ptr 指向 DMMotorInstance 的指针 (daemon owner_id)
 */
static void DM_LostCallback(void* motor_ptr)
{
    DMMotorInstance* motor = (DMMotorInstance*)motor_ptr;
    if (motor == NULL || motor->motor_can_instance == NULL || motor->args == NULL) return;

    LOGWARNING("[dm_motor] motor lost, tx_id=0x%lX", motor->motor_can_instance->tx_id);
    motor->stop_flag = MOTOR_STOP; // 离线后禁止继续使用失联前的控制目标
    motor->pending_cmd = 0U; // 丢弃尚未提交的普通命令，防止反馈恢复后意外执行
    DM_ClearTarget(motor);

    /* 传统模式 args 首成员即 normal.feedback；一拖四拥有独立反馈结构。 */
    if (motor->mode == DM_MODE_1TO4_ENUM)
    {
        DMMotorFeedback1To4* feedback = &((DMMotor1To4Args*)motor->args)->feedback;
        feedback->error = DM_ERROR_OFFLINE;
        feedback->feedback_initialized = 0U; // 恢复后的第一帧重新建立跨圈基准
    }
    else
    {
        DMMotorFeedback* feedback = &((DMMotorNormalArgs*)motor->args)->feedback;
        feedback->error = DM_ERROR_OFFLINE;
        feedback->feedback_initialized = 0U; // 恢复后的第一帧重新建立跨圈基准
    }
}

/* ========================================================================
 * 打包函数: 物理量 → CAN 帧 buffer
 * ======================================================================== */

/**
 * @brief 打包 MIT 模式控制帧到 tx_buff
 *
 * @note 帧格式: pos16 + vel12 + kp12 + kd12 + tff12, 共 8 字节。
 *       打包前对各物理量按映射范围限幅, 再转定点整数。
 *
 * @param motor MIT 模式电机实例
 */
static void DM_PackMIT(DMMotorInstance* motor)
{
    DMMotorMitArgs* args = (DMMotorMitArgs*)motor->args;

    CANInstance* can = motor->motor_can_instance;
    DMMotorNormalArgs* normal = &args->normal;
    uint16_t p, v, kp, kd, t;

    LIMIT_MIN_MAX(args->position, normal->range.p_min, normal->range.p_max);
    LIMIT_MIN_MAX(args->velocity, normal->range.v_min, normal->range.v_max);
    LIMIT_MIN_MAX(args->kp, DM_KP_MIN, DM_KP_MAX);
    LIMIT_MIN_MAX(args->kd, DM_KD_MIN, DM_KD_MAX);
    LIMIT_MIN_MAX(args->torque_ff, normal->range.t_min, normal->range.t_max);

    p = float_to_uint(args->position, normal->range.p_min, normal->range.p_max, 16);
    v = float_to_uint(args->velocity, normal->range.v_min, normal->range.v_max, 12);
    kp = float_to_uint(args->kp, DM_KP_MIN, DM_KP_MAX, 12);
    kd = float_to_uint(args->kd, DM_KD_MIN, DM_KD_MAX, 12);
    t = float_to_uint(args->torque_ff, normal->range.t_min, normal->range.t_max, 12);

    can->tx_buff[0] = (uint8_t)(p >> 8);
    can->tx_buff[1] = (uint8_t)(p & 0xFF);
    can->tx_buff[2] = (uint8_t)(v >> 4);
    can->tx_buff[3] = (uint8_t)(((v & 0x0F) << 4) | (kp >> 8));
    can->tx_buff[4] = (uint8_t)(kp & 0xFF);
    can->tx_buff[5] = (uint8_t)(kd >> 4);
    can->tx_buff[6] = (uint8_t)(((kd & 0x0F) << 4) | (t >> 8));
    can->tx_buff[7] = (uint8_t)(t & 0xFF);
}

/**
 * @brief 打包位置速度模式控制帧到 tx_buff
 *
 * @note 帧格式: float 位置(4字节) + float 速度限幅(4字节), 小端直发。
 *
 * @param motor 位置速度模式电机实例
 */
static void DM_PackPosVel(DMMotorInstance* motor)
{
    DMMotorPosVelArgs* args = (DMMotorPosVelArgs*)motor->args;

    CANInstance* can = motor->motor_can_instance;
    DMMotorNormalArgs* normal = &args->normal;

    LIMIT_MIN_MAX(args->position, normal->range.p_min, normal->range.p_max);
    LIMIT_MIN_MAX(args->velocity_limit, 0.0f, normal->range.v_max);

    memcpy(&can->tx_buff[0], &args->position, 4);
    memcpy(&can->tx_buff[4], &args->velocity_limit, 4);
}

/**
 * @brief 打包速度模式控制帧到 tx_buff
 *
 * @note 帧格式: float 速度(4字节), 小端直发, DLC=4。
 *
 * @param motor 速度模式电机实例
 */
static void DM_PackVel(DMMotorInstance* motor)
{
    DMMotorVelArgs* args = (DMMotorVelArgs*)motor->args;

    CANInstance* can = motor->motor_can_instance;
    DMMotorNormalArgs* normal = &args->normal;

    LIMIT_MIN_MAX(args->velocity, normal->range.v_min, normal->range.v_max);
    memcpy(&can->tx_buff[0], &args->velocity, 4);
}

/**
 * @brief 打包 EMIT (力位混控) 模式控制帧到 tx_buff
 *
 * @note 帧格式: float 位置(4字节) + uint16 速度限幅(rad/s×100) + uint16 电流限幅
 *       (标幺值×10000, 标幺=实际电流/Imax)。上层传入实际电流 A, 此处按 Imax 标幺化。
 *
 * @param motor EMIT 模式电机实例
 */
static void DM_PackEMIT(DMMotorInstance* motor)
{
    DMMotorEmitArgs* args = (DMMotorEmitArgs*)motor->args;

    CANInstance* can = motor->motor_can_instance;
    DMMotorNormalArgs* normal = &args->normal;
    uint16_t vel16, cur16;
    float vel_scaled, cur_scaled;

    LIMIT_MIN_MAX(args->position, normal->range.p_min, normal->range.p_max);

    vel_scaled = args->vel_limit * 100.0f; // rad/s → ×100 定点
    LIMIT_MIN_MAX(vel_scaled, 0.0f, 10000.0f); // 协议上限 10000
    vel16 = (uint16_t)vel_scaled;

    /* imax 已在注册时强制校验，运行期直接使用固定配置完成标幺化。 */
    cur_scaled = args->cur_limit / args->imax * 10000.0f; // 实际电流 → 标幺值×10000
    LIMIT_MIN_MAX(cur_scaled, 0.0f, 10000.0f);
    cur16 = (uint16_t)cur_scaled;

    memcpy(&can->tx_buff[0], &args->position, 4);
    can->tx_buff[4] = (uint8_t)(vel16 & 0xFF);
    can->tx_buff[5] = (uint8_t)(vel16 >> 8);
    can->tx_buff[6] = (uint8_t)(cur16 & 0xFF);
    can->tx_buff[7] = (uint8_t)(cur16 >> 8);
}

/**
 * @brief 将一拖四电流指令写入广播帧对应槽位
 *
 * @note 电流(A)×current_to_out 后按 ±16384 限幅，并按官方 PDF 规定的
 *       “高 8 位在前、低 8 位在后”写入广播帧对应槽位。
 *       实际发送由 DMMotorControl 集中完成。
 *
 * @param motor 一拖四模式电机实例
 */
static void DM_Pack1To4(DMMotorInstance* motor)
{
    DMMotor1To4Args* args = (DMMotor1To4Args*)motor->args;
    if (args->sender_can == NULL) return;

    float target_current = 0.0f;
    float out;
    int16_t value;

    /* 一拖四广播帧只承载电流指令，角度/速度闭环由驱动器或上层完成。 */
    target_current = args->target;

    /* current_max 已在注册时强制校验，所有一拖四命令都必须经过物理电流限幅。 */
    LIMIT_MIN_MAX(target_current, -args->current_max, args->current_max);

    out = target_current * args->current_to_out; // 电流(A) → 广播帧输出量
    LIMIT_MIN_MAX(out, -DM_1TO4_OUT_FULL_SCALE, DM_1TO4_OUT_FULL_SCALE);
    value = (int16_t)out;

    uint8_t slot = args->slot_index;
    uint8_t* buf = args->sender_can->tx_buff;
    buf[slot * 2] = (uint8_t)((uint16_t)value >> 8); // PDF：每个控制电流的高 8 位在前
    buf[slot * 2 + 1] = (uint8_t)((uint16_t)value & 0xFFU); // PDF：低 8 位在后
}

/* ========================================================================
 * 初始化函数
 * ======================================================================== */

/**
 * @brief 校验传统模式的电机 CAN_ID 不超过 15
 *
 * @note 达妙反馈帧 D0 仅用低 4 位表示 CAN_ID, 超过 15 会溢出到状态位导致误报错。
 *
 * @param config 电机初始化配置 (tx_id 为电机 CAN_ID)
 * @return HAL_StatusTypeDef 合法返回 HAL_OK, 否则 HAL_ERROR
 */
static HAL_StatusTypeDef DM_CheckTraditionalCanId(Motor_Init_Config_s* config)
{
    if (config == NULL) return HAL_ERROR;
    uint32_t can_id = config->can_init_config.tx_id;
    if (can_id > 15)
    {
        LOGERROR("[dm_motor] CAN ID too large (max 15), got 0x%lX", can_id);
        return HAL_ERROR;
    }
    return HAL_OK;
}

/**
 * @brief 获取或登记一拖四广播帧 (0x3FE/0x4FE) 的集中发送 CAN 实例
 *
 * @note 同一路 FDCAN 上相同广播 ID 的多个电机共享一条 CAN 帧。不同 FDCAN 即使
 *       使用相同广播 ID，也必须拥有各自独立的发送实例和 8 字节发送缓存。
 *
 * @param tx_id_1to4 广播帧 ID (DM_1TO4_ID_LO / DM_1TO4_ID_HI)
 * @param can        当前电机的 CAN 实例 (首个登记时作为发送器)
 * @return CANInstance* 该广播帧对应的发送器实例, 超出帧数上限返回 NULL
 */
static CANInstance* DM_GetOr1To4Sender(uint32_t tx_id_1to4, CANInstance* can)
{
    if (can == NULL || can->can_handle == NULL) return NULL;

    for (uint8_t i = 0; i < dm_1to4_sender_cnt; i++)
    {
        CANInstance* sender = dm_1to4_senders[i];
        if (sender != NULL && sender->can_handle == can->can_handle && sender->tx_id == tx_id_1to4)
        {
            return sender;
        }
    }

    if (dm_1to4_sender_cnt >= DM_1TO4_SENDER_MAX)
    {
        return NULL;
    }

    dm_1to4_senders[dm_1to4_sender_cnt] = can;
    memset(can->tx_buff, 0, sizeof(can->tx_buff));
    dm_1to4_sender_cnt++;
    return can;
}


/**
 * @brief 校验一拖四实例的反馈 ID、广播槽位及总线内唯一性
 *
 * @note 官方协议固定电机 1~4 使用 0x3FE，反馈 ID 为 0x301~0x304；电机
 *       5~8 使用 0x4FE，反馈 ID 为 0x305~0x308。slot_index 必须与反馈 ID
 *       一一对应，同一 FDCAN、同一广播帧内也不允许重复占用槽位。
 *
 * @param config CAN 初始化配置，用于区分 FDCAN 总线
 * @param args 一拖四注册参数
 * @return HAL_StatusTypeDef 合法返回 HAL_OK，否则返回 HAL_ERROR
 */
static HAL_StatusTypeDef DM_Validate1To4Config(const Motor_Init_Config_s* config,
                                               const DMMotor1To4Args* args)
{
    uint32_t expected_rx_id;

    if (config == NULL || args == NULL || config->can_init_config.can_handle == NULL)
    {
        LOGERROR("[dm_motor] 1-to-4 CAN config is invalid");
        return HAL_ERROR;
    }

    if (args->slot_index > 3U)
    {
        LOGERROR("[dm_motor] 1-to-4 slot_index must be 0~3");
        return HAL_ERROR;
    }

    if (args->tx_id_1to4 == DM_1TO4_ID_LO)
    {
        expected_rx_id = 0x301U + args->slot_index;
    }
    else if (args->tx_id_1to4 == DM_1TO4_ID_HI)
    {
        expected_rx_id = 0x305U + args->slot_index;
    }
    else
    {
        LOGERROR("[dm_motor] 1-to-4 tx_id must be 0x3FE or 0x4FE, got 0x%lX",
                 args->tx_id_1to4);
        return HAL_ERROR;
    }

    if (args->can_rx_id != expected_rx_id)
    {
        LOGERROR("[dm_motor] 1-to-4 slot/rx_id mismatch, expected=0x%lX got=0x%lX",
                 expected_rx_id, args->can_rx_id);
        return HAL_ERROR;
    }

    if (!isfinite(args->current_max) || args->current_max <= 0.0f)
    {
        LOGERROR("[dm_motor] 1-to-4 current_max must be finite and greater than zero");
        return HAL_ERROR;
    }

    if (!isfinite(args->current_to_out_cfg) || args->current_to_out_cfg < 0.0f)
    {
        LOGERROR("[dm_motor] 1-to-4 current_to_out_cfg must be finite and non-negative");
        return HAL_ERROR;
    }

    for (uint8_t i = 0U; i < dm_instance_count; i++)
    {
        DMMotorInstance* registered = dm_motor_instances[i];
        if (registered == NULL || registered->mode != DM_MODE_1TO4_ENUM || registered->args == NULL ||
            registered->motor_can_instance == NULL)
        {
            continue;
        }

        DMMotor1To4Args* registered_args = (DMMotor1To4Args*)registered->args;
        if (registered->motor_can_instance->can_handle == config->can_init_config.can_handle &&
            registered_args->tx_id_1to4 == args->tx_id_1to4 &&
            registered_args->slot_index == args->slot_index)
        {
            LOGERROR("[dm_motor] duplicate 1-to-4 slot on the same CAN bus, tx_id=0x%lX slot=%u",
                     args->tx_id_1to4, (unsigned int)args->slot_index);
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

/**
 * @brief 传统模式初始化公共流程（分配实例→注册 CAN/daemon→登记）
 *
 * @note 注册完成后保持停止态，不自动使能，也不自动保存零点。使能和保存零点都会
 *       改变电机实际状态，必须由应用层在机械条件确认后显式调用对应接口。
 *
 * @param config 电机初始化配置 (tx_id=CAN_ID, rx_id=Master_ID)
 * @param mode   传统控制模式 (MIT/POS_VEL/VEL/EMIT, 其值即 CAN ID 模式偏移)
 * @param args   已分配并填好的对应模式 args
 * @return DMMotorInstance* 成功返回实例指针, 失败返回 NULL
 */
static DMMotorInstance* DM_InitTraditional(Motor_Init_Config_s* config,
                                           DMMotorMode mode,
                                           void* args)
{
    if (config == NULL || args == NULL || !DM_IsTraditionalMode(mode)) return NULL;
    if (DM_CheckTraditionalCanId(config) != HAL_OK) return NULL;
    if (DM_ValidateNormalRange((const DMMotorNormalArgs*)args) != HAL_OK) return NULL;

    DMMotorInstance* motor = DM_AllocBaseMotor(mode, args);
    if (motor == NULL)
    {
        LOGERROR("[dm_motor] instance exceeded, max=%u", (unsigned int)DM_MOTOR_CNT);
        return NULL;
    }

    uint32_t base_id = config->can_init_config.tx_id;
    if (DM_RegisterCanAndDaemon(motor, config, base_id + (uint32_t)mode,
                                config->can_init_config.rx_id, DM_DecodeNormal) != HAL_OK)
    {
        return NULL; /* 初始化失败(致命配置错误), 实例已 zmalloc, 不再 commit */
    }

    DM_CommitMotor(motor);
    LOGINFO("[dm_motor] init OK, mode=%u, tx_id=0x%lX", (unsigned int)mode, motor->motor_can_instance->tx_id);
    return motor;
}

/* 传统模式通用初始化：整体拷贝上层配置后，强制清零模块自行维护的反馈运行时字段。 */
static DMMotorInstance* DM_InitTraditionalCommon(Motor_Init_Config_s* config,
                                                 DMMotorMode mode,
                                                 const void* in_args,
                                                 size_t args_size)
{
    if (in_args == NULL || DM_ValidateNormalRange((const DMMotorNormalArgs*)in_args) != HAL_OK)
    {
        return NULL;
    }

    void* args = zmalloc(args_size);
    if (args == NULL) return NULL;
    memcpy(args, in_args, args_size);

    /* feedback 属于模块运行时状态，不接受上层入参中的残留值。 */
    memset(&((DMMotorNormalArgs*)args)->feedback, 0, sizeof(DMMotorFeedback));
    return DM_InitTraditional(config, mode, args);
}

/**
 * @brief 初始化一个 MIT 模式达妙电机
 *
 * @param config   电机初始化配置 (tx_id=CAN_ID, rx_id=Master_ID)
 * @param mit_args MIT 模式参数 (range 必填, position/velocity/kp/kd/torque_ff 为初值)
 * @return DMMotorInstance* 成功返回实例指针, 入参为空/超限/注册失败返回 NULL
 */
DMMotorInstance* DMMotorInitMIT(Motor_Init_Config_s* config, const DMMotorMitArgs* mit_args)
{
    if (config == NULL || mit_args == NULL)
    {
        LOGERROR("[dm_motor] MIT init args is null");
        return NULL;
    }
    return DM_InitTraditionalCommon(config, DM_MODE_MIT_ENUM, mit_args, sizeof(DMMotorMitArgs));
}

/**
 * @brief 初始化一个位置速度模式达妙电机
 *
 * @param config       电机初始化配置 (tx_id=CAN_ID, rx_id=Master_ID)
 * @param posvel_args  位置速度模式参数 (range 必填, position/velocity_limit 为初值)
 * @return DMMotorInstance* 成功返回实例指针, 失败返回 NULL
 */
DMMotorInstance* DMMotorInitPosVel(Motor_Init_Config_s* config, const DMMotorPosVelArgs* posvel_args)
{
    if (config == NULL || posvel_args == NULL)
    {
        LOGERROR("[dm_motor] POS_VEL init args is null");
        return NULL;
    }
    return DM_InitTraditionalCommon(config, DM_MODE_POS_VEL_ENUM, posvel_args, sizeof(DMMotorPosVelArgs));
}

/**
 * @brief 初始化一个速度模式达妙电机
 *
 * @param config   电机初始化配置 (tx_id=CAN_ID, rx_id=Master_ID)
 * @param vel_args 速度模式参数 (range 必填, velocity 为初值)
 * @return DMMotorInstance* 成功返回实例指针, 失败返回 NULL
 */
DMMotorInstance* DMMotorInitVel(Motor_Init_Config_s* config, const DMMotorVelArgs* vel_args)
{
    if (config == NULL || vel_args == NULL)
    {
        LOGERROR("[dm_motor] VEL init args is null");
        return NULL;
    }
    return DM_InitTraditionalCommon(config, DM_MODE_VEL_ENUM, vel_args, sizeof(DMMotorVelArgs));
}

/**
 * @brief 初始化一个 EMIT (力位混控) 模式达妙电机
 *
 * @note Imax 用于把实际电流换算成协议标幺值，必须与电机上电打印或调试助手中的
 *       最大电流完全一致。本模块不提供跨型号默认值。
 *
 * @param config    电机初始化配置 (tx_id=CAN_ID, rx_id=Master_ID)
 * @param emit_args EMIT 模式参数 (range 必填, position/vel_limit/cur_limit/imax)
 * @return DMMotorInstance* 成功返回实例指针, 失败返回 NULL
 */
DMMotorInstance* DMMotorInitEmit(Motor_Init_Config_s* config, const DMMotorEmitArgs* emit_args)
{
    if (config == NULL || emit_args == NULL)
    {
        LOGERROR("[dm_motor] EMIT init args is null");
        return NULL;
    }

    if (!isfinite(emit_args->imax) || emit_args->imax <= 0.0f)
    {
        LOGERROR("[dm_motor] EMIT imax must be finite and greater than zero");
        return NULL;
    }

    if (DM_ValidateNormalRange(&emit_args->normal) != HAL_OK) return NULL;

    DMMotorEmitArgs* args = (DMMotorEmitArgs*)zmalloc(sizeof(DMMotorEmitArgs));
    if (args == NULL) return NULL;
    memcpy(args, emit_args, sizeof(DMMotorEmitArgs));
    memset(&args->normal.feedback, 0, sizeof(args->normal.feedback)); // 运行时反馈始终从未初始化状态开始

    return DM_InitTraditional(config, DM_MODE_EMIT_ENUM, args);
}

/**
 * @brief 初始化一个一拖四模式达妙电机
 *
 * @note 会校验 slot_index、反馈 ID 与广播帧的固定映射，并派生编码器线数和
 *       电流到协议输出量的换算系数。注册完成后保持停止态，不发送传统模式的
 *       FC/FD/FE/FB 特殊命令，因为一拖四 PDF 只定义了广播电流控制帧。
 *
 * @param config    电机初始化配置 (rx_id 由 args_1to4->can_rx_id 提供)
 * @param args_1to4 一拖四参数（反馈 ID、槽位、广播帧 ID、最大电流等）
 * @return DMMotorInstance* 成功返回实例指针, 失败返回 NULL
 */
DMMotorInstance* DMMotorInit1To4(Motor_Init_Config_s* config, const DMMotor1To4Args* args_1to4)
{
    if (config == NULL || args_1to4 == NULL)
    {
        LOGERROR("[dm_motor] 1-to-4 init args is null");
        return NULL;
    }
    if (DM_Validate1To4Config(config, args_1to4) != HAL_OK) return NULL;

    DMMotor1To4Args* args = (DMMotor1To4Args*)zmalloc(sizeof(DMMotor1To4Args));
    if (args == NULL) return NULL;
    memcpy(args, args_1to4, sizeof(DMMotor1To4Args));
    memset(&args->feedback, 0, sizeof(args->feedback)); // 不继承调用者结构体中的运行时反馈残留
    args->sender_can = NULL; // 发送器只能由当前注册流程按 CAN 总线和广播 ID 分配

    /* 运行时派生字段: 编码器线数 / 电流→输出系数 */
    args->feedback.encoder_per_round = (args->encoder_per_round_cfg > 0)
                                           ? args->encoder_per_round_cfg
                                           : DM_1TO4_ENCODER_PER_ROUND_DEFAULT;
    if (args->current_to_out_cfg > 0.0f)
    {
        args->current_to_out = args->current_to_out_cfg;
    }
    else
    {
        /* PDF：±16384 对应电机最大电流，最大电流由注册配置显式提供。 */
        args->current_to_out = DM_1TO4_OUT_FULL_SCALE / args->current_max;
    }


    DMMotorInstance* motor = DM_AllocBaseMotor(DM_MODE_1TO4_ENUM, args);
    if (motor == NULL)
    {
        LOGERROR("[dm_motor] instance exceeded, max=%u", (unsigned int)DM_MOTOR_CNT);
        return NULL;
    }

    if (DM_RegisterCanAndDaemon(motor, config, args->tx_id_1to4,
                                args->can_rx_id, DM_Decode1To4) != HAL_OK)
    {
        return NULL; /* 初始化失败(致命配置错误), 实例已 zmalloc, 不再 commit */
    }

    args->sender_can = DM_GetOr1To4Sender(args->tx_id_1to4, motor->motor_can_instance);
    if (args->sender_can == NULL)
    {
        LOGERROR("[dm_motor] 1-to-4 sender count exceeded (max %u)",
                 (unsigned int)DM_1TO4_SENDER_MAX);
        return NULL;
    }

    DM_CommitMotor(motor);
    LOGINFO("[dm_motor] 1-to-4 init OK, slot=%u, tx_id=0x%lX",
            (unsigned int)args->slot_index, args->tx_id_1to4);
    return motor;
}

/* ========================================================================
 * 公开接口实现
 * ======================================================================== */

/**
 * @brief 设置 MIT 模式目标 (位置/速度/Kp/Kd/前馈扭矩)
 *
 * @note 仅更新目标值, 实际发送由 DMMotorControl 周期完成。Kp/Kd 在此预限幅。
 *
 * @param motor     DMMotorInitMIT() 返回的 MIT 模式电机实例
 * @param position  目标位置 rad
 * @param velocity  目标速度 rad/s
 * @param kp        位置比例系数 [0,500]
 * @param kd        速度微分系数 [0,5]
 * @param torque_ff 前馈扭矩 N·m
 */
void DMMotorMITSetCommand(DMMotorInstance* motor, float position, float velocity, float kp, float kd, float torque_ff)
{
    if (motor == NULL || motor->args == NULL) return;
    DMMotorMitArgs* args = (DMMotorMitArgs*)motor->args;
    LIMIT_MIN_MAX(kp, DM_KP_MIN, DM_KP_MAX);
    LIMIT_MIN_MAX(kd, DM_KD_MIN, DM_KD_MAX);
    args->position = position;
    args->velocity = velocity;
    args->kp = kp;
    args->kd = kd;
    args->torque_ff = torque_ff;
}

/**
 * @brief 设置位置速度模式目标
 *
 * @param motor          位置速度模式电机实例
 * @param position       目标位置 rad
 * @param velocity_limit 速度限幅 rad/s
 */
void DMMotorPosVelSetCommand(DMMotorInstance* motor, float position, float velocity_limit)
{
    if (motor == NULL || motor->args == NULL) return;
    DMMotorPosVelArgs* args = (DMMotorPosVelArgs*)motor->args;
    args->position = position;
    args->velocity_limit = velocity_limit;
}

/**
 * @brief 设置速度模式目标
 *
 * @param motor    速度模式电机实例
 * @param velocity 目标速度 rad/s
 */
void DMMotorVelSetCommand(DMMotorInstance* motor, float velocity)
{
    if (motor == NULL || motor->args == NULL) return;
    DMMotorVelArgs* args = (DMMotorVelArgs*)motor->args;
    args->velocity = velocity;
}

/**
 * @brief 设置 EMIT (力位混控) 模式目标
 *
 * @param motor     EMIT 模式电机实例
 * @param position  目标位置 rad
 * @param vel_limit 速度限幅 rad/s
 * @param cur_limit 电流限幅 A (实际电流值, 打包时按 Imax 标幺化)
 */
void DMMotorEmitSetCommand(DMMotorInstance* motor, float position, float vel_limit, float cur_limit)
{
    if (motor == NULL || motor->args == NULL) return;
    DMMotorEmitArgs* args = (DMMotorEmitArgs*)motor->args;
    args->position = position;
    args->vel_limit = vel_limit;
    args->cur_limit = cur_limit;
}

/**
 * @brief 设置一拖四广播帧的直接电流指令
 *
 * @param motor  一拖四模式电机实例
 * @param current 目标电流 A
 */
void DMMotor1To4SetCurrent(DMMotorInstance* motor, float current)
{
    if (motor == NULL || motor->args == NULL) return;
    DMMotor1To4Args* args = (DMMotor1To4Args*)motor->args;
    args->target = current;
}

/**
 * @brief 命令请求统一入口 (按运行阶段决定同步发送或异步置标志)
 *
 * @note - 初始化期(调度器未运行): 直接同步发送, 无任务争用 tx_buff, 时序确定;
 *       - 运行期(调度器已运行):   仅置 pending_cmd, 由 DMMotorControl 在自身上下文发送,
 *         命令帧与控制帧分时复用 tx_buff, 消除跨任务竞态。
 *
 * @param motor 电机实例
 * @param cmd   命令码
 */
static void DM_RequestCommand(DMMotorInstance* motor, DMMotorCmd cmd)
{
    if (motor == NULL || motor->mode == DM_MODE_1TO4_ENUM) return;
    if (osKernelGetState() != osKernelRunning)
    {
        DM_SendCommand(motor, cmd);
    }
    else
    {
        motor->pending_cmd = (uint8_t)cmd;
    }
}

/**
 * @brief 停止电机
 *
 * @note 传统模式进入停止态并发送 0xFD 失能命令；一拖四协议没有定义特殊
 *       失能命令，因此只清零该电机的广播电流目标并停止向其槽位写入非零值。
 *
 * @param motor 电机实例
 */
void DMMotorStop(DMMotorInstance* motor)
{
    if (motor == NULL) return;
    motor->stop_flag = MOTOR_STOP;

    if (motor->mode == DM_MODE_1TO4_ENUM)
    {
        if (motor->args != NULL)
        {
            ((DMMotor1To4Args*)motor->args)->target = 0.0f; // 下一周期广播零电流
        }
        return;
    }

    DM_RequestCommand(motor, DM_CMD_RESET_MODE);
}

/**
 * @brief 使能电机
 *
 * @note 传统模式发送 0xFC 进入控制模式；一拖四协议只定义周期电流广播，
 *       因此一拖四实例仅解除模块侧停止标志，不发送 FF...FC 特殊帧。
 *
 * @param motor 电机实例
 */
void DMMotorEnable(DMMotorInstance* motor)
{
    if (motor == NULL) return;
    motor->stop_flag = MOTOR_ENABLED;

    if (motor->mode == DM_MODE_1TO4_ENUM)
    {
        return;
    }

    DM_RequestCommand(motor, DM_CMD_MOTOR_MODE);
}

/**
 * @brief 将电机当前位置保存为零点 (达妙命令 0xFE)
 *
 * @note 达妙编码器出厂已标定, 上电只能设零点, 非"校准编码器"。
 *       初始化期同步发送后短延时, 确保电机收到命令。
 *
 * @param motor 电机实例
 */
void DMMotorSetZero(DMMotorInstance* motor)
{
    if (motor == NULL) return;

    if (motor->mode == DM_MODE_1TO4_ENUM)
    {
        LOGWARNING("[dm_motor] set-zero command is not defined for 1-to-4 protocol");
        return;
    }

    DM_RequestCommand(motor, DM_CMD_ZERO_POSITION);
    if (osKernelGetState() != osKernelRunning)
    {
        DWT_Delay(0.01f);
    }
}

/**
 * @brief 清除电机错误 (达妙命令 0xFB)
 * @param motor 电机实例
 */
void DMMotorClearError(DMMotorInstance* motor)
{
    if (motor == NULL) return;

    if (motor->mode == DM_MODE_1TO4_ENUM)
    {
        LOGWARNING("[dm_motor] clear-error command is not defined for 1-to-4 protocol");
        return;
    }

    DM_RequestCommand(motor, DM_CMD_CLEAR_ERROR);
}

/**
 * @brief 获取传统模式反馈数据指针
 * @param motor 传统模式电机实例
 * @return DMMotorFeedback* 反馈结构指针, 非传统模式返回 NULL
 */
DMMotorFeedback* DMMotorGetNormalFeedback(DMMotorInstance* motor)
{
    if (motor == NULL || motor->args == NULL || motor->mode == DM_MODE_1TO4_ENUM) return NULL;
    /* 传统模式 args 首成员即 normal */
    return &((DMMotorNormalArgs*)motor->args)->feedback;
}

/**
 * @brief 获取一拖四模式反馈数据指针
 * @param motor 一拖四模式电机实例
 * @return DMMotorFeedback1To4* 反馈结构指针, 非一拖四模式返回 NULL
 */
DMMotorFeedback1To4* DMMotorGet1To4Feedback(DMMotorInstance* motor)
{
    if (motor == NULL || motor->args == NULL || motor->mode != DM_MODE_1TO4_ENUM) return NULL;
    return &((DMMotor1To4Args*)motor->args)->feedback;
}

/**
 * @brief 获取电机当前错误码
 * @param motor 电机实例
 * @return DMMotorError 错误码, 非法实例返回 DM_ERROR_DISABLE
 */
DMMotorError DMMotorGetError(DMMotorInstance* motor)
{
    if (motor == NULL || motor->args == NULL) return DM_ERROR_DISABLE;
    /* 传统模式 args 首成员即 normal.feedback; 一拖四单独取 */
    if (motor->mode == DM_MODE_1TO4_ENUM)
    {
        return ((DMMotor1To4Args*)motor->args)->feedback.error;
    }
    return ((DMMotorNormalArgs*)motor->args)->feedback.error;
}

/**
 * @brief 查询电机是否在线 (基于 daemon 喂狗状态)
 * @param motor 电机实例
 * @return bool 在线返回 true
 */
bool DMMotorIsOnline(DMMotorInstance* motor)
{
    if (motor == NULL || motor->motor_daemon == NULL) return false;
    return (bool)DaemonIsOnline(motor->motor_daemon);
}

/* ========================================================================
 * 控制周期
 * ======================================================================== */

static void DMMotorUpdateInstance(DMMotorInstance* motor)
{
    CANInstance* can;

    if (motor == NULL || motor->motor_can_instance == NULL)
        return;

    can = motor->motor_can_instance;

    /* 命令帧优先, 与控制帧分时复用同一 tx_buff。 */
    if (motor->pending_cmd != 0U)
    {
        uint8_t cmd = motor->pending_cmd;
        motor->pending_cmd = 0U;
        DM_SendCommand(motor, (DMMotorCmd)cmd);
        return;
    }

    /*
     * 离线后不自动发送使能命令，也不恢复旧目标。DM_LostCallback() 已将实例置为
     * 停止态；上层必须在确认机械状态后重新设置目标并显式调用 DMMotorEnable()。
     */
    if (!DMMotorIsOnline(motor))
    {
        return;
    }

    if (motor->stop_flag == MOTOR_STOP)
        return;

    switch (motor->mode)
    {
    case DM_MODE_MIT_ENUM:
        DM_PackMIT(motor);
        CANSetDLC(can, 8);
        CANTransmit(can, 1.0f);
        break;

    case DM_MODE_POS_VEL_ENUM:
        DM_PackPosVel(motor);
        CANSetDLC(can, 8);
        CANTransmit(can, 1.0f);
        break;

    case DM_MODE_VEL_ENUM:
        DM_PackVel(motor);
        CANSetDLC(can, 4);
        CANTransmit(can, 1.0f);
        break;

    case DM_MODE_EMIT_ENUM:
        DM_PackEMIT(motor);
        CANSetDLC(can, 8);
        CANTransmit(can, 1.0f);
        break;

    case DM_MODE_1TO4_ENUM:
        DM_Pack1To4(motor);
        break;
    }
}

void DMMotorControl(void)
{
    for (uint8_t i = 0U; i < dm_1to4_sender_cnt; i++)
    {
        memset(dm_1to4_senders[i]->tx_buff, 0, sizeof(dm_1to4_senders[i]->tx_buff));
    }

    for (uint8_t i = 0U; i < dm_instance_count; i++)
    {
        DMMotorUpdateInstance(dm_motor_instances[i]);
    }

    for (uint8_t i = 0U; i < dm_1to4_sender_cnt; i++)
    {
        CANInstance* sender = dm_1to4_senders[i];
        CANSetDLC(sender, 8);
        CANTransmit(sender, 1.0f);
    }
}
