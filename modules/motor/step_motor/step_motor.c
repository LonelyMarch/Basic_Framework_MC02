/**
 * @file step_motor.c
 * @brief 步进电机控制模块实现。
 *
 * 控制方式：PWM 脉冲 + DIR 方向引脚。
 * - 脉冲频率 (Hz) = 转速 (rps) × 每转步数 (steps_per_rev)。
 * - 占空比固定 50%，保证脉冲宽度稳定。
 * - 加减速采用梯形速度曲线：线加速度 accel_rps2。
 *
 * 位置模式含减速距离判断：
 * - 从最大目标速度 v_max 以加速度 a 减速到 0 需要的步数 =
 *   (v_max² / (2*a)) × steps_per_rev，该值在 StepMotorMoveAbs() 中预计算一次。
 * - 当剩余步数 ≤ decel_threshold 时进入减速段，按线性比率降速。
 *
 * 调用者需在高频任务中周期调用 StepMotorUpdate()（推荐 1kHz）。
 *
 * @note 同一实例的所有接口应在同一任务/中断上下文中调用，模块不做任务间互斥。
 */

#include "step_motor.h"
#include "bsp_log.h"
#include <string.h>
#include <math.h>

/* ========================== 静态变量 ========================== */

/** @brief 步进电机实例静态池（不用 malloc）。 */
static StepMotorInstance step_motor_pool[STEP_MOTOR_MAX_CNT];

/** @brief 已注册实例计数。 */
static uint8_t step_motor_idx = 0;

/* ==================== 内部辅助函数 ==================== */

/**
 * @brief 根据当前转速设置 PWM 脉冲频率和占空比。
 *
 * 脉冲频率 (Hz) = 转速 (rps) × 每转步数 (steps_per_rev)。
 * 占空比固定 50%。仅在转速有效（>0）时输出脉冲。
 *
 * @param motor 步进电机实例。
 */
static void StepMotorUpdatePulse(StepMotorInstance* motor)
{
    float speed_abs = fabsf(motor->current_speed_rps);

    if (speed_abs <= 0.0f || motor->mode == STEP_MOTOR_MODE_STOP)
    {
        /* 速度为 0 或停止模式，关闭 PWM 输出 */
        PWMSetDutyRatio(motor->pulse_pwm, 0.0f);
        motor->state = STEP_MOTOR_STATE_IDLE;
        return;
    }

    /*
     * 计算脉冲周期：T_pulse = 1 / (speed * steps_per_rev)
     * 例如：5 rps × 3200 步/转 = 16000 Hz 脉冲频率 → 62.5 μs 周期
     */
    float pulse_period = 1.0f / (speed_abs * (float)motor->steps_per_rev);

    /*
     * PWMSetPeriod() 内部会重新计算 ARR 并写寄存器。
     * 占空比固定 50% 保证脉冲宽度足够，驱动器的 Step 引脚通常
     * 要求最小脉宽 ≥ 1μs（A4988）或 ≥ 100ns（TMC2209），
     * 50% 占空比在此频率范围内满足所有常见驱动器。
     */
    PWMSetPeriod(motor->pulse_pwm, pulse_period);
    PWMSetDutyRatio(motor->pulse_pwm, 0.5f);

    motor->state = STEP_MOTOR_STATE_RUNNING;
}

/**
 * @brief 设置方向引脚电平。
 *
 * @param motor 步进电机实例。
 * @param dir 目标方向。
 */
static void StepMotorSetDir(StepMotorInstance* motor, StepMotor_Dir_e dir)
{
    if (motor->dir_gpio == NULL)
    {
        return;
    }

    motor->dir = dir;

    /* 驱动器 DIR 引脚通常：低电平=CW，高电平=CCW */
    if (dir == STEP_MOTOR_DIR_CW)
    {
        GPIOReset(motor->dir_gpio);
    }
    else
    {
        GPIOSet(motor->dir_gpio);
    }
}

/**
 * @brief 计算从速度 v (rps) 以加速度 a (rps²) 匀减速到零需要的步数。
 *
 * 匀减速运动：减速时间 t = v/a，期间走过的转数 = (v/2) × t = v²/(2a)。
 * 换算为步数：步数 = 转数 × steps_per_rev = (v²/(2a)) × steps_per_rev。
 *
 * @param v 转速绝对值 (rps)。
 * @param a 加速度 (rps²)。
 * @param steps_per_rev 每转步数。
 * @return 减速到零需要的步数（向上取整）。
 */
static int32_t StepMotorCalcDecelSteps(float v, float a, uint32_t steps_per_rev)
{
    if (a <= 0.0f || v <= 0.0f)
    {
        return 0;
    }

    /* v²/(2a) = 转数，再 × steps_per_rev = 减速步数，+0.5f 向上取整 */
    float revolutions = (v * v) / (2.0f * a);
    float decel_steps = revolutions * (float)steps_per_rev;

    return (int32_t)(decel_steps + 0.5f);
}

/**
 * @brief 梯形速度逼近：将当前速度向目标速度线性推进一步。
 *        速度变化量 dv = accel × dt，保证不冲过目标值。
 *
 * @param current 当前速度值（会被修改）。
 * @param target 目标速度值。
 * @param dv 本周期最大速度变化量（>0）。
 */
static void StepMotorApproachSpeed(float* current, float target, float dv)
{
    if (fabsf(*current - target) <= dv)
    {
        *current = target;
    }
    else if (*current < target)
    {
        *current += dv;
    }
    else
    {
        *current -= dv;
    }
}

/* ==================== 公共接口 ==================== */

StepMotorInstance* StepMotorInit(StepMotor_Init_Config_s* config)
{
    StepMotorInstance* motor;

    if (config == NULL)
    {
        LOGERROR("[step_motor] init config is null");
        return NULL;
    }

    if (config->pulse_pwm_config.htim == NULL)
    {
        LOGERROR("[step_motor] pulse PWM timer is null");
        return NULL;
    }

    if (step_motor_idx >= STEP_MOTOR_MAX_CNT)
    {
        LOGERROR("[step_motor] instance exceeded, max [%u]", (unsigned int)STEP_MOTOR_MAX_CNT);
        return NULL;
    }

    /* 从静态池分配 */
    motor = &step_motor_pool[step_motor_idx];
    memset(motor, 0, sizeof(StepMotorInstance));

    /* 注册脉冲 PWM 实例 */
    motor->pulse_pwm = PWMRegister((PWM_Init_Config_s*)&config->pulse_pwm_config);
    if (motor->pulse_pwm == NULL)
    {
        LOGERROR("[step_motor] PWM register failed");
        memset(motor, 0, sizeof(StepMotorInstance));
        return NULL;
    }

    /* 注册方向引脚 GPIO 实例 */
    motor->dir_gpio = GPIORegister(&config->dir_gpio_config);
    if (motor->dir_gpio == NULL)
    {
        LOGERROR("[step_motor] DIR GPIO register failed");
        memset(motor, 0, sizeof(StepMotorInstance));
        return NULL;
    }

    /* 注册使能引脚 GPIO 实例（可选） */
    if (config->en_gpio_config.GPIO_Pin != STEP_MOTOR_NO_EN_PIN)
    {
        motor->en_gpio = GPIORegister(&config->en_gpio_config);
        if (motor->en_gpio == NULL)
        {
            LOGWARNING("[step_motor] EN GPIO register failed, motor will always be enabled");
        }
    }

    /* 保存电机参数 */
    motor->steps_per_rev = config->steps_per_rev;
    motor->max_speed_rps = config->max_speed_rps;
    motor->accel_rps2 = config->accel_rps2;

    /* 初始状态：停止，方向 CW */
    motor->mode = STEP_MOTOR_MODE_STOP;
    motor->dir = STEP_MOTOR_DIR_CW;
    motor->state = STEP_MOTOR_STATE_IDLE;

    StepMotorSetDir(motor, STEP_MOTOR_DIR_CW);
    PWMSetDutyRatio(motor->pulse_pwm, 0.0f); // 初始无脉冲

    step_motor_idx++;

    return motor;
}

void StepMotorSetSpeed(StepMotorInstance* motor, float speed_rps)
{
    if (motor == NULL)
    {
        return;
    }

    /* 限幅到最大转速 */
    if (speed_rps > motor->max_speed_rps)
    {
        speed_rps = motor->max_speed_rps;
    }
    else if (speed_rps < -motor->max_speed_rps)
    {
        speed_rps = -motor->max_speed_rps;
    }

    motor->mode = STEP_MOTOR_MODE_SPEED;
    motor->target_speed_rps = speed_rps;

    /* 根据目标速度符号设置方向 */
    if (speed_rps >= 0.0f)
    {
        StepMotorSetDir(motor, STEP_MOTOR_DIR_CW);
    }
    else
    {
        StepMotorSetDir(motor, STEP_MOTOR_DIR_CCW);
    }
}

void StepMotorMoveRel(StepMotorInstance* motor, int32_t steps, float speed_rps)
{
    if (motor == NULL)
    {
        return;
    }

    StepMotorMoveAbs(motor, motor->current_step + steps, speed_rps);
}

void StepMotorMoveAbs(StepMotorInstance* motor, int32_t step, float speed_rps)
{
    if (motor == NULL)
    {
        return;
    }

    /* 位置模式速度取绝对值（方向由位置差决定） */
    if (speed_rps < 0.0f)
    {
        speed_rps = -speed_rps;
    }
    if (speed_rps > motor->max_speed_rps)
    {
        speed_rps = motor->max_speed_rps;
    }

    motor->mode = STEP_MOTOR_MODE_POSITION;
    motor->target_step = step;
    motor->target_speed_rps = speed_rps;

    /*
     * 预计算减速距离阈值：
     * 从最大目标速度 v_max 以加速度 a 减速到 0 需要的步数。
     * 在 StepMotorMoveAbs() 中一次算出，StepMotorUpdate() 每周期只做比较，
     * 避免每周期重复调用 StepMotorCalcDecelSteps() 带来的浮点开销和
     * 使用"当前速度"导致的减速距离低估。
     */
    motor->decel_threshold = StepMotorCalcDecelSteps(speed_rps,
                                                     motor->accel_rps2,
                                                     motor->steps_per_rev);

    /* 根据目标位置与当前位置的差值确定方向 */
    int32_t delta = step - motor->current_step;

    if (delta >= 0)
    {
        StepMotorSetDir(motor, STEP_MOTOR_DIR_CW);
    }
    else
    {
        StepMotorSetDir(motor, STEP_MOTOR_DIR_CCW);
    }
}

void StepMotorStop(StepMotorInstance* motor)
{
    if (motor == NULL)
    {
        return;
    }

    /*
     * 紧急停止：立即关闭脉冲输出，mode 切 STOP。
     * current_step 保留不归零，以便下次位置模式从此处继续。
     * 如需完全复位位置，调用者应在 Stop 后手动处理位置值。
     */
    motor->mode = STEP_MOTOR_MODE_STOP;
    motor->target_speed_rps = 0.0f;
    motor->current_speed_rps = 0.0f;
    motor->state = STEP_MOTOR_STATE_IDLE;

    PWMSetDutyRatio(motor->pulse_pwm, 0.0f);
}

void StepMotorDisable(StepMotorInstance* motor)
{
    if (motor == NULL)
    {
        return;
    }

    StepMotorStop(motor);

    if (motor->en_gpio != NULL)
    {
        /* EN 高电平 → 驱动器失能，释放力矩 */
        GPIOSet(motor->en_gpio);
    }
}

void StepMotorEnable(StepMotorInstance* motor)
{
    if (motor == NULL)
    {
        return;
    }

    if (motor->en_gpio != NULL)
    {
        /* EN 低电平 → 驱动器使能，上锁力矩（极性取决于驱动器） */
        GPIOReset(motor->en_gpio);
    }

    /*
     * 仅操作 EN 引脚，不恢复之前的控制模式。
     * 调用者需重新发送 StepMotorSetSpeed() 或 StepMotorMoveAbs() 来设定运动。
     */
}

void StepMotorUpdate(StepMotorInstance* motor, float dt_s)
{
    if (motor == NULL || dt_s <= 0.0f)
    {
        return;
    }

    switch (motor->mode)
    {
    case STEP_MOTOR_MODE_SPEED:
        {
            /*
         * 速度模式：梯形加速/减速到目标速度。
         * 当目标速度为 0 且当前速度也已减速到 0 时，自动切到 STOP 模式。
         */
            float target = motor->target_speed_rps;
            float dv = motor->accel_rps2 * dt_s;

            StepMotorApproachSpeed(&motor->current_speed_rps, target, dv);

            /*
         * 当目标速度和当前速度都是 0 时，说明已完全停止。
         * 把 mode 切到 STOP，使状态更清晰。
         */
            if (target == 0.0f && fabsf(motor->current_speed_rps) <= 1e-6f)
            {
                motor->mode = STEP_MOTOR_MODE_STOP;
            }

            /* 更新脉冲频率 */
            StepMotorUpdatePulse(motor);

            /*
         * 位置累加：当前转速 × 每转步数 × dt = 本周期走过的步数增量。
         * 方向由转速符号确定。
         */
            int32_t step_delta = (int32_t)(fabsf(motor->current_speed_rps)
                * (float)motor->steps_per_rev * dt_s);

            if (motor->current_speed_rps >= 0.0f)
            {
                motor->current_step += step_delta;
            }
            else
            {
                motor->current_step -= step_delta;
            }
            motor->total_step += step_delta; // total_step 始终为绝对值累加

            break;
        }

    case STEP_MOTOR_MODE_POSITION:
        {
            /*
         * 位置模式：含预计算减速阈值的梯形速度剖面。
         *
         * 三个阶段（由剩余步数和减速阈值的关系决定）：
         * 1. 加速段/匀速段：速度向 target_speed_rps 逼近。
         * 2. 减速段：remaining ≤ decel_threshold 时开始降速。
         * 3. 完成：remaining == 0 时停止。
         */
            int32_t remaining = motor->target_step - motor->current_step;

            if (remaining == 0)
            {
                /* 已到达目标位置 */
                motor->current_speed_rps = 0.0f;
                motor->mode = STEP_MOTOR_MODE_STOP;
                StepMotorUpdatePulse(motor);
                break;
            }

            /* 目标速度方向：向目标方向 */
            float max_target = motor->target_speed_rps;
            if (remaining < 0)
            {
                max_target = -max_target;
            }

            int32_t remaining_abs = (remaining >= 0) ? remaining : -remaining;

            float dv = motor->accel_rps2 * dt_s;

            if (motor->decel_threshold > 0 && remaining_abs <= motor->decel_threshold)
            {
                /*
             * 减速段：按剩余距离比例缩小目标速度。
             * target_speed = max_target × (remaining_abs / decel_threshold)
             * 当 remaining_abs == decel_threshold 时目标速度 ≈ max_target（平滑过渡），
             * 当 remaining_abs → 0 时目标速度 → 0。
             *
             * 使用渐进式逼近而非直接赋值，防止脉冲频率突变导致电机失步。
             */
                float ratio = (float)remaining_abs / (float)motor->decel_threshold;
                if (ratio > 1.0f) { ratio = 1.0f; }
                float target_speed = max_target * ratio;

                StepMotorApproachSpeed(&motor->current_speed_rps, target_speed, dv);
            }
            else
            {
                /* 加速/匀速段：向 max_target 靠拢 */
                StepMotorApproachSpeed(&motor->current_speed_rps, max_target, dv);
            }

            /* 更新脉冲频率 */
            StepMotorUpdatePulse(motor);

            /* 位置累加 */
            int32_t step_delta = (int32_t)(fabsf(motor->current_speed_rps)
                * (float)motor->steps_per_rev * dt_s);

            if (motor->current_speed_rps >= 0.0f)
            {
                motor->current_step += step_delta;
            }
            else
            {
                motor->current_step -= step_delta;
            }
            motor->total_step += step_delta;

            break;
        }

    case STEP_MOTOR_MODE_STOP:
    default:
        break;
    }
}

void StepMotorControl(float dt_s)
{
    for (uint8_t i = 0U; i < step_motor_idx; i++)
    {
        StepMotorUpdate(&step_motor_pool[i], dt_s);
    }
}

uint8_t StepMotorGetCount(void)
{
    return step_motor_idx;
}

int32_t StepMotorGetPosition(const StepMotorInstance* motor)
{
    if (motor == NULL)
    {
        return 0;
    }

    return motor->current_step;
}

StepMotor_State_e StepMotorGetState(const StepMotorInstance* motor)
{
    if (motor == NULL)
    {
        return STEP_MOTOR_STATE_IDLE;
    }

    return motor->state;
}
