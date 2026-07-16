#include "servo_motor.h"
#include "bsp_log.h"
#include <string.h>

/* ========================== PWM舵机参数 ========================== */
/* 默认参数适用于 SG90 等 50Hz/0.5~2.5ms 脉宽的舵机,按实际型号修改 */

/** @brief PWM舵机默认角度范围,典型 0~180 度。 */
#define PWM_SERVO_ANGLE_MIN 0.0f
#define PWM_SERVO_ANGLE_MAX 180.0f

/** @brief PWM舵机占空比范围,对应 0.5ms~2.5ms @ 50Hz = 2.5%~12.5%。 */
#define PWM_SERVO_DUTY_MIN 0.025f
#define PWM_SERVO_DUTY_MAX 0.125f

/* ========================== 静态变量 ========================== */

/** @brief 舵机实例静态池,避免 malloc() 依赖 heap。 */
static ServoInstance servo_motor_pool[SERVO_MOTOR_CNT];

/** @brief 已注册的舵机实例指针数组,servo_id 作为索引。 */
static ServoInstance* servo_motor_instance[SERVO_MOTOR_CNT] = {NULL};

/** @brief 已注册的舵机实例计数。 */
static uint8_t servo_idx = 0;

/* ==================== 内部辅助函数 ==================== */

/**
 * @brief 总线舵机协议 checksum 计算。
 *        帧格式: 0x55 0x55 LEN CMD ID DATA[0..n] CHECKSUM
 *        CHECKSUM = ~(ID + LEN + CMD + DATA[0] + ... + DATA[n]) & 0xFF
 *
 * @param data 从 LEN 字节开始的待校验数据首地址。
 * @param len  从 LEN 到 DATA 末尾的字节数。
 * @return uint8_t checksum 值。
 */
static uint8_t ServoCalcChecksum(const uint8_t* data, uint8_t len)
{
    uint8_t sum = 0U;

    for (uint8_t i = 0U; i < len; i++)
    {
        sum += data[i];
    }
    return (uint8_t)(~sum);
}

/**
 * @brief 总线舵机反馈解码回调。
 *        由 USART 收到完整一帧后调用,从帧中解析 servo_id 定位实例。
 *
 * @note 当前 USART BSP 回调签名为 void (*)(void),不传递实例参数。
 *       因此需要遍历找到收到数据的 USART 实例,再按帧内 ID 索引。
 */
static void DecodeServo(void)
{
    /*
     * 回复帧格式: 0x55 0x55 LEN CMD ID DATA[0..n] CHECKSUM
     * 角度读取回复 (CMD=0x15): ID 在 DATA[4], 角度在 DATA[6:7] (小端 uint16)。
     */
    for (uint8_t i = 0; i < SERVO_MOTOR_CNT; i++)
    {
        ServoInstance* servo = servo_motor_instance[i];

        if (servo == NULL || servo->servo_type != Bus_Servo)
        {
            continue;
        }

        if (servo->usart_instance == NULL ||
            servo->usart_instance->recv_len < Servo_MAX_BUFF)
        {
            continue;
        }

        uint8_t* rx = servo->usart_instance->recv_buff;

        /* 校验帧头 */
        if (rx[0] != Servo_Frame_First || rx[1] != Servo_Frame_Second)
        {
            continue;
        }

        /* 从帧中提取 ID,直接索引到正确的舵机实例 */
        uint8_t rx_id = rx[4];

        if (rx_id >= SERVO_MOTOR_CNT || servo_motor_instance[rx_id] == NULL)
        {
            continue;
        }

        ServoInstance* target = servo_motor_instance[rx_id];

        if (rx[3] == SERVO_POS_READ_CMD)
        {
            target->recv_angle = (uint16_t)rx[6] |
                ((uint16_t)rx[7] << 8U);
        }
    }
}

/* ==================== 公共接口 ==================== */

ServoInstance* ServoInit(Servo_Init_Config_s* config)
{
    ServoInstance* servo;

    if (config == NULL)
    {
        LOGERROR("[servo] init config is null");
        return NULL;
    }

    if (servo_idx >= SERVO_MOTOR_CNT)
    {
        LOGERROR("[servo] instance exceeded, max [%u]", (unsigned int)SERVO_MOTOR_CNT);
        return NULL;
    }

    /* 从静态池分配,不使用 malloc() */
    servo = &servo_motor_pool[servo_idx];
    memset(servo, 0, sizeof(ServoInstance));

    servo->servo_type = config->servo_type;

    switch (config->servo_type)
    {
    case Bus_Servo:
        {
            USART_Init_Config_s usart_config;
            memset(&usart_config, 0, sizeof(usart_config));
            usart_config.module_callback = DecodeServo;
            usart_config.recv_buff_size = Servo_MAX_BUFF;
            usart_config.usart_handle = config->_handle;

            servo->usart_instance = USARTRegister(&usart_config);
            if (servo->usart_instance == NULL)
            {
                LOGERROR("[servo] USART register failed");
                memset(servo, 0, sizeof(ServoInstance));
                return NULL;
            }
            break;
        }

    case PWM_Servo:
        servo->pwm_instance = PWMRegister(&config->pwm_init_config);
        if (servo->pwm_instance == NULL)
        {
            LOGERROR("[servo] PWM register failed");
            memset(servo, 0, sizeof(ServoInstance));
            return NULL;
        }
        break;

    default:
        LOGERROR("[servo] servo type error [%d]", (int)config->servo_type);
        memset(servo, 0, sizeof(ServoInstance));
        return NULL;
    }

    servo->servo_id = config->servo_id;
    servo_motor_instance[servo->servo_id] = servo;
    servo_idx++;

    return servo;
}

void ServoSetAngle(ServoInstance* servo, float angle)
{
    HAL_StatusTypeDef status;

    if (servo == NULL)
    {
        return;
    }

    switch (servo->servo_type)
    {
    case Bus_Servo:
        {
            /*
         * 构建写入角度命令帧 (栈上临时 buffer,避免全局缓冲区多任务冲突)。
         * 帧格式: 0x55 0x55 LEN(0x08) CMD(0x03) ID ANGLE_L ANGLE_H TIME_L TIME_H CHECKSUM
         * ANGLE: uint16_t 小端, TIME: 默认 0x0320 (800ms)。
         */
            uint8_t tx_frame[10];
            tx_frame[0] = Servo_Frame_First;
            tx_frame[1] = Servo_Frame_Second;
            tx_frame[2] = 0x08; // LEN
            tx_frame[3] = SERVO_MOVE_CMD;
            tx_frame[4] = servo->servo_id;
            tx_frame[5] = (uint8_t)((uint16_t)angle & 0xff);
            tx_frame[6] = (uint8_t)(((uint16_t)angle >> 8) & 0xff);
            tx_frame[7] = 0x20; // 时间低字节 (800ms = 0x0320)
            tx_frame[8] = 0x03; // 时间高字节
            tx_frame[9] = ServoCalcChecksum(&tx_frame[2], 6U);

            status = USARTSend(servo->usart_instance, tx_frame, 10, USART_TRANSFER_DMA);
            if (status != HAL_OK)
            {
                LOGWARNING("[servo] send failed, status [%d]", (int)status);
            }
            break;
        }

    case PWM_Servo:
        {
            /*
         * PWM 舵机: 角度 -> 占空比线性换算。
         * 典型 SG90: 50Hz, 0.5ms(2.5%)~2.5ms(12.5%) 对应 0~180度。
         * 参数由 PWM_SERVO_ANGLE_* / PWM_SERVO_DUTY_* 宏定义控制。
         */
            servo->angle = angle;

            /* 输入限幅 */
            if (angle < PWM_SERVO_ANGLE_MIN) angle = PWM_SERVO_ANGLE_MIN;
            if (angle > PWM_SERVO_ANGLE_MAX) angle = PWM_SERVO_ANGLE_MAX;

            float duty = PWM_SERVO_DUTY_MIN +
                (angle - PWM_SERVO_ANGLE_MIN) /
                (PWM_SERVO_ANGLE_MAX - PWM_SERVO_ANGLE_MIN) *
                (PWM_SERVO_DUTY_MAX - PWM_SERVO_DUTY_MIN);

            PWMSetDutyRatio(servo->pwm_instance, duty);
            break;
        }

    default:
        break;
    }
}
