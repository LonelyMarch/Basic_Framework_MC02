/**
 ******************************************************************************
 * @file    ins_task.c
 * @author  Wang Hongxi
 * @author  annotation and modificaiton by neozng
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "ins_task.h"
#include "QuaternionEKF.h"
#include "bmi088.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_pwm.h"
#include "controller.h"
#include "general_def.h"
#include "spi.h"
#include "tim.h"
#include "user_lib.h"
#include "arm_math.h"
#include <math.h>
#include <string.h>

#define INS_INIT_ACC_SAMPLE_COUNT 100U
#define INS_INIT_ACC_MAX_ATTEMPT 300U
#define INS_BMI088_RETRY_DELAY_S 0.1f
#define INS_EKF_DT_MIN 0.0001f
#define INS_EKF_DT_MAX 0.005f
// BMI088温度读取已降到约1Hz,温控任务不需要跟随1kHz姿态任务高频计算,10Hz足够平滑。
#define INS_TEMP_CTRL_PERIOD_COUNT 100U
// BMI088连续读取失败超过该次数后认为IMU离线,温控停止加热以避免长期使用旧温度。
#define INS_BMI088_OFFLINE_FAIL_COUNT 100U
// IMU离线后的限频日志周期,避免1kHz任务持续刷屏。
#define INS_BMI088_OFFLINE_LOG_PERIOD 1000U
/*
 * BMI088运行期工作模式。
 * 默认保持周期读取,若需要启用DRDY触发模式,可在这里改成BMI088_BLOCK_TRIGGER_MODE。
 */
#ifndef INS_BMI088_WORK_MODE
#define INS_BMI088_WORK_MODE BMI088_BLOCK_PERIODIC_MODE
#endif

static INS_t INS;
static IMU_Param_t IMU_Param;
static PIDInstance TempCtrl = {0};
static PWMInstance* imu_heat_pwm = NULL;
static BMI088Instance* ins_bmi088 = NULL;
static BMI088_Data_t ins_bmi088_data = {0};
static uint32_t ins_bmi088_fail_count = 0U;
static uint32_t ins_bmi088_wait_frame_count = 0U;

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

// 用于获取两次采样之间的时间间隔
static uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
static float RefTemp = 40; // 恒温设定温度

static void IMU_Param_Correction(IMU_Param_t* param, float gyro[3], float accel[3]);


static BMI088Instance* INS_BMI088_Init(void);


static uint8_t INS_BMI088_HasCompleteTriggerFrame(void);


static void INS_BMI088_RecordFailure(const char* reason);


static void IMU_Param_SetDefault(IMU_Param_t* param)
{
    if (param == NULL)
    {
        return;
    }

    /*
     * 默认不做比例修正,也不做安装角补偿。
     * 后续若完成六面标定或整机安装标定,只需要改这里或提供参数加载入口。
     */
    param->gyro_scale[IMU_AXIS_X] = 1.0f;
    param->gyro_scale[IMU_AXIS_Y] = 1.0f;
    param->gyro_scale[IMU_AXIS_Z] = 1.0f;
    param->accel_scale[IMU_AXIS_X] = 1.0f;
    param->accel_scale[IMU_AXIS_Y] = 1.0f;
    param->accel_scale[IMU_AXIS_Z] = 1.0f;
    param->Yaw = 0.0f;
    param->Pitch = 0.0f;
    param->Roll = 0.0f;
    param->flag = 1U;
}

static uint8_t INS_BMI088_HasCompleteTriggerFrame(void)
{
    uint8_t has_frame;
    uint32_t primask;

    if (ins_bmi088 == NULL || ins_bmi088->work_mode != BMI088_BLOCK_TRIGGER_MODE)
    {
        return 1U;
    }

    /*
     * 触发模式下BMI088Acquire()返回0可能只是"完整帧还没到齐"。
     * 在调用前先快照一次DRDY计数,用于区分正常等待和真正读取失败。
     */
    primask = __get_PRIMASK();
    __disable_irq();
    has_frame = ((ins_bmi088->acc_drdy_count != ins_bmi088->acc_read_count) &&
                    (ins_bmi088->gyro_drdy_count != ins_bmi088->gyro_read_count))
                    ? 1U
                    : 0U;
    __set_PRIMASK(primask);

    return has_frame;
}

static void INS_BMI088_RecordFailure(const char* reason)
{
    ins_bmi088_fail_count++;
    if (ins_bmi088_fail_count <= 10U || (ins_bmi088_fail_count % INS_BMI088_OFFLINE_LOG_PERIOD) == 0U)
    {
        LOGWARNING("[ins] BMI088 %s, count=%lu", reason, (unsigned long)ins_bmi088_fail_count);
    }

    if (ins_bmi088_fail_count == INS_BMI088_OFFLINE_FAIL_COUNT ||
        (ins_bmi088_fail_count > INS_BMI088_OFFLINE_FAIL_COUNT &&
            (ins_bmi088_fail_count % INS_BMI088_OFFLINE_LOG_PERIOD) == 0U))
    {
        LOGERROR("[ins] BMI088 offline, disable temperature control, reason=%s, count=%lu",
                 reason,
                 (unsigned long)ins_bmi088_fail_count);
    }
}

static void IMUPWMSet(uint16_t pwm)
{
    if (imu_heat_pwm == NULL || imu_heat_pwm->htim == NULL)
    {
        return;
    }

    // 原温控PID输出的是CCR计数值,这里换算为bsp_pwm使用的0~1占空比语义
    uint64_t period_ticks = (uint64_t)imu_heat_pwm->htim->Instance->ARR + 1ULL;
    if (period_ticks == 0ULL)
    {
        return;
    }

    PWMSetDutyRatio(imu_heat_pwm, (float)pwm / (float)period_ticks);
}

static void IMUTemperatureCtrlDisable(void)
{
    /*
     * BMI088长时间离线时,最近一次温度已经不可信。
     * 此时关闭加热并清空PID历史输出,避免传感器恢复后旧积分造成PWM突跳。
     */
    TempCtrl.Output = 0.0f;
    TempCtrl.Pout = 0.0f;
    TempCtrl.Iout = 0.0f;
    TempCtrl.ITerm = 0.0f;
    TempCtrl.Dout = 0.0f;
    IMUPWMSet(0U);
}

/**
 * @brief 温度控制
 *
 */
static void IMU_Temperature_Ctrl(void)
{
    if (ins_bmi088_fail_count >= INS_BMI088_OFFLINE_FAIL_COUNT)
    {
        IMUTemperatureCtrlDisable();
        return;
    }

    /*
     * 温度来自BMI088最近一次成功采样。温度读取失败时BMI088驱动会保留上一帧温度,
     * 因而热控不会因为一次SPI异常而输出突变。
     */
    PIDCalculate(&TempCtrl, ins_bmi088_data.temperature, RefTemp);
    uint16_t heat_pwm = (uint16_t)float_constrain(float_rounding(TempCtrl.Output), 0, UINT16_MAX);
    IMUPWMSet(heat_pwm);
}

/**
 * @brief 注册并初始化云台板BMI088
 *
 * BMI088初始化发生在RobotInit阶段,此时FreeRTOS尚未启动且全局中断被关闭。
 * 因此这里使用阻塞SPI完成芯片ID检查、软复位、寄存器配置和在线标定;
 * 进入INS任务后仍通过BSP SPI的同步接口读取,由BSP负责总线互斥和片选控制。
 */
static BMI088Instance* INS_BMI088_Init(void)
{
    BMI088_Init_Config_s bmi088_config = {
        .work_mode = INS_BMI088_WORK_MODE,
        .cali_mode = BMI088_CALIBRATE_ONLINE_MODE,
        .spi_gyro_config = {
            .spi_handle = &hspi2,
            .GPIOx = CS2_GYRO_GPIO_Port,
            .cs_pin = CS2_GYRO_Pin,
            .spi_work_mode = SPI_BLOCK_MODE,
        },
        .spi_acc_config = {
            .spi_handle = &hspi2,
            .GPIOx = CS2_ACCEL_GPIO_Port,
            .cs_pin = CS2_ACCEL_Pin,
            .spi_work_mode = SPI_BLOCK_MODE,
        },
        .gyro_int_config = {
            .GPIOx = GYRO_INT_GPIO_Port,
            .GPIO_Pin = GYRO_INT_Pin,
            .pin_state = GPIO_PIN_RESET,
            .exti_mode = GPIO_EXTI_MODE_RISING,
        },
        .acc_int_config = {
            .GPIOx = ACC_INT_GPIO_Port,
            .GPIO_Pin = ACC_INT_Pin,
            .pin_state = GPIO_PIN_RESET,
            .exti_mode = GPIO_EXTI_MODE_RISING,
        },
    };

    while (ins_bmi088 == NULL)
    {
        ins_bmi088 = BMI088Register(&bmi088_config);
        if (ins_bmi088 == NULL)
        {
            /*
             * BMI088是云台姿态闭环的核心传感器,初始化失败时继续重试比带病进入
             * 任务调度更安全。这里仍使用DWT_Delay,符合RobotInit阶段不能依赖RTOS的约束。
             */
            LOGERROR("[ins] BMI088 register failed, retry later");
            DWT_Delay(INS_BMI088_RETRY_DELAY_S);
        }
    }

    /*
     * 先同步BMI088实例中的默认/最近值。若初始化后的第一次主动采样偶发失败,
     * 温控仍会使用驱动内的默认安全温度,不会把全局零初始化值误认为真实温度。
     */
    memcpy(ins_bmi088_data.acc, ins_bmi088->acc, sizeof(ins_bmi088_data.acc));
    memcpy(ins_bmi088_data.gyro, ins_bmi088->gyro, sizeof(ins_bmi088_data.gyro));
    ins_bmi088_data.temperature = ins_bmi088->temperature;

    /*
     * 初始化后主动采一帧,为温控和初始四元数准备一份有效的最近数据。
     * 这里只临时切换软件读取分支,硬件DRDY配置仍由BMI088Register()按初始work_mode决定。
     */
    ins_bmi088->work_mode = BMI088_BLOCK_PERIODIC_MODE;
    (void)BMI088Acquire(ins_bmi088, &ins_bmi088_data);
    ins_bmi088->work_mode = INS_BMI088_WORK_MODE;

    return ins_bmi088;
}

// 使用加速度计的数据初始化Roll和Pitch,而Yaw置0,这样可以避免在初始时候的姿态估计误差
static void InitQuaternion(float* init_q4)
{
    float acc_init[3] = {0};
    float gravity_norm[3] = {0, 0, 1}; // 导航系重力加速度矢量,归一化后为(0,0,1)
    float axis_rot[3] = {0}; // 旋转轴
    uint16_t success_count = 0U;
    uint16_t attempt_count = 0U;

    if (init_q4 == NULL)
    {
        return;
    }

    /*
     * 读取多帧加速度计数据取平均,用于估计初始roll/pitch。
     * 这里只累计成功采样,避免SPI偶发失败把旧数据或全0数据带入姿态初值。
     */
    while (success_count < INS_INIT_ACC_SAMPLE_COUNT && attempt_count < INS_INIT_ACC_MAX_ATTEMPT)
    {
        attempt_count++;
        if (BMI088Acquire(ins_bmi088, &ins_bmi088_data) != 0U)
        {
            acc_init[IMU_AXIS_X] += ins_bmi088_data.acc[IMU_AXIS_X];
            acc_init[IMU_AXIS_Y] += ins_bmi088_data.acc[IMU_AXIS_Y];
            acc_init[IMU_AXIS_Z] += ins_bmi088_data.acc[IMU_AXIS_Z];
            success_count++;
        }
        DWT_Delay(0.001);
    }

    if (success_count == 0U || NormOf3d(acc_init) < 1e-6f)
    {
        /*
         * 如果初始化阶段始终读不到有效加速度,先给EKF一个单位四元数,
         * 后续任务恢复采样后仍能继续运行;同时通过日志提示硬件链路异常。
         */
        LOGERROR("[ins] init quaternion failed: no valid BMI088 sample");
        init_q4[0] = 1.0f;
        init_q4[1] = 0.0f;
        init_q4[2] = 0.0f;
        init_q4[3] = 0.0f;
        return;
    }

    for (uint8_t i = 0; i < 3; ++i)
        acc_init[i] /= (float)success_count;
    Norm3d(acc_init);
    // 计算原始加速度矢量和导航系重力加速度矢量的夹角
    float gravity_dot = float_constrain(Dot3d(acc_init, gravity_norm), -1.0f, 1.0f);
    float angle = acosf(gravity_dot);
    Cross3d(acc_init, gravity_norm, axis_rot);
    if (NormOf3d(axis_rot) < 1e-6f)
    {
        /*
         * 加速度方向和导航系重力几乎共线时,旋转轴长度接近0。
         * 同向时直接使用单位四元数;反向时选择绕X轴旋转180度作为确定解。
         */
        init_q4[0] = (gravity_dot > 0.0f) ? 1.0f : 0.0f;
        init_q4[1] = (gravity_dot > 0.0f) ? 0.0f : 1.0f;
        init_q4[2] = 0.0f;
        init_q4[3] = 0.0f;
        return;
    }
    Norm3d(axis_rot);
    init_q4[0] = cosf(angle / 2.0f);
    for (uint8_t i = 0; i < 2; ++i)
        init_q4[i + 1] = axis_rot[i] * sinf(angle / 2.0f); // 轴角公式,第三轴为0(没有z轴分量)
    init_q4[3] = 0.0f;
}

attitude_t* INS_Init(void)
{
    if (!INS.init)
        INS.init = 1;
    else
        return &INS.attitude;

    PWM_Init_Config_s imu_heat_pwm_config = {
        .htim = &htim3,
        .channel = TIM_CHANNEL_4,
        .period = 0.001f,
        .dutyratio = 0.0f,
    };
    imu_heat_pwm = PWMRegister(&imu_heat_pwm_config);

    INS_BMI088_Init();
    IMU_Param_SetDefault(&IMU_Param);

    float init_quaternion[4] = {0};
    /*
     * 初始四元数估计需要在RobotInit阶段稳定读取加速度,因此临时走周期读取分支。
     * 这不是运行期模式切换接口,不会重新配置BMI088的DRDY寄存器。
     */
    ins_bmi088->work_mode = BMI088_BLOCK_PERIODIC_MODE;
    InitQuaternion(init_quaternion);
    ins_bmi088->work_mode = INS_BMI088_WORK_MODE;
    IMU_QuaternionEKF_Init(init_quaternion, 10, 0.001, 1000000, 1, 0);
    // imu heat init
    PID_Init_Config_s config = {
        .MaxOut = 800,
        .IntegralLimit = 80,
        .DeadBand = 0,
        .Kp = 400,
        .Ki = 5,
        .Kd = 0,
        .Improve = 0x01
    }; // enable integratiaon limit
    PIDInit(&TempCtrl, &config);

    // noise of accel is relatively big and of high freq,thus lpf is used
    INS.AccelLPF = 0.0085;
    DWT_GetDeltaT(&INS_DWT_Count);
    return &INS.attitude;
}

/* 注意以1kHz的频率运行此任务 */
void INS_Task(void)
{
    static uint32_t count = 0;
    const float gravity[3] = {0, 0, 9.81f};

    // ins update
    if ((count % 1) == 0)
    {
        uint8_t bmi088_has_complete_frame = INS_BMI088_HasCompleteTriggerFrame();

        if (BMI088Acquire(ins_bmi088, &ins_bmi088_data) == 0U)
        {
            if (bmi088_has_complete_frame == 0U)
            {
                /*
                 * 触发模式下偶发没有完整帧是正常等待,但连续长期没有完整帧
                 * 通常意味着DRDY中断没有进入或传感器已经异常,需要进入离线保护。
                 */
                ins_bmi088_wait_frame_count++;
                if (ins_bmi088_wait_frame_count >= INS_BMI088_OFFLINE_FAIL_COUNT)
                {
                    ins_bmi088_fail_count = ins_bmi088_wait_frame_count;
                    if (ins_bmi088_wait_frame_count == INS_BMI088_OFFLINE_FAIL_COUNT ||
                        (ins_bmi088_wait_frame_count % INS_BMI088_OFFLINE_LOG_PERIOD) == 0U)
                    {
                        LOGERROR(
                            "[ins] BMI088 offline, disable temperature control, reason=wait trigger frame timeout, count=%lu",
                            (unsigned long)ins_bmi088_wait_frame_count);
                    }
                }

                count++;
                if ((count % INS_TEMP_CTRL_PERIOD_COUNT) == 0)
                {
                    IMU_Temperature_Ctrl();
                }
                return;
            }

            ins_bmi088_wait_frame_count = 0U;
            INS_BMI088_RecordFailure("acquire failed");
        }
        else
        {
            if (ins_bmi088_fail_count >= INS_BMI088_OFFLINE_FAIL_COUNT)
            {
                LOGWARNING("[ins] BMI088 recovered, enable temperature control");
            }
            ins_bmi088_fail_count = 0U;
            ins_bmi088_wait_frame_count = 0U;
            dt = DWT_GetDeltaT(&INS_DWT_Count);
            dt = float_constrain(dt, INS_EKF_DT_MIN, INS_EKF_DT_MAX);
            t += dt;

            INS.attitude.Accel[IMU_AXIS_X] = ins_bmi088_data.acc[IMU_AXIS_X];
            INS.attitude.Accel[IMU_AXIS_Y] = ins_bmi088_data.acc[IMU_AXIS_Y];
            INS.attitude.Accel[IMU_AXIS_Z] = ins_bmi088_data.acc[IMU_AXIS_Z];
            INS.attitude.Gyro[IMU_AXIS_X] = ins_bmi088_data.gyro[IMU_AXIS_X];
            INS.attitude.Gyro[IMU_AXIS_Y] = ins_bmi088_data.gyro[IMU_AXIS_Y];
            INS.attitude.Gyro[IMU_AXIS_Z] = ins_bmi088_data.gyro[IMU_AXIS_Z];

            // demo function,用于修正安装误差,可以不管,本demo暂时没用
            IMU_Param_Correction(&IMU_Param, INS.attitude.Gyro, INS.attitude.Accel);

            // 计算重力加速度矢量和b系的XY两轴的夹角,可用作功能扩展,本demo暂时没用
            // INS.atanxz = -atan2f(INS.attitude.Accel[IMU_AXIS_X], INS.attitude.Accel[IMU_AXIS_Z]) * 180 / PI;
            // INS.atanyz = atan2f(INS.attitude.Accel[IMU_AXIS_Y], INS.attitude.Accel[IMU_AXIS_Z]) * 180 / PI;

            // 核心函数,EKF更新四元数
            IMU_QuaternionEKF_Update(INS.attitude.Gyro[IMU_AXIS_X], INS.attitude.Gyro[IMU_AXIS_Y],
                                     INS.attitude.Gyro[IMU_AXIS_Z],
                                     INS.attitude.Accel[IMU_AXIS_X], INS.attitude.Accel[IMU_AXIS_Y],
                                     INS.attitude.Accel[IMU_AXIS_Z], dt);

            memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

            // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
            BodyFrameToEarthFrame(xb, INS.xn, INS.q);
            BodyFrameToEarthFrame(yb, INS.yn, INS.q);
            BodyFrameToEarthFrame(zb, INS.zn, INS.q);

            // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
            float gravity_b[3];
            EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
            for (uint8_t i = 0; i < 3; ++i) // 同样过一个低通滤波
            {
                INS.MotionAccel_b[i] = (INS.attitude.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) +
                    INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
            }
            BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n

            INS.attitude.Yaw = QEKF_INS.Yaw;
            INS.attitude.Pitch = QEKF_INS.Pitch;
            INS.attitude.Roll = QEKF_INS.Roll;
            INS.attitude.YawTotalAngle = QEKF_INS.YawTotalAngle;

        }
    }

    // 温度由BMI088驱动约1Hz更新,温控降到10Hz即可,避免在1kHz姿态任务中重复计算PID。
    if ((count % INS_TEMP_CTRL_PERIOD_COUNT) == 0)
    {
        IMU_Temperature_Ctrl();
    }

    if ((count++ % 1000) == 0)
    {
        // 1Hz 可以加入monitor函数,检查IMU是否正常运行/离线
    }
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float* vecBF, float* vecEF, float* q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
        (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
        (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
        (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
        (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
        (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
        (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float* vecEF, float* vecBF, float* q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
        (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
        (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
        (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
        (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
        (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
        (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief 修正IMU安装误差与三轴比例误差
 *
 * @note 安装角矩阵同时作用于陀螺仪和加速度计,但两者的比例误差来源不同,
 *       因此分别使用gyro_scale[]和accel_scale[]。默认比例均为1,不会改变数据。
 *
 * @param param IMU参数
 * @param gyro  角速度
 * @param accel 加速度
 */
static void IMU_Param_Correction(IMU_Param_t* param, float gyro[3], float accel[3])
{
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag)
    {
        cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
        cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
        sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
        sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        gyro_temp[i] = gyro[i] * param->gyro_scale[i];

    gyro[IMU_AXIS_X] = c_11 * gyro_temp[IMU_AXIS_X] +
        c_12 * gyro_temp[IMU_AXIS_Y] +
        c_13 * gyro_temp[IMU_AXIS_Z];
    gyro[IMU_AXIS_Y] = c_21 * gyro_temp[IMU_AXIS_X] +
        c_22 * gyro_temp[IMU_AXIS_Y] +
        c_23 * gyro_temp[IMU_AXIS_Z];
    gyro[IMU_AXIS_Z] = c_31 * gyro_temp[IMU_AXIS_X] +
        c_32 * gyro_temp[IMU_AXIS_Y] +
        c_33 * gyro_temp[IMU_AXIS_Z];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        accel_temp[i] = accel[i] * param->accel_scale[i];

    accel[IMU_AXIS_X] = c_11 * accel_temp[IMU_AXIS_X] +
        c_12 * accel_temp[IMU_AXIS_Y] +
        c_13 * accel_temp[IMU_AXIS_Z];
    accel[IMU_AXIS_Y] = c_21 * accel_temp[IMU_AXIS_X] +
        c_22 * accel_temp[IMU_AXIS_Y] +
        c_23 * accel_temp[IMU_AXIS_Z];
    accel[IMU_AXIS_Z] = c_31 * accel_temp[IMU_AXIS_X] +
        c_32 * accel_temp[IMU_AXIS_Y] +
        c_33 * accel_temp[IMU_AXIS_Z];

    lastYawOffset = param->Yaw;
    lastPitchOffset = param->Pitch;
    lastRollOffset = param->Roll;
}

//------------------------------------functions below are not used in this demo-------------------------------------------------
//----------------------------------you can read them for learning or programming-----------------------------------------------
//----------------------------------they could also be helpful for further design-----------------------------------------------

/**
 * @brief        Update quaternion
 */
void QuaternionUpdate(float* q, float gx, float gy, float gz, float dt)
{
    float qa, qb, qc;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);
}

/**
 * @brief        Convert quaternion to eular angle
 */
void QuaternionToEularAngle(float* q, float* Yaw, float* Pitch, float* Roll)
{
    *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}

/**
 * @brief        Convert eular angle to quaternion
 */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float* q)
{
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    Yaw /= 57.295779513f;
    Pitch /= 57.295779513f;
    Roll /= 57.295779513f;
    cosPitch = arm_cos_f32(Pitch / 2);
    cosYaw = arm_cos_f32(Yaw / 2);
    cosRoll = arm_cos_f32(Roll / 2);
    sinPitch = arm_sin_f32(Pitch / 2);
    sinYaw = arm_sin_f32(Yaw / 2);
    sinRoll = arm_sin_f32(Roll / 2);
    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}
