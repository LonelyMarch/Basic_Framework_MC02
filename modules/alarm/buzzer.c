#include "bsp_pwm.h"
#include "buzzer.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "stdlib.h"
#include "string.h"

static PWMInstance *buzzer;
// static uint8_t idx;
static BuzzzerInstance *buzzer_list[BUZZER_DEVICE_CNT] = {0};

/**
 * @brief 蜂鸣器初始化
 *
 */
void BuzzerInit()
{
    PWM_Init_Config_s buzzer_config = {
        .htim = &htim12,
        .channel = TIM_CHANNEL_2,
        .dutyratio = 0,
        .period = 0.001,
    };
    buzzer = PWMRegister(&buzzer_config);
}

BuzzzerInstance *BuzzerRegister(Buzzer_config_s *config)
{
    if (config == NULL)
    {
        LOGERROR("[buzzer] BuzzerRegister received null config");
        return NULL;
    }

    if (config->alarm_level >= BUZZER_DEVICE_CNT) // 报警等级会直接作为数组下标,必须限制在0~BUZZER_DEVICE_CNT-1
    {
        LOGERROR("[buzzer] alarm level exceeds buzzer list range");
        return NULL;
    }

    if (buzzer_list[config->alarm_level] != NULL) // 同一报警等级只允许注册一次,避免覆盖旧实例导致内存泄漏
    {
        LOGERROR("[buzzer] duplicate alarm level register");
        return NULL;
    }

    BuzzzerInstance *buzzer_temp = (BuzzzerInstance *)malloc(sizeof(BuzzzerInstance));
    if (buzzer_temp == NULL)
    {
        LOGERROR("[buzzer] buzzer instance malloc failed");
        return NULL;
    }

    memset(buzzer_temp, 0, sizeof(BuzzzerInstance));

    buzzer_temp->alarm_level = config->alarm_level;
    buzzer_temp->loudness = config->loudness;
    buzzer_temp->octave = config->octave;
    buzzer_temp->alarm_state = ALARM_OFF;

    buzzer_list[config->alarm_level] = buzzer_temp;
    return buzzer_temp;
}

void AlarmSetStatus(BuzzzerInstance *buzzer, AlarmState_e state)
{
    if (buzzer == NULL) // 上层可能在注册失败后仍调用状态设置,这里直接忽略避免空指针访问
    {
        return;
    }

    buzzer->alarm_state = state;
}

void BuzzerTask()
{
    if (buzzer == NULL) // 底层PWM尚未初始化或初始化失败时,任务直接返回
    {
        return;
    }

    BuzzzerInstance *buzz;
    uint8_t has_active_alarm = 0U; // 用于判断本轮是否找到正在响的报警,没有则关闭蜂鸣器

    for (size_t i = 0; i < BUZZER_DEVICE_CNT; ++i)
    {
        buzz = buzzer_list[i];

        if (buzz == NULL) // 允许只注册部分报警等级,未注册的位置跳过即可
        {
            continue;
        }

        if (buzz->alarm_level > ALARM_LEVEL_LOW) // 防御性检查,正常注册流程下不会出现非法等级
        {
            continue;
        }

        if (buzz->alarm_state == ALARM_OFF)
        {
            continue; // 当前等级未触发报警,继续查找更低优先级的报警
        }

        has_active_alarm = 1U; // 从高到低扫描,找到第一个开启的报警就播放并退出
        PWMSetDutyRatio(buzzer, buzz->loudness);
        switch (buzz->octave)
        {
        case OCTAVE_1:
            PWMSetPeriod(buzzer, (float)1 / DoFreq);
            break;
        case OCTAVE_2:
            PWMSetPeriod(buzzer, (float)1 / ReFreq);
            break;
        case OCTAVE_3:
            PWMSetPeriod(buzzer, (float)1 / MiFreq);
            break;
        case OCTAVE_4:
            PWMSetPeriod(buzzer, (float)1 / FaFreq);
            break;
        case OCTAVE_5:
            PWMSetPeriod(buzzer, (float)1 / SoFreq);
            break;
        case OCTAVE_6:
            PWMSetPeriod(buzzer, (float)1 / LaFreq);
            break;
        case OCTAVE_7:
            PWMSetPeriod(buzzer, (float)1 / SiFreq);
            break;
        default:
            PWMSetDutyRatio(buzzer, 0.0f); // 非法音阶不输出声音,避免用上一轮频率继续响
            break;
        }

        break;
    }

    if (has_active_alarm == 0U) // 本轮没有任何开启的报警,确保蜂鸣器保持关闭
    {
        PWMSetDutyRatio(buzzer, 0.0f);
    }
}
