#include "bsp_pwm.h"
#include "buzzer.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "string.h"

static PWMInstance *buzzer;
static BuzzerInstance buzzer_pool[BUZZER_DEVICE_CNT];        // 报警实例静态池,避免FreeRTOS运行期申请堆内存
static BuzzerInstance *buzzer_list[BUZZER_DEVICE_CNT] = {0}; // 按报警等级索引实例,数值越小优先级越高

static float BuzzerLimitLoudness(float loudness)
{
    if (loudness <= 0.0f)
    {
        return 0.0f;
    }

    if (loudness >= 1.0f)
    {
        return 1.0f;
    }

    return loudness;
}

static uint32_t BuzzerGetNoteFreq(BuzzerNote_e note)
{
    switch (note)
    {
    case BUZZER_NOTE_DO:
        return BUZZER_NOTE_DO_FREQ_HZ;
    case BUZZER_NOTE_RE:
        return BUZZER_NOTE_RE_FREQ_HZ;
    case BUZZER_NOTE_MI:
        return BUZZER_NOTE_MI_FREQ_HZ;
    case BUZZER_NOTE_FA:
        return BUZZER_NOTE_FA_FREQ_HZ;
    case BUZZER_NOTE_SO:
        return BUZZER_NOTE_SO_FREQ_HZ;
    case BUZZER_NOTE_LA:
        return BUZZER_NOTE_LA_FREQ_HZ;
    case BUZZER_NOTE_SI:
        return BUZZER_NOTE_SI_FREQ_HZ;
    default:
        return 0U;
    }
}

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

BuzzerInstance *BuzzerRegister(Buzzer_config_s *config)
{
    if (config == NULL)
    {
        LOGERROR("[buzzer] BuzzerRegister received null config");
        return NULL;
    }

    if ((uint32_t)config->alarm_level >= (uint32_t)BUZZER_DEVICE_CNT) // 报警等级会直接作为数组下标,必须限制在0~BUZZER_DEVICE_CNT-1
    {
        LOGERROR("[buzzer] alarm level exceeds buzzer list range");
        return NULL;
    }

    if (buzzer_list[config->alarm_level] != NULL) // 同一报警等级只允许注册一次,避免覆盖旧实例导致内存泄漏
    {
        LOGERROR("[buzzer] duplicate alarm level register");
        return NULL;
    }

    if ((uint32_t)config->note >= (uint32_t)BUZZER_NOTE_COUNT) // 非法音名不注册,避免高优先级错误配置屏蔽低优先级报警
    {
        LOGERROR("[buzzer] invalid note config");
        return NULL;
    }

    BuzzerInstance *buzzer_temp = &buzzer_pool[config->alarm_level];
    memset(buzzer_temp, 0, sizeof(BuzzerInstance));

    buzzer_temp->alarm_level = config->alarm_level;
    buzzer_temp->loudness = BuzzerLimitLoudness(config->loudness); // loudness语义为PWM占空比,注册时限制到0~1
    buzzer_temp->note = config->note;
    buzzer_temp->alarm_state = ALARM_OFF;

    buzzer_list[config->alarm_level] = buzzer_temp;
    return buzzer_temp;
}

void AlarmSetStatus(BuzzerInstance *buzzer, AlarmState_e state)
{
    if (buzzer == NULL) // 上层可能在注册失败后仍调用状态设置,这里直接忽略避免空指针访问
    {
        return;
    }

    if ((uint32_t)state > (uint32_t)ALARM_ON)
    {
        LOGERROR("[buzzer] invalid alarm state");
        return;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    buzzer->alarm_state = state;
    __set_PRIMASK(primask);
}

void BuzzerTask()
{
    if (buzzer == NULL) // 底层PWM尚未初始化或初始化失败时,任务直接返回
    {
        return;
    }

    BuzzerInstance *buzz;
    uint8_t has_active_alarm = 0U; // 用于判断本轮是否找到正在响的报警,没有则关闭蜂鸣器

    for (size_t i = 0; i < BUZZER_DEVICE_CNT; ++i)
    {
        buzz = buzzer_list[i];
        AlarmState_e alarm_state;
        BuzzerNote_e note;
        float loudness;
        uint32_t note_freq;

        if (buzz == NULL) // 允许只注册部分报警等级,未注册的位置跳过即可
        {
            continue;
        }

        if ((uint32_t)buzz->alarm_level > (uint32_t)ALARM_LEVEL_LOW) // 防御性检查,正常注册流程下不会出现非法等级
        {
            continue;
        }

        // 报警状态可能被其他任务修改,这里用极短临界区取一次快照,避免读到一半被更新。
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        alarm_state = buzz->alarm_state;
        note = buzz->note;
        loudness = buzz->loudness;
        __set_PRIMASK(primask);

        if (alarm_state == ALARM_OFF)
        {
            continue; // 当前等级未触发报警,继续查找更低优先级的报警
        }

        note_freq = BuzzerGetNoteFreq(note);
        if (note_freq == 0U)
        {
            continue; // 防御性处理: 若实例数据被破坏,跳过该等级,不屏蔽后续低优先级报警
        }

        has_active_alarm = 1U; // 从高到低扫描,找到第一个开启的报警就播放并退出
        PWMSetPeriod(buzzer, 1.0f / (float)note_freq);
        PWMSetDutyRatio(buzzer, loudness);

        break;
    }

    if (has_active_alarm == 0U) // 本轮没有任何开启的报警,确保蜂鸣器保持关闭
    {
        PWMSetDutyRatio(buzzer, 0.0f);
    }
}
