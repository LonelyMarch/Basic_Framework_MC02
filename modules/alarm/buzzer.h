#ifndef BUZZER_H
#define BUZZER_H
#include "bsp_pwm.h"
#define BUZZER_DEVICE_CNT 5

#define BUZZER_NOTE_DO_FREQ_HZ 523U
#define BUZZER_NOTE_RE_FREQ_HZ 587U
#define BUZZER_NOTE_MI_FREQ_HZ 659U
#define BUZZER_NOTE_FA_FREQ_HZ 698U
#define BUZZER_NOTE_SO_FREQ_HZ 784U
#define BUZZER_NOTE_LA_FREQ_HZ 880U
#define BUZZER_NOTE_SI_FREQ_HZ 988U

typedef enum
{
    BUZZER_NOTE_DO = 0,
    BUZZER_NOTE_RE,
    BUZZER_NOTE_MI,
    BUZZER_NOTE_FA,
    BUZZER_NOTE_SO,
    BUZZER_NOTE_LA,
    BUZZER_NOTE_SI,
    BUZZER_NOTE_COUNT,
} BuzzerNote_e;

typedef enum
{
    ALARM_LEVEL_HIGH = 0,
    ALARM_LEVEL_ABOVE_MEDIUM,
    ALARM_LEVEL_MEDIUM,
    ALARM_LEVEL_BELOW_MEDIUM,
    ALARM_LEVEL_LOW,
} AlarmLevel_e;

typedef enum
{
    ALARM_OFF = 0,
    ALARM_ON,
} AlarmState_e;

typedef struct
{
    AlarmLevel_e alarm_level;
    BuzzerNote_e note;
    float loudness;
} Buzzer_config_s;

typedef struct
{
    float loudness;
    BuzzerNote_e note;
    AlarmLevel_e alarm_level;
    AlarmState_e alarm_state;
} BuzzerInstance;


void BuzzerInit();


void BuzzerTask();


BuzzerInstance* BuzzerRegister(Buzzer_config_s* config);


void AlarmSetStatus(BuzzerInstance* buzzer, AlarmState_e state);
#endif // !BUZZER_H
