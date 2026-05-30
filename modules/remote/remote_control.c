#include "remote_control.h"
#include "string.h"
#include "bsp_usart.h"
#include "stdlib.h"
#include "daemon.h"
#include "bsp_log.h"

#define REMOTE_CONTROL_FRAME_SIZE 18u // 遥控器接收的buffer大小

// 遥控器数据
static RC_ctrl_t rc_ctrl[2];     //[0]:当前数据RC_TEMP,[1]:上一次的数据RC_LAST.用于按键持续按下和切换的判断
static uint8_t rc_init_flag = 0; // 遥控器初始化标志位
static uint8_t rc_lost_logged;    // 离线日志标志,避免daemon离线期间周期性重复刷屏
static uint8_t rc_frame_received; // 至少收到过一帧合法长度数据后,才认为遥控器具备在线基础

// 遥控器拥有的串口实例,因为遥控器是单例,所以这里只有一个,就不封装了
static USARTInstance *rc_usart_instance;
static DaemonInstance *rc_daemon_instance;

static int16_t RemoteControlReadInt16LE(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static int16_t RemoteControlLimitChannel(int16_t channel)
{
    if (abs(channel) > REMOTE_CONTROL_CHANNEL_VALID_LIMIT)
        return 0;

    return channel;
}

static uint8_t RemoteControlIsValidChannel(uint16_t channel)
{
    return (channel >= RC_CH_VALUE_MIN) && (channel <= RC_CH_VALUE_MAX);
}

static uint8_t RemoteControlIsValidSwitch(uint8_t sw)
{
    return (sw == RC_SW_UP) || (sw == RC_SW_MID) || (sw == RC_SW_DOWN);
}

/**
 * @brief 检查DBUS帧的关键字段是否合法。
 *
 * @note 只有合法帧才允许刷新daemon。这样串口噪声或错位数据不会把遥控器误判为在线。
 */
static uint8_t RemoteControlFrameIsValid(const uint8_t *sbus_buf)
{
    uint16_t ch0;
    uint16_t ch1;
    uint16_t ch2;
    uint16_t ch3;
    uint16_t dial;
    uint8_t switch_right;
    uint8_t switch_left;

    if (sbus_buf == NULL)
        return 0;

    ch0 = (uint16_t)((sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff);
    ch1 = (uint16_t)(((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff);
    ch2 = (uint16_t)(((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff);
    ch3 = (uint16_t)(((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff);
    dial = (uint16_t)((sbus_buf[16] | (sbus_buf[17] << 8)) & 0x07ff);
    switch_right = (uint8_t)((sbus_buf[5] >> 4) & 0x03);
    switch_left = (uint8_t)(((sbus_buf[5] >> 4) & 0x0c) >> 2);

    if (RemoteControlIsValidChannel(ch0) == 0 || RemoteControlIsValidChannel(ch1) == 0 ||
        RemoteControlIsValidChannel(ch2) == 0 || RemoteControlIsValidChannel(ch3) == 0 ||
        RemoteControlIsValidChannel(dial) == 0)
    {
        return 0;
    }

    if (RemoteControlIsValidSwitch(switch_right) == 0 || RemoteControlIsValidSwitch(switch_left) == 0)
        return 0;

    if (sbus_buf[12] > 1U || sbus_buf[13] > 1U)
        return 0;

    return 1;
}

/**
 * @brief 遥控器数据解析
 *
 * @param sbus_buf 接收buffer
 */
static void sbus_to_rc(const uint8_t *sbus_buf)
{
    RC_ctrl_t last;
    RC_ctrl_t next;
    uint32_t primask;

    if (sbus_buf == NULL)
        return;

    /* 先快速复制上一帧快照,后续解析都在局部变量中完成,减少关中断时间。 */
    primask = __get_PRIMASK();
    __disable_irq();
    last = rc_ctrl[RC_TEMP];
    __set_PRIMASK(primask);

    next = last;

    // 摇杆,直接解算时减去偏置
    next.rc.rocker_r_ = ((sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff) - RC_CH_VALUE_OFFSET;                              //!< Channel 0
    next.rc.rocker_r1 = (((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 1
    next.rc.rocker_l_ = (((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff) - RC_CH_VALUE_OFFSET; //!< Channel 2
    next.rc.rocker_l1 = (((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 3
    next.rc.dial = ((sbus_buf[16] | (sbus_buf[17] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET;                                 // 左侧拨轮
    next.rc.rocker_l_ = RemoteControlLimitChannel(next.rc.rocker_l_);
    next.rc.rocker_l1 = RemoteControlLimitChannel(next.rc.rocker_l1);
    next.rc.rocker_r_ = RemoteControlLimitChannel(next.rc.rocker_r_);
    next.rc.rocker_r1 = RemoteControlLimitChannel(next.rc.rocker_r1);
    next.rc.dial = RemoteControlLimitChannel(next.rc.dial);
    // 开关,0左1右
    next.rc.switch_right = ((sbus_buf[5] >> 4) & 0x0003);     //!< Switch right
    next.rc.switch_left = ((sbus_buf[5] >> 4) & 0x000C) >> 2; //!< Switch left

    // 鼠标解析
    next.mouse.x = RemoteControlReadInt16LE(&sbus_buf[6]); //!< Mouse X axis
    next.mouse.y = RemoteControlReadInt16LE(&sbus_buf[8]); //!< Mouse Y axis
    next.mouse.press_l = sbus_buf[12];                     //!< Mouse Left Is Press ?
    next.mouse.press_r = sbus_buf[13];                     //!< Mouse Right Is Press ?

    /*
     * 键盘按键为16bit小端位图,直接写union中的keys字段即可。
     * 避免使用指针强转写位域对象,减少别名规则和静态分析告警风险。
     */
    next.key[RC_KEY_PRESS].keys = (uint16_t)(sbus_buf[14] | (sbus_buf[15] << 8));
    if (next.key[RC_KEY_PRESS].ctrl) // ctrl键按下
        next.key[RC_KEY_PRESS_WITH_CTRL] = next.key[RC_KEY_PRESS];
    else
        memset(&next.key[RC_KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    if (next.key[RC_KEY_PRESS].shift) // shift键按下
        next.key[RC_KEY_PRESS_WITH_SHIFT] = next.key[RC_KEY_PRESS];
    else
        memset(&next.key[RC_KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));

    uint16_t key_now = next.key[RC_KEY_PRESS].keys,                       // 当前按键是否按下
        key_last = last.key[RC_KEY_PRESS].keys,                           // 上一次按键是否按下
        key_with_ctrl = next.key[RC_KEY_PRESS_WITH_CTRL].keys,            // 当前ctrl组合键是否按下
        key_with_shift = next.key[RC_KEY_PRESS_WITH_SHIFT].keys,          //  当前shift组合键是否按下
        key_last_with_ctrl = last.key[RC_KEY_PRESS_WITH_CTRL].keys,       // 上一次ctrl组合键是否按下
        key_last_with_shift = last.key[RC_KEY_PRESS_WITH_SHIFT].keys;     // 上一次shift组合键是否按下

    for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
    {
        if (i == RC_KEY_SHIFT || i == RC_KEY_CTRL) // ctrl和shift位只作为组合键修饰,不单独统计边沿
            continue;
        // 如果当前按键按下,上一次按键没有按下,且ctrl和shift组合键没有按下,则按键按下计数加1(检测到上升沿)
        if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
            next.key_count[RC_KEY_PRESS][i]++;
        // 当前ctrl组合键按下,上一次ctrl组合键没有按下,则ctrl组合键按下计数加1(检测到上升沿)
        if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
            next.key_count[RC_KEY_PRESS_WITH_CTRL][i]++;
        // 当前shift组合键按下,上一次shift组合键没有按下,则shift组合键按下计数加1(检测到上升沿)
        if ((key_with_shift & j) && !(key_last_with_shift & j))
            next.key_count[RC_KEY_PRESS_WITH_SHIFT][i]++;
    }

    /*
     * 最后一次性提交解析结果。
     * RC_TEMP保存当前帧,RC_LAST保存上一帧,上层通过RemoteControlGet()读取到的是一致快照。
     */
    primask = __get_PRIMASK();
    __disable_irq();
    rc_ctrl[RC_LAST] = last;
    rc_ctrl[RC_TEMP] = next;
    rc_frame_received = 1;
    __set_PRIMASK(primask);
}

/**
 * @brief 对sbus_to_rc的简单封装,用于注册到bsp_usart的回调函数中
 *
 */
static void RemoteControlRxCallback(void)
{
    if (rc_usart_instance == NULL || rc_daemon_instance == NULL || rc_usart_instance->recv_len != REMOTE_CONTROL_FRAME_SIZE)
        return;
    if (RemoteControlFrameIsValid(rc_usart_instance->recv_buff) == 0)
        return;

    DaemonReload(rc_daemon_instance);         // 合法帧先喂狗,避免和daemon离线回调交错时误清空快照
    sbus_to_rc(rc_usart_instance->recv_buff); // 进行协议解析
    rc_lost_logged = 0;                       // 收到合法长度帧,认为遥控器已重新在线
}

/**
 * @brief 遥控器离线的回调函数,注册到守护进程中,串口掉线时调用
 *
 */
static void RCLostCallback(void *id)
{
    uint32_t primask;

    (void)id;

    primask = __get_PRIMASK();
    __disable_irq();
    /*
     * 如果合法帧刚好在离线回调执行前到来并重载了daemon,则本次离线回调已经过期。
     * 这个判断和清空快照放在同一个短临界区内,避免合法帧提交与离线清空交错。
     */
    if (rc_daemon_instance != NULL && rc_daemon_instance->temp_count > 0U)
    {
        __set_PRIMASK(primask);
        return;
    }

    memset(rc_ctrl, 0, sizeof(rc_ctrl)); // 清空遥控器数据
    rc_frame_received = 0;
    __set_PRIMASK(primask);

    if (rc_lost_logged == 0)
    {
        rc_lost_logged = 1;
        if (rc_usart_instance != NULL)
            USARTServiceInit(rc_usart_instance); // 首次确认离线时尝试重新启动接收
        LOGWARNING("[rc] remote control lost");
    }
}

RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = RemoteControlRxCallback;
    conf.usart_handle = rc_usart_handle;
    conf.recv_buff_size = REMOTE_CONTROL_FRAME_SIZE;
    rc_usart_instance = USARTRegister(&conf);
    if (rc_usart_instance == NULL)
    {
        LOGERROR("[rc] USART register failed");
        return NULL;
    }

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 10, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
        .callback = RCLostCallback,
        .owner_id = NULL, // 只有1个遥控器,不需要owner_id
    };
    rc_daemon_instance = DaemonRegister(&daemon_conf);
    if (rc_daemon_instance == NULL)
    {
        LOGERROR("[rc] daemon register failed");
        return NULL;
    }

    rc_init_flag = 1;
    return rc_ctrl;
}

uint8_t RemoteControlGet(RC_ctrl_t *rc_snapshot)
{
    uint32_t primask;

    if (rc_init_flag == 0 || rc_snapshot == NULL)
        return 0;

    /*
     * 调用方传入大小为2的RC_ctrl_t数组。
     * 复制TEMP和LAST两个槽位,保持原有按键边沿计数/当前值访问习惯。
     */
    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(rc_snapshot, rc_ctrl, sizeof(rc_ctrl));
    __set_PRIMASK(primask);

    return 1;
}

uint8_t RemoteControlIsOnline(void)
{
    DaemonInstance *daemon;
    uint8_t frame_received;
    uint8_t init_flag;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    init_flag = rc_init_flag;
    frame_received = rc_frame_received;
    daemon = rc_daemon_instance;
    __set_PRIMASK(primask);

    if (init_flag == 0 || frame_received == 0 || daemon == NULL)
        return 0;

    return DaemonIsOnline(daemon);
}
