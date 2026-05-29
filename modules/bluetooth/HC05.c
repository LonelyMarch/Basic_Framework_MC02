#include "HC05.h"
#include "bsp_usart.h"
#include "bsp_log.h"
#include <string.h>

#define HC05_FRAME_MAX_SIZE (HC05_DATASIZE + 4U) // 帧头 + 长度 + 数据 + 校验 + 帧尾

#define HC05_FRAME_HEAD 0xAAU // 帧头
#define HC05_FRAME_END  0x55U // 帧尾

typedef enum
{
    HC05_PARSE_WAIT_HEAD = 0, // 等待帧头
    HC05_PARSE_WAIT_LEN,      // 读取有效载荷长度
    HC05_PARSE_WAIT_PAYLOAD,  // 读取有效载荷
    HC05_PARSE_WAIT_CHECKSUM, // 读取校验字节
    HC05_PARSE_WAIT_END,      // 读取帧尾
} HC05ParseState;

static HC05 hc05_msg;                         // HC05通信数据
static USARTInstance *hc05_usart_instance;    // HC05串口通信实例
static volatile uint8_t hc05_init_flag = 0U;  // HC05初始化标志位
static HC05ParseState hc05_parse_state;       // 字节流解析状态机
static uint8_t hc05_parse_len;                // 当前帧声明的有效载荷长度
static uint8_t hc05_parse_index;              // 当前已经接收的有效载荷字节数
static uint8_t hc05_parse_checksum;           // 当前帧计算得到的校验值
static uint8_t hc05_parse_buffer[HC05_DATASIZE]; // 当前正在接收的有效载荷缓存

static uint8_t HC05CalcChecksum(const uint8_t *data, uint8_t len)
{
    uint8_t checksum = len;

    for (uint8_t i = 0U; i < len; ++i)
    {
        checksum ^= data[i];
    }

    return checksum;
}

static void HC05ResetParser(void)
{
    hc05_parse_state = HC05_PARSE_WAIT_HEAD;
    hc05_parse_len = 0U;
    hc05_parse_index = 0U;
    hc05_parse_checksum = 0U;
}

static void HC05ResyncParser(uint8_t byte)
{
    HC05ResetParser();

    /*
     * 字节流错位时,当前错误字节本身也可能是下一帧帧头。
     * 立即重同步可以避免遇到 0xAA 0xAA ... 这类序列时丢掉第二个帧头。
     */
    if (byte == HC05_FRAME_HEAD)
    {
        hc05_parse_state = HC05_PARSE_WAIT_LEN;
    }
}

static void HC05SaveReceivedPayload(void)
{
    uint32_t primask = __get_PRIMASK();

    /*
     * recv_data可能被其他任务读取。这里的拷贝很短,用临界区保证上层不会读到
     * 一半新数据、一半旧数据的中间状态。
     */
    __disable_irq();
    memcpy(hc05_msg.recv_data, hc05_parse_buffer, hc05_parse_len);
    hc05_msg.recv_len = hc05_parse_len;
    __set_PRIMASK(primask);
}

static void HC05ParseByte(uint8_t byte)
{
    switch (hc05_parse_state)
    {
    case HC05_PARSE_WAIT_HEAD:
        if (byte == HC05_FRAME_HEAD)
        {
            hc05_parse_state = HC05_PARSE_WAIT_LEN;
        }
        break;

    case HC05_PARSE_WAIT_LEN:
        if (byte == 0U || byte > HC05_DATASIZE)
        {
            HC05ResyncParser(byte);
        }
        else
        {
            hc05_parse_len = byte;
            hc05_parse_index = 0U;
            hc05_parse_checksum = byte;
            hc05_parse_state = HC05_PARSE_WAIT_PAYLOAD;
        }
        break;

    case HC05_PARSE_WAIT_PAYLOAD:
        hc05_parse_buffer[hc05_parse_index++] = byte;
        hc05_parse_checksum ^= byte;
        if (hc05_parse_index >= hc05_parse_len)
        {
            hc05_parse_state = HC05_PARSE_WAIT_CHECKSUM;
        }
        break;

    case HC05_PARSE_WAIT_CHECKSUM:
        if (byte == hc05_parse_checksum)
        {
            hc05_parse_state = HC05_PARSE_WAIT_END;
        }
        else
        {
            HC05ResyncParser(byte);
        }
        break;

    case HC05_PARSE_WAIT_END:
        if (byte == HC05_FRAME_END)
        {
            HC05SaveReceivedPayload();
            HC05ResetParser();
        }
        else
        {
            HC05ResyncParser(byte);
        }
        break;

    default:
        HC05ResetParser();
        break;
    }
}

// hc05_usart_instance串口回调函数,由bsp_usart在任务上下文中调用
static void HC05RxCallback(void)
{
    if (hc05_usart_instance == NULL || hc05_usart_instance->recv_buff == NULL)
    {
        return;
    }

    /*
     * 蓝牙串口是字节流,一次USART回调不一定刚好对应一帧协议。
     * 因此这里逐字节喂给状态机,可以处理半帧、粘包以及一次回调内多帧的情况。
     */
    for (uint16_t i = 0U; i < hc05_usart_instance->recv_len; ++i)
    {
        HC05ParseByte(hc05_usart_instance->recv_buff[i]);
    }
}

// HC05串口接收初始化
HC05 *HC05Init(UART_HandleTypeDef *hc05_usart_handle)
{
    USART_Init_Config_s conf = {
        .recv_buff_size = HC05_FRAME_MAX_SIZE,
        .usart_handle = hc05_usart_handle,
        .module_callback = HC05RxCallback,
    };

    memset(&hc05_msg, 0, sizeof(hc05_msg));
    HC05ResetParser();

    hc05_usart_instance = USARTRegister(&conf);
    if (hc05_usart_instance == NULL)
    {
        hc05_init_flag = 0U;
        LOGERROR("[hc05] USART register failed");
        return NULL;
    }

    hc05_init_flag = 1U;
    return &hc05_msg;
}

uint8_t HC05_GetData(uint8_t *data, uint8_t max_len)
{
    uint8_t copy_len;
    uint32_t primask;

    if (data == NULL || max_len == 0U)
    {
        return 0U;
    }

    /*
     * 接收回调和上层任务可能同时访问recv_data。读取时也进入短临界区,
     * 保证上层拿到的是同一帧中的长度和数据。
     */
    primask = __get_PRIMASK();
    __disable_irq();
    copy_len = hc05_msg.recv_len;
    if (copy_len > max_len)
    {
        copy_len = max_len;
    }
    memcpy(data, hc05_msg.recv_data, copy_len);
    hc05_msg.recv_len = 0U;
    __set_PRIMASK(primask);

    return copy_len;
}

// HC05串口发送函数，一次最多发送HC05_DATASIZE个数据
HAL_StatusTypeDef HC05_SendData(const uint8_t *data, uint8_t data_num)
{
    HAL_StatusTypeDef status;
    uint8_t frame[HC05_FRAME_MAX_SIZE];
    uint16_t frame_len;
    uint32_t primask;

    if (hc05_init_flag == 0U || data == NULL || data_num == 0U || data_num > HC05_DATASIZE)
    {
        LOGWARNING("[hc05] send argument invalid");
        return HAL_ERROR;
    }

    // 发送数据中加入帧头、长度、校验和帧尾,避免短帧时帧尾没有被发送出去。
    frame[0] = HC05_FRAME_HEAD;
    frame[1] = data_num;

    for (uint8_t i = 0U; i < data_num; ++i)
    {
        frame[2U + i] = data[i];
    }

    frame[2U + data_num] = HC05CalcChecksum(data, data_num);
    frame[3U + data_num] = HC05_FRAME_END;
    frame_len = (uint16_t)data_num + 4U;

    /*
     * send_data用于调试查看最近一帧发送内容。真正发给USART的是局部frame,
     * USARTSend会立即复制到USART实例自己的TX缓冲,因此这里不会悬空。
     */
    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(hc05_msg.send_data, frame, frame_len);
    __set_PRIMASK(primask);

    // 发送数据
    status = USARTSend(hc05_usart_instance, frame, frame_len, USART_TRANSFER_IT);
    if (status != HAL_OK)
    {
        LOGWARNING("[hc05] send failed, status [%d]", status);
    }

    return status;
}
