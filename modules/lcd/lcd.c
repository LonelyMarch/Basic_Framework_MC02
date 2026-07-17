#include "lcd.h"

#include "bsp_gpio.h"
#include "bsp_log.h"
#include "cmsis_os2.h"
#include "lcd_font.h"
#include "main.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#if (LCD_TX_CHUNK_SIZE < 2U) || ((LCD_TX_CHUNK_SIZE % 2U) != 0U)
#error "LCD_TX_CHUNK_SIZE must be an even value and no less than 2."
#endif

#define LCD_CMD_SLEEP_OUT       0x11U
#define LCD_CMD_DISPLAY_ON      0x29U
#define LCD_CMD_DISPLAY_OFF     0x28U
#define LCD_CMD_MEMORY_ACCESS   0x36U
#define LCD_CMD_PIXEL_FORMAT    0x3AU
#define LCD_CMD_COLUMN_ADDRESS  0x2AU
#define LCD_CMD_ROW_ADDRESS     0x2BU
#define LCD_CMD_MEMORY_WRITE    0x2CU

#define LCD_ASCII_FIRST_CHAR    ' '
#define LCD_ASCII_LAST_CHAR     '~'
#define LCD_PRINTF_BUFFER_SIZE  64U
#define LCD_TASK_STACK_SIZE     (1024U * 4U)
#define LCD_FLOAT_INT_WIDTH_MAX 10U
#define LCD_FLOAT_DECIMAL_MAX   6U

#define LCD_CHECK_WRITE(expr)     \
    do                            \
    {                             \
        if (status == HAL_OK)     \
        {                         \
            status = (expr);      \
        }                         \
    } while (0)

typedef enum
{
    LCD_CMD_CLEAR = 0U,
    LCD_CMD_FILL,
    LCD_CMD_DRAW_POINT,
    LCD_CMD_DRAW_LINE,
    LCD_CMD_DRAW_RECTANGLE,
    LCD_CMD_PRINTF,
} LCDCommandType_e;

typedef struct
{
    LCDCommandType_e type;
    uint16_t x1;
    uint16_t y1;
    uint16_t x2;
    uint16_t y2;
    uint16_t color;
    uint16_t bg_color;
    uint8_t sizey;
    uint8_t mode;
    char text[LCD_PRINTF_BUFFER_SIZE];
} LCDCommand_s;

static SPIInstance* lcd_spi;
static GPIOInstance* lcd_dc;
static GPIOInstance* lcd_res;
static GPIOInstance* lcd_blk;
static osMutexId_t lcd_mutex;
static osMessageQueueId_t lcd_cmd_queue;
static osThreadId_t lcd_task_handle;
static uint8_t lcd_is_ready;
static uint8_t lcd_tx_buffer[LCD_TX_CHUNK_SIZE];


static HAL_StatusTypeDef LCDEnsureHardware(void);


static HAL_StatusTypeDef LCDPostCommand(const LCDCommand_s* cmd);


static void LCDTask(void* argument);


static HAL_StatusTypeDef LCDWriteCommandUnlocked(uint8_t cmd);


static HAL_StatusTypeDef LCDWriteData8Unlocked(uint8_t data);


static HAL_StatusTypeDef LCDWriteData16Unlocked(uint16_t data);


static HAL_StatusTypeDef LCDWriteDataBufferUnlocked(const uint8_t* data, uint32_t len);


static void LCDAddressSetUnlocked(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);


static void LCDFillUnlocked(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color);


static void LCDDrawAxisLineUnlocked(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);


static void LCDDrawPointUnlocked(uint16_t x, uint16_t y, uint16_t color);


static void LCDShowCharUnlocked(uint16_t x, uint16_t y, uint8_t num, uint16_t fc, uint16_t bc, uint8_t sizey,
                                uint8_t mode);


static void LCDShowStringUnlocked(uint16_t x, uint16_t y, const uint8_t* p, uint16_t fc, uint16_t bc, uint8_t sizey,
                                  uint8_t mode);


/**
 * @brief 根据调度器状态选择延时方式。
 *
 * LCD 初始化既可能发生在 RobotInit() 阶段，也可能发生在任务里。
 * 调度器启动前只能使用 HAL_Delay()；调度器启动后使用 osDelay()，避免长时间忙等。
 */
static void LCDDelay(uint32_t ms)
{
    if (osKernelGetState() == osKernelRunning)
    {
        (void)osDelay(ms);
    }
    else
    {
        HAL_Delay(ms);
    }
}

/**
 * @brief 创建 LCD 模块级互斥锁。
 *
 * SPI BSP 只能保证同一条 SPI 总线不会被多个设备同时使用。
 * LCD 的“写命令 + 写参数 + 写显存”必须作为一个连续事务看待，
 * 因此 LCD 模块自身还需要一把锁，防止多个任务的绘图命令互相穿插。
 */
static HAL_StatusTypeDef LCDEnsureMutex(void)
{
    static const osMutexAttr_t lcd_mutex_attr = {
        .name = "lcd_mutex",
        .attr_bits = osMutexRecursive | osMutexPrioInherit,
    };
    osKernelState_t kernel_state = osKernelGetState();

    if (lcd_mutex != NULL)
    {
        return HAL_OK;
    }

    if (kernel_state == osKernelInactive || kernel_state == osKernelError)
    {
        return HAL_OK;
    }

    lcd_mutex = osMutexNew(&lcd_mutex_attr);
    if (lcd_mutex == NULL)
    {
        LOGERROR("[lcd] LCD互斥锁创建失败");
        return HAL_ERROR;
    }

    return HAL_OK;
}

static HAL_StatusTypeDef LCDLock(void)
{
    if (LCDEnsureMutex() != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (lcd_mutex != NULL && osKernelGetState() == osKernelRunning)
    {
        if (osMutexAcquire(lcd_mutex, osWaitForever) != osOK)
        {
            LOGERROR("[lcd] LCD互斥锁获取失败");
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

static void LCDUnlock(void)
{
    if (lcd_mutex != NULL && osKernelGetState() == osKernelRunning)
    {
        (void)osMutexRelease(lcd_mutex);
    }
}

/**
 * @brief 将绘图命令投递给 LCD 后台任务。
 *
 * 异步接口只适合在 FreeRTOS 调度器启动后使用。队列满时直接返回 HAL_BUSY，
 * 避免调试显示反向阻塞控制、IMU、CAN 等高实时性任务。
 */
static HAL_StatusTypeDef LCDPostCommand(const LCDCommand_s* cmd)
{
    if (cmd == NULL || lcd_cmd_queue == NULL || osKernelGetState() != osKernelRunning)
    {
        return HAL_ERROR;
    }

    if (lcd_is_ready == 0U)
    {
        return HAL_ERROR;
    }

    if (osMessageQueuePut(lcd_cmd_queue, cmd, 0U, 0U) != osOK)
    {
        return HAL_BUSY;
    }

    return HAL_OK;
}

/**
 * @brief LCD 低优先级绘图任务。
 *
 * 其他任务通过 LCD_AsyncXXX() 投递命令，本任务串行执行真正的 SPI 刷屏。
 * 这样大面积刷新只会占用 LCDTask，不会把调用者任务卡在 SPI 传输上。
 */
static void LCDTask(void* argument)
{
    LCDCommand_s cmd;

    (void)argument;

    for (;;)
    {
        if (osMessageQueueGet(lcd_cmd_queue, &cmd, NULL, osWaitForever) != osOK)
        {
            continue;
        }

        switch (cmd.type)
        {
        case LCD_CMD_CLEAR:
            LCD_Clear(cmd.color);
            break;
        case LCD_CMD_FILL:
            LCD_Fill(cmd.x1, cmd.y1, cmd.x2, cmd.y2, cmd.color);
            break;
        case LCD_CMD_DRAW_POINT:
            LCD_DrawPoint(cmd.x1, cmd.y1, cmd.color);
            break;
        case LCD_CMD_DRAW_LINE:
            LCD_DrawLine(cmd.x1, cmd.y1, cmd.x2, cmd.y2, cmd.color);
            break;
        case LCD_CMD_DRAW_RECTANGLE:
            LCD_DrawRectangle(cmd.x1, cmd.y1, cmd.x2, cmd.y2, cmd.color);
            break;
        case LCD_CMD_PRINTF:
            LCD_ShowString(cmd.x1,
                           cmd.y1,
                           (const uint8_t*)cmd.text,
                           cmd.color,
                           cmd.bg_color,
                           cmd.sizey,
                           cmd.mode);
            break;
        default:
            LOGWARNING("[lcd] 未知异步绘图命令: %u", (unsigned int)cmd.type);
            break;
        }
    }
}

static void LCDGpioSet(GPIOInstance* gpio)
{
    if (gpio != NULL)
    {
        GPIOSet(gpio);
    }
}

static void LCDGpioReset(GPIOInstance* gpio)
{
    if (gpio != NULL)
    {
        GPIOReset(gpio);
    }
}

/**
 * @brief 注册 SPI1 和 LCD 控制 GPIO。
 *
 * 原理图连接关系：
 * - SPI1_CS  -> PE15
 * - SPI1_SCK -> PB03
 * - SPI1_MOSI-> PD07
 * - LCD_DC   -> PD10
 * - LCD_RES  -> PB11
 * - LCD_BLK  -> PB10
 */
static HAL_StatusTypeDef LCDEnsureHardware(void)
{
    if (lcd_spi == NULL)
    {
        SPI_Init_Config_s spi_config = {
            .spi_handle = &hspi1,
            .GPIOx = LCD_CS_GPIO_Port,
            .cs_pin = LCD_CS_Pin,
            .spi_work_mode = LCD_SPI_WORK_MODE,
            .callback = NULL,
            .id = NULL,
        };

        lcd_spi = SPIRegister(&spi_config);
        if (lcd_spi == NULL)
        {
            LOGERROR("[lcd] SPI1 LCD实例注册失败");
            return HAL_ERROR;
        }
    }

    if (lcd_dc == NULL)
    {
        GPIO_Init_Config_s dc_config = {
            .GPIOx = LCD_DC_GPIO_Port,
            .GPIO_Pin = LCD_DC_Pin,
            .pin_state = GPIO_PIN_RESET,
            .exti_mode = GPIO_EXTI_MODE_NONE,
            .gpio_model_callback = NULL,
            .id = NULL,
        };
        lcd_dc = GPIORegister(&dc_config);
    }

    if (lcd_res == NULL)
    {
        GPIO_Init_Config_s res_config = {
            .GPIOx = LCD_RES_GPIO_Port,
            .GPIO_Pin = LCD_RES_Pin,
            .pin_state = GPIO_PIN_RESET,
            .exti_mode = GPIO_EXTI_MODE_NONE,
            .gpio_model_callback = NULL,
            .id = NULL,
        };
        lcd_res = GPIORegister(&res_config);
    }

    if (lcd_blk == NULL)
    {
        GPIO_Init_Config_s blk_config = {
            .GPIOx = LCD_BLK_GPIO_Port,
            .GPIO_Pin = LCD_BLK_Pin,
            .pin_state = GPIO_PIN_RESET,
            .exti_mode = GPIO_EXTI_MODE_NONE,
            .gpio_model_callback = NULL,
            .id = NULL,
        };
        lcd_blk = GPIORegister(&blk_config);
    }

    if (lcd_dc == NULL || lcd_res == NULL || lcd_blk == NULL)
    {
        LOGERROR("[lcd] LCD GPIO实例注册失败");
        return HAL_ERROR;
    }

    return HAL_OK;
}

static HAL_StatusTypeDef LCDSpiTransmitUnlocked(const uint8_t* data, uint16_t len)
{
    HAL_StatusTypeDef status;

    if (data == NULL || len == 0U)
    {
        return HAL_ERROR;
    }

    status = SPITransmit(lcd_spi, (uint8_t*)data, len);
    if (status != HAL_OK)
    {
        LOGERROR("[lcd] SPI写入失败: status=%u", (unsigned int)status);
    }

    return status;
}

static HAL_StatusTypeDef LCDWriteCommandUnlocked(uint8_t cmd)
{
    LCDGpioReset(lcd_dc);
    return LCDSpiTransmitUnlocked(&cmd, 1U);
}

static HAL_StatusTypeDef LCDWriteData8Unlocked(uint8_t data)
{
    LCDGpioSet(lcd_dc);
    return LCDSpiTransmitUnlocked(&data, 1U);
}

static HAL_StatusTypeDef LCDWriteData16Unlocked(uint16_t data)
{
    uint8_t tx[2];

    tx[0] = (uint8_t)(data >> 8U);
    tx[1] = (uint8_t)data;

    LCDGpioSet(lcd_dc);
    return LCDSpiTransmitUnlocked(tx, sizeof(tx));
}

static HAL_StatusTypeDef LCDWriteDataBufferUnlocked(const uint8_t* data, uint32_t len)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint16_t tx_len;

    LCDGpioSet(lcd_dc);
    while (len > 0U)
    {
        tx_len = (len > UINT16_MAX) ? UINT16_MAX : (uint16_t)len;
        status = LCDSpiTransmitUnlocked(data, tx_len);
        if (status != HAL_OK)
        {
            return status;
        }

        data += tx_len;
        len -= tx_len;
    }

    return status;
}

static void LCDWriteColorBurstUnlocked(uint16_t color, uint32_t pixel_count)
{
    uint32_t chunk_pixels;
    uint16_t i;
    const uint8_t color_high = (uint8_t)(color >> 8U);
    const uint8_t color_low = (uint8_t)color;

    LCDGpioSet(lcd_dc);
    while (pixel_count > 0U)
    {
        chunk_pixels = pixel_count;
        if (chunk_pixels > (LCD_TX_CHUNK_SIZE / 2U))
        {
            chunk_pixels = LCD_TX_CHUNK_SIZE / 2U;
        }

        for (i = 0U; i < chunk_pixels; i++)
        {
            lcd_tx_buffer[i * 2U] = color_high;
            lcd_tx_buffer[i * 2U + 1U] = color_low;
        }

        (void)LCDSpiTransmitUnlocked(lcd_tx_buffer, (uint16_t)(chunk_pixels * 2U));
        pixel_count -= chunk_pixels;
    }
}

static void LCDColorStreamAppend(uint16_t color, uint16_t* buffer_len)
{
    if ((*buffer_len + 2U) > LCD_TX_CHUNK_SIZE)
    {
        (void)LCDWriteDataBufferUnlocked(lcd_tx_buffer, *buffer_len);
        *buffer_len = 0U;
    }

    lcd_tx_buffer[*buffer_len] = (uint8_t)(color >> 8U);
    lcd_tx_buffer[*buffer_len + 1U] = (uint8_t)color;
    *buffer_len += 2U;
}

static void LCDColorStreamFlush(uint16_t* buffer_len)
{
    if (*buffer_len > 0U)
    {
        (void)LCDWriteDataBufferUnlocked(lcd_tx_buffer, *buffer_len);
        *buffer_len = 0U;
    }
}

static void LCDAddressSetUnlocked(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
#if LCD_USE_HORIZONTAL == 0U
    (void)LCDWriteCommandUnlocked(LCD_CMD_COLUMN_ADDRESS);
    (void)LCDWriteData16Unlocked(x1);
    (void)LCDWriteData16Unlocked(x2);
    (void)LCDWriteCommandUnlocked(LCD_CMD_ROW_ADDRESS);
    (void)LCDWriteData16Unlocked(y1 + 20U);
    (void)LCDWriteData16Unlocked(y2 + 20U);
#elif LCD_USE_HORIZONTAL == 1U
    (void)LCDWriteCommandUnlocked(LCD_CMD_COLUMN_ADDRESS);
    (void)LCDWriteData16Unlocked(x1);
    (void)LCDWriteData16Unlocked(x2);
    (void)LCDWriteCommandUnlocked(LCD_CMD_ROW_ADDRESS);
    (void)LCDWriteData16Unlocked(y1 + 20U);
    (void)LCDWriteData16Unlocked(y2 + 20U);
#elif LCD_USE_HORIZONTAL == 2U
    (void)LCDWriteCommandUnlocked(LCD_CMD_COLUMN_ADDRESS);
    (void)LCDWriteData16Unlocked(x1 + 20U);
    (void)LCDWriteData16Unlocked(x2 + 20U);
    (void)LCDWriteCommandUnlocked(LCD_CMD_ROW_ADDRESS);
    (void)LCDWriteData16Unlocked(y1);
    (void)LCDWriteData16Unlocked(y2);
#else
    (void)LCDWriteCommandUnlocked(LCD_CMD_COLUMN_ADDRESS);
    (void)LCDWriteData16Unlocked(x1 + 20U);
    (void)LCDWriteData16Unlocked(x2 + 20U);
    (void)LCDWriteCommandUnlocked(LCD_CMD_ROW_ADDRESS);
    (void)LCDWriteData16Unlocked(y1);
    (void)LCDWriteData16Unlocked(y2);
#endif
    (void)LCDWriteCommandUnlocked(LCD_CMD_MEMORY_WRITE);
}

static uint8_t LCDClipPoint(uint16_t x, uint16_t y)
{
    return (x < LCD_W && y < LCD_H) ? 1U : 0U;
}

static void LCDDrawPointUnlocked(uint16_t x, uint16_t y, uint16_t color)
{
    if (!LCDClipPoint(x, y))
    {
        return;
    }

    LCDAddressSetUnlocked(x, y, x, y);
    (void)LCDWriteData16Unlocked(color);
}

/**
 * @brief 在已持有 LCD 锁的前提下填充矩形区域。
 *
 * xend/yend 采用半开区间语义，即实际绘制范围为 [xsta, xend) x [ysta, yend)。
 * 该函数会做屏幕边界裁剪，便于线段、矩形等图元复用同一条批量写入路径。
 */
static void LCDFillUnlocked(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color)
{
    uint32_t pixel_count;

    if (xsta >= LCD_W || ysta >= LCD_H)
    {
        return;
    }

    if (xend > LCD_W)
    {
        xend = LCD_W;
    }
    if (yend > LCD_H)
    {
        yend = LCD_H;
    }
    if (xend <= xsta || yend <= ysta)
    {
        return;
    }

    LCDAddressSetUnlocked(xsta, ysta, xend - 1U, yend - 1U);
    pixel_count = (uint32_t)(xend - xsta) * (uint32_t)(yend - ysta);
    LCDWriteColorBurstUnlocked(color, pixel_count);
}

/**
 * @brief 在已持有 LCD 锁的前提下快速绘制水平线或垂直线。
 *
 * ST7789 写显存时，水平/垂直线本质上都是一个窄矩形。相比逐点绘制，
 * 这里只设置一次地址窗口，再连续写入同一个颜色，复杂图形的边框绘制会明显更快。
 */
static void LCDDrawAxisLineUnlocked(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    uint16_t start;
    uint16_t end;

    if (y1 == y2)
    {
        start = (x1 < x2) ? x1 : x2;
        end = (x1 < x2) ? x2 : x1;
        LCDFillUnlocked(start,
                        y1,
                        (end >= (LCD_W - 1U)) ? LCD_W : (uint16_t)(end + 1U),
                        (uint16_t)(y1 + 1U),
                        color);
        return;
    }

    if (x1 == x2)
    {
        start = (y1 < y2) ? y1 : y2;
        end = (y1 < y2) ? y2 : y1;
        LCDFillUnlocked(x1,
                        start,
                        (uint16_t)(x1 + 1U),
                        (end >= (LCD_H - 1U)) ? LCD_H : (uint16_t)(end + 1U),
                        color);
    }
}

static const uint8_t* LCDGetAsciiFont(uint8_t num, uint8_t sizey, uint8_t* sizex, uint16_t* pixel_count)
{
    if (num < LCD_ASCII_FIRST_CHAR || num > LCD_ASCII_LAST_CHAR)
    {
        num = '?';
    }
    num = (uint8_t)(num - LCD_ASCII_FIRST_CHAR);

    switch (sizey)
    {
    case 12U:
        *sizex = 6U;
        *pixel_count = 6U * 12U;
        return ascii_1206[num];
    case 16U:
        *sizex = 8U;
        *pixel_count = 8U * 16U;
        return ascii_1608[num];
    case 24U:
        *sizex = 12U;
        *pixel_count = 12U * 24U;
        return ascii_2412[num];
    case 32U:
        *sizex = 16U;
        *pixel_count = 16U * 32U;
        return ascii_3216[num];
    default:
        return NULL;
    }
}

static void LCDRenderBitmapFontUnlocked(uint16_t x,
                                        uint16_t y,
                                        uint8_t width,
                                        uint8_t height,
                                        const uint8_t* bitmap,
                                        uint16_t bitmap_bytes,
                                        uint16_t fc,
                                        uint16_t bc,
                                        uint8_t mode)
{
    uint16_t tx_len = 0U;
    uint16_t pixel_index = 0U;
    uint16_t max_pixels = (uint16_t)width * height;
    uint16_t byte_index;
    uint8_t bit_index;
    uint8_t pixel_on;
    uint16_t px = 0U;
    uint16_t py = 0U;

    if (bitmap == NULL || width == 0U || height == 0U || x >= LCD_W || y >= LCD_H)
    {
        return;
    }

    if (mode == LCD_TEXT_MODE_NORMAL)
    {
        if ((x + width) > LCD_W || (y + height) > LCD_H)
        {
            return;
        }
        LCDAddressSetUnlocked(x, y, x + width - 1U, y + height - 1U);
    }

    for (byte_index = 0U; byte_index < bitmap_bytes && pixel_index < max_pixels; byte_index++)
    {
        for (bit_index = 0U; bit_index < 8U && pixel_index < max_pixels; bit_index++)
        {
            pixel_on = (bitmap[byte_index] & (uint8_t)(0x01U << bit_index)) ? 1U : 0U;
            if (mode == LCD_TEXT_MODE_NORMAL)
            {
                LCDColorStreamAppend(pixel_on ? fc : bc, &tx_len);
            }
            else if (pixel_on)
            {
                LCDDrawPointUnlocked(x + px, y + py, fc);
            }

            px++;
            if (px >= width)
            {
                px = 0U;
                py++;
            }
            pixel_index++;
        }
    }

    if (mode == LCD_TEXT_MODE_NORMAL)
    {
        LCDColorStreamFlush(&tx_len);
    }
}

/**
 * @brief 在已持有 LCD 锁时绘制一个 ASCII 字符。
 *
 * 高层字符串/数字接口会一次加锁后连续绘制多个字符，避免多任务同时输出时
 * 字符之间互相穿插；单字符公开接口也复用该函数，保持绘制逻辑只有一份。
 */
static void LCDShowCharUnlocked(uint16_t x, uint16_t y, uint8_t num, uint16_t fc, uint16_t bc, uint8_t sizey,
                                uint8_t mode)
{
    const uint8_t* font;
    uint8_t sizex = 0U;
    uint16_t pixel_count = 0U;
    uint16_t font_bytes;

    font = LCDGetAsciiFont(num, sizey, &sizex, &pixel_count);
    if (font == NULL)
    {
        return;
    }

    font_bytes = (pixel_count + 7U) / 8U;
    LCDRenderBitmapFontUnlocked(x, y, sizex, sizey, font, font_bytes, fc, bc, mode);
}

/**
 * @brief 在已持有 LCD 锁时绘制 ASCII 字符串。
 */
static void LCDShowStringUnlocked(uint16_t x, uint16_t y, const uint8_t* p, uint16_t fc, uint16_t bc, uint8_t sizey,
                                  uint8_t mode)
{
    uint8_t sizex = sizey / 2U;

    if (p == NULL || sizex == 0U)
    {
        return;
    }

    while (*p != '\0')
    {
        if ((x + sizex) > LCD_W)
        {
            x = 0U;
            y = (uint16_t)(y + sizey);
        }
        if ((y + sizey) > LCD_H)
        {
            return;
        }

        LCDShowCharUnlocked(x, y, *p, fc, bc, sizey, mode);
        x = (uint16_t)(x + sizex);
        p++;
    }
}

static uint32_t LCDPow10(uint8_t n)
{
    uint32_t result = 1U;

    while (n-- != 0U)
    {
        result *= 10U;
    }

    return result;
}

/**
 * @brief 解析 LCD 中文字库索引。
 *
 * 达妙官方字库使用 GBK/GB2312 的两个字节作为汉字索引；而当前工程源文件
 * 通常按 UTF-8 保存。这里对官方字库已经包含的“达妙科技”做 UTF-8 到
 * GBK 索引的轻量映射，保证 LCD_ShowChinese(..., "达妙", ...) 这类
 * UTF-8 字符串也能命中字库。若传入本来就是 GBK 两字节字符串，则直接透传。
 */
static uint8_t LCDDecodeChineseIndex(const uint8_t** text, uint8_t index[2])
{
    const uint8_t* p;

    if (text == NULL || *text == NULL || index == NULL)
    {
        return 0U;
    }

    p = *text;
    if (p[0] == 0U)
    {
        return 0U;
    }

    if (p[1] != 0U && p[2] != 0U &&
        p[0] == 0xE8U && p[1] == 0xBEU && p[2] == 0xBEU) // 达 -> GBK B4 EF
    {
        index[0] = 0xB4U;
        index[1] = 0xEFU;
        *text += 3U;
        return 1U;
    }

    if (p[1] != 0U && p[2] != 0U &&
        p[0] == 0xE5U && p[1] == 0xA6U && p[2] == 0x99U) // 妙 -> GBK C3 EE
    {
        index[0] = 0xC3U;
        index[1] = 0xEEU;
        *text += 3U;
        return 1U;
    }

    if (p[1] != 0U && p[2] != 0U &&
        p[0] == 0xE7U && p[1] == 0xA7U && p[2] == 0x91U) // 科 -> GBK BF C6
    {
        index[0] = 0xBFU;
        index[1] = 0xC6U;
        *text += 3U;
        return 1U;
    }

    if (p[1] != 0U && p[2] != 0U &&
        p[0] == 0xE6U && p[1] == 0x8AU && p[2] == 0x80U) // 技 -> GBK BC BC
    {
        index[0] = 0xBCU;
        index[1] = 0xBCU;
        *text += 3U;
        return 1U;
    }

    if (p[0] < 0x80U || p[1] == 0U)
    {
        return 0U;
    }

    index[0] = p[0];
    index[1] = p[1];
    *text += 2U;
    return 1U;
}

HAL_StatusTypeDef LCD_Init(void)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (LCDLock() != HAL_OK)
    {
        return HAL_ERROR;
    }

    do
    {
        status = LCDEnsureHardware();
        if (status != HAL_OK)
        {
            break;
        }

        lcd_is_ready = 0U;

        /* 硬复位时序沿用达妙官方例程，保证 ST7789V2 从确定状态启动。 */
        LCDGpioReset(lcd_res);
        LCDDelay(100U);
        LCDGpioSet(lcd_res);
        LCDDelay(100U);

        LCDGpioSet(lcd_blk);
        LCDDelay(100U);

        LCD_CHECK_WRITE(LCDWriteCommandUnlocked(LCD_CMD_SLEEP_OUT));
        LCDDelay(120U);

        LCD_CHECK_WRITE(LCDWriteCommandUnlocked(LCD_CMD_MEMORY_ACCESS));
#if LCD_USE_HORIZONTAL == 0U
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x00U));
#elif LCD_USE_HORIZONTAL == 1U
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0xC0U));
#elif LCD_USE_HORIZONTAL == 2U
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x70U));
#else
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0xA0U));
#endif

        LCD_CHECK_WRITE(LCDWriteCommandUnlocked(LCD_CMD_PIXEL_FORMAT));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x05U)); // RGB565, 每像素16bit

        LCD_CHECK_WRITE(LCDWriteCommandUnlocked(0xB2U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x0CU));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x0CU));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x00U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x33U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x33U));

        LCD_CHECK_WRITE(LCDWriteCommandUnlocked(0xB7U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x35U));

        LCD_CHECK_WRITE(LCDWriteCommandUnlocked(0xBBU));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x32U));

        LCD_CHECK_WRITE(LCDWriteCommandUnlocked(0xC2U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x01U));

        LCD_CHECK_WRITE(LCDWriteCommandUnlocked(0xC3U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x15U));

        LCD_CHECK_WRITE(LCDWriteCommandUnlocked(0xC4U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x20U));

        LCD_CHECK_WRITE(LCDWriteCommandUnlocked(0xC6U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x0FU));

        LCD_CHECK_WRITE(LCDWriteCommandUnlocked(0xD0U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0xA4U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0xA1U));

        LCD_CHECK_WRITE(LCDWriteCommandUnlocked(0xE0U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0xD0U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x08U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x0EU));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x09U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x09U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x05U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x31U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x33U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x48U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x17U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x14U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x15U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x31U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x34U));

        LCD_CHECK_WRITE(LCDWriteCommandUnlocked(0xE1U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0xD0U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x08U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x0EU));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x09U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x09U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x15U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x31U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x33U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x48U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x17U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x14U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x15U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x31U));
        LCD_CHECK_WRITE(LCDWriteData8Unlocked(0x34U));

        LCD_CHECK_WRITE(LCDWriteCommandUnlocked(0x21U)); // 颜色反转，沿用官方例程配置
        LCD_CHECK_WRITE(LCDWriteCommandUnlocked(LCD_CMD_DISPLAY_ON));
        if (status == HAL_OK)
        {
            lcd_is_ready = 1U;
        }
    }
    while (0);

    LCDUnlock();
    return status;
}

HAL_StatusTypeDef LCDTaskInit(void)
{
    static const osThreadAttr_t lcd_task_attr = {
        .name = "lcdtask",
        .stack_size = LCD_TASK_STACK_SIZE,
        .priority = osPriorityLow,
    };
    osKernelState_t kernel_state = osKernelGetState();

    if (kernel_state == osKernelInactive || kernel_state == osKernelError)
    {
        LOGERROR("[lcd] LCD任务初始化失败: FreeRTOS内核尚未初始化");
        return HAL_ERROR;
    }

    /*
     * LCD_Init() 通常发生在调度器启动前,当时不会持有RTOS互斥锁。
     * 在创建LCD任务和其他业务任务前统一创建mutex,避免任务运行后首次绘图时并发懒创建。
     */
    if (LCDEnsureMutex() != HAL_OK)
    {
        return HAL_ERROR;
    }

    /*
     * application层当前只统一调用LCDTaskInit()。这里顺手完成屏幕硬件初始化，
     * 确保异步绘图任务创建后LCD_AsyncXXX()接口能够直接工作。
     */
    if (lcd_is_ready == 0U && LCD_Init() != HAL_OK)
    {
        LOGERROR("[lcd] LCD硬件初始化失败");
        return HAL_ERROR;
    }

    if (lcd_cmd_queue == NULL)
    {
        lcd_cmd_queue = osMessageQueueNew(LCD_ASYNC_QUEUE_DEPTH, sizeof(LCDCommand_s), NULL);
        if (lcd_cmd_queue == NULL)
        {
            LOGERROR("[lcd] LCD异步绘图队列创建失败");
            return HAL_ERROR;
        }
    }

    if (lcd_task_handle == NULL)
    {
        lcd_task_handle = osThreadNew(LCDTask, NULL, &lcd_task_attr);
        if (lcd_task_handle == NULL)
        {
            LOGERROR("[lcd] LCD绘图任务创建失败");
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef LCD_SetBacklight(uint8_t enable)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (LCDLock() != HAL_OK)
    {
        return HAL_ERROR;
    }

    status = LCDEnsureHardware();
    if (status == HAL_OK)
    {
        if (enable != 0U)
        {
            LCDGpioSet(lcd_blk);
        }
        else
        {
            LCDGpioReset(lcd_blk);
        }
    }

    LCDUnlock();
    return status;
}

HAL_StatusTypeDef LCD_DisplayOn(void)
{
    HAL_StatusTypeDef status;

    if (LCDLock() != HAL_OK)
    {
        return HAL_ERROR;
    }

    status = LCDEnsureHardware();
    if (status == HAL_OK)
    {
        status = LCDWriteCommandUnlocked(LCD_CMD_DISPLAY_ON);
        LCDGpioSet(lcd_blk);
    }

    LCDUnlock();
    return status;
}

HAL_StatusTypeDef LCD_DisplayOff(void)
{
    HAL_StatusTypeDef status;

    if (LCDLock() != HAL_OK)
    {
        return HAL_ERROR;
    }

    status = LCDEnsureHardware();
    if (status == HAL_OK)
    {
        LCDGpioReset(lcd_blk);
        status = LCDWriteCommandUnlocked(LCD_CMD_DISPLAY_OFF);
    }

    LCDUnlock();
    return status;
}

void LCD_Clear(uint16_t color)
{
    LCD_Fill(0U, 0U, LCD_W, LCD_H, color);
}

void LCD_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color)
{
    if (LCDLock() != HAL_OK)
    {
        return;
    }

    if (LCDEnsureHardware() != HAL_OK || lcd_is_ready == 0U)
    {
        LCDUnlock();
        return;
    }

    LCDFillUnlocked(xsta, ysta, xend, yend, color);

    LCDUnlock();
}

void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color)
{
    if (LCDLock() != HAL_OK)
    {
        return;
    }

    if (LCDEnsureHardware() == HAL_OK && lcd_is_ready != 0U)
    {
        LCDDrawPointUnlocked(x, y, color);
    }

    LCDUnlock();
}

void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    int32_t delta_x = (int32_t)x2 - (int32_t)x1;
    int32_t delta_y = (int32_t)y2 - (int32_t)y1;
    int32_t inc_x;
    int32_t inc_y;
    int32_t distance;
    int32_t xerr = 0;
    int32_t yerr = 0;
    int32_t x = x1;
    int32_t y = y1;

    if (LCDLock() != HAL_OK)
    {
        return;
    }

    if (LCDEnsureHardware() != HAL_OK || lcd_is_ready == 0U)
    {
        LCDUnlock();
        return;
    }

    if (x1 == x2 || y1 == y2)
    {
        LCDDrawAxisLineUnlocked(x1, y1, x2, y2, color);
        LCDUnlock();
        return;
    }

    inc_x = (delta_x > 0) ? 1 : ((delta_x == 0) ? 0 : -1);
    inc_y = (delta_y > 0) ? 1 : ((delta_y == 0) ? 0 : -1);
    if (delta_x < 0)
    {
        delta_x = -delta_x;
    }
    if (delta_y < 0)
    {
        delta_y = -delta_y;
    }
    distance = (delta_x > delta_y) ? delta_x : delta_y;

    for (int32_t i = 0; i <= distance; i++)
    {
        if (x >= 0 && y >= 0)
        {
            LCDDrawPointUnlocked((uint16_t)x, (uint16_t)y, color);
        }
        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance)
        {
            xerr -= distance;
            x += inc_x;
        }
        if (yerr > distance)
        {
            yerr -= distance;
            y += inc_y;
        }
    }

    LCDUnlock();
}

void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    uint16_t xmin = (x1 < x2) ? x1 : x2;
    uint16_t xmax = (x1 < x2) ? x2 : x1;
    uint16_t ymin = (y1 < y2) ? y1 : y2;
    uint16_t ymax = (y1 < y2) ? y2 : y1;

    if (LCDLock() != HAL_OK)
    {
        return;
    }

    if (LCDEnsureHardware() != HAL_OK || lcd_is_ready == 0U)
    {
        LCDUnlock();
        return;
    }

    /*
     * 矩形边框由四条轴向线组成。这里一次加锁后直接调用 unlocked 快速线函数，
     * 避免 LCD_DrawLine() 被调用四次导致重复获取锁，也避免逐点重复设置地址窗口。
     */
    LCDDrawAxisLineUnlocked(xmin, ymin, xmax, ymin, color);
    if (ymax != ymin)
    {
        LCDDrawAxisLineUnlocked(xmin, ymax, xmax, ymax, color);
    }
    if ((uint32_t)ymax > ((uint32_t)ymin + 1U))
    {
        LCDDrawAxisLineUnlocked(xmin,
                                (uint16_t)(ymin + 1U),
                                xmin,
                                (uint16_t)(ymax - 1U),
                                color);
        if (xmax != xmin)
        {
            LCDDrawAxisLineUnlocked(xmax,
                                    (uint16_t)(ymin + 1U),
                                    xmax,
                                    (uint16_t)(ymax - 1U),
                                    color);
        }
    }

    LCDUnlock();
}

void LCD_DrawCircle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color)
{
    int32_t a = 0;
    int32_t b = r;

    if (LCDLock() != HAL_OK)
    {
        return;
    }

    if (LCDEnsureHardware() != HAL_OK || lcd_is_ready == 0U)
    {
        LCDUnlock();
        return;
    }

    while (a <= b)
    {
        LCDDrawPointUnlocked((uint16_t)(x0 - b), (uint16_t)(y0 - a), color);
        LCDDrawPointUnlocked((uint16_t)(x0 + b), (uint16_t)(y0 - a), color);
        LCDDrawPointUnlocked((uint16_t)(x0 - a), (uint16_t)(y0 + b), color);
        LCDDrawPointUnlocked((uint16_t)(x0 - a), (uint16_t)(y0 - b), color);
        LCDDrawPointUnlocked((uint16_t)(x0 + b), (uint16_t)(y0 + a), color);
        LCDDrawPointUnlocked((uint16_t)(x0 + a), (uint16_t)(y0 - b), color);
        LCDDrawPointUnlocked((uint16_t)(x0 + a), (uint16_t)(y0 + b), color);
        LCDDrawPointUnlocked((uint16_t)(x0 - b), (uint16_t)(y0 + a), color);
        a++;
        if ((a * a + b * b) > ((int32_t)r * r))
        {
            b--;
        }
    }

    LCDUnlock();
}

void LCD_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    if (LCDLock() != HAL_OK)
    {
        return;
    }

    if (LCDEnsureHardware() != HAL_OK || lcd_is_ready == 0U)
    {
        LCDUnlock();
        return;
    }

    LCDShowCharUnlocked(x, y, num, fc, bc, sizey, mode);

    LCDUnlock();
}

void LCD_ShowString(uint16_t x, uint16_t y, const uint8_t* p, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    if (LCDLock() != HAL_OK)
    {
        return;
    }

    if (LCDEnsureHardware() != HAL_OK || lcd_is_ready == 0U)
    {
        LCDUnlock();
        return;
    }

    LCDShowStringUnlocked(x, y, p, fc, bc, sizey, mode);

    LCDUnlock();
}

void LCD_ShowChinese(uint16_t x, uint16_t y, const uint8_t* s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    uint8_t index[2];
    const uint8_t* text = s;

    if (s == NULL)
    {
        return;
    }

    while (LCDDecodeChineseIndex(&text, index) != 0U)
    {
        switch (sizey)
        {
        case 12U:
            LCD_ShowChinese12x12(x, y, index, fc, bc, sizey, mode);
            break;
        case 16U:
            LCD_ShowChinese16x16(x, y, index, fc, bc, sizey, mode);
            break;
        case 24U:
            LCD_ShowChinese24x24(x, y, index, fc, bc, sizey, mode);
            break;
        case 32U:
            LCD_ShowChinese32x32(x, y, index, fc, bc, sizey, mode);
            break;
        default:
            return;
        }

        x = (uint16_t)(x + sizey);
        if ((x + sizey) > LCD_W)
        {
            x = 0U;
            y = (uint16_t)(y + sizey);
        }
    }
}

void LCD_ShowChinese12x12(uint16_t x, uint16_t y, const uint8_t* s, uint16_t fc, uint16_t bc, uint8_t sizey,
                          uint8_t mode)
{
    (void)sizey;
    if (LCDLock() != HAL_OK)
    {
        return;
    }

    if (s == NULL || LCDEnsureHardware() != HAL_OK || lcd_is_ready == 0U)
    {
        LCDUnlock();
        return;
    }

    for (uint16_t k = 0U; k < (sizeof(tfont12) / sizeof(tfont12[0])); k++)
    {
        if (tfont12[k].Index[0] == s[0] && tfont12[k].Index[1] == s[1])
        {
            LCDRenderBitmapFontUnlocked(x, y, 12U, 12U, tfont12[k].Msk, sizeof(tfont12[k].Msk), fc, bc, mode);
            LCDUnlock();
            return;
        }
    }

    LCDUnlock();
}

void LCD_ShowChinese16x16(uint16_t x, uint16_t y, const uint8_t* s, uint16_t fc, uint16_t bc, uint8_t sizey,
                          uint8_t mode)
{
    (void)sizey;
    if (LCDLock() != HAL_OK)
    {
        return;
    }

    if (s == NULL || LCDEnsureHardware() != HAL_OK || lcd_is_ready == 0U)
    {
        LCDUnlock();
        return;
    }

    for (uint16_t k = 0U; k < (sizeof(tfont16) / sizeof(tfont16[0])); k++)
    {
        if (tfont16[k].Index[0] == s[0] && tfont16[k].Index[1] == s[1])
        {
            LCDRenderBitmapFontUnlocked(x, y, 16U, 16U, tfont16[k].Msk, sizeof(tfont16[k].Msk), fc, bc, mode);
            LCDUnlock();
            return;
        }
    }

    LCDUnlock();
}

void LCD_ShowChinese24x24(uint16_t x, uint16_t y, const uint8_t* s, uint16_t fc, uint16_t bc, uint8_t sizey,
                          uint8_t mode)
{
    (void)sizey;
    if (LCDLock() != HAL_OK)
    {
        return;
    }

    if (s == NULL || LCDEnsureHardware() != HAL_OK || lcd_is_ready == 0U)
    {
        LCDUnlock();
        return;
    }

    for (uint16_t k = 0U; k < (sizeof(tfont24) / sizeof(tfont24[0])); k++)
    {
        if (tfont24[k].Index[0] == s[0] && tfont24[k].Index[1] == s[1])
        {
            LCDRenderBitmapFontUnlocked(x, y, 24U, 24U, tfont24[k].Msk, sizeof(tfont24[k].Msk), fc, bc, mode);
            LCDUnlock();
            return;
        }
    }

    LCDUnlock();
}

void LCD_ShowChinese32x32(uint16_t x, uint16_t y, const uint8_t* s, uint16_t fc, uint16_t bc, uint8_t sizey,
                          uint8_t mode)
{
    (void)sizey;
    if (LCDLock() != HAL_OK)
    {
        return;
    }

    if (s == NULL || LCDEnsureHardware() != HAL_OK || lcd_is_ready == 0U)
    {
        LCDUnlock();
        return;
    }

    for (uint16_t k = 0U; k < (sizeof(tfont32) / sizeof(tfont32[0])); k++)
    {
        if (tfont32[k].Index[0] == s[0] && tfont32[k].Index[1] == s[1])
        {
            LCDRenderBitmapFontUnlocked(x, y, 32U, 32U, tfont32[k].Msk, sizeof(tfont32[k].Msk), fc, bc, mode);
            LCDUnlock();
            return;
        }
    }

    LCDUnlock();
}

void LCD_ShowIntNum(uint16_t x, uint16_t y, uint16_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t sizey)
{
    uint8_t sizex = sizey / 2U;
    uint8_t started = 0U;

    if (LCDLock() != HAL_OK)
    {
        return;
    }

    if (LCDEnsureHardware() != HAL_OK || lcd_is_ready == 0U)
    {
        LCDUnlock();
        return;
    }

    for (uint8_t t = 0U; t < len; t++)
    {
        uint8_t digit = (uint8_t)((num / LCDPow10((uint8_t)(len - t - 1U))) % 10U);
        if (started == 0U && t < (len - 1U) && digit == 0U)
        {
            LCDShowCharUnlocked((uint16_t)(x + t * sizex), y, ' ', fc, bc, sizey, LCD_TEXT_MODE_NORMAL);
        }
        else
        {
            started = 1U;
            LCDShowCharUnlocked((uint16_t)(x + t * sizex), y, (uint8_t)(digit + '0'), fc, bc, sizey,
                                LCD_TEXT_MODE_NORMAL);
        }
    }

    LCDUnlock();
}

void LCD_ShowFloatNum(uint16_t x, uint16_t y, float num, uint8_t len, uint8_t decimal, uint16_t fc, uint16_t bc,
                      uint8_t sizey)
{
    uint8_t sizex = sizey / 2U;
    int display_len = (len > LCD_FLOAT_INT_WIDTH_MAX) ? (int)LCD_FLOAT_INT_WIDTH_MAX : (int)len;
    int display_decimal = (decimal > LCD_FLOAT_DECIMAL_MAX) ? (int)LCD_FLOAT_DECIMAL_MAX : (int)decimal;
    uint32_t scale = LCDPow10((uint8_t)display_decimal);
    int32_t scaled = (int32_t)(num * (float)scale + ((num >= 0.0f) ? 0.5f : -0.5f));
    uint32_t abs_scaled;
    uint32_t int_part;
    uint32_t frac_part;
    char text[32];

    if (scaled < 0)
    {
        abs_scaled = (uint32_t)(-scaled);
    }
    else
    {
        abs_scaled = (uint32_t)scaled;
    }

    int_part = abs_scaled / scale;
    frac_part = abs_scaled % scale;

    /*
     * len/decimal来自上层参数,理论最大可到255。若直接用于snprintf的动态宽度,
     * 编译器会按最坏情况推导出超长字符串。这里先限制显示宽度,既避免告警,
     * 也防止LCD文本区域被异常参数拉得过大。
     */
    if (display_decimal == 0)
    {
        if (scaled < 0)
        {
            (void)snprintf(text, sizeof(text), "-%*lu",
                           display_len,
                           (unsigned long)int_part);
        }
        else
        {
            (void)snprintf(text, sizeof(text), " %*lu",
                           display_len,
                           (unsigned long)int_part);
        }
    }
    else if (scaled < 0)
    {
        (void)snprintf(text, sizeof(text), "-%*lu.%0*lu",
                       display_len,
                       (unsigned long)int_part,
                       display_decimal,
                       (unsigned long)frac_part);
    }
    else
    {
        (void)snprintf(text, sizeof(text), " %*lu.%0*lu",
                       display_len,
                       (unsigned long)int_part,
                       display_decimal,
                       (unsigned long)frac_part);
    }

    if (LCDLock() != HAL_OK)
    {
        return;
    }

    if (LCDEnsureHardware() == HAL_OK && lcd_is_ready != 0U)
    {
        LCDFillUnlocked(x, y, (uint16_t)(x + strlen(text) * sizex), (uint16_t)(y + sizey), bc);
        LCDShowStringUnlocked(x, y, (const uint8_t*)text, fc, bc, sizey, LCD_TEXT_MODE_NORMAL);
    }

    LCDUnlock();
}

void LCD_ShowFloatNum1(uint16_t x, uint16_t y, float num, uint8_t len, uint8_t decimal, uint16_t fc, uint16_t bc,
                       uint8_t sizey)
{
    if (num < 0.0f)
    {
        num = 0.0f;
    }

    LCD_ShowFloatNum(x, y, num, len, decimal, fc, bc, sizey);
}

void LCD_ShowPicture(uint16_t x, uint16_t y, uint16_t length, uint16_t width, const uint8_t pic[])
{
    uint32_t total_bytes;
    uint32_t offset = 0U;
    uint16_t tx_len;

    if (pic == NULL || length == 0U || width == 0U)
    {
        return;
    }

    if (LCDLock() != HAL_OK)
    {
        return;
    }

    if (LCDEnsureHardware() != HAL_OK || lcd_is_ready == 0U ||
        x >= LCD_W || y >= LCD_H || (x + length) > LCD_W || (y + width) > LCD_H)
    {
        LCDUnlock();
        return;
    }

    LCDAddressSetUnlocked(x, y, x + length - 1U, y + width - 1U);
    total_bytes = (uint32_t)length * (uint32_t)width * 2U;
    while (offset < total_bytes)
    {
        tx_len = ((total_bytes - offset) > LCD_TX_CHUNK_SIZE) ? LCD_TX_CHUNK_SIZE : (uint16_t)(total_bytes - offset);
        (void)LCDWriteDataBufferUnlocked(&pic[offset], tx_len);
        offset += tx_len;
    }

    LCDUnlock();
}

void LCD_Printf(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode, const char* fmt, ...)
{
    char buffer[LCD_PRINTF_BUFFER_SIZE];
    va_list args;

    if (fmt == NULL)
    {
        return;
    }

    va_start(args, fmt);
    (void)vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (LCDLock() != HAL_OK)
    {
        return;
    }

    if (LCDEnsureHardware() == HAL_OK && lcd_is_ready != 0U)
    {
        LCDShowStringUnlocked(x, y, (const uint8_t*)buffer, fc, bc, sizey, mode);
    }

    LCDUnlock();
}

HAL_StatusTypeDef LCD_AsyncClear(uint16_t color)
{
    LCDCommand_s cmd = {
        .type = LCD_CMD_CLEAR,
        .color = color,
    };

    return LCDPostCommand(&cmd);
}

HAL_StatusTypeDef LCD_AsyncFill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color)
{
    LCDCommand_s cmd = {
        .type = LCD_CMD_FILL,
        .x1 = xsta,
        .y1 = ysta,
        .x2 = xend,
        .y2 = yend,
        .color = color,
    };

    return LCDPostCommand(&cmd);
}

HAL_StatusTypeDef LCD_AsyncDrawPoint(uint16_t x, uint16_t y, uint16_t color)
{
    LCDCommand_s cmd = {
        .type = LCD_CMD_DRAW_POINT,
        .x1 = x,
        .y1 = y,
        .color = color,
    };

    return LCDPostCommand(&cmd);
}

HAL_StatusTypeDef LCD_AsyncDrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    LCDCommand_s cmd = {
        .type = LCD_CMD_DRAW_LINE,
        .x1 = x1,
        .y1 = y1,
        .x2 = x2,
        .y2 = y2,
        .color = color,
    };

    return LCDPostCommand(&cmd);
}

HAL_StatusTypeDef LCD_AsyncDrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    LCDCommand_s cmd = {
        .type = LCD_CMD_DRAW_RECTANGLE,
        .x1 = x1,
        .y1 = y1,
        .x2 = x2,
        .y2 = y2,
        .color = color,
    };

    return LCDPostCommand(&cmd);
}

HAL_StatusTypeDef LCD_AsyncPrintf(uint16_t x,
                                  uint16_t y,
                                  uint16_t fc,
                                  uint16_t bc,
                                  uint8_t sizey,
                                  uint8_t mode,
                                  const char* fmt,
                                  ...)
{
    LCDCommand_s cmd = {
        .type = LCD_CMD_PRINTF,
        .x1 = x,
        .y1 = y,
        .color = fc,
        .bg_color = bc,
        .sizey = sizey,
        .mode = mode,
    };
    va_list args;

    if (fmt == NULL)
    {
        return HAL_ERROR;
    }

    va_start(args, fmt);
    (void)vsnprintf(cmd.text, sizeof(cmd.text), fmt, args);
    va_end(args);

    return LCDPostCommand(&cmd);
}
