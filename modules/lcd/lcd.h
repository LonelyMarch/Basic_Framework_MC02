/**
 * @file lcd.h
 * @brief 达妙 DM_TFT-V1.1 彩色 LCD 模块驱动。
 *
 * LCD 控制器为 ST7789V2，主控板侧使用 SPI1 写入显示数据，并通过
 * CS/DC/RES/BLK 四根 GPIO 控制片选、命令数据选择、复位和背光。
 */
#ifndef LCD_MODULE_H
#define LCD_MODULE_H

#include "bsp_spi.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * LCD 显示方向。
 *
 * 0/1：竖屏，显示区域为 240 x 280。
 * 2/3：横屏，显示区域为 280 x 240。
 *
 * 默认值 2 与达妙主控板 LCD 官方例程保持一致。
 */
#ifndef LCD_USE_HORIZONTAL
#define LCD_USE_HORIZONTAL 2U
#endif

#if (LCD_USE_HORIZONTAL == 0U) || (LCD_USE_HORIZONTAL == 1U)
#define LCD_W 240U
#define LCD_H 280U
#else
#define LCD_W 280U
#define LCD_H 240U
#endif

#define LCD_WIDTH  LCD_W
#define LCD_HEIGHT LCD_H

/*
 * LCD SPI 工作模式。
 *
 * 默认使用阻塞模式，因为 LCD_Init() 可能在 FreeRTOS 调度器启动前调用。
 * 若后续确认所有 LCD 绘图都在任务上下文执行，并且 CubeMX 已开启 SPI1 TX DMA
 * 与 SPI1 全局中断，可以在工程编译宏中改为 SPI_DMA_MODE。
 */
#ifndef LCD_SPI_WORK_MODE
#define LCD_SPI_WORK_MODE SPI_BLOCK_MODE
#endif

/*
 * LCD 单次批量写入字节数。
 *
 * 该值用于区域填充、文字渲染和图片发送的分块传输。若使用 SPI_DMA_MODE，
 * 该值不能超过 bsp_spi 内部 DMA 中转缓冲区大小。
 */
#ifndef LCD_TX_CHUNK_SIZE
#define LCD_TX_CHUNK_SIZE 512U
#endif

/*
 * LCD 异步绘图队列深度。
 *
 * LCD 是低优先级调试显示外设，队列不宜过深。队列满时异步接口会返回
 * HAL_BUSY，由上层自行决定是否丢弃本次刷新。
 */
#ifndef LCD_ASYNC_QUEUE_DEPTH
#define LCD_ASYNC_QUEUE_DEPTH 8U
#endif

typedef enum
{
    LCD_TEXT_MODE_NORMAL = 0U,  // 同时写前景色和背景色，会覆盖整个字符矩形区域
    LCD_TEXT_MODE_OVERLAY = 1U, // 只写前景像素，背景区域保持原显示内容不变
} LCD_TextMode_e;

/* RGB565 常用颜色定义。 */
#define LCD_WHITE      0xFFFFU
#define LCD_BLACK      0x0000U
#define LCD_BLUE       0x001FU
#define LCD_BRED       0xF81FU
#define LCD_GRED       0xFFE0U
#define LCD_GBLUE      0x07FFU
#define LCD_RED        0xF800U
#define LCD_MAGENTA    0xF81FU
#define LCD_GREEN      0x07E0U
#define LCD_CYAN       0x7FFFU
#define LCD_YELLOW     0xFFE0U
#define LCD_BROWN      0xBC40U
#define LCD_BRRED      0xFC07U
#define LCD_GRAY       0x8430U
#define LCD_DARKBLUE   0x01CFU
#define LCD_LIGHTBLUE  0x7D7CU
#define LCD_GRAYBLUE   0x5458U
#define LCD_LIGHTGREEN 0x841FU
#define LCD_LGRAY      0xC618U
#define LCD_LGRAYBLUE  0xA651U
#define LCD_LBBLUE     0x2B12U

HAL_StatusTypeDef LCD_Init(void);
HAL_StatusTypeDef LCDTaskInit(void);
HAL_StatusTypeDef LCD_SetBacklight(uint8_t enable);
HAL_StatusTypeDef LCD_DisplayOn(void);
HAL_StatusTypeDef LCD_DisplayOff(void);

void LCD_Clear(uint16_t color);
void LCD_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color);
void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color);
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void LCD_DrawCircle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color);

void LCD_ShowChinese(uint16_t x, uint16_t y, const uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);
void LCD_ShowChinese12x12(uint16_t x, uint16_t y, const uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);
void LCD_ShowChinese16x16(uint16_t x, uint16_t y, const uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);
void LCD_ShowChinese24x24(uint16_t x, uint16_t y, const uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);
void LCD_ShowChinese32x32(uint16_t x, uint16_t y, const uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);

void LCD_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);
void LCD_ShowString(uint16_t x, uint16_t y, const uint8_t *p, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);
void LCD_ShowIntNum(uint16_t x, uint16_t y, uint16_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t sizey);
void LCD_ShowFloatNum(uint16_t x, uint16_t y, float num, uint8_t len, uint8_t decimal, uint16_t fc, uint16_t bc, uint8_t sizey);
void LCD_ShowFloatNum1(uint16_t x, uint16_t y, float num, uint8_t len, uint8_t decimal, uint16_t fc, uint16_t bc, uint8_t sizey);
void LCD_ShowPicture(uint16_t x, uint16_t y, uint16_t length, uint16_t width, const uint8_t pic[]);
void LCD_Printf(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode, const char *fmt, ...);

HAL_StatusTypeDef LCD_AsyncClear(uint16_t color);
HAL_StatusTypeDef LCD_AsyncFill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color);
HAL_StatusTypeDef LCD_AsyncDrawPoint(uint16_t x, uint16_t y, uint16_t color);
HAL_StatusTypeDef LCD_AsyncDrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
HAL_StatusTypeDef LCD_AsyncDrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
HAL_StatusTypeDef LCD_AsyncPrintf(uint16_t x,
                                  uint16_t y,
                                  uint16_t fc,
                                  uint16_t bc,
                                  uint8_t sizey,
                                  uint8_t mode,
                                  const char *fmt,
                                  ...);

#ifdef __cplusplus
}
#endif

#endif // LCD_MODULE_H
