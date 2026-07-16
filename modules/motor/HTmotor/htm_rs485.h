#ifndef HTM_RS485_H
#define HTM_RS485_H

#include <stdint.h>

#include "bsp_usart.h"

#define HTM_RS485_BUS_CNT 2U
#define HTM_RS485_MOTOR_PER_BUS 16U

typedef struct HTMRS485Bus HTMRS485Bus;

typedef struct
{
    UART_HandleTypeDef *usart_handle;
    USART_TRANSFER_MODE transfer_mode;
    uint16_t response_timeout_ms;
} HTMRS485Bus_Init_Config_s;

HTMRS485Bus *HTMRS485BusInit(const HTMRS485Bus_Init_Config_s *config);
void HTMRS485Control(void);

#endif
