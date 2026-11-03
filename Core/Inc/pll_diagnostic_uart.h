#ifndef __PLL_DIAGNOSTIC_UART_H
#define __PLL_DIAGNOSTIC_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lissajous_pll.h"

/* USART1_TX = PA9, 115200 baud, 8 data bits, no parity, 1 stop bit. */
void PLL_DiagnosticUart_Init(void);
void PLL_DiagnosticUart_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* __PLL_DIAGNOSTIC_UART_H */
