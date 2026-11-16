// =====================================================================
//  Minimal pico-sdk UART shim — HOST SYNTAX CHECK ONLY.
//
//  Just enough of hardware/uart.h for ariadna_uart_dma.h to compile off
//  the target. Nothing here does anything.
// =====================================================================
#ifndef PICO_UART_STUB_H
#define PICO_UART_STUB_H

#include <stdint.h>

typedef struct uart_inst uart_inst_t;
extern uart_inst_t *const uart0;
extern uart_inst_t *const uart1;

typedef struct { volatile uint32_t dr; } uart_hw_t;
inline uart_hw_t *uart_get_hw(uart_inst_t *) {
    static uart_hw_t hw;
    return &hw;
}
inline unsigned uart_get_dreq(uart_inst_t *, bool) { return 0; }

#endif
