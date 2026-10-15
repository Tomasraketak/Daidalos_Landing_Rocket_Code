// Host shim: one UART with a controllable "TX FIFO empty" flag.
#ifndef BENCH_UART_STUB_H
#define BENCH_UART_STUB_H
#include <stdint.h>

typedef struct uart_inst uart_inst_t;
typedef struct { volatile uint32_t fr; } uart_hw_t;
#define UART_UARTFR_TXFE_BITS 0x80u

extern uart_hw_t g_fake_uart_hw;
inline uart_hw_t *uart_get_hw(uart_inst_t *) { return &g_fake_uart_hw; }
#endif
