#ifndef STUB_HARDWARE_WATCHDOG_H
#define STUB_HARDWARE_WATCHDOG_H
#include <stdint.h>
inline void watchdog_enable(uint32_t, bool) {}
inline void watchdog_update(void) {}
#endif
