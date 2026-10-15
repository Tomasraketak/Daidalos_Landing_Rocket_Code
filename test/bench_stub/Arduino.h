// =====================================================================
//  Host shim for bench_test.cpp — just enough of arduino-pico for
//  Ariadna_V2/Bench_Tests/common/bench_common.h to run on a workstation.
//
//  Unlike stub/Arduino.h this one DOES something: the serial port is a
//  pair of byte queues and the clock is a variable, so the bench link can
//  be exercised end to end without a board.
// =====================================================================
#ifndef BENCH_ARDUINO_STUB_H
#define BENCH_ARDUINO_STUB_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <deque>
#include <vector>

extern uint32_t g_fake_us;
inline uint32_t micros(void) { return g_fake_us; }
inline uint32_t millis(void) { return g_fake_us / 1000u; }
inline void     delayMicroseconds(uint32_t) {}
inline float    analogReadTemp(void) { return 25.0f; }

class SerialUART {
public:
    std::deque<uint8_t>  in;     // bytes the board would receive
    std::vector<uint8_t> out;    // bytes the board transmitted
    void   setTX(uint8_t) {}
    void   setRX(uint8_t) {}
    bool   setFIFOSize(size_t) { return true; }
    void   begin(unsigned long) {}
    int    available(void) { return (int)in.size(); }
    int    read(void) { if (in.empty()) return -1; int b = in.front(); in.pop_front(); return b; }
    size_t write(const uint8_t *p, size_t n) { out.insert(out.end(), p, p + n); return n; }
};

class USBStub {
public:
    size_t write(const uint8_t *, size_t n) { return n; }   // console output discarded
};
extern USBStub Serial;

#include "hardware/uart.h"

#endif
