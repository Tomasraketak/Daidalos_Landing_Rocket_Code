// =====================================================================
//  Arduino / arduino-pico shim for the big sketches (Master, Position) —
//  HOST SYNTAX CHECK ONLY (`make syntax`). Nothing here does anything. It
//  exists so a typo, a wrong argument type, a missing declaration or a bad
//  printf format shows up on a workstation instead of in front of a board.
//  It does NOT replace building with arduino-cli before flashing.
// =====================================================================
#ifndef FULL_ARDUINO_STUB_H
#define FULL_ARDUINO_STUB_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define HIGH          1
#define LOW           0
#define INPUT         0
#define OUTPUT        1
#define INPUT_PULLUP  2
#define MSBFIRST      1
#define SPI_MODE0     0
#define DEG_TO_RAD    0.017453292519943295
#define RAD_TO_DEG    57.29577951308232

enum { D0 = 26, D1, D2, D3, D4 = 6, D5, D6 = 0, D7, D8, D9 = 4, D10 = 3, D11 = 21, D12 = 20 };
#define PIN_NEOPIXEL   22
#define NEOPIXEL_POWER 23

inline void     pinMode(uint8_t, uint8_t) {}
inline void     digitalWrite(uint8_t, int) {}
inline int      digitalRead(uint8_t) { return HIGH; }
inline int      analogRead(uint8_t) { return 0; }
inline void     analogReadResolution(int) {}
inline uint32_t millis(void) { return 0; }
inline uint32_t micros(void) { return 0; }
inline void     delay(uint32_t) {}
inline void     delayMicroseconds(uint32_t) {}
inline void     tone(uint8_t, uint32_t, uint32_t = 0) {}
inline void     noTone(uint8_t) {}
inline void     tight_loop_contents(void) {}

typedef struct { int dummy; } mutex_t;
inline void mutex_init(mutex_t *) {}
inline bool mutex_try_enter(mutex_t *, uint32_t *) { return true; }
inline void mutex_exit(mutex_t *) {}

class Stream {
public:
    void   begin(uint32_t) {}
    void   setTX(uint8_t) {}
    void   setRX(uint8_t) {}
    bool   setFIFOSize(size_t) { return true; }
    int    available(void) { return 0; }
    int    availableForWrite(void) { return 64; }
    int    read(void) { return -1; }
    size_t write(const uint8_t *, size_t) { return 0; }
    size_t write(uint8_t) { return 0; }
    size_t print(const char *) { return 0; }
    size_t print(char) { return 0; }
    size_t println(const char *) { return 0; }
    size_t println(void) { return 0; }
    void   flush(void) {}
    explicit operator bool() const { return true; }
    __attribute__((format(printf, 2, 3)))
    size_t printf(const char *, ...) { return 0; }
};
typedef Stream HardwareSerial;
class SerialPIO : public Stream {
public:
    SerialPIO(uint8_t, uint8_t, size_t = 32) {}
    void end(void) {}
};
extern Stream Serial;
extern Stream Serial1;
extern Stream Serial2;

#include "hardware/uart.h"

#endif
