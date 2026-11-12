// =====================================================================
//  Arduino / arduino-pico shim for the Examples/ sketches — HOST SYNTAX
//  CHECK ONLY (`make syntax`). Nothing here does anything; it exists so a
//  typo, a wrong argument type or a bad printf format in an example shows
//  up on a workstation instead of in front of a board.
// =====================================================================
#ifndef EXAMPLE_ARDUINO_STUB_H
#define EXAMPLE_ARDUINO_STUB_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

#define HIGH          1
#define LOW           0
#define INPUT         0
#define OUTPUT        1
#define INPUT_PULLUP  2

#define D0   26
#define D1   27
#define D2   28
#define D3   29
#define D6    0
#define D7    1
#define D8    2
#define D9    4

inline void     pinMode(uint8_t, uint8_t) {}
inline void     digitalWrite(uint8_t, int) {}
inline int      digitalRead(uint8_t) { return HIGH; }
inline int      analogRead(uint8_t) { return 0; }
inline void     analogReadResolution(int) {}
inline uint32_t millis(void) { return 0; }
inline void     delay(uint32_t) {}
inline void     tone(uint8_t, uint32_t) {}
inline void     noTone(uint8_t) {}

class SerialStub {
public:
    void   begin(uint32_t) {}
    void   setTX(uint8_t) {}
    void   setRX(uint8_t) {}
    bool   setFIFOSize(size_t) { return true; }
    int    available(void) { return 0; }
    int    availableForWrite(void) { return 64; }
    int    read(void) { return -1; }
    size_t write(const uint8_t *, size_t) { return 0; }
    size_t println(const char *) { return 0; }
    __attribute__((format(printf, 2, 3)))
    size_t printf(const char *, ...) { return 0; }
};
extern SerialStub Serial;
extern SerialStub Serial1;
extern SerialStub Serial2;

#endif
