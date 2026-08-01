// =====================================================================
//  Minimal Arduino / arduino-pico shim — HOST SYNTAX CHECK ONLY.
//
//  This exists so `make syntax` can compile Daidalos_Landing_Guidance.ino
//  on a workstation. It is not a simulator: nothing here does anything.
//  It catches the class of mistake that otherwise only shows up when a
//  board is in front of you — a typo, a wrong argument type, a rename in
//  guidance.h that the sketch was not updated for.
//
//  The real behaviour is verified by host_test.cpp against the flight
//  algorithm, which is why the algorithm is kept out of the .ino.
// =====================================================================
#ifndef ARDUINO_STUB_H
#define ARDUINO_STUB_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

#define HIGH          1
#define LOW           0
#define INPUT         0
#define OUTPUT        1
#define INPUT_PULLUP  2
#define LED_BUILTIN   25

#define DEG_TO_RAD    0.017453292519943295
#define RAD_TO_DEG    57.29577951308232

// XIAO RP2350 silkscreen aliases, as the arduino-pico board variant
// defines them. The numbers here are placeholders — this shim only needs
// the names to exist and to be distinct, because nothing is driven.
#define D0   26
#define D1   27
#define D2   28
#define D3   29
#define D4    6
#define D5    7
#define D6    0
#define D7    1
#define D8    2
#define D9    4
#define D10   3

inline void     pinMode(uint8_t, uint8_t) {}
inline void     digitalWrite(uint8_t, int) {}
inline int      digitalRead(uint8_t) { return HIGH; }
inline void     analogWrite(uint8_t, int) {}
inline int      analogRead(uint8_t) { return 0; }
inline void     analogReadResolution(int) {}
inline void     analogWriteFreq(uint32_t) {}
inline void     analogWriteRange(uint32_t) {}
inline uint32_t micros(void) { return 0; }
inline uint32_t millis(void) { return 0; }
inline void     delay(uint32_t) {}
inline void     tight_loop_contents(void) {}

class SerialStub {
public:
    void   begin(uint32_t) {}
    void   setTX(uint8_t) {}
    void   setRX(uint8_t) {}
    int    available(void) { return 0; }
    int    read(void) { return -1; }
    size_t write(const uint8_t *, size_t) { return 0; }
    void   println(const char *) {}
    void   println(void) {}
    // format checking is the entire point of compiling this file
    __attribute__((format(printf, 2, 3)))
    void printf(const char *, ...) {}
};
extern SerialStub Serial;
extern SerialStub Serial1;
extern SerialStub Serial2;

#endif
