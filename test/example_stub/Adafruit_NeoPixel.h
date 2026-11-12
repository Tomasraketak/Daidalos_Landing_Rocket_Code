// Adafruit_NeoPixel shim for `make syntax` — compile only, does nothing.
#ifndef EXAMPLE_NEOPIXEL_STUB_H
#define EXAMPLE_NEOPIXEL_STUB_H
#include <stdint.h>
#define NEO_GRB     0x52
#define NEO_KHZ800  0x0000
class Adafruit_NeoPixel {
public:
    Adafruit_NeoPixel(uint16_t, int16_t, uint16_t) {}
    void begin(void) {}
    void clear(void) {}
    void show(void) {}
    void setPixelColor(uint16_t, uint32_t) {}
    static uint32_t Color(uint8_t r, uint8_t g, uint8_t b) {
        return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
    }
};
#endif
