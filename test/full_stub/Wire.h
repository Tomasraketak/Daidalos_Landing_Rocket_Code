#ifndef FULL_WIRE_STUB_H
#define FULL_WIRE_STUB_H
#include <Arduino.h>
class TwoWire {
public:
    void    setSDA(uint8_t) {}
    void    setSCL(uint8_t) {}
    void    begin(void) {}
    void    setClock(uint32_t) {}
    void    setTimeout(uint32_t, bool = false) {}
    void    beginTransmission(uint8_t) {}
    size_t  write(uint8_t) { return 1; }
    size_t  write(const uint8_t *, size_t n) { return n; }
    uint8_t endTransmission(bool = true) { return 0; }
    int     requestFrom(int, int) { return 0; }
    int     available(void) { return 0; }
    int     read(void) { return 0; }
};
extern TwoWire Wire;
extern TwoWire Wire1;
#endif
