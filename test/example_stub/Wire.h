// Wire shim for `make syntax` — compile only, does nothing.
#ifndef EXAMPLE_WIRE_STUB_H
#define EXAMPLE_WIRE_STUB_H
#include <stdint.h>
#include <stddef.h>
class TwoWireStub {
public:
    void    setSDA(uint8_t) {}
    void    setSCL(uint8_t) {}
    void    setTimeout(uint32_t, bool) {}
    void    begin(void) {}
    void    setClock(uint32_t) {}
    void    beginTransmission(uint8_t) {}
    size_t  write(uint8_t) { return 1; }
    uint8_t endTransmission(bool = true) { return 0; }
    uint8_t requestFrom(int, int) { return 0; }
    int     read(void) { return -1; }
};
extern TwoWireStub Wire;
#endif
