#ifndef FULL_SPI_STUB_H
#define FULL_SPI_STUB_H
#include <Arduino.h>
class SPISettings { public: SPISettings(uint32_t, int, int) {} SPISettings() {} };
class SPIClass {
public:
    void    setRX(uint8_t) {}
    void    setTX(uint8_t) {}
    void    setSCK(uint8_t) {}
    void    begin(void) {}
    void    beginTransaction(SPISettings) {}
    void    endTransaction(void) {}
    uint8_t transfer(uint8_t) { return 0; }
};
extern SPIClass SPI;
extern SPIClass SPI1;
#endif
