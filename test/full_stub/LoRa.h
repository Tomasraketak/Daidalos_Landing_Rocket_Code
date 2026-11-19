#ifndef FULL_LORA_STUB_H
#define FULL_LORA_STUB_H
#include <SPI.h>
class LoRaClass {
public:
    void   setSPI(SPIClass &) {}
    void   setPins(int, int, int) {}
    int    begin(long) { return 1; }
    void   setSpreadingFactor(int) {}
    void   setSignalBandwidth(long) {}
    void   setCodingRate4(int) {}
    void   setPreambleLength(long) {}
    void   setTxPower(int) {}
    void   enableCrc(void) {}
    int    beginPacket(int = 0) { return 1; }
    size_t write(const uint8_t *, size_t n) { return n; }
    int    endPacket(bool = false) { return 1; }
    int    parsePacket(int = 0) { return 0; }
    void   receive(int = 0) {}
    void   idle(void) {}
    void   sleep(void) {}
    int    available(void) { return 0; }
    int    read(void) { return -1; }
    size_t readBytes(uint8_t *, size_t n) { return n; }
    int    packetRssi(void) { return 0; }
    float  packetSnr(void) { return 0.0f; }
};
extern LoRaClass LoRa;
#endif
