#ifndef FULL_EEPROM_STUB_H
#define FULL_EEPROM_STUB_H
#include <Arduino.h>
class EEPROMClass {
public:
    void begin(size_t) {}
    template <typename T> T &get(int, T &t) { memset((void *)&t, 0, sizeof(T)); return t; }
    template <typename T> const T &put(int, const T &t) { return t; }
    bool commit(void) { return true; }
};
extern EEPROMClass EEPROM;
#endif
