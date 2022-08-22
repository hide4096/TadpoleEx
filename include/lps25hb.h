#ifndef LPS25HB_H
#define LPS25HB_H

#include <SPI.h>
#include <Arduino.h>

class lps25hb{
    public:
        int init(SPIClass* wire,const uint32_t _clock,const int _cs);
        float press();
        float temp();
        float altitude();
    private:
        SPIClass* mywire;
        SPISettings spi_settings;
        int cs;
        float zero_press;
        float zero_temp;
        uint8_t readWHO_AM_I();
        uint8_t readRegister(const uint8_t _adrs);
        void writeRegister(const uint8_t _adrs,const uint8_t _data);
        uint16_t readRegister_2Byte(const uint8_t _Hadrs,const uint8_t _Ladrs);
        uint32_t readRegister_3Byte(const uint8_t _Hadrs,const uint8_t _Ladrs,const uint8_t _XLadrs);
};

#endif