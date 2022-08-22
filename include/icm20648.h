#ifndef ICM_20648_H
#define ICM_20648_H

#include <SPI.h>
#include <Arduino.h>

class icm20648{
    public:
        int init(SPIClass* wire,const uint32_t _clock,const int _cs);
        uint16_t accelX();
        uint16_t accelY();
        uint16_t accelZ();
        uint16_t gyroX();
        uint16_t gyroY();
        uint16_t gyroZ();
        float temp();
    private:
        SPIClass* mywire;
        SPISettings spi_settings;
        int cs;
        uint8_t readWHO_AM_I();
        uint8_t readRegister(const uint8_t _adrs);
        void writeRegister(const uint8_t _adrs,const uint8_t _data);
        uint16_t readRegister_2Byte(const uint8_t _Hadrs,const uint8_t _Ladrs);

};

#endif