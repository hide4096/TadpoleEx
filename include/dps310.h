#ifndef DPS310_H
#define DPS310_H

#include <SPI.h>

#define DPS310_WHO_AM_I 0x10
#define RETRY_INIT 5

#define PRC_1 524288
#define PRC_2 1572864
#define PRC_4 3670016
#define PRC_8 7864320
#define PRC_16 253952
#define PRC_32 516096
#define PRC_64 1040384
#define PRC_128 2088960

class dps310{
    public:
        int init(SPIClass* wire,int _cs,uint8_t _PM_RATE,uint8_t _PM_PRC,uint8_t _TMP_RATE,uint8_t _TMP_PRC);
        void updateSensor();
        void setZeroPoint();
        float getAltitude();
        float pressure,temp,altitude;
        int32_t raw_pressure,raw_temp;
        int32_t c0,c1,c00,c10,c01,c11,c20,c21,c30;
        int32_t kP,kT;
    private:
        int32_t getRawPressure();
        int32_t getRawTemperature();
        uint8_t readRegister(uint8_t);
        void writeRegister(uint8_t,uint8_t);
        void getTwoComplement(int32_t*,uint8_t);

        SPIClass* mywire;
        SPISettings spi_settings;
        uint8_t PM_RATE;
        uint8_t PM_PRC;
        uint8_t TMP_RATE;
        uint8_t TMP_PRC;
        float zero_press;
        float zero_temp;
};

#endif