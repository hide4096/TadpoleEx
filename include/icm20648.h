/* 
########################
    ICM-20648 Driver
########################
SPDX-FileCopyrightText: 2022 Aso Hidetoshi asouhide2002@gmail.com
SPDX-License-Identifier: BSD-3-Clause

    Date			Author              Notes
    2022/8/22      Aso Hidetoshi       First release
*/
#ifndef ICM_20648_H
#define ICM_20648_H

#include <SPI.h>
#include <Arduino.h>

class icm20648{
    public:
        int init(SPIClass* wire,const uint32_t _clock,const int _cs);
        int changeSensitivity(const uint8_t _gyro,const uint8_t _accel);
        int16_t accelX_raw();
        int16_t accelY_raw();
        int16_t accelZ_raw();
        int16_t gyroX_raw();
        int16_t gyroY_raw();
        int16_t gyroZ_raw();
        float temp();
        float accelX();
        float accelY();
        float accelZ();
        float gyroX();
        float gyroY();
        float gyroZ();
    private:
        SPIClass* mywire;
        SPISettings spi_settings;
        int cs;
        float gyro_sensitivity;
        float accel_sensitivity;
        uint8_t readWHO_AM_I();
        uint8_t readRegister(const uint8_t _adrs);
        void writeRegister(const uint8_t _adrs,const uint8_t _data);
        uint16_t readRegister_2Byte(const uint8_t _Hadrs,const uint8_t _Ladrs);

};

#endif