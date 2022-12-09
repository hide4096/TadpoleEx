/* 
########################
    LPS-25HB Driver
########################
SPDX-FileCopyrightText: 2022 Aso Hidetoshi asouhide2002@gmail.com
SPDX-License-Identifier: BSD-3-Clause

    Date			Author              Notes
    2022/8/22      Aso Hidetoshi       First release
*/
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