#include "icm20648.h"

#define WHO_AM_I 0xE0
#define TEMP_SENSITIVITY 333.87
#define TEMP_OFFSET 21.0
#define RETRY_INIT 5

uint8_t icm20648::readRegister(const uint8_t _adrs){
    uint8_t recv = 0,adrs = _adrs | 0x80;

    mywire->beginTransaction(spi_settings);

    digitalWrite(cs,LOW);
    mywire->transfer(adrs);
    recv = mywire->transfer(0);
    digitalWrite(cs,HIGH);

    mywire->endTransaction();
    
    return recv;
}

void icm20648::writeRegister(const uint8_t _adrs,const uint8_t _data){
    //delay(1);
    uint8_t adrs = _adrs & 0x7F;
    mywire->beginTransaction(spi_settings);

    digitalWrite(cs,LOW);
    mywire->transfer(_adrs);
    mywire->transfer(_data);
    digitalWrite(cs,HIGH);

    mywire->endTransaction();
}

uint16_t icm20648::readRegister_2Byte(const uint8_t _Hadrs,const uint8_t _Ladrs){
    uint8_t Hrecv,Lrecv;
    Hrecv = readRegister(_Hadrs);
    Lrecv = readRegister(_Ladrs);
    return Hrecv << 8 | Lrecv;
}

int icm20648::init(SPIClass* wire,const uint32_t _clock,const int _cs){
    mywire = wire;
    cs = _cs;
    spi_settings = SPISettings(_clock,MSBFIRST,SPI_MODE3);

    pinMode(cs,OUTPUT);
    digitalWrite(cs,HIGH);
    wire->begin();

    int errcnt = 0;
    uint8_t whoami = readWHO_AM_I();
    while(whoami != WHO_AM_I){
        delay(100);
        whoami = readWHO_AM_I();
        errcnt++;
        if(errcnt >= RETRY_INIT){
            wire->end();
            return -1;
        }
    }

    //スリープ解除
    writeRegister(0x06,0x01);
    //レンジの調整
    writeRegister(0x7F,0x20);
    writeRegister(0x01,0x06);
    writeRegister(0x14,0x02);
    writeRegister(0x7F,0x00);

    return 0;
}

uint8_t icm20648::readWHO_AM_I(){
    return readRegister(0x00);
}

uint16_t icm20648::accelX(){
    return readRegister_2Byte(0x2D,0x2E);
}

uint16_t icm20648::accelY(){
    return readRegister_2Byte(0x2F,0x30);
}

uint16_t icm20648::accelZ(){
    return readRegister_2Byte(0x31,0x32);
}

uint16_t icm20648::gyroX(){
    return readRegister_2Byte(0x33,0x34);
}

uint16_t icm20648::gyroY(){
    return readRegister_2Byte(0x35,0x36);
}

uint16_t icm20648::gyroZ(){
    return readRegister_2Byte(0x37,0x38);
}

float icm20648::temp(){
    uint16_t temp = readRegister_2Byte(0x39,0x3A);
    return temp/TEMP_SENSITIVITY+TEMP_OFFSET;
}