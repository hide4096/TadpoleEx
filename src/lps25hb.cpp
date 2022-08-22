#include "lps25hb.h"

#define WHO_AM_I 0xBD
#define RETRY_INIT 5

uint8_t lps25hb::readRegister(const uint8_t _adrs){
    uint8_t recv = 0,adrs = _adrs | 0x80;

    mywire->beginTransaction(spi_settings);

    digitalWrite(cs,LOW);
    mywire->transfer(adrs);
    recv = mywire->transfer(0);
    digitalWrite(cs,HIGH);

    mywire->endTransaction();
    
    return recv;
}

void lps25hb::writeRegister(const uint8_t _adrs,const uint8_t _data){
    //delay(1);
    uint8_t adrs = _adrs & 0x7F;
    mywire->beginTransaction(spi_settings);

    digitalWrite(cs,LOW);
    mywire->transfer(_adrs);
    mywire->transfer(_data);
    digitalWrite(cs,HIGH);

    mywire->endTransaction();
}

uint16_t lps25hb::readRegister_2Byte(const uint8_t _Hadrs,const uint8_t _Ladrs){
    uint8_t Hrecv,Lrecv;
    Hrecv = readRegister(_Hadrs);
    Lrecv = readRegister(_Ladrs);
    return Hrecv << 8 | Lrecv;
}

uint32_t lps25hb::readRegister_3Byte(const uint8_t _Hadrs,const uint8_t _Ladrs,const uint8_t _XLadrs){
    uint8_t Hrecv,Lrecv,XLrecv;
    Hrecv = readRegister(_Hadrs);
    Lrecv = readRegister(_Ladrs);
    XLrecv = readRegister(_XLadrs);
    return Hrecv << 16 | Lrecv << 8 | XLrecv;
}

int lps25hb::init(SPIClass* wire,const uint32_t _clock,const int _cs){
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

    writeRegister(0x20,0b11000000);
    writeRegister(0x21,0b01000000);
    writeRegister(0x2E,0b11000111);

    zero_press = press();
    zero_temp = temp();
    Serial.println(zero_press);

    return 0;
}

uint8_t lps25hb::readWHO_AM_I(){
    return readRegister(0x0F);
}

float lps25hb::press(){
    int32_t press = readRegister_3Byte(0x2A,0x29,0x28);
    return (float)press / 4096.0;
}

float lps25hb::temp(){
    int16_t rawData = readRegister_2Byte(0x2C,0x2B);
    return (float)rawData / 480 + 42.5;
}

float lps25hb::altitude(){
    return (zero_temp/-0.0065)*(pow((press()/zero_press),0.19026324)-1);
}