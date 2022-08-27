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
    changeSensitivity(2,1);

    return 0;
}

/*
    ジャイロ
    0   →   ±250dps
    1   →   ±500dps
    2   →   ±1000dps
    3   →   ±2000dps
    加速度
    0   →   ±2g
    1   →   ±4g
    2   →   ±8g
    3   →   ±16g
*/
int icm20648::changeSensitivity(const uint8_t _gyro,const uint8_t _accel){
    if(_gyro > 0b11 || _accel > 0b11) return -1;

    switch(_gyro){
        case 0:
            gyro_sensitivity = 250.0 / 32768.0;
            break;
        case 1:
            gyro_sensitivity = 500.0 / 32768.0;
            break;
        case 2:
            gyro_sensitivity = 1000.0 / 32768.0;
            break;
        case 3:
            gyro_sensitivity = 2000.0 / 32768.0;
            break;
    }
    switch(_accel){
        case 0:
            accel_sensitivity = 2.0 / 32768.0;
            break;
        case 1:
            accel_sensitivity = 4.0 / 32768.0;
            break;
        case 2:
            accel_sensitivity = 8.0 / 32768.0;
            break;
        case 3:
            accel_sensitivity = 16.0 / 32768.0;
            break;
    }
    uint8_t gyro_config=_gyro << 1,accel_config=_accel<<1;

    writeRegister(0x7F,0x20);   //バンク切り替え(バンク2)
    writeRegister(0x01,gyro_config);
    writeRegister(0x14,accel_config);
    writeRegister(0x7F,0x00);   //バンク切り替え(バンク0)

    return 0;
}

uint8_t icm20648::readWHO_AM_I(){
    return readRegister(0x00);
}

int16_t icm20648::accelX_raw(){
    return readRegister_2Byte(0x2D,0x2E);
}

int16_t icm20648::accelY_raw(){
    return readRegister_2Byte(0x2F,0x30);
}

int16_t icm20648::accelZ_raw(){
    return readRegister_2Byte(0x31,0x32);
}

int16_t icm20648::gyroX_raw(){
    return readRegister_2Byte(0x33,0x34);
}

int16_t icm20648::gyroY_raw(){
    return readRegister_2Byte(0x35,0x36);
}

int16_t icm20648::gyroZ_raw(){
    return readRegister_2Byte(0x37,0x38);
}

float icm20648::temp(){
    uint16_t temp = readRegister_2Byte(0x39,0x3A);
    return temp/TEMP_SENSITIVITY+TEMP_OFFSET;
}

float icm20648::gyroX(){
    return (float)gyroX_raw() * gyro_sensitivity;
}

float icm20648::gyroY(){
    return (float)gyroY_raw() * gyro_sensitivity;
}

float icm20648::gyroZ(){
    return (float)gyroZ_raw() * gyro_sensitivity;
}

float icm20648::accelX(){
    return (float)accelX_raw() * accel_sensitivity;
}

float icm20648::accelY(){
    return (float)accelY_raw() * accel_sensitivity;
}

float icm20648::accelZ(){
    return (float)accelZ_raw() * accel_sensitivity;
}