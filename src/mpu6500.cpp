#include"mpu6500.h"
#include<Arduino.h>

uint8_t mpu6500::readRegister(const uint8_t _adrs){
    uint8_t recv = 0,adrs = _adrs | 0x80;

    mywire->beginTransaction(spi_settings);

    digitalWrite(cs,LOW);
    mywire->transfer(adrs);
    recv = mywire->transfer(0);
    digitalWrite(cs,HIGH);

    mywire->endTransaction();
    
    return recv;
}

void mpu6500::writeRegister(const uint8_t _adrs,const uint8_t _data){
    vTaskDelay(1/portTICK_PERIOD_MS);
    uint8_t adrs = _adrs & 0x7F;
    mywire->beginTransaction(spi_settings);

    digitalWrite(cs,LOW);
    mywire->transfer(adrs);
    mywire->transfer(_data);
    digitalWrite(cs,HIGH);

    mywire->endTransaction();
}

uint16_t mpu6500::readRegister_2Byte(const uint8_t _Hadrs,const uint8_t _Ladrs){
    uint8_t Hrecv,Lrecv;
    Hrecv = readRegister(_Hadrs);
    Lrecv = readRegister(_Ladrs);
    return Hrecv << 8 | Lrecv;
}

int mpu6500::init(SPIClass* wire,const int _cs){
    mywire = wire;
    cs = _cs;
    spi_settings = SPISettings(7000000,MSBFIRST,SPI_MODE3);

    pinMode(cs,OUTPUT);
    digitalWrite(cs,HIGH);

    int errcnt = 0;
    uint8_t whoami = readWHO_AM_I();
    while(whoami != MPU6500_WHO_AM_I){
        delay(100);
        whoami = readWHO_AM_I();
        Serial.println(whoami,HEX);
        errcnt++;
        if(errcnt >= RETRY_INIT){
            wire->end();
            return -1;
        }
    }

    //IMUリセット
    writeRegister(0x6B,0b10000000);
    vTaskDelay(10/portTICK_PERIOD_MS);
    //スリープモード無効&温度センサ無効
    writeRegister(0x06,0b00001001);
    //レンジの調整
    if(changeSensitivity(3,3) == -1){
        wire->end();
        return -1;
    }

    return 0;
}

/*
    ジャイロ
    0   →   ±500dps
    1   →   ±1000dps
    2   →   ±2000dps
    3   →   ±4000dps
    加速度
    0   →   ±4g
    1   →   ±8g
    2   →   ±16g
    3   →   ±30g
*/
int mpu6500::changeSensitivity(uint8_t _gyro,uint8_t _accel){
    if(_gyro > 0b11 || _accel > 0b11) return -1;
    uint8_t gyro_config=_gyro << 3,accel_config=_accel<<3;

    writeRegister(0x1B,gyro_config);
    writeRegister(0x1C,accel_config);
    if((readRegister(0x1B) & 0b11000) != gyro_config) return -1;
    if((readRegister(0x1C) & 0b11000) != accel_config) return -1;

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

    return 0;
}

uint8_t mpu6500::readWHO_AM_I(){
    return readRegister(0x75);
}

int16_t mpu6500::accelX_raw(){
    return readRegister_2Byte(0x3B,0x3C);
}

int16_t mpu6500::accelY_raw(){
    return readRegister_2Byte(0x3D,0x3E);
}

int16_t mpu6500::accelZ_raw(){
    return readRegister_2Byte(0x3F,0x40);
}

int16_t mpu6500::gyroX_raw(){
    return readRegister_2Byte(0x67,0x68);
}

int16_t mpu6500::gyroY_raw(){
    return readRegister_2Byte(0x69,0x6A);
}

int16_t mpu6500::gyroZ_raw(){
    return readRegister_2Byte(0x6B,0x6C);
}

float mpu6500::temp(){
    uint16_t temp = readRegister_2Byte(0x41,0x42);
    return temp/TEMP_SENSITIVITY+TEMP_OFFSET;
}

float mpu6500::gyroX(){
    return (float)gyroX_raw() * gyro_sensitivity;
}

float mpu6500::gyroY(){
    return (float)gyroY_raw() * gyro_sensitivity;
}

float mpu6500::gyroZ(){
    return (float)gyroZ_raw() * gyro_sensitivity;
}

float mpu6500::accelX(){
    return (float)accelX_raw() * accel_sensitivity;
}

float mpu6500::accelY(){
    return (float)accelY_raw() * accel_sensitivity;
}

float mpu6500::accelZ(){
    return (float)accelZ_raw() * accel_sensitivity;
}