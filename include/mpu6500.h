#ifndef MPU6500_H
#define MPU6500_H

#include <SPI.h>

#define MPU6500_WHO_AM_I 0x70
#define RETRY_INIT 5
#define TEMP_SENSITIVITY 333.87
#define TEMP_OFFSET 21.0

class mpu6500{
    public:
        int init(SPIClass* wire,const int _cs);
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