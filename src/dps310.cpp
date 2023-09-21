#include"dps310.h"
#include<Arduino.h>

uint8_t dps310::readRegister(uint8_t _adrs){
    uint8_t recv = 0,adrs = _adrs | 0x80;

    mywire->beginTransaction(spi_settings);

    digitalWrite(cs,LOW);
    mywire->transfer(adrs);
    recv = mywire->transfer(0);
    digitalWrite(cs,HIGH);

    mywire->endTransaction();
    
    return recv;
}

void dps310::writeRegister(uint8_t _adrs,uint8_t _data){
    vTaskDelay(1/portTICK_PERIOD_MS);
    uint8_t adrs = _adrs & 0x7F;
    mywire->beginTransaction(spi_settings);

    digitalWrite(cs,LOW);
    mywire->transfer(adrs);
    mywire->transfer(_data);
    digitalWrite(cs,HIGH);

    mywire->endTransaction();
}

void dps310::getTwoComplement(int32_t *raw,uint8_t length){
    if(*raw & ((uint32_t)1 << (length - 1))){
        *raw -= (uint32_t)1<<length;
    }
}

void dps310::setZeroPoint(){
    updateSensor();
    zero_press = pressure;
    zero_temp = temp;
}

int dps310::init(SPIClass* wire,int _cs,uint8_t _PM_RATE,uint8_t _PM_PRC,uint8_t _TMP_RATE,uint8_t _TMP_PRC){
    cs = _cs;
    mywire = wire;
    spi_settings = SPISettings(10000000,MSBFIRST,SPI_MODE3);

    pinMode(cs,OUTPUT);
    digitalWrite(cs,HIGH);

    PM_RATE = _PM_RATE;
    PM_PRC = _PM_PRC;
    TMP_RATE = _TMP_RATE;
    TMP_PRC = _TMP_PRC;

    switch(PM_PRC){
        case 0b0000:
            kP = PRC_1;
            break;
        case 0b0001:
            kP = PRC_2;
            break;
        case 0b0010:
            kP = PRC_4;
            break;
        case 0b0011:
            kP = PRC_8;
            break;
        case 0b0100:
            kP = PRC_16;
            break;
        case 0b0101:
            kP = PRC_32;
            break;
        case 0b0110:
            kP = PRC_64;
            break;
        case 0b0111:
            kP = PRC_128;
            break;
        default:
            return -1;
    }
    switch(TMP_PRC){
        case 0b0000:
            kT = PRC_1;
            break;
        case 0b0001:
            kT = PRC_2;
            break;
        case 0b0010:
            kT = PRC_4;
            break;
        case 0b0011:
            kT = PRC_8;
            break;
        case 0b0100:
            kT = PRC_16;
            break;
        case 0b0101:
            kT = PRC_32;
            break;
        case 0b0110:
            kT = PRC_64;
            break;
        case 0b0111:
            kT = PRC_128;
            break;
        default:
            return -1;
    }

    int errcnt = 0;
    uint8_t whoami = readRegister(0x0D);
    while(whoami != DPS310_WHO_AM_I){
        delay(100);
        whoami = readRegister(0x0D);
        Serial.println(whoami,HEX);
        errcnt++;
        if(errcnt >= RETRY_INIT){
            wire->end();
            return -1;
        }
    }

    writeRegister(0x28,0b10000000);   //温度センサ指定
    vTaskDelay(1/portTICK_PERIOD_MS);
    if(readRegister(0x28) >> 7 != 1){
        wire->end();
        return -1;
    }
    //読み込み速度の指定
    uint8_t PRS_CFG = PM_RATE << 4 | PM_PRC;
    uint8_t TMP_CFG = 1 << 7 | TMP_RATE << 4 | TMP_PRC;
    uint8_t MEAS_CFG = 0b111;
    writeRegister(0x06,PRS_CFG);   //気圧のレートと解像度（オーバーサンプリング数）
    writeRegister(0x07,TMP_CFG);   //気温のレートと解像度（オーバーサンプリング数）
    writeRegister(0x08,MEAS_CFG);   //BackGroundで温度と気圧を読み込む

    uint8_t CFG_REG = 0;
    if(PM_PRC >= 0b0011) CFG_REG |= 0b100;
    if(TMP_PRC >= 0b0011) CFG_REG |= 0b1000;
    writeRegister(0x09,CFG_REG);   //FIFOは使わない

    if(readRegister(0x06) != PRS_CFG && readRegister(0x07) != TMP_CFG &&
        (readRegister(0x08) & 0b1111) != MEAS_CFG && readRegister(0x09) != CFG_REG){
        wire->end();
        return -1;
    }

    //センサ定数の準備完了まで待機
    uint8_t status = readRegister(0x08);
    while(status >> 7 == 0){
        status = readRegister(0x08);
        ets_delay_us(100);
    }

    //定数の取得
    uint8_t coef[18];
    for(int i = 0;i < 18;i++){
        coef[i] = readRegister(0x10 + i);
    }
    c0 = coef[0] << 4 | coef[1] >> 4;
    getTwoComplement(&c0,12);
    c1 = (coef[1] & 0b1111) << 8 | coef[2];
    getTwoComplement(&c1,12);
    c00 = coef[3] << 12 | coef[4] << 4 | coef[5] >> 4;
    getTwoComplement(&c00,20);
    c10 = (coef[5] & 0b1111) << 16 | coef[6] << 8 | coef[7];
    getTwoComplement(&c10,20);
    c01 = coef[8] << 8 | coef[9];
    getTwoComplement(&c01,16);
    c11 = coef[10] << 8 | coef[11];
    getTwoComplement(&c11,16);
    c20 = coef[12] << 8 | coef[13];
    getTwoComplement(&c20,16);
    c21 = coef[14] << 8 | coef[15];
    getTwoComplement(&c21,16);
    c30 = coef[16] << 8 | coef[17];
    getTwoComplement(&c30,16);

    //センサの初期化完了まで待機
    status = readRegister(0x08);
    while(((status >> 6) && 0b01) == 0){
        status = readRegister(0x08);
        ets_delay_us(100);
    }

    vTaskDelay(100/portTICK_PERIOD_MS);

    setZeroPoint();

    return 0;
}

int32_t dps310::getRawPressure(){
    int32_t P_raw =  readRegister(0x00) << 16 | readRegister(0x01) << 8 | readRegister(0x02);
    getTwoComplement(&P_raw,24);
    return P_raw;
}

int32_t dps310::getRawTemperature(){
    int32_t T_raw =  readRegister(0x03) << 16 | readRegister(0x04) << 8 | readRegister(0x05);
    getTwoComplement(&T_raw,24);
    return T_raw;
}

void dps310::updateSensor(){
    raw_pressure = getRawPressure();
    raw_temp = getRawTemperature();
    float P_raw = raw_pressure / (float)kP;
    float T_raw = raw_temp / (float)kT;
    pressure = c00 + P_raw * (c10 + P_raw * (c20 + P_raw * c30)) + T_raw * c01 + T_raw * P_raw * (c11 + P_raw * c21);
    temp = (c0*0.5) + (T_raw * c1);
    altitude = -67.85 * (zero_temp + 273.15) * log10(pressure/zero_press);
}