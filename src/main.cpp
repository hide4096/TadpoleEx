#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "icm20648.h"
#include "lps25hb.h"
#include "VL53L1X.h"
#include "esp32-hal-ledc.h"
#include "sbus.h"

//SBUS入力
bfs::SbusRx sbus_rx(&Serial2);
std::array<int16_t,bfs::SbusRx::NUM_CH()> sbus_data;

//大気圧センサ

const uint8_t _ALT_CS = 15;
lps25hb alt;

//6軸IMUセンサ
const uint8_t _IMU_CS = 5;
icm20648 imu;


/*
  PWM_CH  基板の表示   サーボ
  0       CH1         AIL(未使用)
  1       CH2         ELE
  2       CH3         THR
  3       CH4         RUD
  4       CH5         AUX(投下機構)
*/
const uint16_t _PWM_DutyMin = 3542;
const uint16_t _PWM_DutyMax = 6194;
const uint16_t _PWM_DutyCenter = (_PWM_DutyMax - _PWM_DutyMin) / 2 + _PWM_DutyMin;
const uint8_t _CH_Port[5] = {27,26,25,33,32};
const uint16_t _CH_Neutral[5] = {_PWM_DutyCenter,_PWM_DutyCenter,_PWM_DutyMin,_PWM_DutyCenter,_PWM_DutyCenter};

const float _SBUS2PWM = 2.021;

String duty;
/*
  CH5_State
  0 → 右投下
  1 → 左投下
*/
uint8_t CH5_State = 0;
bool is_CH5_reset = true;

VL53L1X dist;

void setup() {
  Serial.begin(115200);
  sbus_rx.Begin(16,17);

  for(uint8_t i = 0;i<5;i++){
    ledcSetup(i,50,16);
    ledcAttachPin(_CH_Port[i],i);
    ledcWrite(i,_CH_Neutral[i]);
  }

  //センサ初期化
  /*
  alt = new SPIClass(HSPI);
  alt->begin();
  alt->beginTransaction(SPISettings(1000*1000,MSBFIRST,SPI_MODE3));
  digitalWrite(ALT_CS,LOW);

  pinMode(IMU_CS,OUTPUT);
  digitalWrite(IMU_CS,HIGH);
  imu = new SPIClass(VSPI);
  imu->begin();

  pinMode(4,OUTPUT);
  digitalWrite(4,LOW);

  Wire.begin();
  Wire.setClock(400 * 1000);
  dist.setTimeout(1000);

  dist.init();
  dist.setDistanceMode(VL53L1X::Long);
  dist.setMeasurementTimingBudget(50000);
  dist.startContinuous(50);
  */

 if(imu.init(new SPIClass(VSPI),1000000,_IMU_CS) == -1) while(1);
 if(alt.init(new SPIClass(HSPI),1000000,_ALT_CS) == -1) while(1);

}

void loop() {
  if(sbus_rx.Read()){
    sbus_data = sbus_rx.ch();
    for(uint8_t i = 0;i<4;i++){
      uint16_t DutyRatio = (sbus_data[i] - 1024) * _SBUS2PWM + _PWM_DutyCenter;
      ledcWrite(i,DutyRatio);
    }
    if(sbus_data[4] > 1024){
      if(is_CH5_reset){
        if(CH5_State){ 
          ledcWrite(4,_PWM_DutyMax);
        }else{
          ledcWrite(4,_PWM_DutyMin);
        }
        CH5_State = 1 - CH5_State;
        is_CH5_reset = false;
      }
    }else{
      is_CH5_reset = true;
      ledcWrite(4,_PWM_DutyCenter);
    }
  }
 //Serial.println(dist.read());
}