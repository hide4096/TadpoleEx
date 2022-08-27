#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Ticker.h>
#include "icm20648.h"
#include "lps25hb.h"
#include "VL53L1X.h"
#include "esp32-hal-ledc.h"
#include "sbus.h"
#include "MadgwickAHRS.h"


#define AUTOLED 4

bfs::SbusRx sbus_rx(&Serial2);  //SBUS入力
std::array<int16_t,bfs::SbusRx::NUM_CH()> sbus_data;

lps25hb alt;  //大気圧センサ
const uint8_t _ALT_CS = 15;

icm20648 imu; //6軸IMUセンサ
const uint8_t _IMU_CS = 5;

VL53L1X dist; //ToFセンサ

//Madgwickフィルタ
Madgwick mdf;
float pitch,yaw,roll;
hw_timer_t* timer = NULL;

//姿勢取得
void IRAM_ATTR getAttitude(){
  mdf.updateIMU(imu.gyroX(),imu.gyroY(),imu.gyroZ(),imu.accelX(),imu.accelY(),imu.accelZ());
  roll = mdf.getPitch()*-1.0;
  yaw = mdf.getYaw()*-1.0;
  pitch = mdf.getRoll()*-1.0;
}

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
int16_t DutyRatio[5];

/*
  CH5_State
  0 → 右投下
  1 → 左投下
*/
uint8_t CH5_State = 0;
bool is_CH5_reset = true;

void controlServo(){
  //投下機構
  if(sbus_data[4] > 1024){
    if(is_CH5_reset){
      if(CH5_State == 0){
        DutyRatio[4] = 0;
      }else{
        DutyRatio[4] = 2047;
      }
      CH5_State = 1 - CH5_State;
      is_CH5_reset = false;
    }
  }else{
      is_CH5_reset = true;
      DutyRatio[4] = 980;
  }

  //PWM書き込み
  for(uint8_t i = 0;i<5;i++){
    if(DutyRatio[i] > 2047) DutyRatio[i] = 2047;
    else if(DutyRatio[i] < 0) DutyRatio[i] = 0;
    ledcWrite(i,(DutyRatio[i] - 1024) * _SBUS2PWM + _PWM_DutyCenter);
  }
}

Ticker fetcher;

//メイン処理(1kHz)
void fetchSensors(){
  getAttitude();
}

void controlServos(void *pvParameters){
  while(1){
  if(sbus_rx.Read()){
    sbus_data = sbus_rx.ch();
    for(uint8_t i = 0;i<4;i++){
      DutyRatio[i] = sbus_data[i];
    }
  }

  //投下機構
  if(sbus_data[4] > 1024){
    if(is_CH5_reset){
      if(CH5_State == 0){
        DutyRatio[4] = 0;
      }else{
        DutyRatio[4] = 2047;
      }
      CH5_State = 1 - CH5_State;
      is_CH5_reset = false;
    }
  }else{
      is_CH5_reset = true;
      DutyRatio[4] = 980;
  }

  //PWM書き込み
  for(uint8_t i = 0;i<5;i++){
    if(DutyRatio[i] > 2047) DutyRatio[i] = 2047;
    else if(DutyRatio[i] < 0) DutyRatio[i] = 0;
    ledcWrite(i,(DutyRatio[i] - 1024) * _SBUS2PWM + _PWM_DutyCenter);
  }
  }
}

void setup() {
  Serial.begin(115200);
  sbus_rx.Begin(16,17);

  for(uint8_t i = 0;i<5;i++){
    ledcSetup(i,50,16);
    ledcAttachPin(_CH_Port[i],i);
    ledcWrite(i,_CH_Neutral[i]);
  }

  pinMode(AUTOLED,OUTPUT);
  digitalWrite(AUTOLED,LOW);

  /*
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

  mdf.begin(1000);

  fetcher.attach_ms(1,fetchSensors);
  xTaskCreateUniversal(controlServos,"controlServos",8192,NULL,1,NULL,APP_CPU_NUM);
}

void loop() {
  Serial.printf("%f,%f,%f\r\n",pitch,yaw,roll);
  delay(100);
}