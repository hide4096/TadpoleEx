#include <Arduino.h>
#include "esp32-hal-ledc.h"
#include "sbus.h"

bfs::SbusRx sbus_rx(&Serial2);
std::array<int16_t,bfs::SbusRx::NUM_CH()> sbus_data;

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

void setup() {
  Serial.begin(115200);
  sbus_rx.Begin(16,17);

  for(uint8_t i = 0;i<5;i++){
    ledcSetup(i,50,16);
    ledcAttachPin(_CH_Port[i],i);
    ledcWrite(i,_CH_Neutral[i]);
  }
}

void loop() {
  if(sbus_rx.Read()){
    sbus_data = sbus_rx.ch();
    for(uint8_t i = 0;i<5;i++){
      uint16_t DutyRatio = (sbus_data[i] - 1024) * _SBUS2PWM + _PWM_DutyCenter;
      ledcWrite(i,DutyRatio);
    }
  }
}