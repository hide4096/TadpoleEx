#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Ticker.h>
#include <WiFi.h>
#include <AsyncUDP.h>
#include "icm20648.h"
#include "lps25hb.h"
#include "VL53L1X.h"
#include "esp32-hal-ledc.h"
#include "sbus.h"
#include "MadgwickAHRS.h"

#define AUTOLED 4
#define WIFI_TIMEOUTMS 5000
#define LPF_GAIN 0.001

//セマフォとかタイマーとか
volatile SemaphoreHandle_t sp_control;
volatile SemaphoreHandle_t sp_sensor;
volatile SemaphoreHandle_t sp_wifi;
hw_timer_t* tm_control = NULL;
hw_timer_t* tm_sensor = NULL;
hw_timer_t* tm_wifi = NULL;

//デバッグ用WiFiAP
const char *ssid = "TadpoleEx";
const char *pass = "07033208416";
AsyncUDP udp; 

//SBUS入力
bfs::SbusRx sbus_rx(&Serial2);
std::array<int16_t,bfs::SbusRx::NUM_CH()> sbus_data;

//大気圧センサ
lps25hb alt;
const uint8_t _ALT_CS = 15;

//6軸IMUセンサ
icm20648 imu;
const uint8_t _IMU_CS = 5;
float pitch,yaw,roll;
float x,y,z;
float lx,ly,lz;
float ox=0,oy=0,oz=0;
Madgwick mdf;
void getAttitude(){
  x = imu.gyroX()-ox,y = imu.gyroY()-oy,z=imu.gyroZ()-oz;
  lx = LPF_GAIN * lx + (1.0 - LPF_GAIN) * x;
  ly = LPF_GAIN * ly + (1.0 - LPF_GAIN) * y;
  lz = LPF_GAIN * lz + (1.0 - LPF_GAIN) * z;

  mdf.updateIMU(lx*-1.0,ly,lz*-1.0,imu.accelX()*-1.0,imu.accelY(),imu.accelZ()*-1.0);
  pitch = mdf.getRoll();
  yaw = mdf.getYaw();
  roll = mdf.getPitch();
}

//ToFセンサ
VL53L1X dist;

/*
  PWM_CH  基板の表示   サーボ
  0       CH1         AIL(未使用)
  1       CH2         ELE
  2       CH3         THR
  3       CH4         RUD
  4       CH5         AUX(投下機構)

  CH5_State
  0 → 右投下
  1 → 左投下
*/

const uint16_t _PWM_DutyMin = 3542;
const uint16_t _PWM_DutyMax = 6194;
const uint16_t _PWM_DutyCenter = (_PWM_DutyMax - _PWM_DutyMin) / 2 + _PWM_DutyMin;
const uint8_t _CH_Port[5] = {27,26,25,33,32};
const uint16_t _CH_Neutral[5] = {_PWM_DutyCenter,_PWM_DutyCenter,_PWM_DutyMin,_PWM_DutyCenter,_PWM_DutyCenter};
const float _SBUS2PWM = 2.021;
int16_t DutyRatio[5];
uint8_t CH5_State = 0;
bool is_CH5_reset = true;

//サーボ制御
void controlServos(){
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
      DutyRatio[4] = 960;
  }

  //PWM書き込み
  for(uint8_t i = 0;i<5;i++){
    if(DutyRatio[i] > 2047) DutyRatio[i] = 2047;
    else if(DutyRatio[i] < 0) DutyRatio[i] = 0;
    ledcWrite(i,(DutyRatio[i] - 1024) * _SBUS2PWM + _PWM_DutyCenter);
  }
}

//SBUS取得
void readSBUS(){
  if(sbus_rx.Read()){
    sbus_data = sbus_rx.ch();
    for(uint8_t i = 0;i<5;i++){
      DutyRatio[i] = sbus_data[i];
    }
  }
}

//機体情報をUDPで放送
void sendUDP(){
    char str[128];
    sprintf(str,"%ldms,%.3f,%.3f,%.3f,%d,%d,%d,%.3f",
      esp_timer_get_time()/1000,
      pitch,yaw,roll,
      sbus_data[1],sbus_data[2],sbus_data[3],
      imu.accelY()
    );
    udp.broadcastTo(str,8901);
    delay(1);
}

//PID制御
float before_pitch,I_pitch;
float pitch_kp=10.0,pitch_ki=0.0,pitch_kd=8.0;
float target_pitch = 15.0;
float before_roll,I_roll;
float roll_kp=10.0,roll_ki=0.0,roll_kd=8.0;
float target_roll = 50.0;
void PID(){
  if(sbus_data[5] > 1024) return;
  float diff_pitch = target_pitch - pitch;
  DutyRatio[1] -= diff_pitch * pitch_kp + I_pitch * pitch_ki + (before_pitch - pitch) * pitch_kd;
  I_pitch += diff_pitch;
  before_pitch = pitch;

  float diff_roll = target_roll - roll;
  DutyRatio[3] -= diff_roll * roll_kp + I_roll * roll_ki + (before_roll - roll) * roll_kd;
  I_roll += diff_roll;
  before_roll = roll;
}

//制御系（100Hz）
void control(void *pvParam){
  while(1){
    xSemaphoreTake(sp_control,portMAX_DELAY);
    readSBUS();
    PID();
    controlServos();
  }
}

//センサ処理(1kHz)
void fetchSensors(void *pvParam){
  while(1){
    xSemaphoreTake(sp_sensor,portMAX_DELAY);
    getAttitude();
  }
}

//WiFi通信(100Hz,CPU0)
void commViaWiFi(void *pvParam){
  while (1){
    xSemaphoreTake(sp_wifi,portMAX_DELAY);
    sendUDP();
  }
}

void IRAM_ATTR onTimer_control(){
  xSemaphoreGiveFromISR(sp_control,NULL);
}
void IRAM_ATTR onTimer_sensor(){
  xSemaphoreGiveFromISR(sp_sensor,NULL);
}
void IRAM_ATTR onTimer_wifi(){
  xSemaphoreGiveFromISR(sp_wifi,NULL);
}

void setup() {
  //シリアル通信開始
  Serial.begin(115200);
  sbus_rx.Begin(16,17);

  //PWM設定
  for(uint8_t i = 0;i<5;i++){
    ledcSetup(i,50,16);
    ledcAttachPin(_CH_Port[i],i);
    ledcWrite(i,_CH_Neutral[i]);
  }

  //センサ初期化
  if(imu.init(new SPIClass(VSPI),1000000,_IMU_CS) == -1){
    Serial.printf("IMU init failed.\r\n");
    while(1);
  }

  vTaskDelay(3000/portTICK_RATE_MS);
  for(int i=0;i<1000;i++){
    ox+=imu.gyroX();
    oy+=imu.gyroY();
    oz+=imu.gyroZ();
    vTaskDelay(1/portTICK_RATE_MS);
  }
  ox/=1000.0,oy/=1000.0,oz/=1000.0;

  /*
  if(alt.init(new SPIClass(HSPI),1000000,_ALT_CS) == -1){
    Serial.printf("ALT init failed.\r\n");
    while(1);
    }
  */

  /*
  Wire.begin();
  Wire.setClock(400 * 1000);
  dist.setTimeout(1000);
  dist.init();
  dist.setDistanceMode(VL53L1X::Long);
  dist.setMeasurementTimingBudget(50000);
  dist.startContinuous(50);
  */

  //LED設定
  pinMode(AUTOLED,OUTPUT);
  digitalWrite(AUTOLED,LOW);

  //セマフォ生成
  sp_control = xSemaphoreCreateBinary();
  sp_sensor = xSemaphoreCreateBinary();
  sp_wifi = xSemaphoreCreateBinary();

  //WiFi接続
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid,pass);
  if(WiFi.waitForConnectResult(WIFI_TIMEOUTMS) != WL_CONNECTED) Serial.println("fail");

  //フィルタ起動
  mdf.begin(1000);

  //タスク追加
  xTaskCreateUniversal(control,"control",8192,NULL,1,NULL,APP_CPU_NUM);
  xTaskCreateUniversal(fetchSensors,"sensor",8192,NULL,configMAX_PRIORITIES,NULL,APP_CPU_NUM);
  xTaskCreateUniversal(commViaWiFi,"wifi",8192,NULL,1,NULL,PRO_CPU_NUM);

  //タイマー設定
  //タイマー1が死んでるw
  tm_control = timerBegin(0,getApbFrequency()/1000000,true);
  tm_sensor = timerBegin(2,getApbFrequency()/1000000,true);
  tm_wifi = timerBegin(3,getApbFrequency()/1000000,true);
  timerAttachInterrupt(tm_control, &onTimer_control, true);
  timerAttachInterrupt(tm_sensor, &onTimer_sensor, true);
  timerAttachInterrupt(tm_wifi, &onTimer_wifi, true);

  //割り込み設定
  timerAlarmWrite(tm_control,10000,true);
  timerAlarmWrite(tm_sensor,1000,true);
  timerAlarmWrite(tm_wifi,10000,true);

  //タイマー起動
  timerAlarmEnable(tm_control);
  timerAlarmEnable(tm_sensor);
  timerAlarmEnable(tm_wifi);

  Serial.printf("setup finished.\r\n");
  }


void loop() {
  delay(1);
}