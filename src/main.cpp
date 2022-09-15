#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Ticker.h>
#include <WiFi.h>
#include <AsyncUDP.h>
#include <math.h>
#include "icm20648.h"
#include "VL53L1X.h"
#include "esp32-hal-ledc.h"
#include "sbus.h"
#include "MadgwickAHRS.h"
#include <WiFiScan.h>

#define AUTOLED 4
#define WIFI_TIMEOUT_MS 5000
#define LPF_GAIN 0.1
#define LPF_GAIN_ALT 0.8
#define LPF_GAIN_RSSI 0.2
#define DEG2RAD M_PI/180.0
#define I_MAX 10000

#define CRUSE_ALT 500
#define R_TXPOWER -55
#define L_TXPOWER -55
#define LAND_RSSI -35
#define DROP_RSSI -50

#define R_GAIN 20.0
#define L_GAIN 20.0
  
#define AIL 0
#define ELE 1
#define THR 2
#define RUD 3
#define CH5 4
#define CH6 5
#define CH7 6
#define CH8 7

const int _servofreq_Hz = 50;
const int _beaconfreq_Hz = 25;
const int _controlcycle_us = 1000000 / _servofreq_Hz;
const int _rssicycle_us = 1000000 / _beaconfreq_Hz;

//セマフォとかタイマーとか
volatile SemaphoreHandle_t sp_control;
volatile SemaphoreHandle_t sp_imu;
volatile SemaphoreHandle_t sp_rssi;
hw_timer_t* tm_control = NULL;
hw_timer_t* tm_imu = NULL;
hw_timer_t* tm_rssi = NULL;

//デバッグ用WiFiAP
const char *ssid = "TadpoleEx";
const char *pass = "07033208416"; //作者の電話番号
AsyncUDP udp; 

//SBUS入力
bfs::SbusRx sbus_rx(&Serial2);
std::array<int16_t,bfs::SbusRx::NUM_CH()> sbus_data;

//6軸IMUセンサ
icm20648 imu;
const uint8_t _IMU_CS = 5;
float pitch,yaw,roll;
float x_acc,y_acc,z_acc;
float pitch_gyr,yaw_gyr,roll_gyr;
float ax=0,ay=0,az=0;
float lx=0,ly=0,lz=0;
float ox=0,oy=0,oz=0;
Madgwick mdf;

//ToFセンサ
VL53L1X dist;
float distance_Tof = 0.0;
float altitude = 0.0;
float climb_speed = 0.0;

//クォータニオン
float q0q0,q1q1,q2q2,q3q3;
float q0q1,q0q2,q0q3;
float q1q2,q1q3;
float q2q3;

void IRAM_ATTR readSensor(){
  pitch = mdf.getRollRadians();
  yaw = mdf.getYawRadians();
  roll = mdf.getPitchRadians();
  pitch_gyr = lx;
  yaw_gyr = lz;
  roll_gyr = ly;

  distance_Tof = (q0q0 - q1q1 - q2q2 + q3q3) * dist.read();
  altitude = LPF_GAIN_ALT * altitude + (1.0 - LPF_GAIN_ALT) * distance_Tof;
}

void IRAM_ATTR getPosture(){
  //ジャイロにオフセットとローパスフィルタ
  float gx = (imu.gyroX()-ox)*-1.0,gy = imu.gyroY()-oy,gz=(imu.gyroZ()-oz)*-1.0;
  lx = LPF_GAIN * lx + (1.0 - LPF_GAIN) * gx;
  ly = LPF_GAIN * ly + (1.0 - LPF_GAIN) * gy;
  lz = LPF_GAIN * lz + (1.0 - LPF_GAIN) * gz;
  //加速度
  ax = imu.accelX()*-1.0,ay = imu.accelY(),az = imu.accelZ()*-1.0;
  //姿勢計算
  mdf.updateIMU(lx,ly,lz,ax,ay,az);

  //絶対座標系に変換
  float* q;
  mdf.getQuaternion(q);
  q0q0 = q[0]*q[0],q1q1 = q[1]*q[1],q2q2 = q[2]*q[2],q3q3 = q[3]*q[3];
  q0q1 = q[0]*q[1],q0q2 = q[0]*q[2],q0q3 = q[0]*q[3];
  q1q2 = q[1]*q[2],q1q3 = q[1]*q[3];
  q2q3 = q[2]*q[3];

  x_acc = (q0q0 + q1q1 - q2q2 - q3q3)*ax + 2*(q1q2 - q0q3)*ay + 2*(q1q3 + q0q2)*az;
  y_acc = 2*(q1q2 + q0q3)*ax + (q0q0 - q1q1 + q2q2 - q3q3)*ay + 2*(q2q3 - q0q1)*az;
  z_acc = 2*(q1q3 - q0q2)*ax + 2*(q2q3 + q0q1)*ay + (q0q0 - q1q1 - q2q2 + q3q3)*az;
}

//WiFi強度の取得
float RW_R = 0,RW_L = 0;
float RW_diff = 0.0;
int DROP = 0,RW_R_raw = 0,RW_L_raw = 0;
void GetRSSI(){
  int num_ap= WiFi.scanNetworks(false,false,false,30,9);
  for(int i = 0;i<num_ap;i++){
    String ssid_fetch = WiFi.SSID(i);
    if(ssid_fetch == "Runway_L") RW_L_raw = WiFi.RSSI(i);
    else if(ssid_fetch == "Runway_R") RW_R_raw = WiFi.RSSI(i);
    else if(ssid_fetch == "TadpoleEx") DROP = WiFi.RSSI(i);
  }
  if(RW_L_raw >= 0) RW_L_raw = -100.0;
  RW_L = pow(10.0,(L_TXPOWER - RW_L_raw) / L_GAIN);
  if(RW_R_raw >= 0) RW_R_raw = -100.0; 
  RW_R = pow(10.0,(R_TXPOWER - RW_R_raw) / R_GAIN);
  RW_diff = RW_diff * LPF_GAIN_RSSI + (RW_R - RW_L) * (1.0-LPF_GAIN_RSSI);
}

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

  SBUSのレンジ
  2^11(0~2047)
*/

const uint16_t _PWM_DutyMin = 3542;
const uint16_t _PWM_DutyMax = 6194;
const uint16_t _PWM_DutyCenter = (_PWM_DutyMax - _PWM_DutyMin) / 2 + _PWM_DutyMin;
const uint8_t _CH_Port[5] = {27,26,25,33,32};
const uint16_t _CH_Neutral[5] = {_PWM_DutyCenter,_PWM_DutyCenter,_PWM_DutyMin,_PWM_DutyCenter,_PWM_DutyCenter};
const float _SBUS2PWM = 2.021;
int16_t Output_SBUS[5];
uint8_t CH5_State = 0;
bool is_CH5_reset = true;

//サーボ制御
void IRAM_ATTR controlServos(){
  //投下機構
  if(sbus_data[CH5] > 1024){
    if(is_CH5_reset){
      if(CH5_State == 0){
        Output_SBUS[CH5] = 0;
      }else{
        Output_SBUS[CH5] = 2047;
      }
      CH5_State = 1 - CH5_State;
      is_CH5_reset = false;
    }
  }else{
      is_CH5_reset = true;
      Output_SBUS[CH5] = 960;
  }

  //PWM書き込み
  for(uint8_t i = 0;i<5;i++){
    if(Output_SBUS[i] > 2047) Output_SBUS[i] = 2047;
    else if(Output_SBUS[i] < 0) Output_SBUS[i] = 0;
    ledcWrite(i,(Output_SBUS[i] - 1024) * _SBUS2PWM + _PWM_DutyCenter);
  }
}

//SBUS取得
void IRAM_ATTR readSBUS(){
  if(sbus_rx.Read()){
    sbus_data = sbus_rx.ch();
  }
}

/*
  オートパイロット
  0 → 無効
  1 → 離陸
  2 → 着陸進入
  3 → 着陸
*/
uint8_t autopilot = 0;
bool is_drop = false;
uint16_t autothr = 0;

//PID制御
float before_pitch,I_pitch,target_pitch;
float pitch_kp=1000.0,pitch_ki = 0.0,pitch_kd=2000.0;

float before_roll,I_roll,target_roll;
float roll_kp=1000.0,roll_ki = 0.0,roll_kd=1000.0;

float before_alt,I_alt,target_alt;
float alt_kp=1.0,alt_ki = 0.0, alt_kd=0.5;

//方位（起動時の機首方向を0）指定で飛行
float before_auto,I_auto,target_auto;
float auto_kp=1.0,auto_ki = 0.0, auto_kd=0.0;

void IRAM_ATTR PIDcontrol(){
  float diff_pitch = target_pitch - pitch;
  Output_SBUS[ELE] += diff_pitch * pitch_kp + I_pitch * pitch_ki - (before_pitch - pitch) * pitch_kd;
  before_pitch = pitch;
  I_pitch += diff_pitch;
  if(I_pitch > I_MAX ) I_pitch = I_MAX;
  else if(I_pitch < -I_MAX) I_pitch = -I_MAX;
  
  switch(autopilot){
    case 1:
      Output_SBUS[THR] = 1800;
      break;
    case 2:
      Output_SBUS[THR] = 650;
      break;
    case 3:
      Output_SBUS[THR] = 0;
      break;
    default:
      break;
  }

  if(target_alt > 0){
    float diff_alt = target_alt - altitude;
    Output_SBUS[THR] += diff_alt * alt_kp + I_alt * alt_ki - (before_alt - altitude) * alt_kd;
    before_alt = diff_alt;
    I_alt += diff_alt;
    if(I_alt > I_MAX ) I_alt = I_MAX;
    else if(I_alt < -I_MAX) I_alt = -I_MAX;
  }

  

  if(autopilot == 2){
    float diff_auto = yaw - target_auto;
    target_roll = diff_auto * auto_kp + I_auto * auto_ki - (before_auto - diff_auto) * auto_kd;
    before_auto = diff_auto;
    I_auto += diff_auto;
    if(I_auto > I_MAX ) I_auto = I_MAX;
    else if(I_auto < -I_MAX) I_auto = -I_MAX;

    if(is_drop) sbus_data[CH5] = 0;
  }

  float diff_roll = target_roll - roll;
  Output_SBUS[RUD] -= diff_roll * roll_kp + I_roll * roll_ki - (before_roll - roll) * roll_kd;
  before_roll = roll;
  I_roll += diff_roll;
  if(I_roll > I_MAX ) I_roll = I_MAX;
  else if(I_roll < -I_MAX) I_roll = -I_MAX;
}

bool is_first_run = true;

void Modecontrol(){
  for(int i = 0;i<4;i++){
    Output_SBUS[i] = sbus_data[i];
  }
  //CH6で自動操縦切り替え
  if(sbus_data[CH6] > 1024){
    is_first_run = true;
    autopilot = false;
    digitalWrite(AUTOLED,LOW);
    return;
  }

  if(is_first_run){
    I_pitch = 0.0;
    I_roll = 0.0;
    I_alt = 0.0;
    before_alt = 0.0;
    before_pitch = 0.0;
    before_roll = 0.0;
    digitalWrite(AUTOLED,HIGH);
  }

  if(sbus_data[CH8] > 1536){
    if(is_first_run) target_alt = -1.0;
    target_roll = 0;
    target_pitch = 20.0 * DEG2RAD;
  }else if(sbus_data[CH8] < 512){
    if(is_first_run) target_alt = -1.0;
    target_roll = -35*DEG2RAD;
    target_pitch = 15.0 * DEG2RAD;
  }else{
    //自動操縦
    if(is_first_run){
      autopilot = 1;
      target_pitch = 15.0 * DEG2RAD;
      target_roll = 0.0;
      target_alt = -1.0;
      is_drop = false;
    }
    if(autopilot == 1){
      I_pitch = 0.0;
      I_roll = 0.0;
    }
    if(autopilot == 1 && altitude > 50.0){
      target_pitch = 5.0 * DEG2RAD;
      target_alt = CRUSE_ALT;
      target_auto = 45.0 * DEG2RAD;
      autopilot = 2;
    }
    if(autopilot == 2){
      if((RW_L + RW_R < 1.0)) autopilot = 3;
      if(DROP > DROP_RSSI) is_drop = true;
    }
    if(autopilot == 3){
      target_roll = 0.0;
      target_pitch = 0.0;
      target_alt = -1.0;
    }
  }
  if(is_first_run) is_first_run = false;
  PIDcontrol();
  
  return;
}

/*
  機体情報をUDPで放送
  
  起動からの経過時間[ms]
  ピッチ[rad] ヨー[rad] ロール[rad]
  エレベーター[SBUS] スロットル[SBUS] ラダー[SBUS]
  WiFi強度の差分,
  対地高度[mm]

*/

void IRAM_ATTR sendUDP(){
    char str[128];
    if(sbus_data[CH7] > 1024){
      sprintf(str,"%ld,%.3f,%.3f,%.3f,%d,%d,%d,%.3f,%.3f,%.3f,%d,%d,%d",
        esp_timer_get_time()/1000,
        pitch,yaw,roll,
        Output_SBUS[ELE],Output_SBUS[THR],Output_SBUS[RUD],
        (RW_L+RW_R)/2.0,RW_diff,
        altitude,autopilot,RW_L_raw,RW_R_raw
      );
    }else{
      sprintf(str,"Soiya");
    }
    udp.broadcastTo(str,8901);
}

//制御系（サーボのPWM周波数に合わせて50Hz）
void control(void *pvParam){
  while(1){
    xSemaphoreTake(sp_control,portMAX_DELAY);
    Modecontrol();
    controlServos();
  }
}

//センサ処理
const int _sensorfreq_Hz = 1000;
const int _sensorcycle_us = 1000000 / _sensorfreq_Hz;
void fetchIMU(void *pvParam){
  while(1){
    xSemaphoreTake(sp_imu,portMAX_DELAY);
    getPosture();
  }
}

//各種センサ取得
void fetchInput(void *pvParam){
  while (1){
    xSemaphoreTake(sp_control,portMAX_DELAY);
    readSBUS();
    readSensor();
    delay(1);
  }
}

//RSSI取得
void fetchRSSI(void *pvParam){
  while (1){
    xSemaphoreTake(sp_rssi,portMAX_DELAY);
    GetRSSI();
    delay(1);
  }
}

void IRAM_ATTR onTimer_control(){
  xSemaphoreGiveFromISR(sp_control,NULL);
}
void IRAM_ATTR onTimer_imu(){
  xSemaphoreGiveFromISR(sp_imu,NULL);
}
void IRAM_ATTR onTimer_rssi(){
  xSemaphoreGiveFromISR(sp_rssi,NULL);
}

bool is_udp_enable = true;

void setup() {
  //シリアル通信開始
  Serial.begin(115200);
  sbus_rx.Begin(16,17);

  //PWM設定
  for(uint8_t i = 0;i<5;i++){
    ledcSetup(i,_servofreq_Hz,16);
    ledcAttachPin(_CH_Port[i],i);
    ledcWrite(i,_CH_Neutral[i]);
  }

  //センサ初期化
  if(imu.init(new SPIClass(VSPI),1000*1000,_IMU_CS) == -1){
    Serial.printf("IMU init failed.\r\n");
    while(1);
  }

  //ジャイロオフセット
  vTaskDelay(3000/portTICK_RATE_MS);
  for(int i=0;i<1000;i++){
    ox+=imu.gyroX();
    oy+=imu.gyroY();
    oz+=imu.gyroZ();
    //vTaskDelay(1/portTICK_RATE_MS);
  }
  ox/=1000.0,oy/=1000.0,oz/=1000.0;

  //ToFセンサ
  Wire.begin();
  Wire.setClock(400 * 1000);
  dist.setTimeout(1000);
  dist.init();
  dist.setDistanceMode(VL53L1X::Medium);
  dist.setMeasurementTimingBudget(50000);
  dist.startContinuous(50);

  //LED設定
  pinMode(AUTOLED,OUTPUT);
  digitalWrite(AUTOLED,LOW);

  //セマフォ生成
  sp_control = xSemaphoreCreateBinary();
  sp_imu = xSemaphoreCreateBinary();
  sp_rssi = xSemaphoreCreateBinary();

  //フィルタ起動
  mdf.begin(_sensorfreq_Hz);

  //タスク追加
  xTaskCreateUniversal(control,"control",8192,NULL,1,NULL,APP_CPU_NUM);
  xTaskCreateUniversal(fetchIMU,"IMU",8192,NULL,configMAX_PRIORITIES,NULL,APP_CPU_NUM);
  xTaskCreateUniversal(fetchInput,"Input",8192,NULL,1,NULL,PRO_CPU_NUM);
  xTaskCreateUniversal(fetchRSSI,"RSSI",8192,NULL,1,NULL,PRO_CPU_NUM);

  //タイマー設定
  //タイマー1が死んでるw
  const int _microsecond = getApbFrequency()/1000000;
  tm_control = timerBegin(0,_microsecond,true);
  tm_imu = timerBegin(2,_microsecond,true);
  tm_rssi = timerBegin(3,_microsecond,true);
  timerAttachInterrupt(tm_control, &onTimer_control, true);
  timerAttachInterrupt(tm_imu, &onTimer_imu, true);
  timerAttachInterrupt(tm_rssi, &onTimer_rssi, true);

  //割り込み設定
  timerAlarmWrite(tm_control,_controlcycle_us,true);
  timerAlarmWrite(tm_imu,_sensorcycle_us,true);
  timerAlarmWrite(tm_rssi,_rssicycle_us,true);

  //WiFi接続
  WiFi.mode(WIFI_STA);
  //WiFi.disconnect();
  WiFi.begin(ssid,pass);
  if(WiFi.waitForConnectResult(WIFI_TIMEOUT_MS) != WL_CONNECTED){
    is_udp_enable = false;
    Serial.println("fail");
  }
  
  //タイマー起動
  timerAlarmEnable(tm_rssi);
  timerAlarmEnable(tm_control);
  timerAlarmEnable(tm_imu);

  Serial.printf("setup finished.\r\n");
  }


void loop() {
  sendUDP();
  delay(1);
}