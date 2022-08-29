#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Ticker.h>
#include <WiFi.h>
#include <AsyncUDP.h>
#include <math.h>
#include "icm20648.h"
#include "lps25hb.h"
#include "VL53L1X.h"
#include "esp32-hal-ledc.h"
#include "sbus.h"
#include "MadgwickAHRS.h"

#define AUTOLED 4
#define WIFI_TIMEOUT_MS 5000
#define LPF_GAIN 0.5
#define BROADCAST_FREQ_HZ 100
#define DEG2RAD M_PI/180.0

#define AIL 0
#define ELE 1
#define THR 2
#define RUD 3
#define CH5 4
#define CH6 5
#define CH7 6
#define VR  7


//セマフォとかタイマーとか
volatile SemaphoreHandle_t sp_control;
volatile SemaphoreHandle_t sp_sensor;
volatile SemaphoreHandle_t sp_wifi;
hw_timer_t* tm_control = NULL;
hw_timer_t* tm_sensor = NULL;
hw_timer_t* tm_wifi = NULL;

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
float front_acc,right_acc,up_acc;
float pitch_gyr,yaw_gyr,roll_gyr;
float ax=0,ay=0,az=0;
float lx=0,ly=0,lz=0;
float ox=0,oy=0,oz=0;
Madgwick mdf;

void IRAM_ATTR getPosture(){
  float gx = (imu.gyroX()-ox)*-1.0,gy = imu.gyroY()-oy,gz=(imu.gyroZ()-oz)*-1.0;
  lx = LPF_GAIN * lx + (1.0 - LPF_GAIN) * gx;
  ly = LPF_GAIN * ly + (1.0 - LPF_GAIN) * gy;
  lz = LPF_GAIN * lz + (1.0 - LPF_GAIN) * gz;

  ax = imu.accelX()*-1.0,ay = imu.accelY(),az = imu.accelZ()*-1.0;

  mdf.updateIMU(lx,ly,lz,ax,ay,az);
}

//ToFセンサ
VL53L1X dist;
//大気圧センサ
lps25hb alt;
const uint8_t _ALT_CS = 15;
float distance_Tof = -1.0;
float before_yaw;
float altitude_press;

void IRAM_ATTR readSensor(){
  distance_Tof = dist.read();
  //altitude_press = alt.altitude();
  pitch = mdf.getRollRadians();
  before_yaw = yaw;
  yaw = mdf.getYawRadians();
  roll = mdf.getPitchRadians();
  front_acc = ay*-1.0;
  right_acc = ax*-1.0;
  up_acc = az*-1.0;
  pitch_gyr = lx;
  yaw_gyr = lz;
  roll_gyr = ly;
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
  機体情報をUDPで放送
  
  起動からの経過時間[ms]
  ピッチ[rad] ヨー[rad] ロール[rad]
  エレベーター[SBUS] スロットル[SBUS] ラダー[SBUS]
  前方加速度[G] 右方加速度[G] 上方加速度[G]
  対地高度[mm] 気圧高度[m]

*/

void IRAM_ATTR sendUDP(){
    char str[128];
    if(sbus_data[CH7] > 1024){
      sprintf(str,"%ld,%.3f,%.3f,%.3f,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
        esp_timer_get_time()/1000,
        pitch,yaw,roll,
        Output_SBUS[ELE],Output_SBUS[THR],Output_SBUS[RUD],
        front_acc,right_acc,up_acc,
        pitch_gyr,yaw_gyr,roll_gyr,
        distance_Tof,(yaw - before_yaw)*10.0
      );
    }else{
      sprintf(str,"Soiya");
    }
    udp.broadcastTo(str,8901);
}

//PD制御
float before_pitch;
float pitch_kp=1000.0,pitch_kd=100.0;
float target_pitch = 15.0*DEG2RAD;
float before_roll;
float roll_kp=1000.0,roll_kd=100.0;
float target_roll = 70.0*DEG2RAD;

void IRAM_ATTR PDcontrol(){
  for(int i = 0;i<5;i++){
    Output_SBUS[i] = sbus_data[i];
  }
  //CH6で自動操縦切り替え
  if(sbus_data[CH6] > 1024) return;


  float diff_pitch = target_pitch - pitch;
  Output_SBUS[ELE] -= diff_pitch * pitch_kp + (before_pitch - pitch) * pitch_kd;
  before_pitch = pitch;

  float diff_roll = target_roll - (before_yaw - yaw)*10.0;
  Output_SBUS[RUD] -= diff_roll * roll_kp + (before_roll - roll) * roll_kd;
  before_roll = roll;
}

//制御系（サーボのPWM周波数に合わせて50Hz）
void control(void *pvParam){
  while(1){
    xSemaphoreTake(sp_control,portMAX_DELAY);
    readSBUS();
    PDcontrol();
    controlServos();
  }
}

//センサ処理
const int _sensorfreq_Hz = 1000;
const int _sensorcycle_us = 1000000 / _sensorfreq_Hz;
void fetchIMU(void *pvParam){
  while(1){
    xSemaphoreTake(sp_sensor,portMAX_DELAY);
    getPosture();
  }
}

//WiFi通信(CPU0)
void commViaWiFi(void *pvParam){
  while (1){
    xSemaphoreTake(sp_wifi,portMAX_DELAY);
    readSensor();
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

const int _servofreq_Hz = 50;
const int _controlcycle_us = 1000000 / _servofreq_Hz;

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
    vTaskDelay(1/portTICK_RATE_MS);
  }
  ox/=1000.0,oy/=1000.0,oz/=1000.0;

  if(alt.init(new SPIClass(HSPI),1000*1000,_ALT_CS) == -1){
    Serial.printf("ALT init failed.\r\n");
    //while(1);
  }

  //ToFセンサ
  Wire.begin();
  Wire.setClock(400 * 1000);
  dist.setTimeout(1000);
  dist.init();
  dist.setDistanceMode(VL53L1X::Long);
  dist.setMeasurementTimingBudget(50000);
  dist.startContinuous(50);

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
  if(WiFi.waitForConnectResult(WIFI_TIMEOUT_MS) != WL_CONNECTED) Serial.println("fail");

  //フィルタ起動
  mdf.begin(_sensorfreq_Hz);

  //タスク追加
  xTaskCreateUniversal(control,"control",8192,NULL,1,NULL,APP_CPU_NUM);
  xTaskCreateUniversal(fetchIMU,"IMU",8192,NULL,configMAX_PRIORITIES,NULL,APP_CPU_NUM);
  xTaskCreateUniversal(commViaWiFi,"wifi",8192,NULL,1,NULL,PRO_CPU_NUM);

  //タイマー設定
  //タイマー1が死んでるw
  const int _microsecond = getApbFrequency()/1000000;
  tm_control = timerBegin(0,_microsecond,true);
  tm_sensor = timerBegin(2,_microsecond,true);
  tm_wifi = timerBegin(3,_microsecond,true);
  timerAttachInterrupt(tm_control, &onTimer_control, true);
  timerAttachInterrupt(tm_sensor, &onTimer_sensor, true);
  timerAttachInterrupt(tm_wifi, &onTimer_wifi, true);

  //割り込み設定
  timerAlarmWrite(tm_control,_controlcycle_us,true);
  timerAlarmWrite(tm_sensor,_sensorcycle_us,true);
  timerAlarmWrite(tm_wifi,BROADCAST_FREQ_HZ,true);

  //タイマー起動
  timerAlarmEnable(tm_control);
  timerAlarmEnable(tm_sensor);
  timerAlarmEnable(tm_wifi);

  Serial.printf("setup finished.\r\n");
  }


void loop() {
  delay(1);
}