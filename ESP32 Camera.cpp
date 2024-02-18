// HEADER
#include <Arduino.h>
#include "soc/soc.h"          // 브라운 아웃 해결
#include "soc/rtc_cntl_reg.h" // 브라운 아웃 해결
#include "esp_camera.h"       // camera 라이브러리
#include <BluetoothSerial.h>
#include "SD_MMC.h"
#include "RtcDS1302.h"

// CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22


// Object define
camera_config_t config;
camera_fb_t * fb = NULL;
BluetoothSerial SerialBT;
sensor_t * s = esp_camera_sensor_get();
// File LOG_FILE;
struct SensorData
{
  float WeightData;
  RtcDateTime DateTime;
};


// User Function
void init_camera();
void init_BT();
void init_microSD();
// File init_log();
void capture(RtcDateTime date, int filename, size_t fb_size, camera_fb_t * fb);
// int readLastLineNumber(File log, SensorData tmpData);
void capture_log(int uniqueNumber, SensorData tmp_data);

// User var
unsigned long startTime = millis();
unsigned long Period = 1000;
SensorData recivedData;
int uNumber = 0;

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);

  // if(!init_camera()){
  //   Serial.println("Cam configuration fail... 5s after restart.");
  //   delay(5000);
  //   ESP.restart();
  // }

  init_camera();
  init_microSD();
  // LOG_FILE = init_log();
  init_BT();

  Serial.print("Trying to connect to ESP32 TTGO...");
  while(!SerialBT.hasClient()) {
    Serial.print('.');
    delay(500);
  }
  Serial.printf("\nConnected to ESP32 TTGO\n");
}

void loop() {  
  if(millis() - startTime > 30000){
    if(SerialBT.hasClient()){
      startTime = millis();
    }
    else{
      init_BT();
      Serial.print("TTGO disconnected, try reconnect to TTGO...");
      while(!SerialBT.hasClient()){
        Serial.print('.');
        delay(500);
      }
      Serial.printf("\nReconnected to ESP32 TTGO\n");
      startTime = millis();
    }
  }

  if(SerialBT.available() >= sizeof(float)){
    SerialBT.readBytes((char*)&recivedData, sizeof(recivedData));

    Serial.print("Received time : ");
    Serial.println(String(recivedData.DateTime.Year()) + "-" + 
                  String(recivedData.DateTime.Month()) + "-" + 
                  String(recivedData.DateTime.Day()) + "_" + 
                  String(recivedData.DateTime.Hour()) + ":" + 
                  String(recivedData.DateTime.Minute()) + ":" + 
                  String(recivedData.DateTime.Second()));
    Serial.print("Received Weight : ");
    Serial.println(recivedData.WeightData, 2);

    fb = esp_camera_fb_get();
    if(!fb){
      Serial.println("사진 촬영 실패");
      delay(500);
      ESP.restart();
    }

    // int uniqenumber = readLastLineNumber(LOG_FILE, recivedData);

    capture(recivedData.DateTime, uNumber, fb->len, fb);
    capture_log(uNumber, recivedData);
    esp_camera_fb_return(fb);
    delay(250);
  }
}



/*
    User Function Description
*/

void init_camera(){
  // unsigned long TimeNow = millis();
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz =  20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  config.grab_mode = CAMERA_GRAB_LATEST;

  // PSRAM 확인 후 버퍼 크기 및 이미지 화질 변경
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 5;  //0-63 낮을수록 고품질 10
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 7;  //0-63 낮을수록 고품질 12
    config.fb_count = 1;
  }
  
  // 카메라 초기화
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("카메라 초기화 실패 오류코드 : 0x%x", err);
    delay(1000);
    ESP.restart();
  }
  // while(millis() - TimeNow < 1000);

  // s->set_brightness(s, 0);     // -2 to 2
  // s->set_contrast(s, 0);       // -2 to 2
  // s->set_saturation(s, 0);     // -2 to 2
  // s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  // s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
  // s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
  // s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  // s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
  // s->set_aec2(s, 0);           // 0 = disable , 1 = enable
  // s->set_ae_level(s, 0);       // -2 to 2
  // s->set_aec_value(s, 600);    // 0 to 1200
  // s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
  // s->set_agc_gain(s, 0);       // 0 to 30
  // s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
  // s->set_bpc(s, 0);            // 0 = disable , 1 = enable
  // s->set_wpc(s, 1);            // 0 = disable , 1 = enable
  // s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
  // s->set_lenc(s, 1);           // 0 = disable , 1 = enable
  // s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
  // s->set_vflip(s, 0);          // 0 = disable , 1 = enable
  // s->set_dcw(s, 1);            // 0 = disable , 1 = enable
  // s->set_colorbar(s, 0);       // 0 = disable , 1 = enable
}

void init_BT(){
  SerialBT.begin("ESP_CAM_BT");
}

/*File init_log(){
  File logFile = SD_MMC.open("/photo/log.txt", FILE_APPEND);

  if(!logFile){
    File logFile = SD_MMC.open("/photo/log.txt", FILE_WRITE);
    return logFile;
  }
  return logFile;
}
*/

void init_microSD(){
  // unsigned long TimeNow = millis();
  if(!SD_MMC.begin()){
    Serial.println("SD card initialization fail.");
  }
  // while(millis() - TimeNow < 1000);
}

/*int readLastLineNumber(File log, SensorData tmpData){
  if (log) {
    // 파일 크기 확인
    size_t fileSize = log.size();

    if (fileSize > 0) {
      // 파일 포인터를 파일 끝으로 이동
      log.seek(fileSize - 1);

      // 개행 문자('\n')를 찾을 때까지 포인터를 앞으로 이동
      while (log.peek() != '\n' && log.position() > 0) {
        log.seek(log.position() - 1);
      }

      // 마지막 줄을 읽어옴
      String lastLine;
      while (log.available()) {
        char c = log.read();
        if (c == '\n') {
          break;
        }
        lastLine += c;
      }

      // 각 줄을 ','로 분리
      int commaIndex = lastLine.lastIndexOf(',');
      if (commaIndex != -1) {
        // ','를 기준으로 문자열을 분리
        String uniqueNumberStr = lastLine.substring(0, commaIndex);
        int uniqueNumber = uniqueNumberStr.toInt();

      // 마지막 줄을 반환
        return uniqueNumber;
      
      // 여기에서 고유번호를 추출하여 사용하면 됩니다.
      }
    }
    else {
      log.println()
      return 1;
    }
  }
}
*/

void capture(RtcDateTime date, int value, size_t fb_size, camera_fb_t * fb){
  // UART 통신을 통해 보내는 코드
  /*
  Serial.print('|');
  Serial.println(fb->len);
  Serial.write('|');
  Serial.write(fb->buf, fb->len);
  */

  // SD 카드에 fb 내용을 저장하는 코드
  String fileName = "/photo/" + 
                    // String(date.Year()) + "-" + 
                    // String(date.Month()) + "-" + 
                    // String(date.Day()) + "_" + 
                    // String(date.Hour()) + ":" + 
                    // String(date.Minute()) + ":" + 
                    // String(date.Second()) + "_" + 
                    String(value) + ".jpg";

  File file = SD_MMC.open(fileName.c_str(), FILE_WRITE);
  if(!file){
    Serial.println("파일 저장 실패");
    return;
  }

  file.write(fb->buf, fb->len);
  Serial.println("사진 저장 완료 : " + fileName);
  file.close();

  // SD_MMC.end();
}

void capture_log(int uniqueNumber, SensorData tmp_data){
  File log = SD_MMC.open("/photo/log.txt", FILE_APPEND);
  if(!log){
    Serial.println("파일 열기 실패");
    return;
  }

  log.printf("%d %s %.2f\n", uniqueNumber, 
                    (String(tmp_data.DateTime.Year()) + "-" + 
                    String(tmp_data.DateTime.Month()) + "-" + 
                    String(tmp_data.DateTime.Day()) + "_" + 
                    String(tmp_data.DateTime.Hour()) + ":" + 
                    String(tmp_data.DateTime.Minute()) + ":" + 
                    String(tmp_data.DateTime.Second())).c_str(), tmp_data.WeightData);
  Serial.println("파일 저장 완료");
  uNumber++;
  log.close();
}