// define Header
#include <Arduino.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <HX711.h>
#include <SPI.h> 
#include <BluetoothSerial.h>
#include "RtcDS1302.h"
#include <Wire.h>


// define Pin
#define button_take    2
#define pin_RTC_DAT   13
#define pin_RTC_SCK   12
#define pin_RTC_RST   15
#define button_tare   17
#define loadcell_DOUT 21
#define loadcell_SCK  22


// define Class Object
BluetoothSerial SerialBT;
TFT_eSPI display = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h
HX711 loadcell;
ThreeWire myWire(pin_RTC_DAT, pin_RTC_SCK, pin_RTC_RST);
RtcDS1302<ThreeWire> rtc(myWire);

struct SensorData {
  float WeightData;
  RtcDateTime dateTime;
};


// define value
bool buttonPressed_take = false;
bool buttonPressed_tare = false;
float calibrationFactor = 417.236; // 로드셀 마다 다름
float RawData = 0.0;
float filteredWeight = 0.0;
unsigned long startTime = millis();
SensorData sendData;
// bool buttonState_take = false;
// bool buttonState_tare = false;
// unsigned long lastDebounceTime_take = 0;
// unsigned long lastDebounceTime_tare = 0;
// unsigned long debounceDelay = 500;
unsigned long lastTakeTime = 0;
unsigned long lastTareTime = 0;
unsigned long Period = 1000;


// define User Function
void init_BT();
void init_loadcell();
void init_tft_LCD();
void init_rtc();
void update_display(float tmp);
float LPFilter(float RawWeight, float FilteredWeight);
void buttonInterrupt_take();
void buttonInterrupt_tare();

// void readButtons();
// void taring_loadcell();


void setup(void) {
  Serial.begin(115200);

  // LCD 화면에 현재 작동과 관련해서 표시하는 기능
  init_tft_LCD();
  // 로드셀 초기화 및 LPF 적용방안
  init_loadcell();
  init_rtc();
  
  init_BT();
  SerialBT.begin("TTGO_BT", true);

  display.fillScreen(TFT_BLACK);
  display.setCursor(0,0);
  display.print("Trying to connect to ESP32-CAM...");
  Serial.print("Trying to connect to ESP32-CAM...");
  while(!SerialBT.connected()) {
    display.print('.');
    SerialBT.connect("ESP_CAM_BT");
  }
  display.fillScreen(TFT_BLACK);
  display.setCursor(0, 0);
  display.print("Connected to ESP32-CAM");
  Serial.printf("\nConnected to ESP32-CAM\n");

  // 추가 필요 기능

  
  // PIN 입력을 받는 부분 및 버튼 인터럽트
  pinMode(button_take, INPUT);
  attachInterrupt(digitalPinToInterrupt(button_take), buttonInterrupt_take, FALLING);
  
  // 연점 조절 기능 버튼도 필요
  pinMode(button_tare, INPUT);
  attachInterrupt(digitalPinToInterrupt(button_tare), buttonInterrupt_tare, FALLING);  
}

void loop() {
  // taring_loadcell();
  if(millis() - startTime > 30000){
    if(SerialBT.hasClient()){
      startTime = millis();
    }
    else{
      SerialBT.end();
      init_BT();
      Serial.print("Cam disconnected, try reconnect to Cam...");
      display.fillScreen(TFT_BLACK);
      display.setCursor(0, 0);
      display.print("Cam disconnected, try again to connect...");
      while(!SerialBT.connected()){
        display.print('.');
        SerialBT.connect("ESP_CAM_BT");
      }
      display.fillScreen(TFT_BLACK);
      display.setCursor(0, 0);
      display.print("Reconnected to ESP32 CAM");
      Serial.printf("\nReconnected to ESP32 CAM\n");
      startTime = millis();
    }
  }

  

  if(buttonPressed_tare && (millis() - lastTareTime > 1000)){
    buttonPressed_tare = false;
    lastTareTime = millis();
    loadcell.tare();
    display.fillScreen(TFT_BLACK);
    display.setCursor(0, 0);
    display.print("Tare done.");
    while(millis() - lastTareTime < 1000);
  }

  RawData = loadcell.get_units(5);
  filteredWeight = LPFilter(RawData, filteredWeight);
  update_display(filteredWeight);

  if(buttonPressed_take && (millis() - lastTakeTime > 1000)) {
    buttonPressed_take = false;
    lastTakeTime = millis();
    display.fillScreen(TFT_BLACK);
    display.setCursor(0, 0);
    display.printf("take photo, %s\n", String(filteredWeight, 2).c_str());
    sendData.dateTime = rtc.GetDateTime();
    sendData.WeightData = filteredWeight;
    display.printf("%04d-%02d-%02d_%02d:%02d:%02d\n", 
               sendData.dateTime.Year(), sendData.dateTime.Month(), sendData.dateTime.Day(),
               sendData.dateTime.Hour(), sendData.dateTime.Minute(), sendData.dateTime.Second());
    SerialBT.write((uint8_t*)&sendData, sizeof(sendData));
    while(millis() - lastTakeTime < 1000);
  }

  // SerialBT.write((uint8_t*)&numberToSend, sizeof(numberToSend));   BT 통해서 보내는 구문 8비트 정수만.
  delay(250);
}


void init_BT(){
  unsigned long TimeNow = millis();
  SerialBT.begin("ESP_TTGO_BT", true);    // 블루투스 마스터 모드를 위해서 해당 기능 필요
  display.fillScreen(TFT_BLACK);
  display.setCursor(0, 0);
  display.print("init_BT done.");
  while(millis() - TimeNow < Period);
}

void init_loadcell(){
  unsigned long TimeNow = millis();
  loadcell.begin(loadcell_DOUT, loadcell_SCK);
  loadcell.set_scale(calibrationFactor);
  loadcell.tare();
  display.fillScreen(TFT_BLACK);
  display.setCursor(0, 0);
  display.print("init_loadcell done.");
  while(millis() - TimeNow < Period);
}

void init_tft_LCD(){
  unsigned long TimeNow = millis();
  display.begin();
  display.setRotation(3);
  display.fillScreen(TFT_BLACK);
  display.setCursor(0,0);
  display.print("init_display done.");
  display.setTextColor(TFT_WHITE);
  display.setTextFont(4);
  while(millis() - TimeNow < Period);
}

void update_display(float tmp){
  display.fillScreen(TFT_BLACK);
  display.setCursor(50,50);
  display.print(tmp, 2);
  display.print("g");
}

float LPFilter(float RawWeight, float FilteredWeight){
  float LPF_Factor = 0.0055;
  return (LPF_Factor * FilteredWeight) + ((1 - LPF_Factor) * RawWeight);
}

void buttonInterrupt_take(){
  buttonPressed_take = true;
}

void buttonInterrupt_tare(){
  buttonPressed_tare = true;
}

void init_rtc(){
  unsigned long TimeNow = millis();
  rtc.Begin();
  rtc.SetDateTime(RtcDateTime(2024, 1, 23, 9, 25, 0));    //업로드 할때마다 변경필요
  while(millis() - TimeNow < Period);
}

/*void taring_loadcell(){
  if (loadcell.is_ready()) {
    loadcell.set_scale();    
    Serial.println("Tare... remove any weights from the scale.");
    delay(5000);
    loadcell.tare();
    Serial.println("Tare done...");
    Serial.print("Place a known weight on the scale...");
    delay(5000); 
    long reading = loadcell.get_units(10);
    Serial.print("Result: ");
    Serial.println(reading);
  } 
  else {
    Serial.println("HX711 not found.");
  }
  Serial.print("one reading:\t");
  Serial.print(loadcell.get_units(), 1);
  Serial.print("\t| average:\t");
  Serial.println(loadcell.get_units(10), 5);
}
*/

/*void readButtons() {
  int reading_take = digitalRead(button_take);
  int reading_tare = digitalRead(button_tare);

  // 버튼 상태가 변했다면 디바운싱을 적용
  if (reading_take != buttonState_take) {
    lastDebounceTime_take = millis();
  }

  if (reading_tare != buttonState_tare) {
    lastDebounceTime_tare = millis();
  }

  // 디바운싱 시간이 지났을 때만 버튼 상태 업데이트
  if ((millis() - lastDebounceTime_take) > debounceDelay) {
    buttonState_take = reading_take;
  }

  if ((millis() - lastDebounceTime_tare) > debounceDelay) {
    buttonState_tare = reading_tare;
  }

  // 버튼이 눌렸고 디바운싱이 지났다면 해당 플래그 설정
  if (buttonState_take == HIGH && reading_take == HIGH) {
    buttonPressed_take = true;
  }
  else
    buttonState_take = false;

  if (buttonState_tare == HIGH && reading_tare == HIGH) {
    buttonPressed_tare = true;
  }
  else
    buttonState_tare = false;
}*/



/*// Calibrating the load cell
#include <Arduino.h>
#include "soc/rtc.h"
#include "HX711.h"

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 21;
const int LOADCELL_SCK_PIN = 22;

HX711 scale;

void setup() {
  Serial.begin(115200);
  // rtc_clk_cpu_freq_set(RTC_CPU_FREQ_80M);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
}

void loop() {

  if (scale.is_ready()) {
    scale.set_scale();    
    Serial.println("Tare... remove any weights from the scale.");
    delay(5000);
    scale.tare();
    Serial.println("Tare done...");
    Serial.print("Place a known weight on the scale...");
    delay(5000);
    long reading = scale.get_units(10);
    Serial.print("Result: ");
    Serial.println(reading);
  } 
  else {
    Serial.println("HX711 not found.");
  }
  delay(1000);
}
*/