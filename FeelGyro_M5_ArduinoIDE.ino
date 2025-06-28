#include <M5StickCPlus2.h>
#include <M5GFX.h>
#include <Wire.h>
#include "UGOKU_Pad_Controller.hpp"

//---- サウンド定義 ----
#define Ef5 622
#define Bf4 466
#define Af4 415
#define Ef4 311

//---- I2C 定義 ----
#define I2C_DEV_ADDR 0x55

//---- インスタンス ----
UGOKU_Pad_Controller controller;
LGFX_Sprite sprite(&M5.Lcd);

//---- ピン定義 ----
const int buzzerPin       = 2;
const int analogPin       = 36;
const int BUTTON_SET_HIGH = 0;   // 全開ボタン
const int BUTTON_SET_LOW  = 26;  // 停止／ブレーキボタン
const int CONTROL_PIN     = 32;  // LSB
const int CONTROL_PIN_2   = 33;  // MSB

//---- 電圧計測定数 ----
const float R1 = 2000.0;
const float R2 = 220.0;
const float voltageDividerRatio = R2 / (R1 + R2);
const float adcMax = 4095.0;
const float vRef = 3.3;

//---- グローバル変数 ----
bool     isConnected    = false;
int      ControlState   = 0;      // 0b00:停止, 0b01:全開, 0b11:ブレーキ
int      rpm            = 0;
uint8_t  lastCh1Val     = 0xFF, lastCh2Val = 0xFF, lastCh3Val = 0xFF;

//---- ボタン＆デバウンス用 ----
bool     prevHighState  = false;
bool     prevLowState   = false;
unsigned long lowPressStart  = 0;
bool     lowLongHandled      = false;
const unsigned long DEBOUNCE_MS  = 50;
const unsigned long LONGPRESS_MS = 2000;

//---- バッテリーオフ閾値 ----
const float cutoffVoltage = 5.0;  // V

//---- 文字の色 ----
uint16_t SFGreen = sprite.color565(0, 255, 180);

//---- コールバック ----
void onDeviceConnect() {
  Serial.println("[BLE] Connected");
  isConnected = true;
}
void onDeviceDisconnect() {
  Serial.println("[BLE] Disconnected");
  isConnected = false;
}

//---- ヘルパー関数 ----
void playXPSound() {
  float q = 0.3;
  tone(buzzerPin, Ef5); delay(q*3/4*1500);
  tone(buzzerPin, Ef4); delay(q*500);
  tone(buzzerPin, Bf4); delay(q*1000);
  tone(buzzerPin, Af4); delay(q*1500);
  tone(buzzerPin, Ef5); delay(q*1000);
  tone(buzzerPin, Bf4); delay(q*2000);
  noTone(buzzerPin);
}

void showWelcomeScreen() {
  M5.Lcd.fillRect(0, 0, 240, 45, 0x0010);
  M5.Lcd.fillRect(0, 18, 240, 120, 0x03BF);
  M5.Lcd.fillRect(0, 117, 240, 45, 0x0010);
  M5.Lcd.drawLine(0, 18, 240, 18, 0x7DDF);
  M5.Lcd.drawLine(0, 117, 240, 117, 0xF800);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(WHITE, 0x03BF);
  M5.Lcd.setCursor(120, 135/2 - 8);
  M5.Lcd.print("Welcome");
}

// 電圧補正（最小二乗法などで求めた補正式を適用）
float calibrateVoltage(float raw) {
  return raw + 1.2;
}

// バッテリーアイコン描画
void drawBattery(float voltage, float percentage) {
  int margin=2, x=40, y=75, w=35, h=50;
  int usable = h - margin*2;
  int lvl = map((int)percentage, 0, 100, 0, usable);
  uint16_t col = (percentage<20? RED : (percentage<60? YELLOW : GREEN));
  sprite.drawRect(x, y, w, h, SFGreen);
  sprite.drawRect(x+8, y-6, 19, 7, SFGreen);
  if (lvl > 0) {
    sprite.fillRect(x+margin, y+h-margin-lvl, w-margin*2, lvl, col);
  }
  if (percentage < 10.0) {
    sprite.setTextSize(4);
    sprite.setTextColor(RED, BLACK);
    sprite.setCursor(x+7, y+11);
    sprite.print("!");
    sprite.setTextColor(SFGreen, BLACK);
  }
}

void setup(){
  Serial.begin(115200);
  M5.begin();
  M5.Lcd.setRotation(3);
  showWelcomeScreen();
  analogReadResolution(12);
  playXPSound();

  sprite.createSprite(240, 135);
  sprite.setTextColor(sprite.color565(0,255,180), BLACK);

  pinMode(BUTTON_SET_HIGH, INPUT_PULLUP);
  pinMode(BUTTON_SET_LOW,  INPUT_PULLUP);
  pinMode(CONTROL_PIN,     OUTPUT);
  pinMode(CONTROL_PIN_2,   OUTPUT);
  digitalWrite(CONTROL_PIN,   0);
  digitalWrite(CONTROL_PIN_2, 0);

  controller.setup("GYRO");
  controller.setOnConnectCallback(onDeviceConnect);
  controller.setOnDisconnectCallback(onDeviceDisconnect);

  prevHighState = false;
  prevLowState  = false;

  // **ここで I2C マスターを準備**
  Wire.begin(32, 33);   // SDA=32, SCL=33
}

void loop(){
  //--- BLE 読み取り ---
  if (isConnected) {
    uint8_t err = controller.read_data();
    if (err == no_err) {
      uint8_t cnt = controller.getLastPairsCount();
      for (uint8_t i = 0; i < cnt; i++) {
        uint8_t ch = controller.getDataByChannel(i+1);
        if (ch == HIGH) {
          if      (i+1 == 1) { ControlState = 0b01; Serial.println("[BLE] Full"); }
          else if (i+1 == 2) { ControlState = 0b00; Serial.println("[BLE] Stop"); }
          else if (i+1 == 3) { ControlState = 0b11; Serial.println("[BLE] Brake"); }
        }
      }
      controller.write_data(10, rpm/100);
    }
  }

  //--- ボタン読み取り & デバウンス ---
  static unsigned long lastHighChange = 0, lastLowChange = 0;
  bool rawHigh = (digitalRead(BUTTON_SET_HIGH) == LOW);
  bool rawLow  = (digitalRead(BUTTON_SET_LOW)  == LOW);

  // High ボタン 短押し
  if (rawHigh != prevHighState && millis() - lastHighChange > DEBOUNCE_MS) {
    lastHighChange = millis();
    prevHighState  = rawHigh;
    if (rawHigh) {
      ControlState = 1;
      Serial.println("[BTN] High → Full");
    }
  }

  // Low ボタン デバウンス&長押し判定
  if (rawLow != prevLowState && millis() - lastLowChange > DEBOUNCE_MS) {
    lastLowChange = millis();
    if (rawLow) {
      // 押し始め
      lowPressStart = millis();
      lowLongHandled = false;
    } else {
      // リリース
      unsigned long dur = millis() - lowPressStart;
      if (!lowLongHandled && dur < LONGPRESS_MS) {
        ControlState = 0;
        Serial.println("[BTN] Low → Stop");
      }
    }
    prevLowState = rawLow;
  }
  // 長押し検出
  if (prevLowState && !lowLongHandled && millis() - lowPressStart >= LONGPRESS_MS) {
    ControlState   = 2;
    lowLongHandled = true;
    Serial.println("[BTN] Low long → Brake");
  }

  // 送信
  Wire.beginTransmission(I2C_DEV_ADDR);
  Wire.write((uint8_t)ControlState);
  uint8_t txErr = Wire.endTransmission(true);
  Serial.printf("TX state=%u, err=%u\n", ControlState, txErr);
  
  delay(5);  // 5msほど待って、STM32がHAL_I2C_Slave_Transmitに入るのを待つ

  // 受信
  uint8_t i2cBuf[4] = {0};
  uint8_t len = Wire.requestFrom(I2C_DEV_ADDR, (uint8_t)4, true);
  Serial.printf("RX len=%u\n", len);
  for (uint8_t i = 0; i < len; i++) {
    i2cBuf[i] = Wire.read();
    Serial.printf("  i2cBuf[%u]=0x%02X\n", i, i2cBuf[i]);
  }
  if (len == 4) {
    uint32_t v =  (uint32_t)i2cBuf[0]
                | ((uint32_t)i2cBuf[1] << 8)
                | ((uint32_t)i2cBuf[2] << 16)
                | ((uint32_t)i2cBuf[3] << 24);
    Serial.printf("  rpm=%lu\n", v);
    rpm = v;
  }

  

  //--- 電圧&バッテリー計算 ---
  int raw = analogRead(analogPin);
  float vpin  = (raw / adcMax) * vRef;
  float battV = calibrateVoltage(vpin / voltageDividerRatio);
  float perc  = constrain((battV - 20.5) / (25.2 - 20.5) * 100.0, 0.0, 100.0);

  //--- 画面描画 ---
  sprite.fillSprite(BLACK);
  // RPM
  sprite.setTextSize(4);
  char strBuf[16]; sprintf(strBuf, "%lu", rpm);
  int tw = sprite.textWidth(strBuf);
  sprite.setCursor(160 + 24*3 - tw - 6, 17);
  sprite.print(strBuf);
  sprite.setTextSize(3);
  sprite.setCursor(130 + (24*4 - 18*3), 50);
  sprite.print("RPM");
  // バッテリー%
  sprite.setTextSize(3);
  sprite.setCursor(30, 10);
  sprite.printf("%3.0f%%", perc);
  sprite.setCursor(12, 40);
  sprite.printf("%.1fV", battV);
  // バッテリーアイコン
  sprite.setTextSize(1);
  drawBattery(battV, perc);
  sprite.pushSprite(0, 0);

  //--- 低電圧自動オフ ---
  if (battV < cutoffVoltage) {
    Serial.println("[SYSTEM] Low voltage detected, going to deep sleep");
    M5.Lcd.fillScreen(BLACK);
    delay(100);
    esp_deep_sleep_start();
  }

  delay(50);
}
