#include <M5StickCPlus2.h>
#include <M5GFX.h>  
#include "UGOKU_Pad_Controller.hpp" 

#define Ef5 622
#define Bf4 466
#define Af4 415
#define Ef4 311

UGOKU_Pad_Controller controller;      // Instantiate the UGOKU Pad Controller object
uint8_t CH;                           // Variable to store the channel number
uint8_t VAL;                          // Variable to store the value for the servo control

bool isConnected = false;             // Boolean flag to track BLE connection status

const int buzzerPin = 2; 

const int analogPin = 36;
const float R1 = 2000.0;
const float R2 = 220.0;
const float voltageDividerRatio = R2 / (R1 + R2);
const float adcMax = 4095.0;
const float vRef = 3.3;

int rpm = 0; 
int temp = 23;

//Button
const int BUTTON_SET_LOW = 26;
const int BUTTON_SET_HIGH = 0;

//TEMP GPIO Control 
const int CONTROL_PIN = 32;  // G32
const int CONTROL_PIN_2 = 33;

bool prevBtnHigh = HIGH;
bool prevBtnLow = HIGH;
bool setControlState = LOW;
int prevValCh0 = HIGH;
int prevValCh1 = HIGH;
int prevValCh2 = HIGH;

uint8_t lastCh1Val = 0xFF;
uint8_t lastCh2Val = 0xFF;

//LGFX_Sprite for M5GFX
LGFX_Sprite sprite = LGFX_Sprite(&M5.Lcd);
const int spriteWidth = 240;
const int spriteHeight = 135;

uint16_t SFGreen = sprite.color565(0, 255, 180);

uint8_t lastPrintedCh;
uint8_t lastPrintedVal;

void playXPSound() {
  float qn = 0.3;
  tone(buzzerPin, Ef5); delay(qn * 3 / 4 * 1500);     
  tone(buzzerPin, Ef4); delay(qn * 500);     
  tone(buzzerPin, Bf4); delay(qn * 1000);        
  tone(buzzerPin, Af4); delay(qn * 1500);   
  tone(buzzerPin, Ef5); delay(qn * 1000);         
  tone(buzzerPin, Bf4); delay(qn * 2000);     
  noTone(buzzerPin);
}

void showWelcomeScreen() {
  //background
  M5.Lcd.fillRect(0, 0, 240, 45, 0x0010);   // dark blue
  M5.Lcd.fillRect(0, 18, 240, 120, 0x03BF);  // mid blue
  M5.Lcd.fillRect(0, 117, 240, 45, 0x0010);  // dark blue
  
  //water line
  M5.Lcd.drawLine(0, 18, 240, 18, 0x7DDF);
  
  //red line
  M5.Lcd.drawLine(0, 117, 240, 117, 0xF800);
  
  //welcome
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(WHITE, 0x03BF);
  M5.Lcd.setCursor(120, 135/2 - 16/2);
}

// 電圧計測補正関数（テスタでの測定値3点を元に最小二乗法で導出） 
float calibrateVoltage(float rawVoltage) {
  return rawVoltage * 0.8186 + 4.23;
}

// バッテリー表示（スプライト描画用）
void drawBattery(float voltage, float percentage) {
  int margin = 2;
  int x = 40, y = 75;
  int w = 35, h = 50;
  int usableHeight = h - margin * 2;
  int levelHeight = map((int)percentage, 0, 100, 0, usableHeight);

  // 色決定
  uint16_t fillColor = GREEN;
  if (percentage < 20) fillColor = RED;
  else if (percentage < 60) fillColor = YELLOW;

  // バッテリー枠と端子
  sprite.drawRect(x, y, w, h, WHITE);
  sprite.drawRect(x + 8, y - 6, 19, 7, WHITE);

  // バッテリー中身
  if (levelHeight > 0) {
    int fillY = y + h - margin - levelHeight;
    sprite.fillRect(x + margin, fillY, w - margin * 2, levelHeight, fillColor);
  }

  // 注意マーク（10%未満）
  if (percentage < 10.0) {
    sprite.setTextSize(4);
    sprite.setTextColor(RED, BLACK);
    sprite.setCursor(x + 7, y + 11);
    sprite.print("!");
    sprite.setTextColor(WHITE, BLACK); // 元に戻す
  }
}

// Function called when a BLE device connects
void onDeviceConnect() {
  Serial.println("Device connected!");  // Print connection message
  isConnected = true;                   // Set the connection flag to true
}

// Function called when a BLE device disconnects
void onDeviceDisconnect() {
  Serial.println("Device disconnected!");  // Print disconnection message
  isConnected = false;                     // Set the connection flag to false
}

void setup() {
  Serial.begin(115200);  
  M5.begin();
  M5.Lcd.setRotation(3);
  showWelcomeScreen();
  analogReadResolution(12);
  playXPSound();

  // スプライト初期化
  sprite.createSprite(spriteWidth, spriteHeight);
  sprite.setTextColor(SFGreen, BLACK);

  //仮GPIOコントロール
  pinMode(BUTTON_SET_HIGH, INPUT_PULLUP);
  pinMode(BUTTON_SET_LOW, INPUT_PULLUP);
  pinMode(CONTROL_PIN, OUTPUT);
  digitalWrite(CONTROL_PIN, setControlState);

  // Setup the BLE connection
  controller.setup("GYRO");       // Set the BLE device name to "My ESP32"

  // Set callback functions for when a device connects and disconnects
  controller.setOnConnectCallback(onDeviceConnect);   // Function called on device connection
  controller.setOnDisconnectCallback(onDeviceDisconnect);  // Function called on device disconnection
}

void loop() {
  // 電圧計算
  int raw = analogRead(analogPin);
  float voltageAtPin = (raw / adcMax) * vRef;
  float batteryVoltage = calibrateVoltage(voltageAtPin / voltageDividerRatio);
  float percentage = constrain((batteryVoltage - 20.5) / (25.2 - 20.5) * 100.0, 0.0, 100.0);

  // 回転数（仮：固定値 or センサー値に置換）
  if(setControlState == LOW){
    rpm = 0;
  }else if(setControlState == HIGH){
    rpm = 7000; 
  }

  // スプライトに描画
  sprite.fillSprite(BLACK);
  sprite.setTextColor(SFGreen, BLACK); 

  // 回転数右寄
  sprite.setTextSize(4);
  char rpmStr[10];
  sprintf(rpmStr, "%d", rpm);
  
  int textW = sprite.textWidth(rpmStr);  // 表示幅を計算
  int x = 160 + 24 * 3 - textW - 6;     // 右端から詰めて配置
  
  sprite.setCursor(x, 17);               // Y位置はそのままでOK
  sprite.print(rpmStr);

  // 回転数の描画（スプライトに）
  sprite.setTextSize(3);    
  sprite.setCursor(130 + (24*4 - 18*3) , 50);  
  sprite.printf("RPM");

  // 巻き線温度
  sprite.setFont(&fonts::Font0);  
  sprite.setTextSize(4);       
  sprite.setCursor(130 + (24*4 - (24*2 + 18 + 10)), 88);           
  sprite.printf("%d", temp);
  sprite.drawCircle(130 + (24*4 - (24*2 + 18 + 10)) + 18*2 + 16, 88 + 9, 3, SFGreen);
  sprite.setTextSize(3); 
  sprite.setCursor(130 + (24*4 - (24*2 + 18 + 5)) + 48 + 5,88+6);  
  sprite.printf("C");

  // 数字表示（左寄せ調整）
  sprite.setTextSize(3);
  sprite.setCursor(30, 10);
  sprite.printf("%3.0f%%", percentage);
  sprite.setCursor(30-18, 40);
  sprite.printf("%.1fV", batteryVoltage);

  // バッテリー表示
  sprite.setTextSize(1);
  drawBattery(batteryVoltage, percentage);

  // 画面に表示
  sprite.pushSprite(0, 0);

  if (batteryVoltage < 5.0) {  // 電源切れ時の「ありえない低い電圧」で判定
    M5.Lcd.fillScreen(BLACK);  // LCD消す（見た目対策）
    delay(100);                // 表示反映の時間
    esp_deep_sleep_start();    // 自動スリープ！
  }

  //仮GPIOコントロール
  bool currBtnHigh = digitalRead(BUTTON_SET_HIGH);
  bool currBtnLow = digitalRead(BUTTON_SET_LOW);

  // G0押された（HIGHにする）
  if (prevBtnHigh == HIGH && currBtnHigh == LOW) {  // G0押された（HIGHにする）
    setControlState = HIGH;
  }else if(prevBtnLow == HIGH && currBtnLow == LOW) {  // G26押された（LOWにする）
    setControlState = LOW;
  }

if (isConnected) {
    uint8_t err = controller.read_data();
    if (err == no_err) {
      uint8_t cnt = controller.getLastPairsCount();
      if (cnt > 0) {
        // チャンネル1: HIGH → setControlState = HIGH
        uint8_t ch1 = controller.getDataByChannel(1);
        if (ch1 != 0xFF && ch1 != lastCh1Val) {
          lastCh1Val = ch1;
          if (ch1 == HIGH) setControlState = HIGH;
        }
        // チャンネル2: HIGH → setControlState = LOW
        uint8_t ch2 = controller.getDataByChannel(2);
        if (ch2 != 0xFF && ch2 != lastCh2Val) {
          lastCh2Val = ch2;
          if (ch2 == HIGH) setControlState = LOW;
        }
      }
    } else if (err == cs_err) {
      Serial.println("Checksum error");
    } else if (err == data_err) {
      Serial.println("Packet length error");
    }
    controller.write_data(2, rpm / 100);
  }

  digitalWrite(CONTROL_PIN, setControlState);

  prevBtnHigh = currBtnHigh;
  prevBtnLow = currBtnLow;

  delay(20);
}
