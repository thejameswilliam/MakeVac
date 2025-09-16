#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "ControlProtocol.h"
#include <U8g2lib.h>
#include <Wire.h>

// Pins (adjust to your board; ADC1 only for WiFi/ESP-NOW compatibility)
#define PIN_JOY_X    34
#define PIN_JOY_Y    35
#define PIN_BTN_MODE 18  // toggle autonomous/remote
#define PIN_BTN_VAC  19  // toggle shop-vac

// Peer (onboard) MAC will be provided from main.cpp
static uint8_t PEER_MAC[6] = {0};

static bool modeLatched = false; // false=REMOTE, true=AUTO
static bool vacLatched  = false;

// Debug/graph settings
#define GRAPH_WIDTH 41   // odd number so center index is (WIDTH-1)/2
#define DEBUG_PRINT_INTERVAL_MS 100   // 10 Hz printing to avoid flooding

static uint16_t centerX = 2048;
static uint16_t centerY = 2048;
static bool     calibrated = false;
static uint32_t calibStartMs = 0;
static uint32_t calibDurationMs = 2000; // collect 2s of samples
static uint32_t calibSamples = 0;
static uint64_t sumX = 0;
static uint64_t sumY = 0;

static uint32_t lastDebugPrint = 0;

// OLED display (SSD1306 128x64 over I2C). If your module uses another controller, we can swap this constructor.
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 22, /* data=*/ 21);

// Status from onboard (received via ESP-NOW)
static volatile StatusPacket lastStatus{};
static volatile uint32_t lastStatusMs = 0;
static portMUX_TYPE statusMux = portMUX_INITIALIZER_UNLOCKED;

static void onRecvStatus(const uint8_t* mac, const uint8_t* data, int len) {
  if (len != sizeof(StatusPacket)) {
    return;
  }
  StatusPacket s{}; 
  
  memcpy(&s, data, sizeof(s));


  if (s.version != PROTO_VERSION) {
    return;
  }
  taskENTER_CRITICAL(&statusMux);
  memcpy((void*)&lastStatus, &s, sizeof(s));
  lastStatusMs = millis();
  taskEXIT_CRITICAL(&statusMux);
}

// Helper to decode 16-bit error mask to string (0xFFFF = read-fail sentinel)
static const char* decodeError(uint16_t mask) {
  if (mask == 0)      return "OK";
  if (mask == 0xFFFF) return "ReadFail";
  if (mask & 0x0001)  return "M1OverCur";
  if (mask & 0x0002)  return "M2OverCur";
  if (mask & 0x0004)  return "E-Stop";
  if (mask & 0x0008)  return "TempErr";
  if (mask & 0x0010)  return "MainHigh";
  if (mask & 0x0020)  return "MainLow";
  if (mask & 0x0040)  return "LogicHigh";
  if (mask & 0x0080)  return "LogicLow";
  if (mask & 0x0100)  return "M1DrvFault";
  if (mask & 0x0200)  return "M2DrvFault";
  if (mask & 0x0400)  return "M1SpdErr";
  if (mask & 0x0800)  return "M2SpdErr";
  if (mask & 0x1000)  return "M1PosErr";
  if (mask & 0x2000)  return "M2PosErr";
  if (mask & 0x4000)  return "Warn1";
  if (mask & 0x8000)  return "Warn2";
  return "Unknown";
}

static void drawHUD(int16_t mappedX, int16_t mappedY, bool modeAuto, bool vacOn) {
  // Copy status atomically
  StatusPacket s{}; uint32_t sMs = 0; uint32_t now = millis();
  taskENTER_CRITICAL(&statusMux);
  memcpy(&s, (void*)&lastStatus, sizeof(s));
  sMs = lastStatusMs;
  taskEXIT_CRITICAL(&statusMux);

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);

  // Line 1: Mode / Vac / RX age
  char l1[64];
  uint32_t age = (sMs==0) ? 0xFFFFFFFF : (now - sMs);
  snprintf(l1, sizeof(l1), "Mode:%s  Vac:%s  %s", modeAuto?"AUTO":"REM", vacOn?"ON":"OFF", (sMs&&age<1000)?"OK":"--");
  u8g2.drawStr(0, 10, l1);

  // Line 2: Joystick values
  char l2[64]; snprintf(l2, sizeof(l2), "X:%4d  Y:%4d", mappedX, mappedY);
  u8g2.drawStr(0, 22, l2);

  // Line 3: Errors and limit switches
  char l3[64];
  if (sMs==0) {
    snprintf(l3, sizeof(l3), "Err: n/a");
  } else {
    // Show error string
    snprintf(l3, sizeof(l3), "Err: %s", decodeError(s.errMask));
    // Show limit switches if any are triggered
    if (s.limitMask) {
      char limits[32] = " ";
      if (s.limitMask & 0x01) strcat(limits, "Limit L");
      if (s.limitMask & 0x02) strcat(limits, "Limit R");
      if (s.limitMask & 0x04) strcat(limits, "Limit Front");
      if (s.limitMask & 0x08) strcat(limits, "Limit Back");
      strcat(l3, limits);
    }
  }
  u8g2.drawStr(0, 34, l3);

  // Line 4: Voltage graph at bottom
  // Graph area: x=0..127, y=54..63 (10px high)
  const int graphX0 = 0, graphY0 = 54, graphW = 128, graphH = 10;
  u8g2.drawFrame(graphX0, graphY0, graphW, graphH);
  if (sMs != 0 && s.mainV10 > 0) {
    // Map voltage to bar: e.g. 18V = full, 10V = empty
    float v = s.mainV10 / 11.0f;
    float vMin = 10.0f, vMax = 13.4f;
    int barW = (int)((v - vMin) * (graphW - 2) / (vMax - vMin));
    if (barW < 0) barW = 0;
    if (barW > graphW - 2) barW = graphW - 2;
    u8g2.drawBox(graphX0 + 1, graphY0 + 1, barW, graphH - 2);
    // Draw voltage value above graph
    char vStr[16]; snprintf(vStr, sizeof(vStr), "V: %.1f", v);
    u8g2.drawStr(graphX0, graphY0 - 2, vStr);
  } else {
    u8g2.drawStr(graphX0, graphY0 - 2, "V: n/a");
  }

  // Mini crosshair for stick
  int cx = 100, cy = 40, r = 10;
  u8g2.drawCircle(cx, cy, r, U8G2_DRAW_ALL);
  int px = cx + (mappedX * r) / 1000;
  int py = cy - (mappedY * r) / 1000;
  u8g2.drawDisc(px, py, 2, U8G2_DRAW_ALL);

  u8g2.sendBuffer();
}

static int16_t mapAxis(int raw) {
  // 12-bit ADC: 0..4095; center ~2048; deadband ±80
  const int center = 2048;
  int v = raw - center;
  if (abs(v) < 80) v = 0;
  long out = (long)v * 1000 / 2048;
  if (out > 1000) out = 1000;
  if (out < -1000) out = -1000;
  return (int16_t)out;
}

static int16_t mapAxisCentered(int raw, int center) {
  int v = raw - center;
  long out = (long)v * 1000 / 2048; // -1000..+1000 range
  if (out > 1000) out = 1000;
  if (out < -1000) out = -1000;
  return (int16_t)out;
}

static void onSent(const uint8_t*, esp_now_send_status_t) {
  // optional: observe delivery status
}

void handheld_setup(const uint8_t peer_mac[6]) {
  memcpy(PEER_MAC, peer_mac, 6);
  WiFi.mode(WIFI_STA);

  pinMode(PIN_BTN_MODE, INPUT_PULLUP);
  pinMode(PIN_BTN_VAC,  INPUT_PULLUP);
  Serial.println("Pins set up");

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  calibStartMs = millis();
  calibrated = false;
  sumX = sumY = 0;
  calibSamples = 0;

  // OLED init
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 12, "HUD init...");
  u8g2.sendBuffer();

  // ESP-NOW init
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1) delay(1000);
  }
  Serial.println("ESP-NOW initialized");
  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onRecvStatus);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, PEER_MAC, 6);
  peer.ifidx   = WIFI_IF_STA;
  peer.channel = 0;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Add peer failed");
    while (1) delay(1000);
  }
  Serial.println("Peer added");
}

void handheld_loop() {
  static uint32_t lastBtnSample = 0, lastSend = 0;
  uint32_t now = millis();

  // Poll buttons every 20ms
  if (now - lastBtnSample >= 20) {
    lastBtnSample = now;
    static uint8_t mPrev=1, vPrev=1;
    uint8_t m = digitalRead(PIN_BTN_MODE);
    uint8_t v = digitalRead(PIN_BTN_VAC);
    if (m==0 && mPrev==1) {
      modeLatched = !modeLatched; // toggle on press
    //   Serial.print("Mode button pressed, modeLatched = ");
    //   Serial.println(modeLatched ? "AUTO" : "REMOTE");
    }
    if (v==0 && vPrev==1) {
      vacLatched  = !vacLatched;
    //   Serial.print("Vac button pressed, vacLatched = ");
    //   Serial.println(vacLatched ? "ON" : "OFF");
    }
    // Serial.print("Button states - Mode: ");
    // Serial.print(m);
    // Serial.print(", Vac: ");
    // Serial.println(v);
    mPrev=m; vPrev=v;
  }

  // Send at ~50 Hz
  if (now - lastSend >= 20) {
    lastSend = now;
    int rawX = analogRead(PIN_JOY_X);
    int rawY = analogRead(PIN_JOY_Y);


    // Calibration phase on startup: average raw samples for 2 seconds
    if (!calibrated) {
      if (millis() - calibStartMs <= calibDurationMs) {
        sumX += rawX; sumY += rawY; ++calibSamples;
      } else if (calibSamples > 0) {
        centerX = (uint16_t)(sumX / calibSamples);
        centerY = (uint16_t)(sumY / calibSamples);
        calibrated = true;
        Serial.print("Calibrated centers -> X:"); Serial.print(centerX);
        Serial.print(" Y:"); Serial.println(centerY);
      }
    }

    // Map using calibrated centers
    int16_t mappedX = mapAxisCentered(rawX, centerX);
    int16_t mappedY = mapAxisCentered(rawY, centerY);

    // ASCII graph at 10 Hz
    if (millis() - lastDebugPrint >= DEBUG_PRINT_INTERVAL_MS) {
      lastDebugPrint = millis();
      // HUD update
      drawHUD(mappedX, mappedY, modeLatched, vacLatched);
    }

    // Build and send packet
    ControlPacket p{};
    p.version = PROTO_VERSION;
    p.x = mappedX;
    p.y = mappedY;
    p.mode = modeLatched ? MODE_AUTO : MODE_REMOTE;
    p.vac  = vacLatched ? 1 : 0;
    p._rsvd0 = 0; // keep explicit for deterministic packet

    esp_now_send(PEER_MAC, (uint8_t*)&p, sizeof(p));
  }
}