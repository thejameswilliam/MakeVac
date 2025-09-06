#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <RoboClaw.h>
#include "ControlProtocol.h"

// ==== Hardware config ====
// RoboClaw UART2: TX=GPIO17 -> S1 (RX), RX=GPIO16 <- S2 (TX via 5V->3V3 level shift)
#define ROBOCLAW_ADDR 0x80
static HardwareSerial& RC = Serial2;
static RoboClaw roboclaw(&RC, 10000); // 10ms timeout

// Relay for shop-vac
#define RELAY_PIN         23
#define RELAY_ACTIVE_HIGH 1  // set to 0 if your module is active-LOW

// Safety & loop timing
#define LINK_TIMEOUT_MS 300    // stop if no packet within this window
#define LOOP_INTERVAL_MS 20    // ~50 Hz

// Center deadzone with hysteresis (units: -1000..+1000 command space)
#define START_MOVE_THRESHOLD  240  // must exceed this to start moving
#define STOP_MOVE_THRESHOLD   160  // must fall below this to stop

// Peer (handheld) MAC will be provided from main.cpp
static uint8_t PEER_MAC[6] = {0};

// State updated from ESP-NOW callback (protected by spinlock)
static ControlPacket lastPkt{};   // non-volatile; guarded by critical section
static uint32_t lastPktMs = 0;
static bool havePkt = false;

// FreeRTOS spinlock for critical sections on ESP32 (SMP-safe)
static portMUX_TYPE espnowMux = portMUX_INITIALIZER_UNLOCKED;

static bool driveActive = false;  // latched state to add hysteresis around center

static void printRcErrors(uint16_t e) {
  if (e == 0) { Serial.println("RC Errors: NONE"); return; }
  Serial.print("RC Errors: 0x"); Serial.println(e, HEX);
  if (e & 0x0001) Serial.println(" - M1 OverCurrent");
  if (e & 0x0002) Serial.println(" - M2 OverCurrent");
  if (e & 0x0004) Serial.println(" - E-Stop");
  if (e & 0x0008) Serial.println(" - Temperature Error");
  if (e & 0x0010) Serial.println(" - Main Battery High");
  if (e & 0x0020) Serial.println(" - Main Battery Low");
  if (e & 0x0040) Serial.println(" - Logic Battery High");
  if (e & 0x0080) Serial.println(" - Logic Battery Low");
  if (e & 0x0100) Serial.println(" - M1 Driver Fault");
  if (e & 0x0200) Serial.println(" - M2 Driver Fault");
  if (e & 0x0400) Serial.println(" - Main Battery High Warn");
  if (e & 0x0800) Serial.println(" - Main Battery Low Warn");
}

static uint16_t lastErrorMask = 0xFFFF; // invalid sentinel to force first print
static uint32_t lastErrorPollMs = 0;
#define ERROR_POLL_INTERVAL_MS 200  // poll at 5 Hz

static inline void setVac(bool on) {
  if (RELAY_ACTIVE_HIGH) digitalWrite(RELAY_PIN, on ? HIGH : LOW);
  else                   digitalWrite(RELAY_PIN, on ? LOW  : HIGH);
}

static inline void stopMotors() {
  roboclaw.DutyM1M2(ROBOCLAW_ADDR, 0, 0);
}

// Deadzone is applied in onboard_loop() so mixing always receives intended values.
static int16_t scaleLimit1000_toDuty(int16_t v, int16_t limit) {
  // map -1000..+1000 to -32767..+32767, clamp to ±limit
  long d = (long)v * 32767 / 1000;
  if (d >  limit) d =  limit;
  if (d < -limit) d = -limit;
  return (int16_t)d;
}

static void driveFromXY(int16_t x, int16_t y) {
  long left  = (long)y + (long)x;
  long right = (long)y - (long)x;
  if (left  > 1000) left  = 1000; if (left  < -1000) left  = -1000;
  if (right > 1000) right = 1000; if (right < -1000) right = -1000;

  // Limit to ~35% duty for safety (≈11500)
  const int16_t limit = 11500;
  int16_t dL = scaleLimit1000_toDuty((int16_t)left,  limit);
  int16_t dR = scaleLimit1000_toDuty((int16_t)right, limit);

  roboclaw.DutyM1M2(ROBOCLAW_ADDR, dL, dR);
}

static void onRecv(const uint8_t* mac, const uint8_t* data, int len) {
  if (len != sizeof(ControlPacket)) return;
  ControlPacket pkt; memcpy(&pkt, data, sizeof(pkt));
  if (pkt.version != PROTO_VERSION) return;

  // Optionally: allowlist by handheld MAC (PEER_MAC). If you want strict check:
  // if (memcmp(mac, PEER_MAC, 6) != 0) return;

  taskENTER_CRITICAL(&espnowMux);
  memcpy(&lastPkt, &pkt, sizeof(lastPkt));
  lastPktMs = millis();
  havePkt = true;
  taskEXIT_CRITICAL(&espnowMux);
}

// Added helper and status sending
static uint32_t statusSeq = 0;
static void sendStatusNow() {
  bool eOK=false, vOK=false;
  uint16_t err = roboclaw.ReadError(ROBOCLAW_ADDR, &eOK);
  uint16_t mv10 = roboclaw.ReadMainBatteryVoltage(ROBOCLAW_ADDR, &vOK);
  StatusPacket s{};
  s.version = PROTO_VERSION;
  s.errMask = eOK ? err : 0xFFFF; // 0xFFFF to indicate read-failure
  s.mainV10 = vOK ? mv10 : 0;
  s.seq = ++statusSeq;
  esp_now_send(PEER_MAC, (const uint8_t*)&s, sizeof(s));
}

void onboard_setup(const uint8_t peer_mac[6]) {
  memcpy(PEER_MAC, peer_mac, 6);

  // Relay
  pinMode(RELAY_PIN, OUTPUT);
  setVac(false);

  // RoboClaw UART
  RC.begin(38400, SERIAL_8N1, 16, 17);
  roboclaw.begin(38400);

  char ver[48] = {0};
  if (roboclaw.ReadVersion(ROBOCLAW_ADDR, ver)) {
    Serial.print("RoboClaw FW: "); Serial.println(ver);
    bool eOK = false;
    uint16_t err = roboclaw.ReadError(ROBOCLAW_ADDR, &eOK);
    if (eOK) { lastErrorMask = err; printRcErrors(err); }
    else     { Serial.println("Failed to read RC errors at startup"); }
  } else {
    Serial.println("RoboClaw comms not responding.");
  }

  // ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed"); while (1) delay(1000);
  }
  esp_now_register_recv_cb(onRecv);

  // Add handheld as peer (optional but recommended)
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, PEER_MAC, 6);
  peer.ifidx   = WIFI_IF_STA;
  peer.channel = 0;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Add peer failed"); while (1) delay(1000);
  }

  stopMotors();
}

void onboard_loop() {
  static uint32_t lastTick = 0;
  uint32_t now = millis();
  if (now - lastTick < LOOP_INTERVAL_MS) { delay(1); return; }
  lastTick = now;

  ControlPacket pktLocal{};
  uint32_t pktMsLocal = 0;
  bool havePktLocal = false;

  taskENTER_CRITICAL(&espnowMux);
  memcpy(&pktLocal, &lastPkt, sizeof(pktLocal));
  pktMsLocal = lastPktMs;
  havePktLocal = havePkt;
  taskEXIT_CRITICAL(&espnowMux);

  bool linkOk = havePktLocal && ((now - pktMsLocal) <= LINK_TIMEOUT_MS);
  if (!linkOk) { stopMotors(); return; }

  // Poll and log RoboClaw errors (print on change)
  if (now - lastErrorPollMs >= ERROR_POLL_INTERVAL_MS) {
    lastErrorPollMs = now;
    bool eOK = false;
    uint16_t err = roboclaw.ReadError(ROBOCLAW_ADDR, &eOK);
    if (eOK) {
      if (err != lastErrorMask) { lastErrorMask = err; printRcErrors(err); }
    } else {
      Serial.println("Error: failed to read RC error mask");
    }
  }

  // Optional: voltage sampling at 1 Hz
  static uint32_t lastVms = 0; 
  if (now - lastVms >= 1000) {
    lastVms = now;
    bool vOK = false;
    uint16_t mv10 = roboclaw.ReadMainBatteryVoltage(ROBOCLAW_ADDR, &vOK);
    if (vOK) { Serial.print("Main V: "); Serial.println(mv10 / 10.0f); }
  }

  // Send status at ~2 Hz
  static uint32_t lastStatusSendMs = 0;
  if (now - lastStatusSendMs >= 500) { // 2 Hz
    lastStatusSendMs = now;
    sendStatusNow();
  }

  setVac(pktLocal.vac != 0);

  if (pktLocal.mode == MODE_REMOTE) {
    // Compute joystick magnitude (L-infinity norm is sufficient and cheap)
    int16_t ax = abs(pktLocal.x);
    int16_t ay = abs(pktLocal.y);
    int16_t mag = (ax > ay) ? ax : ay; // max(|x|,|y|)

    // Hysteresis: require larger deflection to start, smaller to stop
    if (!driveActive) {
      if (mag >= START_MOVE_THRESHOLD) driveActive = true;
    } else { // currently active
      if (mag <= STOP_MOVE_THRESHOLD) driveActive = false;
    }

    if (!driveActive) {
      // Inside deadzone -> hard stop to eliminate twitch
      stopMotors();
    } else {
      // Outside deadzone -> drive normally
      driveFromXY(pktLocal.x, pktLocal.y);
    }
  } else {
    // MODE_AUTO placeholder: currently stop (add autonomous logic later)
    stopMotors();
  }
}