#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <RoboClaw.h>
#include "ControlProtocol.h"

// ---- Protocol compile-time guards ----
static_assert(sizeof(ControlPacket) == 8,  "ControlPacket size mismatch: check packing/fields");
static_assert(sizeof(StatusPacket)  == 12, "StatusPacket size mismatch: check packing/fields");
#ifndef PROTO_VERSION
#error "PROTO_VERSION must be defined in ControlProtocol.h"
#endif

// ==== Hardware config ====
// RoboClaw UART2: TX=GPIO17 -> S1 (RX), RX=GPIO16 <- S2 (TX via 5V->3V3 level shift)
#define ROBOCLAW_ADDR 0x80
static HardwareSerial& RC = Serial2;
static RoboClaw roboclaw(&RC, 100000); // 100ms timeout (more tolerant under load/EMI)

// Relay for shop-vac
#define RELAY_PIN         23
#define RELAY_ACTIVE_HIGH 1  // set to 0 if your module is active-LOW

// Limit switches (active-LOW with internal pullups). One on each side of chassis.
#define LIMIT_LEFT_PIN     18
#define LIMIT_RIGHT_PIN    19
#define LIMIT_ACTIVE_LOW   0   // set to 0 if your switches are active-HIGH

// Extend error bits reported in StatusPacket.errMask so handheld can display
// These bits do not collide with documented RoboClaw bits (we use 0x9000/0x9500)
#define ERRBIT_LIMIT_LEFT  0x9000
#define ERRBIT_LIMIT_RIGHT 0x9500

// Safety & loop timing
#define LINK_TIMEOUT_MS 300    // stop if no packet within this window
#define LOOP_INTERVAL_MS 20    // ~50 Hz

// Center deadzone with hysteresis (units: -1000..+1000 command space)
#define START_MOVE_THRESHOLD  200  // must exceed this to start moving
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
// Tracks when we're commanding near-full forward so we can snapshot diagnostics if motion drops
static bool fullFwdCommanded = false;

static bool limitLeft = false;
static bool limitRight = false;

static inline bool readLimitPin(uint8_t pin){
  int v = digitalRead(pin);
  if (LIMIT_ACTIVE_LOW) {
    return (v == HIGH);
  } else {
    return (v == LOW);
  }
}

static void sampleLimitSwitches(){
  // Simple polling; LOOP_INTERVAL_MS=20ms provides inherent debounce
  limitLeft  = readLimitPin(LIMIT_LEFT_PIN);
  limitRight = readLimitPin(LIMIT_RIGHT_PIN);
  Serial.print("Limit Left: "); Serial.print(limitLeft ? "TRIGGERED" : "OK");
  Serial.print(" | Limit Right: "); Serial.println(limitRight ? "TRIGGERED" : "OK");
}

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

// Robust ReadError() with small retry/backoff and RX flush to handle EMI/timeouts
static bool rcReadError(uint16_t &errOut) {
  const int MAX_TRIES = 3;
  for (int i = 0; i < MAX_TRIES; ++i) {
    bool ok = false;
    uint16_t e = roboclaw.ReadError(ROBOCLAW_ADDR, &ok);
    
    if (ok) { 
      errOut = e; 
      return true; 
    }
    // Flush any garbage from RX to resync framing before retry
    while (RC.available()) { 
      RC.read(); 
    }
    delay(3); // brief backoff to ride out motor switching noise
  }
  Serial.println("rcReadError: all retries failed");
  return false;
}

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

static bool manualMode = false;

static void driveFromXY(int16_t x, int16_t y) {

    // Improved mixing: scale X and Y so neither left nor right saturates
    int16_t sx = x;
    int16_t sy = y;
    int16_t maxMag = max(abs(x + y), abs(y - x));
    if (maxMag > 1000) {
      sx = x * 1000 / maxMag;
      sy = y * 1000 / maxMag;
    }
    int16_t left  = sy + sx;
    int16_t right = sy - sx;
    // No need to clamp, scaling above guarantees range
    const int16_t limit = 32767;
    int16_t dL = scaleLimit1000_toDuty(left,  limit);
    int16_t dR = scaleLimit1000_toDuty(right, limit);

    if (manualMode) {
      if (limitLeft || limitRight) {
        dL = 0;
        dR = 0;
      }
    }

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
  bool vOK=false;
  uint16_t err = 0;
  bool eOK = rcReadError(err);
  // err is a 16-bit mask. If read fails, we transmit 0xFFFF as a sentinel so the handheld can show 'ReadFail'.
  uint16_t mv10 = roboclaw.ReadMainBatteryVoltage(ROBOCLAW_ADDR, &vOK);
  uint16_t mergedErr = eOK ? (uint16_t)err : (uint16_t)0xFFFF;

  // Set limitMask bits: bit 0 = left, bit 1 = right, bit 2 = front, bit 3 = rear
  uint8_t limitMask = 0;
  if (limitLeft)  limitMask |= 0x01;
  if (limitRight) limitMask |= 0x02;
  // Add more switches here if needed

  StatusPacket s{};
  s.version = PROTO_VERSION;
  s.limitMask = limitMask;
  s.errMask = mergedErr;
  s.mainV10 = vOK ? (uint16_t)mv10 : (uint16_t)0;
  s.seq = ++statusSeq;
  esp_now_send(PEER_MAC, (const uint8_t*)&s, sizeof(s));
}
 
void onboard_setup(const uint8_t peer_mac[6]) {
  memcpy(PEER_MAC, peer_mac, 6);

  // Relay
  pinMode(RELAY_PIN, OUTPUT);
  setVac(false);

  // Limit switches
  pinMode(LIMIT_LEFT_PIN,  LIMIT_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
  pinMode(LIMIT_RIGHT_PIN, LIMIT_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
  sampleLimitSwitches();

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

    Serial.print("PROTO_VERSION="); Serial.println((int)PROTO_VERSION);
    Serial.print("sizeof(ControlPacket)="); Serial.println((int)sizeof(ControlPacket));
    Serial.print("sizeof(StatusPacket)=");  Serial.println((int)sizeof(StatusPacket));
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

  sampleLimitSwitches();

  ControlPacket pktLocal{};
  uint32_t pktMsLocal = 0;
  bool havePktLocal = false;

  taskENTER_CRITICAL(&espnowMux);
  memcpy(&pktLocal, &lastPkt, sizeof(pktLocal));
  pktMsLocal = lastPktMs;
  havePktLocal = havePkt;
  taskEXIT_CRITICAL(&espnowMux);

  bool linkOk = havePktLocal && ((now - pktMsLocal) <= LINK_TIMEOUT_MS);
  if (!linkOk) {
    // Send one last status snapshot on link loss
    sendStatusNow();
    stopMotors();
    return;
  }

  // Poll and log RoboClaw errors (print on change)
  if (now - lastErrorPollMs >= ERROR_POLL_INTERVAL_MS) {
    lastErrorPollMs = now;
    uint16_t err = 0;
    if (rcReadError(err)) {
      if (err != lastErrorMask) {
        lastErrorMask = err;
        printRcErrors(err);
        // Push an immediate status update on error change
        sendStatusNow();
      }
    } else {
      Serial.println("Error: failed to read RC error mask (timeout/CRC)"); // single line on failure
    }
  }

  // Optional: voltage sampling at 0.5 Hz
  static uint32_t lastVms = 0; 
  if (now - lastVms >= 2000) {
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

  manualMode = (pktLocal.mode == MODE_REMOTE);

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

    // Track when we're commanding near-full forward (within a simple window)
    bool nearFullForward = (abs(pktLocal.y) >= 800) && (abs(pktLocal.x) < 120);
    if (driveActive && nearFullForward) {
      fullFwdCommanded = true;
    }
    // If we were in a near-full-forward command and we just dropped out of driveActive,
    // capture and send a diagnostics snapshot to the handheld immediately.
    if (!driveActive && fullFwdCommanded) {
      fullFwdCommanded = false;
      // One-shot status packet with current error/voltage snapshot
      sendStatusNow();
    }
    if (!driveActive) {
      // Inside deadzone or failure condition - hard stop
      stopMotors();
    } else {
      // Outside deadzone - drive normally
      driveFromXY(pktLocal.x, pktLocal.y);
    }
  } else {
    // MODE_AUTO placeholder: currently stop (add autonomous logic later)
    fullFwdCommanded = false;
    stopMotors();
  }
}