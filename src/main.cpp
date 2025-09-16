#include <Arduino.h>
#include <WiFi.h>
#include "ControlProtocol.h"

// === CONFIG: set these to your actual device MAC addresses ===
static const uint8_t MAC_HANDHELD[6] = { 0x68, 0x25, 0xDD, 0x32, 0xA2, 0x24 }; 
static const uint8_t MAC_ONBOARD [6] = { 0x68, 0x25, 0xDD, 0x32, 0x08, 0x50 };

// Forward declarations implemented in handheld.cpp / onboard.cpp
void handheld_setup(const uint8_t peer_mac[6]);
void handheld_loop();

void onboard_setup(const uint8_t peer_mac[6]);
void onboard_loop();

// Role dispatch
enum Role { ROLE_UNKNOWN=0, ROLE_HANDHELD, ROLE_ONBOARD };
static Role role = ROLE_UNKNOWN;

// Simple MAC compare helper
static bool macEq(const uint8_t a[6], const uint8_t b[6]) {
  for (int i=0; i<6; i++) {
    if (a[i]!=b[i]) {
      return false;
    }
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(100);

  WiFi.mode(WIFI_STA); // required before macAddress()
  String macStr = WiFi.macAddress();
  uint8_t my[6] = {0};
  WiFi.macAddress(my);

  Serial.print("Device MAC: "); Serial.println(macStr);

  if (macEq(my, MAC_HANDHELD)) {
    role = ROLE_HANDHELD;
    Serial.println("Role: HANDHELD");
    handheld_setup(MAC_ONBOARD);
  } else if (macEq(my, MAC_ONBOARD)) {
    role = ROLE_ONBOARD;
    Serial.println("Role: ONBOARD");
    onboard_setup(MAC_HANDHELD);
  } else {
    Serial.println("Role: UNKNOWN (MAC not recognized). Please check MAC definitions in main.cpp");
  }
}

void loop() {
  switch (role) {
    case ROLE_HANDHELD: handheld_loop(); break;
    case ROLE_ONBOARD:  onboard_loop();  break;
    default: delay(1000); break;
  }
}