#pragma once
#include <stdint.h>

#define PROTO_VERSION 1

// Modes
enum : uint8_t { MODE_REMOTE = 0, MODE_AUTO = 1 };

// Handheld → Onboard payload (fits in ESP-NOW)
typedef struct __attribute__((packed)) {
  uint8_t  version;   // PROTO_VERSION
  int16_t  x;         // turn axis, -1000..+1000 (left..right)
  int16_t  y;         // fwd axis,  -1000..+1000 (back..fwd)
  uint8_t  mode;      // MODE_REMOTE or MODE_AUTO
  uint8_t  vac;       // 0=off, 1=on
  uint32_t seq;       // incrementing sequence
} ControlPacket;

// Onboard -> Handheld status payload
typedef struct __attribute__((packed)) {
  uint8_t  version;   // PROTO_VERSION
  uint16_t errMask;   // RoboClaw error bitmask
  uint16_t mainV10;   // Main battery voltage in 0.1 V units
  uint32_t seq;       // incrementing sequence
} StatusPacket;