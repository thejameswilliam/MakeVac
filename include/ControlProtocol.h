#pragma once
#include <stdint.h>

// Bump this if/when packet layout changes
#define PROTO_VERSION 1

// Modes
enum : uint8_t { MODE_REMOTE = 0, MODE_AUTO = 1 };

// Ensure identical packing on both targets
#if defined(__GNUC__)
#  define PACKED __attribute__((packed))
#else
#  define PACKED
#endif

// Handheld → Onboard payload (fixed 8 bytes)
typedef struct PACKED {
  uint8_t  version;   // PROTO_VERSION
  int16_t  x;         // turn axis, -1000..+1000 (left..right)
  int16_t  y;         // fwd axis,  -1000..+1000 (back..fwd)
  uint8_t  mode;      // MODE_REMOTE or MODE_AUTO
  uint8_t  vac;       // 0=off, 1=on
  // total = 1+2+2+1+1 = 7 -> add 1 byte pad to reach 8 bytes
  // (explicit to keep sender/receiver in lockstep)
  uint8_t  _rsvd0;
} ControlPacket;

// Onboard → Handheld status payload (fixed 12 bytes)
typedef struct PACKED {
  uint8_t  version;   // PROTO_VERSION
  uint8_t  _rsvd0;    // keep 16-bit alignment below (and match 12-byte target size)
  uint16_t errMask;   // RoboClaw error bitmask (0xFFFF = read-fail sentinel)
  uint16_t mainV10;   // Main battery voltage in 0.1 V units (0 = read-fail)
  uint32_t seq;       // incrementing sequence
  uint16_t _rsvd1;    // pad to 12 bytes; reserved for future fields
} StatusPacket;

// Compile-time guards to catch drift early
static_assert(sizeof(ControlPacket) == 8,  "ControlPacket must be 8 bytes");
static_assert(sizeof(StatusPacket)  == 12, "StatusPacket must be 12 bytes");