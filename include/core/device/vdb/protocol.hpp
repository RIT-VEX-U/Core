#pragma once
#include <vex_thread.h>
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include "types.hpp"

namespace VDB {
uint32_t time_ms();
void delay_ms(uint32_t ms);
}  // namespace VDB

// #define VDPTRACE
#define VDPDEBUG
#define VDPWARN

#ifdef VDPWARN
#define VDPWarnf(fmt, ...) printf("WARN: " fmt "\n", ##__VA_ARGS__)
#else
#define VDPWarnf(...)
#endif

#ifdef VDPDEBUG
#define VDPDebugf(fmt, ...) printf("DEBUG: " fmt "\n", ##__VA_ARGS__)
#else
#define VDPDebugf(...)
#endif

#ifdef VDPTRACE
#define VDPTracef(fmt, ...) printf("TRACE: " fmt "\n", ##__VA_ARGS__)
#else
#define VDPTracef(...)
#endif

namespace VDP {
constexpr size_t MAX_CHANNELS = 256;

using Packet = std::vector<uint8_t>;

using ChannelID = uint8_t;

/**
 * defines what byte value is what type in a packet
 */

enum class PacketType : uint8_t { Data = 0b00000000, Broadcast = 0b00000001, Acknow };

enum class PacketFunction : uint8_t {
  Send = 0b00000000,
  Recieve = 0b00000001,
};

std::string to_string(TypeId t);

TypeId parse_type(std::string);

template <typename T>
class Channel {
 public:
  Channel(ChannelID id, Field<T> data);

  ChannelID get_id() const;

  Field<T> get_data() {
    return data;
  }

  bool apply_update(Field<T> recieved) {
    return data.apply_update(recieved);
  }

  VDP::Packet serialize();

 private:
  ChannelID id_;
  Field<T> data;
};

}  // namespace VDP
