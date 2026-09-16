#pragma once
#include <sys/types.h>
#include <vex_thread.h>

#include <array>
#include <bit>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include "crc32.hpp"
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

enum PacketValidity : uint8_t {
  Ok,
  BadChecksum,
  TooSmall,
};

VDP::PacketValidity validate_packet(const VDP::Packet& packet);

VDP::Packet checksum_pac(VDP::Packet in);

using Packet = std::vector<uint8_t>;

using ChannelID = uint8_t;

/**
 * defines what byte value is what type in a packet
 */

enum class PacketType : uint8_t {
  Data = 0b00000000,
  Schema = 0b00000001,
};

enum class PacketFunction : uint8_t {
  Send = 0b00000010,
  Acknowledge = 0b00000100,
  Holding = 0b00000110,
};

struct PacketHeader {
  PacketType type;
  PacketFunction func;
};

uint8_t make_header_byte(PacketHeader head);

PacketHeader decode_header_byte(uint8_t hb);

Packet checksum_pac(VDP::Packet in);

template <typename T>
  requires IsField<std::remove_cvref_t<T>>::value
class Channel {
 public:
  explicit Channel(T data) : id_(0), data(std::move(data)), acked(false) {}

  void set_id(ChannelID id) { id_ = id; }

  ChannelID get_id() const {
    return id_;
  };

  T& get_data() { return data; }

  void acknowledge() { acked = true; }

  bool apply_update(VDP::Packet data_packet) { return data.apply_update(data_packet); }

  VDP::Packet serialize(PacketType pac_type) {
    VDP::Packet out;
    out.push_back((uint8_t)PacketFunction::Send | (uint8_t)PacketType::Schema);
    out.push_back((uint8_t)id_);

    VDP::Packet packet_body; 
    if(pac_type == PacketType::Data) {
      packet_body = data.serialize_data();
    }
    else if(pac_type == PacketType::Schema) {
      packet_body = data.serialize_schema();
    }

    out.insert(out.end(), packet_body.begin(), packet_body.end());
    VDP::Packet checksum = checksum_pac(out);
    out.insert(out.end(), checksum.begin(), checksum.end());
    return out;
  };

  std::string schema_to_string() const {
    std::string out = "";
    out += "{\n  id : " + std::to_string(id_) + ",\n";
    out += data.schema_to_string(1) + "\n}";
    return out;
  }

  std::string data_to_string() const {
    std::string out = "";
    out += "{\n  id : " + std::to_string(id_) + ",\n";
    out += data.data_to_string(1) + "\n}";
    return out;
  }

 private:
  ChannelID id_;
  T data;
  bool acked;
};

template <typename T>
Channel(ChannelID, T) -> Channel<T>;

}  // namespace VDP
