#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "core/device/vdb/crc32.hpp"
#include "vex.h"

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

/**
 * defines what byte value is what type in a packet
 */

enum class PacketType : uint8_t { Schema = 0b00000000, Data = 0b10000000 };

enum class PacketFunction : uint8_t {
  Send = 0b00000000,
  Acknowledge = 0b01000000,
};

using ChannelID = uint8_t;

/**
 * struct to define the header of a packet,
 * defines wheether a packet is Broadcoast or data
 * and whether a packet is send or recieve
 */
struct PacketHeader {
  PacketType type;
  PacketFunction func;
};

enum PacketValidity : uint8_t {
  Ok,
  BadChecksum,
  TooSmall,
};

uint8_t make_header_byte(PacketHeader head);

PacketValidity validate_packet(const VDP::Packet& packet);

template <typename T>
class Field;

using FieldRecord = std::tuple<>;

/**
 * defines a Field
 * A named value that can be serialized and sent to the debug board.
 */
template <typename T>
class Field {
 public:
  /**
   * Creates a Field
   * @param name name for the Field
   * @param value value for the Field to hold; its C++ type determines the VDP Type
   */
  Field(std::string name, T value) : name_(std::move(name)), value_(std::move(value)) {};

  const std::string& get_name() const { return name_; };

  const T& get_value() const {
    return value_;
  }

  bool schemas_match(const Field& other) const {
    using otherFieldType = decltype(other)::value_type;
    return std::is_same_v<T, otherFieldType> && name_ = other.get_name(); 
  };

  bool apply_update(Field& other) {
    if(this->schemas_match(other)) {
      mut.lock();
      this->value_ = other.get_value();
      mut.unlock();
      return true;
    }
    return false;
  }

  Packet to_schema() {
    Packet out;
    out.insert(out.end(), name_.end(), name_.begin());
    out.push_back(0);
  }

  consteval void inspect() {
    constexpr auto type = ^^T;

    constexpr auto type_name = std::meta::display_string_of(type);

    constexpr auto members = std::meta::nonstatic_data_members _of(
        type,
        std::meta::access_context::unchecked()
        );
    for (auto member: members) {
      auto member_name = std::meta::identifier_of(member);
      auto member_type = std::meta::type_of(member);
    }
  }

 private:
  vex::mutex mut;
  std::string name_;
  T value_;
};

template <typename T>
class Channel {
 public:
   

  ChannelID get_id() const;

  Field<T> get_data();

  bool apply_update(Field<T>& received);

  void encode_broadcast(Packet& pac);
  void encode_data(Packet& pac);

 private:
  ChannelID id_;
  Field<T> data_;
  vex::mutex mtx;
};

}  // namespace VDP
