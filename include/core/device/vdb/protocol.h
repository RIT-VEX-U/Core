#pragma once
#include <sys/types.h>
#include <v5_api.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "types.h"

namespace VDB {
uint32_t time_ms();
void delay_ms(uint32_t ms);
}  // namespace VDB

#define VDPWarnf(fmt, ...) printf("WARN: " fmt "\n", ##__VA_ARGS__)
#define VDPDebugf(fmt, ...) printf("DEBUG: " fmt "\n", ##__VA_ARGS__)
#define VDPTracef(fmt, ...) printf("TRACE: " fmt "\n", ##__VA_ARGS__)

namespace VDP {
constexpr size_t MAX_CHANNELS = 256;

/// channel ids are in the form of a byte
using ChannelID = uint8_t;

/**
 * Packet Types are either
 * Data Packets, holding raw data to be sent to the debug board
 * Schema Packets, holding a schema to tell the debug board what each channel looks like
 */
enum class PacketType : uint8_t {
  Data = 0b00000000,
  Schema = 0b00000001,
};

/**
 * Packet Functions are either
 * Send, just sending information to the board
 * Acknowledge, acknoledging a specific channel of data
 */
enum class PacketFunction : uint8_t {
  Send = 0b00000010,
  Acknowledge = 0b00000100,
};

/**
 * PacketValidities can be
 * Ok, packet successfully validated
 * BadChecksum, packet's chceksum does not match recreated checksum
 * TooSmall, packet is too small to be a real packet
 */
enum PacketValidity : uint8_t {
  Ok,
  BadChecksum,
  TooSmall,
};

/// A packet header is byte that combines a packet function and packet type
struct PacketHeader {
  PacketType type;
  PacketFunction func;
};

/**
 * @brief Checks the validity of a packet
 * @param packet the packet to checksum
 * @return the type of validity of the packet
 */
VDP::PacketValidity validate_packet(const VDP::Packet& packet);

/**
 * @brief Creates a checksum for a packet
 * @param in the packet to create a checksum for
 * @return the checksum in packet form
 */
VDP::Packet checksum_pac(VDP::Packet in);

/// Creates a header byte from a PacketHeader
uint8_t make_header_byte(PacketHeader head);

/// Creates a PacketHeader from a packet
PacketHeader decode_header_byte(uint8_t hb);

/// creates a checksum for a packet in the form of a packet
Packet checksum_pac(VDP::Packet in);

/// A channel of data to be send to the debug board
template <typename T>
  requires IsField<std::remove_cvref_t<T>>::value
class Channel {
 public:
  /**
   * @brief Constructs a channel of data to be sent to the debug board
   * @param data a field of data to be send through this channel
   * @param id a byte designating the channel
   */
  explicit Channel(T& data, ChannelID id) : id_(id), data_(data), last_received_(0) {}

  /// Gets the channel's id
  ChannelID get_id() const { return id_; };

  /// Gets the data stored inside the channel
  T& get_data() { return data_; }

  /**
   * @brief Applies an update from a received packet of data to the data held by the channel
   * @param data_packet the received packet of data
   * @return wether or not the update was successful
   */
  bool apply_update(VDP::PacketSpan data_packet) {
    std::memcpy(&last_received_, data_packet.subspan(0, 4).data(), sizeof(uint32_t));
    return data_.apply_update(data_packet.subspan(4));
  }

  /**
   * @brief Serializes the channel as a packet to be sent over a wire to the debug board
   * @param pac_type the type of packet to create, schema or data
   */
  VDP::Packet serialize(PacketType pac_type) {
    VDP::Packet out;
    // add the header and channel id to the packet
    out.push_back((uint8_t)PacketFunction::Send | (uint8_t)pac_type);
    out.push_back((uint8_t)id_);

    // either serialize the data held by the channel or the channel schema
    VDP::Packet packet_body(sizeof(uint32_t));
    if (pac_type == PacketType::Data) {
      // add the timestamp to the packet
      uint32_t timestamp = vexSystemTimeGet();

      VDP::Packet timestamp_pac(sizeof(uint32_t));

      // Copy the timestamp into the vector's underlying storage
      std::memcpy(timestamp_pac.data(), &timestamp, sizeof(uint32_t));

      out.insert(out.end(), timestamp_pac.begin(), timestamp_pac.end());

      packet_body = data_.serialize_data();
    } else if (pac_type == PacketType::Schema) {
      packet_body = data_.serialize_schema();
    }
    out.insert(out.end(), packet_body.begin(), packet_body.end());

    // add the checksum to the packet
    VDP::Packet checksum = checksum_pac(out);
    out.insert(out.end(), checksum.begin(), checksum.end());
    return out;
  };

  /// Gets the channel's schema as a string
  std::string schema_to_string() const {
    std::string out = "";
    out += "{\n  id : " + std::to_string(id_) + ",\n";
    out += data_.schema_to_string(1) + "\n}";
    return out;
  }

  /// Gets the channel's data as a string
  std::string data_to_string() const {
    std::string out = "";
    out += "{\n  id : " + std::to_string(id_) + ",\n";
    out += "  last received: " + std::to_string(id_) + ",\n";
    out += data_.data_to_string(1) + "\n}";
    return out;
  }

  bool acked = false;

 private:
  ChannelID id_;
  T data_;
  uint32_t last_received_;
};

/// deduction guide so that template arguments are not required when creating a channel
template <typename T>
Channel(ChannelID, T) -> Channel<T>;

}  // namespace VDP
