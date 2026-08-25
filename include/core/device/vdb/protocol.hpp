#pragma once
#include <array>
#include <cstdio>
#include <cstring>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <variant>
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
enum class Type : uint8_t {
  Record = 0,
  Boolean = 1,
  String = 2,

  Double = 3,
  Float = 4,

  Uint8 = 5,
  Uint16 = 6,
  Uint32 = 7,
  Uint64 = 8,

  Int8 = 9,
  Int16 = 10,
  Int32 = 11,
  Int64 = 12,

  Q3_4 = 13,
  Q4_4 = 14,
  Q7_1 = 15,
  Q1_7 = 16,
  Q6_2 = 17,
  Q2_6 = 18,
  Q7_8 = 19,
  Q8_8 = 20,
  Q15_1 = 21,
  Q1_15 = 22,
  Q10_6 = 23,
  Q9_6 = 24,
  Q12_12 = 25,
  Q16_8 = 26,
  Q8_16 = 27,
  Q15_16 = 28,
  Q16_16 = 29,
  Q24_8 = 30,
  Q8_24 = 31,
  Q31_32 = 32,
  Q32_32 = 33,

  UNKNOWN = 34,
};

enum class PacketType : uint8_t { Data = 0b00000000, Broadcast = 0b00000001, Acknow };

enum class PacketFunction : uint8_t {
  Send = 0b00000000,
  Recieve = 0b00000001,
};

std::string to_string(Type t);

Type parse_type(std::string);

class Field;
using Record = std::vector<Field>;

/**
 * Stores the raw integer representation of a fixed-point value.
 * The Type template argument keeps fixed-point formats with the same storage
 * type distinct from one another.
 */
template <Type FixedPointType, typename Storage>
struct FixedPoint {
  static constexpr Type type = FixedPointType;

  Storage raw_value;
};

using ChannelID = uint8_t;

using Q3_4 = FixedPoint<Type::Q3_4, int8_t>;
using Q4_4 = FixedPoint<Type::Q4_4, uint8_t>;
using Q7_1 = FixedPoint<Type::Q7_1, int8_t>;
using Q1_7 = FixedPoint<Type::Q1_7, int8_t>;
using Q6_2 = FixedPoint<Type::Q6_2, int8_t>;
using Q2_6 = FixedPoint<Type::Q2_6, int8_t>;

using Q7_8 = FixedPoint<Type::Q7_8, int16_t>;
using Q8_8 = FixedPoint<Type::Q8_8, uint16_t>;
using Q15_1 = FixedPoint<Type::Q15_1, int16_t>;
using Q1_15 = FixedPoint<Type::Q1_15, int16_t>;
using Q10_6 = FixedPoint<Type::Q10_6, uint16_t>;
using Q9_6 = FixedPoint<Type::Q9_6, int16_t>;

using Q12_12 = FixedPoint<Type::Q12_12, int32_t>;
using Q16_8 = FixedPoint<Type::Q16_8, int32_t>;
using Q8_16 = FixedPoint<Type::Q8_16, int32_t>;
using Q15_16 = FixedPoint<Type::Q15_16, int32_t>;
using Q16_16 = FixedPoint<Type::Q16_16, uint32_t>;
using Q24_8 = FixedPoint<Type::Q24_8, int32_t>;
using Q8_24 = FixedPoint<Type::Q8_24, int32_t>;

using Q31_32 = FixedPoint<Type::Q31_32, int64_t>;
using Q32_32 = FixedPoint<Type::Q32_32, uint64_t>;

using FieldValue = std::variant<Record, std::string, bool, double, float, uint8_t, uint16_t, uint32_t, uint64_t, int8_t,
                                int16_t, int32_t, int64_t, Q3_4, Q4_4, Q7_1, Q1_7, Q6_2, Q2_6, Q7_8, Q8_8, Q15_1, Q1_15,
                                Q10_6, Q9_6, Q12_12, Q16_8, Q8_16, Q15_16, Q16_16, Q24_8, Q8_24, Q31_32, Q32_32>;

/**
 * defines a Field
 * A named value that can be serialized and sent to the debug board.
 */
class Field {
 public:
  /**
   * Creates a Field
   * @param name name for the Field
   * @param value value for the Field to hold; its C++ type determines the VDP Type
   */
  Field(std::string name, FieldValue value);

  const std::string& get_name() const;

  Type get_type() const;

  template <typename T>
  const T& get_value() const {
    return std::get<T>(value_);
  }

  const FieldValue& get_raw_value();

 private:
  std::string name_;
  FieldValue value_;
};

bool SchemasMatch(Record a, Record b);

class Channel {
 public:
  Channel(ChannelID id, Record data);

  ChannelID get_id() const;

  Record get_data();

  bool apply_update(Record& received);

  VDP::Packet serialize();

 private:
  ChannelID id_;
  Record data_;
  vex::mutex mtx;
};

/*
 * Defines a PacketReader, it reads packets
 */
class PacketReader {
 public:
  /**
   * Defines a PacketReader to read a packet
   * @param pac the packet to read
   */
  PacketReader(Packet pac);
  /**
   * Defines a PacketReader to read a packet with a set start location for the packet
   * @param pac the packet to read
   * @param start the start location for the reader to start reading from
   */
  PacketReader(Packet pac, size_t start);
  /**
   * @return the current byte the reader is on
   */
  uint8_t get_byte();
  /**
   * @return the type of the current byte the reader is on
   */
  Type get_type();
  /**
   * @return a string of bytes the reader is reading until the next 0 byte (end of the Packet)
   */
  std::string get_string();

  /**
   * @return the value stored by a Number Part
   */
  template <typename Number>
  Number get_number() {
    // ensures that the function is only used on numbers
    static_assert(std::is_floating_point<Number>::value || std::is_integral<Number>::value,
                  "This function should only be used on numbers");
    // checks that the size of the number its trying to read combined with its location
    // doesnt put it past the packet size
    if (read_head + sizeof(Number) > pac.size()) {
      printf(
          "%s:%d: Reading a number[%d] at position %d would read past "
          "buffer of "
          "size %d\n",
          __FILE__, __LINE__, sizeof(Number), read_head, pac.size());
      return 0;
    }
    Number value = 0;
    // copies the the number at the reader head to the Number's stored value and
    // adds the size of the number to the read head so it moves on to the next set of bits
    std::memcpy(&value, &pac[read_head], sizeof(Number));
    read_head += sizeof(Number);
    return value;
  }

 private:
  Packet pac;
  size_t read_head;
};
/**
 * Defines a PacketWriter, it writes packets
 */
class PacketWriter {
 public:
  /**
   * creates a packet writer
   * @param scratch_space the packet for the writer to write to
   */
  explicit PacketWriter(Packet& scratch_space);
  /**
   * clears the packet the writer is writing to
   */
  void clear();
  /**
   * @return the size of the packet
   */
  size_t size();
  /**
   * writes a byte to the end of the packet
   * @param b the byte to write
   */
  void write_byte(uint8_t b);
  /**
   * writes a VDP type to the packet in the form of a byte
   * @param t the VDP type to write to the packet
   */
  void write_type(Type t);
  /**
   * writes a string to the packet
   * @param str the string to write to the packet
   */
  void write_string(const std::string& str);
  /**
   * writes a broadcast acknowledgement of a channel to the packet
   * @param chan the channel to write the acknowledgement for
   */
  void write_channel_acknowledge(const Channel& chan);
  /**
   * writes a broadcast of a channel schematic to the packet
   * @param chan the channel to write the schematic from
   */
  void write_channel_broadcast(const Channel& chan);
  /**
   * writes a response packet to the packets
   * @param chan the Channel to write the data from
   */
  void write_response(std::deque<Channel>& channels);
  /**
   * writes the data from a channel to the packet
   * @param chan the Channel to write the data from
   */
  void write_data_message(const Channel& part);
  /**
   * writes a request for a channel schematic to the packets
   * @param chan the Channel to write the data from
   */
  void write_request();
  /**
   * @return the packet the writer is writing to
   */
  const Packet& get_packet() const;
  /**
   * writes a number to the end of the packet
   */
  template <typename Number>
  void write_number(const Number& num) {
    std::array<uint8_t, sizeof(Number)> bytes;
    std::memcpy(&bytes, &num, sizeof(Number));
    for (const uint8_t b : bytes) {
      write_byte(b);
    }
  }

 private:
  Packet& sofar;
};

}  // namespace VDP
