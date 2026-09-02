#include "core/device/vdb/protocol.hpp"

#include <stdio.h>

#include <cstdint>
#include <cstring>
#include <functional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "core/device/vdb/types.hpp"

namespace VDP {

std::string to_string(Type t) {
  switch (t) {
    case Type::Record:
      return "record";
    case Type::Boolean:
      return "boolean";
    case Type::String:
      return "string";

    case Type::Float:
      return "float";
    case Type::Double:
      return "double";

    case Type::Uint8:
      return "uint8";
    case Type::Uint16:
      return "uint16";
    case Type::Uint32:
      return "uint32";
    case Type::Uint64:
      return "uint64";

    case Type::Int8:
      return "int8";
    case Type::Int16:
      return "int16";
    case Type::Int32:
      return "int32";
    case Type::Int64:
      return "int64";

    case Type::Q3_4:
      return "Q3_4";
    case Type::Q4_4:
      return "Q4_4";
    case Type::Q7_1:
      return "Q7_1";
    case Type::Q1_7:
      return "Q1_7";
    case Type::Q6_2:
      return "Q6_2";
    case Type::Q2_6:
      return "Q2_6";
    case Type::Q7_8:
      return "Q7_8";
    case Type::Q8_8:
      return "Q8_8";
    case Type::Q15_1:
      return "Q15_1";
    case Type::Q1_15:
      return "Q1_15";
    case Type::Q9_6:
      return "Q9_6";
    case Type::Q10_6:
      return "Q10_6";
    case Type::Q12_12:
      return "Q12_12";
    case Type::Q16_8:
      return "Q16_8";
    case Type::Q8_16:
      return "Q8_16";
    case Type::Q15_16:
      return "Q15_16";
    case Type::Q16_16:
      return "Q16_16";
    case Type::Q24_8:
      return "Q24_8";
    case Type::Q8_24:
      return "Q8_24";
    case Type::Q31_32:
      return "Q31_32";
    case Type::Q32_32:
      return "Q32_32";
    default:
      return "unknown";
  }

  return "<<UNKNOWN TYPE>>";
}

Type parse_type(std::string str) {
  if (str == "record") return Type::Record;
  if (str == "string") return Type::String;

  if (str == "float") return Type::Float;
  if (str == "double") return Type::Double;

  if (str == "uint8") return Type::Uint8;
  if (str == "uint16") return Type::Uint16;
  if (str == "uint32") return Type::Uint32;
  if (str == "uint64") return Type::Uint64;

  if (str == "int8") return Type::Int8;
  if (str == "int16") return Type::Int16;
  if (str == "int32") return Type::Int32;
  if (str == "int64") return Type::Int64;

  if (str == "Q3_4") return Type::Q3_4;
  if (str == "Q4_4") return Type::Q4_4;
  if (str == "Q7_1") return Type::Q7_1;
  if (str == "Q1_7") return Type::Q1_7;
  if (str == "Q6_2") return Type::Q6_2;
  if (str == "Q2_6") return Type::Q2_6;
  if (str == "Q7_8") return Type::Q7_8;
  if (str == "Q8_8") return Type::Q8_8;
  if (str == "Q15_1") return Type::Q15_1;
  if (str == "Q1_15") return Type::Q1_15;
  if (str == "Q9_6") return Type::Q9_6;
  if (str == "Q10_6") return Type::Q10_6;
  if (str == "Q12_12") return Type::Q12_12;
  if (str == "Q16_8") return Type::Q16_8;
  if (str == "Q8_16") return Type::Q8_16;
  if (str == "Q15_16") return Type::Q15_16;
  if (str == "Q16_16") return Type::Q16_16;
  if (str == "Q24_8") return Type::Q24_8;
  if (str == "Q8_24") return Type::Q8_24;
  if (str == "Q31_32") return Type::Q31_32;
  if (str == "Q32_32") return Type::Q32_32;
  return Type::UNKNOWN;
}

/**
 * Creates a Field
 * @param name name for the Field
 * @param value value for the Field to hold
 */

Field::Field(std::string name, FieldValue value) : name_(std::move(name)), value_(std::move(value)) {}

const std::string& Field::get_name() const { return name_; }

Type Field::get_type() const {
  return std::visit(
      [](const auto& value) -> Type {
        using ValueType = std::decay_t<decltype(value)>;

        if constexpr (std::is_same_v<ValueType, Record>) {
          return Type::Record;
        } else if constexpr (std::is_same_v<ValueType, std::string>) {
          return Type::String;
        } else if constexpr (std::is_same_v<ValueType, bool>) {
          return Type::Boolean;
        } else if constexpr (std::is_same_v<ValueType, double>) {
          return Type::Double;
        } else if constexpr (std::is_same_v<ValueType, float>) {
          return Type::Float;
        } else if constexpr (std::is_same_v<ValueType, uint8_t>) {
          return Type::Uint8;
        } else if constexpr (std::is_same_v<ValueType, uint16_t>) {
          return Type::Uint16;
        } else if constexpr (std::is_same_v<ValueType, uint32_t>) {
          return Type::Uint32;
        } else if constexpr (std::is_same_v<ValueType, uint64_t>) {
          return Type::Uint64;
        } else if constexpr (std::is_same_v<ValueType, int8_t>) {
          return Type::Int8;
        } else if constexpr (std::is_same_v<ValueType, int16_t>) {
          return Type::Int16;
        } else if constexpr (std::is_same_v<ValueType, int32_t>) {
          return Type::Int32;
        } else if constexpr (std::is_same_v<ValueType, int64_t>) {
          return Type::Int64;
        } else {
          return ValueType::type;
        }
      },
      value_);
}

const FieldValue& Field::get_raw_value() { return value_; }

bool SchemasMatch(Record a, Record b) {
  for (int i; i < a.size(); i++) {
    if ((a.at(i).get_name() == b.at(i).get_name()) && (a.at(i).get_type() == b.at(i).get_type())) {
      if (a.at(i).get_type() == Type::Record) {
        return SchemasMatch(a.at(i).get_value<Record>(), b.at(i).get_value<Record>());
      }
    } else {
      return false;
    }
  }
  return true;
};

Channel::Channel(ChannelID id, Record data) : id_(id), data_(data) {}

ChannelID Channel::get_id() const { return id_; }

Record Channel::get_data() {
  mtx.lock();
  Record copy = data_;
  mtx.unlock();
  return copy;
}

bool Channel::apply_update(Record& received) {
  mtx.lock();
  if (SchemasMatch(data_, received)) {
    this->data_ = received;
    mtx.unlock();
    return true;
  }
  mtx.unlock();
  return false;
}

VDP::Packet Channel::serialize() {
  VDP::Packet pac;
  make_header_byte(PacketType::Data, PacketFunction::Send);
}

/**
 * Defines a PacketReader to read a packet
 * @param pac the packet to read
 */
PacketReader::PacketReader(Packet pac) : pac(std::move(pac)), read_head(0) {}
/**
 * Defines a PacketReader to read a packet with a set start location for the packet
 * @param pac the packet to read
 * @param start the start location for the reader to start reading from
 */
PacketReader::PacketReader(Packet pac, size_t start) : pac(std::move(pac)), read_head(start) {}
/**
 * checks a packets validility
 * @param packet the packet to check the validity of
 * @return the PacketValidility (TooSmall, BadChecksum, or Ok)
 */
VDP::PacketValidity validate_packet(const VDP::Packet& packet) {
  VDPTracef("Validating packet of size %d", (int)packet.size());

  // packet header byte + checksum = 5 bytes,
  static constexpr size_t min_packet_size = 5;

  // checks that the minimum packet size is met
  if (packet.size() < min_packet_size) {
    return VDP::PacketValidity::TooSmall;
  }
  // calculates the checksum for the packet
  uint32_t checksum = CRC32::calculate(packet.data(), packet.size() - 4);

  // recreates the checksum manually
  auto size = packet.size();
  const uint32_t written_checksum = (uint32_t(packet[size - 1]) << 24) | (uint32_t(packet[size - 2]) << 16) |
                                    (uint32_t(packet[size - 3]) << 8) | uint32_t(packet[size - 4]);
  // checks if both checksums match
  if (checksum != written_checksum) {
    VDPWarnf("Checksums do not match: expected: %08lx, got: %08lx", checksum, written_checksum);
    return VDP::PacketValidity::BadChecksum;
  }
  // if no problems with the packet are found, packet is Ok
  return VDP::PacketValidity::Ok;
}
/**
 * @return the current byte the reader is on
 */
uint8_t PacketReader::get_byte() {
  const uint8_t b = pac[read_head];
  read_head++;
  return b;
}
/**
 * @return the current type the reader is on
 */
Type PacketReader::get_type() {
  const uint8_t val = get_byte();
  return (Type)val;
}
/**
 * @return the string the reader is at the start of
 */
std::string PacketReader::get_string() {
  std::string s;
  // iterates through the string until it reaches a 0 (end of the string)
  while (1) {
    const uint8_t c = get_byte();
    if (c == 0) {
      break;
    }
    s.push_back((char)c);
  }
  return s;
}

/**
 * creates a packet writer
 * @param scratch_space the packet for the writer to write to
 */
PacketWriter::PacketWriter(VDP::Packet& scratch) : sofar(scratch) {}
/**
 * clears the packet the writer is writing to
 */
void PacketWriter::clear() { sofar.clear(); }
/**
 * @return the size of the packet
 */
size_t PacketWriter::size() { return sofar.size(); }
/**
 * writes a byte to the end of the packet
 * @param b the byte to write
 */
void PacketWriter::write_byte(uint8_t b) { sofar.push_back(b); }
/**
 * writes a VDP type to the packet in the form of a byte
 * @param t the VDP type to write to the packet
 */
void PacketWriter::write_type(Type t) { write_byte((uint8_t)t); }
/**
 * writes a string to the packet
 * @param str the string to write to the packet
 */
void PacketWriter::write_string(const std::string& str) {
  // inserts a string into the end of the packet in bytes
  sofar.insert(sofar.end(), str.begin(), str.end());
  // adds a 0 byte after the string to signal the end of the string
  sofar.push_back(0);
}

/**
 * @return the packet the writer is writing to
 */
const Packet& PacketWriter::get_packet() const { return sofar; }

/**
 * writes a broadcast acknowledgement of a channel to the packet
 * @param chan the channel to write the acknowledgement for
 */
void PacketWriter::write_channel_acknowledge(const Channel& chan) {
  clear();
  // makes a header byte with the type broadcast and the function acknowledgement
  const uint8_t header = make_header_byte(PacketHeader{PacketType::Broadcast, PacketFunction::Acknowledge});

  // writes the header byte and channel id to the packet
  write_number<uint8_t>(header);
  write_number<ChannelID>(chan.getID());

  // creates and writes the Checksum to the packet
  uint32_t crc = CRC32::calculate(sofar.data(), sofar.size());
  write_number<uint32_t>(crc);
}
/**
 * writes a broadcast of a channel schematic to the packet
 * @param chan the channel to write the schematic from
 */
void PacketWriter::write_channel_broadcast(const Channel& chan) {
  clear();
  // makes a header byte with the type broadcast and function send
  const uint8_t header = make_header_byte(PacketHeader{PacketType::Broadcast, PacketFunction::Send});
  // writes the header byte and channel id to the packet
  write_number<uint8_t>(header);
  write_number<ChannelID>(chan.getID());

  // writes the packet schematic from the channel to the packet
  chan.data->write_schema(*this);

  // creates and writes the Checksum to the packet
  uint32_t crc = CRC32::calculate(sofar.data(), sofar.size());
  write_number<uint32_t>(crc);
}

/**
 * writes the data from a channel to the packet
 * @param chan the Channel to write the data from
 */
void PacketWriter::write_data_message(const Channel& chan) {
  clear();
  // makes a header byte with the type data and function send
  const uint8_t header = make_header_byte(PacketHeader{PacketType::Data, PacketFunction::Send});

  // writes the header byte and channel id to the packet
  write_number<uint8_t>(header);
  write_number<ChannelID>(chan.getID());

  // writes the data from the channel to the packet
  chan.data->write_message(*this);

  // creates and writes the Checksum to the packet
  uint32_t crc = CRC32::calculate(sofar.data(), sofar.size());
  // printf("data checksum: %08lx\n", crc);
  write_number<uint32_t>(crc);
}

/**
 * writes a request for a channel schematic to the packet
 * @param chan the channel to request
 */
void PacketWriter::write_request() {
  clear();
  // makes a header byte with the type broadcast and the function acknowledgement
  const uint8_t header = make_header_byte(PacketHeader{PacketType::Broadcast, PacketFunction::Request});
  // writes the header byte and channel id to the packet
  write_number<uint8_t>(header);
  // creates and writes the Checksum to the packet
  uint32_t crc = CRC32::calculate(sofar.data(), sofar.size());
  write_number<uint32_t>(crc);
}
/**
 * writes a response packet to the brain
 * @param response_queue the queue of channels to respond with
 */
void PacketWriter::write_response(std::deque<Channel>& response_queue) {
  clear();
  // makes a header byte with the type broadcast and the function Receive
  const uint8_t header = make_header_byte(PacketHeader{PacketType::Data, PacketFunction::Response});

  // writes the header byte and number of responses in the queue
  write_number<uint8_t>(header);
  write_number<uint8_t>(response_queue.size());
  // writes the channel id for the channel we are responding to
  write_number<ChannelID>(response_queue.front().getID());
  response_queue.front().data->write_message(*this);
  // removes the response from the queue
  response_queue.pop_front();

  // creates and writes the Checksum to the packet
  uint32_t crc = CRC32::calculate(sofar.data(), sofar.size());
  write_number<uint32_t>(crc);
}

/**
 *  deleter for the device, used to delete it when it is no longer needed
 */
AbstractDevice::~AbstractDevice() {}
/**
 * creates a decoder to decode a packet
 * @param pac the packet reader to make a decoder from
 * @return the Part Pointer for the data from the packet
 */
PartPtr make_decoder(PacketReader& pac) {
  /**
   * gets the type and name of the packet and contstructs a Part pointer from it
   */
  const Type t = pac.get_type();
  const std::string name = pac.get_string();

  switch (t) {
    case Type::String:
      return PartPtr(new String(name));

    case Type::Float:
      return PartPtr(new Float(name));
    case Type::Double:
      return PartPtr(new Double(name));

    case Type::Uint8:
      return PartPtr(new Uint8(name));
    case Type::Uint16:
      return PartPtr(new Uint16(name));
    case Type::Uint32:
      return PartPtr(new Uint32(name));
    case Type::Uint64:
      return PartPtr(new Uint64(name));

    case Type::Int8:
      return PartPtr(new Int8(name));
    case Type::Int16:
      return PartPtr(new Int16(name));
    case Type::Int32:
      return PartPtr(new Int32(name));
    case Type::Int64:
      return PartPtr(new Int64(name));
  }
  return nullptr;
}
static constexpr auto PACKET_TYPE_BIT_MASK = 0b10000000;
static constexpr auto PACKET_FUNCTION_BIT_MASK = 0b01100000;

uint8_t make_header_byte(PacketHeader head) { return (uint8_t)head.type | (uint8_t)head.func; }

PacketHeader decode_header_byte(uint8_t hb) {
  const PacketType pt = (PacketType)(hb & PACKET_TYPE_BIT_MASK);
  const PacketFunction func = (PacketFunction)(hb & PACKET_FUNCTION_BIT_MASK);

  return {pt, func};
}
/**
 * Decodes the broadcast in a packet
 * @param packet the packet to decode
 * @return the pair of the Channel ID and the Part Pointer of the packet schematic
 */
std::pair<ChannelID, PartPtr> decode_broadcast(const Packet& packet) {
  VDPTracef("Decoding broadcast of size: %d", (int)packet.size());
  PacketReader reader(packet);
  // reads the header byte, which had to be read to know were a braodcast
  (void)reader.get_byte();
  // checks the channel id from the packet
  const ChannelID id = reader.get_number<ChannelID>();
  // constructs the schematic for the packet from the byte as a Part Pointer
  const PartPtr schema = make_decoder(reader);
  // returns the pair of the channel id and the packet shematic
  return {id, schema};
}

}  // namespace VDP
