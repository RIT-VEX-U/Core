#pragma once
#include "core/device/vdb/crc32.hpp"
#include <array>
#include <cstdio>
#include <cstring>
#include <deque>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace VDB {
uint32_t time_ms();
void delay_ms(uint32_t ms);
} // namespace VDB

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

class Part;
// Shared Part Pointer to delete an object that has no pointer pointing to it
//
using PartPtr = std::shared_ptr<Part>;
// Packet of bytes stored in a vector of 8 bit unsigned integers
using Packet = std::vector<uint8_t>;

// defines a channel id as an 8bit unsigned integer
using ChannelID = uint8_t;
class Channel {
  public:
    template <typename MutexType> friend class RegistryListener;
    friend class RegistryController;
    /**
     * Creates a channel used for sending data to the brain
     * @param data Part Pointer of data to be stored at the channel
     */
    explicit Channel(PartPtr data) : data(data) {}
    PartPtr data;
    /*
     * @return The Channel ID from 0 - 256
     */
    ChannelID getID() const;

  private:
    /**
     * Creates a channel used for sending data to the brain
     * @param data Part Pointer of data to be stored at the channel
     * @param channel_id The Channel ID to assign the channel from 0 - 256
     */
    Channel(PartPtr data, ChannelID channel_id) : data(data), id(channel_id) {}

    ChannelID id = 0;
    Packet packet_scratch_space;
    bool acked = false;
    // std::vector
};

/**
 * Prints out a packet of data
 */
void dump_packet_hex(const Packet &pac);
void dump_packet_8bit(const Packet &pac);

/**
 * defines what byte value correspondes to what packet type or packet function
 */
enum class PacketType : uint8_t {
  Broadcast = 0b00000000,
  Data = 0b10000000
};
enum class PacketFunction : uint8_t {
  Send = 0b00000000,
  Acknowledge = 0b00100000,
  Response = 0b01000000,
  Request = 0b01100000
};
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
PacketValidity validate_packet(const VDP::Packet &packet);
/**
 * defines what byte value is what type in a packet
 */
enum class Type : uint8_t {
    Record = 0,
    String = 1,
    // Enum

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

};

std::string to_string(Type t);

class PacketReader;
class PacketWriter;
class Visitor;
/**
 * adds indents to a stringstream
 * @param ss the stringstream to add indents to
 * @param indent the amount of double spaced indents to add
 */
void add_indents(std::stringstream &ss, size_t indent);

/**
 * defines a Part, which has a name and contains data
 * essentially defines data formatted so that it can be sent to the debug board
 */
class Part {
    friend class PacketReader;
    friend class PacketWriter;
    friend class Record;

  public:
    /**
     * Creates a Part with a name
     * a part is essentially data formatted so that it can be sent to the debug board
     * @param name name for the Part
     */
    Part(std::string name);
    /*
     * Deleter for the Part, used to delete the data once it is no longer needed
     * i.e after it has been sent to the debug board
     */
    virtual ~Part();
    /**
     *  @return a string of the Part with the format "name: string"
     */
    std::string pretty_print() const;
    /**
     * @return a string of the Part's data with the format "name: value"
     */
    std::string pretty_print_data() const;
    /*
     * sets the data the part contains, meant to be overrided
     */
    virtual void fetch() = 0;

    virtual void response();

    template<typename T>
      PartPtr to_PartPtr(){
        return std::make_shared<T>(this);
      };

    virtual VDP::PartPtr clone() = 0;
    /**
     * sets the data the part contains to the data from a packet, meant to be overrided
     * @param reader the PacketReader to read data from
     */
    virtual void read_data_from_message(PacketReader &reader) = 0;

    std::string get_name() const;

    virtual void Visit(Visitor *) = 0;

  protected:
    // These are needed to decode correctly but you shouldn't call them directly
    /**
     * writes the Part schematic to a packet so that it can be sent to the debug board, meant to be overrided
     * @param sofar the packet writer to write with
     */
    virtual void write_schema(PacketWriter &sofar) const = 0;
    /**
     * writes the value of the Part to a packet so that it can be sent to the debug board
     * @param sofar the packet writer to write with
     */
    virtual void write_message(PacketWriter &sofar) const = 0;
    /**
     * changes a stringstream to a specified format, meant to be overrided
     * @param ss the stream of strings to change
     * @param indent the amount of double spaced indents to add to the string
     */
    virtual void pprint(std::stringstream &ss, size_t indent) const = 0;
    /**
     * changes a stringstream to the contain the Part's data in a specified format, meant to be overrided
     * @param ss the stream of strings to change
     * @param indent the amount of double spaced indents to add to the string
     */
    virtual void pprint_data(std::stringstream &ss, size_t indent) const = 0;

    std::string name;
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
    template <typename Number> Number get_number() {
        // ensures that the function is only used on numbers
        static_assert(
          std::is_floating_point<Number>::value || std::is_integral<Number>::value,
          "This function should only be used on numbers"
        );
        // checks that the size of the number its trying to read combined with its location
        // doesnt put it past the packet size
        if (read_head + sizeof(Number) > pac.size()) {
            printf(
              "%s:%d: Reading a number[%d] at position %d would read past "
              "buffer of "
              "size %d\n",
              __FILE__, __LINE__, sizeof(Number), read_head, pac.size()
            );
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
    explicit PacketWriter(Packet &scratch_space);
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
    void write_string(const std::string &str);
    /**
     * writes a broadcast acknowledgement of a channel to the packet
     * @param chan the channel to write the acknowledgement for
     */
    void write_channel_acknowledge(const Channel &chan);
    /**
     * writes a broadcast of a channel schematic to the packet
     * @param chan the channel to write the schematic from
     */
    void write_channel_broadcast(const Channel &chan);
    /**
     * writes a response packet to the packets
     * @param chan the Channel to write the data from
     */
    void write_response(std::deque<Channel> &channels);
    /**
     * writes the data from a channel to the packet
     * @param chan the Channel to write the data from
     */
    void write_data_message(const Channel &part);
    /**
     * writes a request for a channel schematic to the packets
     * @param chan the Channel to write the data from
     */
    void write_request();
    /**
     * @return the packet the writer is writing to
     */
    const Packet &get_packet() const;
    /**
     * writes a number to the end of the packet
     */
    template <typename Number> void write_number(const Number &num) {
        std::array<uint8_t, sizeof(Number)> bytes;
        std::memcpy(&bytes, &num, sizeof(Number));
        for (const uint8_t b : bytes) {
            write_byte(b);
        }
    }

  private:
    Packet &sofar;
};
/**
 * defines a generic device to trasmit packets through
 */
class AbstractDevice {
  public:
    /** Sends a packet over some transmission medium
     * It is not specified how the packet reaches the partner
     * The transmission medium and wire format are left to the user
     * @param packet the packet to send through the device
     * @return whether the packet was sent sucessfully or not
     */
    virtual bool send_packet(const VDP::Packet &packet) = 0;
    /**
     * a callback to function that runs when a new packet is available
     * @param the function for the callback to call
     * me when my ex-wife
     */
    virtual void register_receive_callback(std::function<void(const VDP::Packet &packet)> callback) = 0;
    /**
     *  deleter for the device, used to delete it when it is no longer needed
     */
    virtual ~AbstractDevice();
};
/**
 * creates a decoder to decode a packet
 * @param pac the packet reader to make a decoder from
 * @return the Part Pointer for the data from the packet
 */
PartPtr make_decoder(PacketReader &pac);
/**
 * creates a byte from a given packet header
 * @return the header byte created
 */
uint8_t make_header_byte(PacketHeader head);
/**
 * @param hb the header byte to decode
 * @return a PacketHeader with the Function and Type from the byte decoded
 */
PacketHeader decode_header_byte(uint8_t hb);
/**
 * Decodes the broadcast in a packet
 * @param packet the packet to decode
 * @return the pair of the Channel ID and the Part Pointer of the packet schematic
 */
std::pair<ChannelID, PartPtr> decode_broadcast(const Packet &packet);

std::pair<ChannelID, PartPtr> decode_data(const Packet &packet);

} // namespace VDP
