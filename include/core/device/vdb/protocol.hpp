#pragma once
#include <sys/types.h>
#include <v5_api.h>

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

/**
 * checks the validity of a packet
 * @param packet the packet to checksum
 * @return the type of validity of the packet
 */
VDP::PacketValidity validate_packet(const VDP::Packet& packet);

/**
 * creates a checksum for a packet
 * @param in the packet to create a checksum for
 * @return the checksum in packet form
 */
VDP::Packet checksum_pac(VDP::Packet in);

using Packet = std::vector<uint8_t>;

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
 * Holding, the debug board has data it is waiting to send to the brain
 */
enum class PacketFunction : uint8_t {
    Send = 0b00000010,
    Acknowledge = 0b00000100,
    Holding = 0b00000110,
};

/// A packet header is byte that combines a packet function and packet type
struct PacketHeader {
    PacketType type;
    PacketFunction func;
};

/// creates a header byte from a PacketHeader
uint8_t make_header_byte(PacketHeader head);

/// creates a PacketHeader from a packet
PacketHeader decode_header_byte(uint8_t hb);

/// creates a checksum for a packet in the form of a packet
Packet checksum_pac(VDP::Packet in);

/**
 * A channel of data to be send to the debug board
 */
template <typename T>
    requires IsField<std::remove_cvref_t<T>>::value
class Channel {
   public:
    /**
     * @brief Constructs a channel of data to be send to the debug board
     * @param data a field of data to be send through this channel
     */
    explicit Channel(T& data, ChannelID id)
        : id_(id), data_(data), acked(false), last_recieved(0) {}

    /**
     * @return the channel id
     */
    ChannelID get_id() const { return id_; };

    /**
     * @return the data stored inside the channel
     */
    T& get_data() { return data_; }

    /**
     * @brief acknowledge the channel
     */
    void acknowledge() { acked = true; }

    /**
     * @brief unacknowledge the channel
     */
    void unacknowledge() { acked = false; }

    /**
     * @return whether the channel is acknowledged or not
     */
    bool is_acknowledged() { return acked; }

    /**
     * @brief applies an update from a recieved packet of data to the data held by the channel
     * @param data_packet the recieved packet of data
     * @return wether or not the update was successful
     */
    bool apply_update(VDP::Packet data_packet) {
        printf("applying update, packet size so far %d\n", data_packet.size());

        uint8_t time_bytes[4] = {
                data_packet.at(0), data_packet.at(1), data_packet.at(2), data_packet.at(3)
        };
        std::memcpy(&last_recieved, time_bytes, sizeof(uint32_t));
        return data_.apply_update({data_packet.begin() + 4, data_packet.end()});
    }

    /**
     * Serializes the channel as a packet to be sent over a wire to the debug board
     * @param pac_type the type of packet to create, schema or data
     */
    VDP::Packet serialize(PacketType pac_type) {
        VDP::Packet out;
        // add the header and channel id to the packet
        out.push_back((uint8_t)PacketFunction::Send | (uint8_t)pac_type);
        out.push_back((uint8_t)id_);

        // either serialize the data held by the channel or the schema
        VDP::Packet packet_body;
        if (pac_type == PacketType::Data) {
            // add the timestamp to the packet
            uint32_t timeSent = vexSystemTimeGet();

            out.push_back((timeSent >> 24) & 0xFF);
            out.push_back((timeSent >> 16) & 0xFF);
            out.push_back((timeSent >> 8) & 0xFF);
            out.push_back((timeSent) & 0xFF);

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

    /**
     * @brief gets the channel's schema as a string
     * @return a string representation of the channel's schema
     */
    std::string schema_to_string() const {
        std::string out = "";
        out += "{\n  id : " + std::to_string(id_) + ",\n";
        out += data_.schema_to_string(1) + "\n}";
        return out;
    }

    /**
     * @brief gets the channel's data as a string
     * @return a string representation of the channel's data
     */
    std::string data_to_string() const {
        std::string out = "";
        out += "{\n  id : " + std::to_string(id_) + ",\n";
        out += "  last recieved: " + std::to_string(id_) + ",\n";
        out += data_.data_to_string(1) + "\n}";
        return out;
    }

   private:
    ChannelID id_;
    T data_;
    bool acked;
    uint32_t last_recieved;
};

/// deduction guide so that template arguments are not required when creqting a channel
template <typename T>
Channel(ChannelID, T) -> Channel<T>;

}  // namespace VDP
