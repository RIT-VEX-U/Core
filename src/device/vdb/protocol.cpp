#include "core/device/vdb/protocol.hpp"

#include <cstdint>
#include <cstring>
#include <functional>
#include <sstream>
#include <stdio.h>
#include <string>
#include <utility>
#include <vector>

#include "core/device/vdb/types.hpp"

namespace VDP {
/**
 * @return the channel's id
 */
ChannelID Channel::getID() const { return id; }
/*
 * prints out the packet in individual bytes
 */
void dump_packet(const Packet &pac) {
    int i = 0;
    for (const uint8_t d : pac) {
        if (i % 16 == 0 && i != 0) {
            printf("\n");
        }
        printf("%02x ", (int)d);
        i++;
    }
    printf("\n");
}
/**
 * @param t the VDP type to return a string of
 * @return a string of the VDP type
 */
std::string to_string(Type t) {
    switch (t) {
    case Type::Record:
        return "record";
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
    }

    return "<<UNKNOWN TYPE>>";
}
/**
 * adds indents to a stringstream
 * @param ss the stringstream to add indents to
 * @param indent the amount of double spaced indents to add
 */
void add_indents(std::stringstream &ss, size_t indent) {
    for (size_t i = 0; i < indent; i++) {
        ss << "  ";
    }
}
/**
 * Creates a Part with a name
 * a part is essentially data formatted so that it can be sent to the debug board
 * @param name name for the Part
 */
Part::Part(std::string name) : name(std::move(name)) {}
/*
 * Deleter for the Part, used to delete the data once it is no longer needed
 * i.e after it has been sent to the debug board
 */
Part::~Part() {}

void Part::response() {}
/**
 *  @return a stringstream of the Part with the format "name: string"
 */
std::string Part::pretty_print() const {
    std::stringstream ss;
    this->pprint(ss, 0);

    return ss.str();
}
/**
 * @return a stringstream of the Part's data with the format "name: value"
 */
std::string Part::pretty_print_data() const {
    std::stringstream ss;
    this->pprint_data(ss, 0);
    return ss.str();
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
PacketWriter::PacketWriter(VDP::Packet &scratch) : sofar(scratch) {}
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
void PacketWriter::write_string(const std::string &str) {
    // inserts a string into the end of the packet in bytes
    sofar.insert(sofar.end(), str.begin(), str.end());
    // adds a 0 byte after the string to signal the end of the string
    sofar.push_back(0);
}
/**
 * writes a broadcast acknowledgement of a channel to the packet
 * @param chan the channel to write the acknowledgement for
 */
void PacketWriter::write_channel_acknowledge(const Channel &chan) {
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
void PacketWriter::write_channel_broadcast(const Channel &chan) {
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
 * writes a receive packet to the packets
 * @param chan the channel to request
 */
void PacketWriter::write_response(const std::vector<Channel> &channels) {
    clear();
    // makes a header byte with the type broadcast and the function Receive
    const uint8_t header = make_header_byte(PacketHeader{PacketType::Data, PacketFunction::Response});

    // writes the header byte and number of channels to send to the packet
    write_number<uint8_t>(header);

    write_number<size_t>(channels.size());
    // writes each channel to the packet
    for (Channel chan : channels) {
        const uint8_t header = make_header_byte(PacketHeader{PacketType::Data, PacketFunction::Response});

        // writes the header byte and channel id to the packet
        write_number<uint8_t>(header);
        write_number<ChannelID>(chan.getID());

        // writes the data from the channel to the packet
        chan.data->write_message(*this);
    }

    // creates and writes the Checksum to the packet
    uint32_t crc = CRC32::calculate(sofar.data(), sofar.size());
    write_number<uint32_t>(crc);
}
/**
 * writes the data from a channel to the packet
 * @param chan the Channel to write the data from
 */
void PacketWriter::write_data_message(const Channel &chan) {
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
    write_number<uint32_t>(crc);
}
/**
 * @return the packet the writer is writing to
 */
const Packet &PacketWriter::get_packet() const { return sofar; }
/**
 *  deleter for the device, used to delete it when it is no longer needed
 */
AbstractDevice::~AbstractDevice() {}
/**
 * creates a decoder to decode a packet
 * @param pac the packet reader to make a decoder from
 * @return the Part Pointer for the data from the packet
 */
PartPtr make_decoder(PacketReader &pac) {
    /**
     * gets the type and name of the packet and contstructs a Part pointer from it
     */
    const Type t = pac.get_type();
    const std::string name = pac.get_string();

    switch (t) {
    case Type::String:
        return PartPtr(new String(name));
    case Type::Record:
        return PartPtr(new Record(name, pac));

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
static constexpr auto PACKET_TYPE_BIT_LOCATION = 7;
static constexpr auto PACKET_FUNCTION_BIT_LOCATION = 5;
/**
 * creates a byte from a given packet header
 * @return the header byte created
 */
uint8_t make_header_byte(PacketHeader head) {

    uint8_t b = 0;
    // ORs the byte with the header type bit shifted over to the packet type but location
    b |= ((uint8_t)head.type) << PACKET_TYPE_BIT_LOCATION;
    // ORs the byte with the header function bit shifted over to the packet function bit location
    b |= ((uint8_t)head.func) << PACKET_FUNCTION_BIT_LOCATION;
    return b;
}
/**
 * @param hb the header byte to decode
 * @return a PacketHeader with the Function and Type from the byte decoded
 */
PacketHeader decode_header_byte(uint8_t hb) {
    // sets the packet type to the header byte shifted by the type bit location ANDed with 1
    const PacketType pt = (PacketType)((hb >> PACKET_TYPE_BIT_LOCATION) & 1);
    // sets the packet function to the header byte shifted by the function location bit ANDed with 1
    const PacketFunction func = (PacketFunction)((hb >> PACKET_FUNCTION_BIT_LOCATION) & 1);
    return {pt, func};
}
/**
 * Decodes the broadcast in a packet
 * @param packet the packet to decode
 * @return the pair of the Channel ID and the Part Pointer of the packet schematic
 */
std::pair<ChannelID, PartPtr> decode_broadcast(const Packet &packet) {
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
} // namespace VDP
