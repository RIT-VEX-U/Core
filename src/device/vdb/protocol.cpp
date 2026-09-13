#include "core/device/vdb/protocol.hpp"

#include <stdio.h>

#include <bit>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "core/device/vdb/types.hpp"

namespace VDP {
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

VDP::Packet checksum_pac(VDP::Packet in) {
  VDP::Packet out;
  uint32_t checksum = CRC32::calculate(in.data(), in.size());

  for (std::size_t i = 0; i < sizeof(checksum); ++i) {
    out.push_back(static_cast<uint8_t>(checksum >> (i * 8)));
  }
  return out;
}

uint8_t make_header_byte(PacketHeader head) { return (uint8_t)head.type | (uint8_t)head.func; }

PacketHeader decode_header_byte(uint8_t header_byte) {
  const PacketType pt = (PacketType)(header_byte & 0b11111110);
  const PacketFunction func = (PacketFunction)(header_byte & 0b00000001);

  return {pt, func};
}

}  // namespace VDP
