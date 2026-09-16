#pragma once
#include <array>
#include <cstdint>
#include <deque>
#include <functional>
#include <iterator>
#include <tuple>
#include <span>

#include "core/device/cobs_device.h"
#include "core/device/vdb/protocol.hpp"
#include "vex.h"

/**
 * Defines a COBS Serial Device to transmit VDB data through
 */
namespace VDB {
template <typename... Fields>
class Device : public COBSSerialDevice {
 public:
  static constexpr int32_t NO_ACTIVITY_DELAY = 2;  // ms
  static constexpr std::size_t MAX_OUT_QUEUE_SIZE = 50;
  static constexpr std::size_t MAX_IN_QUEUE_SIZE = 50;

  enum SEND_PACKET_STATE { SUCCESS, NONE_QUEUED, ERROR };

  /**
   * creates a COBS Serial device for VDB data at a specified port with a specified baud rate
   * @param port the port the debug board is connected to
   * @param baud_rate the baud rate for the debug board to use
   */
  explicit Device(int32_t port, int32_t baud_rate, VDP::Channel<Fields>... channels) : COBSSerialDevice(port, baud_rate), channels_(std::move(channels)...) {

    static_assert(sizeof...(Fields) <= VDP::MAX_CHANNELS, "There can be no more than 256 Channels sent to a VDB Device");
    size_t next_id = 0;
    std::apply([&](auto&... channel) {
      (channel.set_id(static_cast<VDP::ChannelID>(next_id++)), ...);
    }, channels_);
    serial_task = vex::task(Device::serial_thread, (void*)this, vex::thread::threadPriorityHigh);
  }

  bool add_to_queue(const VDP::Packet& packet) {
    if (packet.empty()) {
      printf("VDP WARNING: Empty Packets are not allowed\n");
      return false;
    }
    outbound_mutex.lock();
    if (outbound_packets.size() >= MAX_OUT_QUEUE_SIZE) {
      outbound_mutex.unlock();
      return false;
    }
    outbound_packets.push_back(packet);
    outbound_mutex.unlock();
    return true;
  }

  /**
   * writes a packet to the device as soon as it is available
   */
  SEND_PACKET_STATE write_packet_from_queue() {
  // packet to write to the device
  VDP::Packet outbound_packet = {};
  // lock the serial port
  outbound_mutex.lock();
  // check if we have a packet to write
  if (outbound_packets.size() > 0) {
    // of we do take the latest packet out of the vector of packets we have
    outbound_packet = std::move(outbound_packets.front());
    outbound_packets.pop_front();
  }
  // unlock
  outbound_mutex.unlock();
  if (outbound_packet.size() == 0) {
    return NONE_QUEUED;
  }

  int sent = send_cobs_packet_blocking(outbound_packet.data(), outbound_packet.size());

  if (sent >= 0) {
    return SUCCESS;
  } else {
    printf("Failed to send packet (%d):\n", sent);
    hexdump(outbound_packet.data(), outbound_packet.size());
    return ERROR;
  }
}

  /**
   * the thread for sending data to the wire
   */
  static int serial_thread(void* vself) {
    // defines itself within the thread
    Device& self = *(Device*)vself;

    // serial thread loop
    while (true) {
      bool did_something = false;
      // Lame replacement for blocking IO. We can't just wait and tell the
      // scheduler to go work on something else while we wait for packets so
      // instead, if we're getting nothing in and have nothing to send, block
      // ourselves.

      // Writing
      SEND_PACKET_STATE send_state = self.write_packet_from_queue();
      if (send_state != NONE_QUEUED) {
        did_something = true;
      }
      // Reading
      if (self.poll_incoming_data_once()) {
        Packet decoded = {};
        decoded = self.get_last_decoded_packet();
        self.apply_packet(decoded);
        did_something = true;
      }
      if (!did_something) {
        vexDelay(NO_ACTIVITY_DELAY);
      }
    }
    return 0;
  }

void send_channel(VDP::ChannelID id) {
  VDP::Channel to_send = std::get<id>(channels_);
  std::apply([&](const auto&... channel) {
    ([&] {
      if (id == channel.get_id()) {
        if (channel.acknowledged == true) {
          this->add_to_queue(channel.serialize(VDP::PacketType::Data));
        }
        else {
          this->add_to_queue(channel.serialize(VDP::PacketType::Schema));
        }
      }
    }(), ...);
  }, channels_);
}

void apply_packet(VDP::Packet in) {
  const VDP::PacketValidity status = VDP::validate_packet(in);

  if (status == VDP::PacketValidity::BadChecksum) {
    VDPWarnf("Controller: Bad packet checksum. Skipping");
    return;
  } else if (status == VDP::PacketValidity::TooSmall) {
    VDPWarnf("Controller: Packet too small to be valid (%d bytes). Skipping", (int)in.size());
    return;
  } else if (status != VDP::PacketValidity::Ok) {
    VDPWarnf("Controller: Unknown validity of packet (BAD). Skipping");
    return;
  }
  VDP::PacketHeader header = VDP::decode_header_byte(in[0]);
  switch (header.func) {
    case VDP::PacketFunction::Send: {
      // decode packet and apply to channel
      VDP::ChannelID id_to_update = in[1];
      std::apply([&](const auto&... channel) {
        ([&] {
         if (id_to_update == channel.get_id()) {
          channel.apply_update(std::span(in).subspan(2));
         }
        }(), ...);
      }, channels_);
      break;
    }
    // mark channel as acknowledged, dont do anything if data packet
    case VDP::PacketFunction::Acknowledge:
      if (header.type == VDP::PacketType::Schema) {
        VDP::ChannelID acked_id = in[1];
          std::apply([&](const auto&... channel) {
            ([&] {
             if (acked_id == channel.get_id()) {
                channel.acknowledge();
             }
            }(), ...);
          }, channels_);
      }
      break;
      // send out ack packet when empty
    case VDP::PacketFunction::Holding:
      if (this->outbound_packets.empty()) {
        VDP::Packet out;
        out.push_back(VDP::make_header_byte({.type = VDP::PacketType::Schema, .func = VDP::PacketFunction::Acknowledge}));
        VDP::Packet checksum = VDP::checksum_pac(out);
        out.insert(out.end(), checksum.begin(), checksum.end());
        this->add_to_queue(out);
      }
      break;
  }
}

 private:
  /**
   * @brief Packets that have been encoded and are waiting for their turn
   * to be sent out on the wire
   */
  std::deque<VDP::Packet> outbound_packets{};
  vex::mutex outbound_mutex;
  std::tuple<VDP::Channel<Fields>...> channels_;

  bool waiting_to_recieve = false;
  /**
   * @brief Packets that have been read from the wire and split up but that are
   * still COBS encoded
   */
  std::deque<WirePacket> inbound_packets;

  /**
   * @brief Working buffer that the reading thread uses to assemble packets
   * until it finds a full COBS packet
   */
  WirePacket inbound_buffer;

  // Task that deals with the low level writing and reading bytes from the wire
  vex::task serial_task;

  std::function<void(const VDP::Packet& packet)> callback;
};
template <typename... Fields>
Device(int32_t, int32_t, VDP::Channel<Fields>...) -> Device<Fields...>;

}  // namespace VDB
