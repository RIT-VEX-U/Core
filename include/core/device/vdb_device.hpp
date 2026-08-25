#pragma once
#include <deque>

#include "core/device/cobs_device.h"
#include "core/device/vdb/protocol.hpp"
#include "vex.h"

/**
 * Defines a COBS Serial Device to transmit VDB data through
 */
namespace VDB {
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
  explicit Device(int32_t port, int32_t baud_rate);

  bool add_to_queue(const VDP::Packet& packet);

 private:
  /**
   * @brief Packets that have been encoded and are waiting for their turn
   * to be sent out on the wire
   */
  std::deque<VDP::Packet> outbound_packets{};
  vex::mutex outbound_mutex;
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

  /**
   * the thread for sending data to the wire
   */
  static int serial_thread(void* self);

  SEND_PACKET_STATE write_packet_from_queue();

  // Task that deals with the low level writing and reading bytes from the wire
  vex::task serial_task;

  std::function<void(const VDP::Packet& packet)> callback;
};

}  // namespace VDB
