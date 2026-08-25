#include "core/device/vdb_device.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <vector>
namespace VDB {
/**
 * delay for ms time
 * @param ms the ms to delay for
 */
void delay_ms(uint32_t ms) { vexDelay(ms); }
/**
 * @return the time in ms of the bot since startup
 */
uint32_t time_ms() { return vexSystemTimeGet(); }
/**
 * the thread for sending data to the wire
 */
int Device::serial_thread(void* vself) {
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
      if (self.callback) {
        self.callback(decoded);
      }
      did_something = true;
    }
    if (!did_something) {
      vexDelay(NO_ACTIVITY_DELAY);
    }
  }
  return 0;
}
/**
 * creates a COBS Serial device for VDB data at a specified port with a specified baud rate
 * @param port the port the debug board is connected to
 * @param baud_rate the baud rate for the debug board to use
 */
Device::Device(int32_t port, int32_t baud_rate) : COBSSerialDevice(port, baud_rate) {
  serial_task = vex::task(Device::serial_thread, (void*)this, vex::thread::threadPriorityHigh);
}

bool Device::add_to_queue(const VDP::Packet& packet) {
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
Device::SEND_PACKET_STATE Device::write_packet_from_queue() {
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

}  // namespace VDB
