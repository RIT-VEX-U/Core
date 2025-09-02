#pragma once
#include "core/device/cobs_device.h"
#include "core/device/vdb/protocol.hpp"
#include "vex.h"
#include <deque>

/**
 * Defines a COBS Serial Device to transmit VDB data through
 */
namespace VDB {
class Device : public VDP::AbstractDevice, public COBSSerialDevice {
  public:
    static constexpr int32_t NO_ACTIVITY_DELAY = 2; // ms
    static constexpr std::size_t MAX_OUT_QUEUE_SIZE = 50;
    static constexpr std::size_t MAX_IN_QUEUE_SIZE = 50;
    /**
     * creates a COBS Serial device for VDB data at a specified port with a specified baud rate
     * @param port the port the debug board is connected to
     * @param baud_rate the baud rate for the debug board to use
     */
    explicit Device(int32_t port, int32_t baud_rate);

    bool send_packet(const VDP::Packet &packet);
    /**
     * defines a callback to a functions that calls when the register recieves data from the debug board
     * @param callback the callback function to call
     */
    void register_receive_callback(std::function<void(const VDP::Packet &packet)> callback
    ) override; // From VDP::AbstractDevice

  private:
    /**
     * @brief Packets that have been encoded and are waiting for their turn
     * to be sent out on the wire
     */
    std::deque<WirePacket> outbound_packets{};
    vex::mutex outbound_mutex;
    /**
     * @brief Packets that have been read from the wire and split up but that are
     * still COBS encoded
     */
    std::deque<WirePacket> inbound_packets;

    vex::mutex inbound_mutex;
    /**
     * @brief Working buffer that the reading thread uses to assemble packets
     * until it finds a full COBS packet
     */
    WirePacket inbound_buffer;
    /**
     * the thread for decoding data from the wire
     */
    static int decode_thread(void *self);

    /**
     * the thread for sending data to the wire
     */
    static int serial_thread(void *self);

    bool write_packet_if_avail();
    
    // Task that deals with the low level writing and reading bytes from the wire
    vex::task serial_task;

    bool write_request();
    std::function<void(const VDP::Packet &packet)> callback;
};

} // namespace VDB