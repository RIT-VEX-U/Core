#pragma once
#include "../core/include/device/cobs_device.h"
#include "../core/include/device/vdb/protocol.hpp"
#include "vex.h"


/**
 * Defines a COBS Serial Device to transmit VDB data through
 */
namespace VDB {
class Device : public VDP::AbstractDevice, COBSSerialDevice {
  public:
    /**
     * creates a COBS Serial device for VDB data at a specified port with a specified baud rate
     * @param port the port the debug board is connected to
     * @param baud_rate the baud rate for the debug board to use
     */
    explicit Device(int32_t port, int32_t baud_rate);
    /**
     * sends a packet of data to the debug board
     */
    bool send_packet(const VDP::Packet &packet) override; // From VDP::AbstractDevice
    /**
     * defines a callback to a functions that calls when the register recieves data from the debug board
     * @param callback the callback function to call
     */
    void register_receive_callback(std::function<void(const VDP::Packet &packet)> callback
    ) override; // From VDP::AbstractDevice
    /**
     * defines a callback to a packet that is called when the cobs device decodes a packet
     * @param pac the packet to get the callback from
     */
    void cobs_packet_callback(const Packet &pac);

  private:
    std::function<void(const VDP::Packet &packet)> callback;
};

} // namespace VDB