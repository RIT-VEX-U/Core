#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "../core/include/device/wrapper_device.hpp"

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
 * creates a COBS Serial device for VDB data at a specified port with a specified baud rate
 * @param port the port the debug board is connected to
 * @param baud_rate the baud rate for the debug board to use
 */
Device::Device(int32_t port, int32_t baud_rate) : COBSSerialDevice(port, baud_rate) {}
/**
 * sends a packet of data to the device
 * @param packet the packet of data to send to  the device
 */
bool Device::send_packet(const VDP::Packet &packet) {
    return COBSSerialDevice::send_cobs_packet_blocking(packet.data(), packet.size());
}
/**
 * defines a callback to a functions that calls when the register recieves data from the device
 * @param callback the callback function to call
 */
void Device::register_receive_callback(std::function<void(const VDP::Packet &packet)> new_callback) {
    callback = std::move(new_callback);
}
/**
 * defines a callback to a packet that is called when the cobs device decodes a packet
 * @param pac the packet to get the callback from
 */
void Device::cobs_packet_callback(const Packet &pac) { callback(pac); }

} // namespace VDB
