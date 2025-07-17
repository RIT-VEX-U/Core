#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "core/device/wrapper_device.hpp"
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
int Device::serial_thread(void *vself) {
    // defines itself within the thread
    Device &self = *(Device *)vself;

    // sets up a buffer
    static constexpr size_t buflen = 4096;
    static uint8_t buf[buflen] = {0};
    vex::timer timer;
    // loop for the thread
    while (true) {
        bool did_something = false;
        // Lame replacement for blocking IO. We can't just wait and tell the
        // scheduler to go work on something else while we wait for packets so
        // instead, if we're getting nothing in and have nothing to send, block
        // ourselves.

        // Writing
        if (self.write_packet_if_avail()) {
            did_something = true;
        }
        // Reading
        if (self.poll_incoming_data_once()) {
            Packet decoded = {};
            decoded = self.get_last_decoded_packet();
            self.callback(decoded);
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
    serial_task = vex::task(Device::serial_thread, (void *)this, vex::thread::threadPriorityHigh);
}

bool Device::send_packet(const VDP::Packet &packet) {
    if (outbound_packets.size() >= MAX_OUT_QUEUE_SIZE) {
        return false;
    }
    outbound_packets.push_front(packet);
    return true;
}

/**
 * writes a packet to the device as soon as it is available
 */
bool Device::write_packet_if_avail() {
    // packet to write to the device
    WirePacket outbound_packet = {};
    // lock the serial port
    outbound_mutex.lock();
    // check if we have a packet to write
    if (outbound_packets.size() > 0) {
        // of we do take the latest packet out of the vector of packets we have
        outbound_packet = outbound_packets.back();
        outbound_packets.pop_back();
    }
    // unlock
    outbound_mutex.unlock();
    if (outbound_packet.size() == 0) {
        return false;
    }
    send_cobs_packet_blocking(outbound_packet.data(), outbound_packet.size());

    return true;
}
/**
 * defines a callback to a functions that calls when the register recieves data from the device
 * @param callback the callback function to call
 */
void Device::register_receive_callback(std::function<void(const VDP::Packet &packet)> new_callback) {
    callback = std::move(new_callback);
}

} // namespace VDB
